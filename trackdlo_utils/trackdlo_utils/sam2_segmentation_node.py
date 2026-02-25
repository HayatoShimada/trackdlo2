#!/usr/bin/env python3
"""SAM2 Segmentation Node: Segment Anything Model 2 for DLO segmentation.

User clicks on the DLO in the first frame to provide a prompt.
SAM2 ImagePredictor runs per-frame inference, using the click points
and the previous frame's bounding box as prompts.

Install: pip install sam2-pytorch  (or: pip install git+https://github.com/facebookresearch/sam2.git)
"""

import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class SAM2SegmentationNode(Node):
    def __init__(self):
        super().__init__('sam2_segmentation')
        self.bridge = CvBridge()

        self.declare_parameter('rgb_topic', '/camera/color/image_raw')
        self.declare_parameter('sam2_checkpoint', 'facebook/sam2.1-hiera-small')
        self.declare_parameter('force_cpu', False)

        rgb_topic = self.get_parameter('rgb_topic').value
        checkpoint = self.get_parameter('sam2_checkpoint').value
        force_cpu = self.get_parameter('force_cpu').value

        # Device selection
        import torch
        if not force_cpu and torch.cuda.is_available():
            self.device = torch.device('cuda')
            self.get_logger().info('Using CUDA for SAM2')
        else:
            self.device = torch.device('cpu')
            self.get_logger().info('Using CPU for SAM2 (inference may be slow)')

        # State
        self.click_points = []
        self.click_labels = []
        self.prompt_image = None
        self.prompting_done = False
        self.predictor = None
        self.latest_image = None
        self.prev_mask = None
        self._inference_running = False
        self._inference_lock = threading.Lock()

        # Load SAM2 model
        self.get_logger().info(f'Loading SAM2 model: {checkpoint}...')
        try:
            from sam2.build_sam import build_sam2
            from sam2.sam2_image_predictor import SAM2ImagePredictor
            self.SAM2ImagePredictor = SAM2ImagePredictor
            sam2_model = build_sam2(checkpoint, device=self.device)
            self.predictor = SAM2ImagePredictor(sam2_model)
            self.get_logger().info('SAM2 model loaded successfully.')
        except ImportError:
            self.get_logger().error(
                'SAM2 not installed. Install with: '
                'pip install git+https://github.com/facebookresearch/sam2.git')
            raise
        except Exception as e:
            self.get_logger().error(f'Failed to load SAM2 model: {e}')
            self.get_logger().info(
                'Trying alternative loading method with from_pretrained...')
            try:
                from sam2.sam2_image_predictor import SAM2ImagePredictor
                self.SAM2ImagePredictor = SAM2ImagePredictor
                self.predictor = SAM2ImagePredictor.from_pretrained(
                    checkpoint, device=self.device)
                self.get_logger().info('SAM2 model loaded via from_pretrained.')
            except Exception as e2:
                self.get_logger().error(f'All loading methods failed: {e2}')
                raise

        self.rgb_sub = self.create_subscription(
            Image, rgb_topic, self.image_callback, 10)
        self.mask_pub = self.create_publisher(
            Image, '/trackdlo/segmentation_mask', 10)

        self.timer = self.create_timer(1.0 / 30.0, self.gui_timer_callback)
        self.get_logger().info(
            'SAM2 node started. Click on the DLO in the first frame, '
            'then press ENTER to confirm. Right-click for background points.')

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.click_points.append([x, y])
            self.click_labels.append(1)
            self.get_logger().info(f'Foreground point at ({x}, {y})')
        elif event == cv2.EVENT_RBUTTONDOWN:
            self.click_points.append([x, y])
            self.click_labels.append(0)
            self.get_logger().info(f'Background point at ({x}, {y})')

    def image_callback(self, msg):
        # 1. カメラ画像を受信し、OpenCV形式(BGR)に変換
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        if not self.prompting_done:
            # 2. 初期プロンプト（クリック入力）が未完了の場合は、最初の1フレーム目を保持してスキップ
            if self.prompt_image is None:
                self.prompt_image = self.latest_image.copy()
            return

        # 3. バックグラウンドでの推論スレッドの起動（画像受信コールバックをブロックさせないため）
        with self._inference_lock:
            if self._inference_running:
                return  # 既に推論中の場合は、新しいフレームはスキップ（ドロップ）する
            self._inference_running = True
        thread = threading.Thread(target=self._inference_thread, daemon=True)
        thread.start()

    def _inference_thread(self):
        try:
            self.run_inference()
        finally:
            with self._inference_lock:
                self._inference_running = False

    def gui_timer_callback(self):
        if self.prompting_done or self.prompt_image is None:
            cv2.waitKey(1)
            return

        display = self.prompt_image.copy()
        for pt, label in zip(self.click_points, self.click_labels):
            color = (0, 255, 0) if label == 1 else (0, 0, 255)
            cv2.circle(display, tuple(pt), 5, color, -1)
            cv2.circle(display, tuple(pt), 7, color, 2)

        cv2.putText(display,
                    'L-click: foreground | R-click: background | ENTER: confirm | R: reset',
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.imshow('SAM2 - Click on DLO', display)
        cv2.setMouseCallback('SAM2 - Click on DLO', self.mouse_callback)

        key = cv2.waitKey(1) & 0xFF
        if key == 13 and len(self.click_points) > 0:  # Enter
            self.get_logger().info(
                f'Prompt confirmed with {len(self.click_points)} points. Starting segmentation...')
            cv2.destroyWindow('SAM2 - Click on DLO')
            # ユーザーのプロンプト入力が完了したフラグを立て、毎フレーム推論フェーズへ移行
            self.prompting_done = True
            # 最初のフレーム画像に対して最初の推論を実行する
            # Run initial inference on the prompt image
            self.run_inference_on(self.prompt_image)
        elif key == ord('r'):
            self.click_points.clear()
            self.click_labels.clear()
            self.get_logger().info('Points reset.')

    def run_inference_on(self, image):
        """Run SAM2 inference on a single image with accumulated prompts."""
        try:
            # 4. SAM2モデルへの入力準備（RGB変換と画像のセット）
            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            self.predictor.set_image(image_rgb)

            # 5. Pointプロンプトの構築（GUIでクリックした前景/背景点）
            point_coords = np.array(self.click_points, dtype=np.float32)
            point_labels = np.array(self.click_labels, dtype=np.int32)

            # 6. Boxプロンプトの構築（トラッキング用：前フレームのマスクから計算）
            # Use previous mask's bounding box as additional prompt
            input_box = None
            if self.prev_mask is not None:
                coords = np.where(self.prev_mask > 0)
                if len(coords[0]) > 0:
                    y_min, y_max = coords[0].min(), coords[0].max()
                    x_min, x_max = coords[1].min(), coords[1].max()
                    # 予測のブレを吸収するため、計算されたバウンディングボックスを上下左右に少し広げる
                    margin = 20
                    h, w = image.shape[:2]
                    input_box = np.array([
                        max(0, x_min - margin),
                        max(0, y_min - margin),
                        min(w, x_max + margin),
                        min(h, y_max + margin),
                    ], dtype=np.float32)

            # 7. SAM2モデルによる複数マスク（マルチマスク）の推論
            masks, scores, _ = self.predictor.predict(
                point_coords=point_coords,
                point_labels=point_labels,
                box=input_box,
                multimask_output=True,
            )

            # 8. 出力されたマスク候補のうち、最もスコア(信頼度)の高いものを選択し、2値化画像(0 or 255)に変換
            best_idx = np.argmax(scores)
            mask = (masks[best_idx] > 0).astype(np.uint8) * 255

            # 9. 次フレームのBoxプロンプト探索用に結果を保存し、ROS 2トピックとしてパブリッシュ
            self.prev_mask = mask
            self.publish_mask(mask)

        except Exception as e:
            self.get_logger().warn(f'SAM2 inference error: {e}', throttle_duration_sec=2.0)

    def run_inference(self):
        if self.latest_image is None:
            return
        self.run_inference_on(self.latest_image)

    def publish_mask(self, mask):
        mask_msg = self.bridge.cv2_to_imgmsg(mask, 'mono8')
        mask_msg.header.stamp = self.get_clock().now().to_msg()
        self.mask_pub.publish(mask_msg)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SAM2SegmentationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
