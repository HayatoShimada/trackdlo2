# trackdlo_ros2 設計ドキュメント

## 概要

trackdlo2リポジトリからWebカメラ（RealSenseシリーズ）認識、TrackDLOによるDLOモデル生成、プレビューウィンドウに特化した別プロジェクト「trackdlo_ros2」を作成する。

## 方針

- trackdlo2をGitHub上でフォークし、不要部分を削除する（ミニマル削除アプローチ）
- ROS2ベースを維持（Humble + Jazzy対応）
- セグメンテーションを差し替え可能な構成にする
- 他のROS2プロジェクトとhost network経由で連携可能にする

## プロジェクト構成

```
trackdlo_ros2/
├── trackdlo_perception/       # CPD-LLEコア + trackdlo_node + 初期化
├── trackdlo_utils/            # composite_view, hsv_tuner, param_tuner
├── trackdlo_bringup/          # launch, config, rviz
├── trackdlo_msgs/             # カスタムメッセージ（将来用）
├── trackdlo_segmentation/     # 【新規】セグメンテーション基底クラス + HSV実装
├── docker/
│   ├── Dockerfile.base        # ROS2 (ARG ROS_DISTRO=humble|jazzy) + 共通依存
│   ├── Dockerfile.core        # perception + utils + HSV segmentation
│   ├── Dockerfile.sam2        # SAM2セグメンテーションノード
│   ├── Dockerfile.yolo        # YOLOセグメンテーションノード（将来）
│   ├── docker-compose.yml     # メイン構成 (profiles でセグメンテーション選択)
│   ├── docker-compose.nvidia.yml  # NVIDIA GPUオーバーライド
│   ├── build.sh
│   └── run.sh
└── README.md
```

## 削除対象

### パッケージ
- `trackdlo_moveit/` — UR5ロボット制御（全体）
- `trackdlo_description/` — URDF/SDF ロボットモデル（全体）

### Dockerファイル
- `docker/Dockerfile.gazebo` — Gazebo Fortress
- `docker/Dockerfile.moveit` — MoveIt2
- `docker/docker-compose.yml`（フルスタック用） — 新しいcomposeに置き換え
- `docker/docker-compose.amd.yml` — 不要（既に削除済み）
- `docker/docker-compose.realsense.amd.yml` — 不要（既に削除済み）

### Launchファイル
- `sim_full_pipeline.launch.py` — Gazeboシミュレーション
- `full_pipeline.launch.py` — シミュレーション + ロボット制御
- `ur5_gazebo.launch.py` — Gazebo UR5起動
- `visualize_output.launch.py` — composite_viewに統合済み

### 設定ファイル
- `trackdlo_bringup/config/trackdlo_params.yaml` — シミュレーション用パラメータ

## Launchファイル構成

### 残す / リネームするファイル

| ファイル | 変更 | 用途 |
|---|---|---|
| `realsense_test.launch.py` | `trackdlo.launch.py`にリネーム | メインlaunch |
| `camera.launch.py` | そのまま | カメラ + TFのみ |
| `evaluation.launch.py` | そのまま | 評価モード |

### 起動方法

```bash
# 基本（HSV内蔵）
ros2 launch trackdlo_bringup trackdlo.launch.py

# 外部セグメンテーション使用時
ros2 launch trackdlo_bringup trackdlo.launch.py use_external_mask:=true

# RealSense機種指定
ros2 launch trackdlo_bringup trackdlo.launch.py camera_model:=D415
```

## セグメンテーションアーキテクチャ

### 設計原則
- 統一トピックインターフェース: 全セグメンテーションノードが `/trackdlo/segmentation_mask` (sensor_msgs/Image, mono8) にpublish
- 基底クラスによる共通化
- Docker profilesで起動するセグメンテーションを選択

### 新規パッケージ `trackdlo_segmentation`

```
trackdlo_segmentation/          # ament_python パッケージ
├── package.xml
├── setup.py
├── trackdlo_segmentation/
│   ├── __init__.py
│   ├── base.py                # SegmentationNodeBase（基底クラス）
│   └── hsv_node.py            # HSV実装（trackdlo_utilsのhsv_tuner_nodeから移動）
```

### 基底クラス

```python
from abc import abstractmethod
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class SegmentationNodeBase(Node):
    """全セグメンテーションノードの共通インターフェース"""

    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.bridge = CvBridge()
        self.mask_pub = self.create_publisher(
            Image, '/trackdlo/segmentation_mask', 10
        )
        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self._on_image, 10
        )

    def _on_image(self, msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        mask = self.segment(cv_image)
        mask_msg = self.bridge.cv2_to_imgmsg(mask, 'mono8')
        mask_msg.header = msg.header
        self.mask_pub.publish(mask_msg)

    @abstractmethod
    def segment(self, cv_image: np.ndarray) -> np.ndarray:
        """入力: BGR画像 (H, W, 3), 出力: バイナリマスク (H, W), 値は0または255"""
        ...
```

### HSVセグメンテーションの扱い

- `trackdlo_segmentation/hsv_node.py`: `SegmentationNodeBase`を継承したHSVセグメンテーション。GUIスライダー付き（現在の`hsv_tuner_node.py`の機能を統合）。coreコンテナに同梱。
- `trackdlo_utils/hsv_tuner_node.py`: 移動後に削除。

### 外部セグメンテーション（別コンテナ / 別パッケージ）

SAM2, YOLO, DeepLab等は `SegmentationNodeBase` を継承して実装。同リポジトリ内に置くことも、別リポジトリにすることも可能。依存は `trackdlo_segmentation` パッケージのみ。

### 通信フロー

```
[RealSense] → /camera/color/image_raw
                    │
         ┌──────────┼──────────┐
         ↓          ↓          ↓
     [HSV node] [SAM2 node] [YOLO node]   ← どれか1つだけ起動
         │          │          │
         └──────────┼──────────┘
                    ↓
    /trackdlo/segmentation_mask (mono8)
                    ↓
            [trackdlo_node]  ← use_external_mask=true
```

## Docker構成

### docker-compose.yml

```yaml
services:
  trackdlo-core:
    build:
      context: ..
      dockerfile: docker/Dockerfile.core
      args:
        ROS_DISTRO: humble
    network_mode: host
    environment:
      - DISPLAY
      - RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
      - ROS_DOMAIN_ID=42
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix
    devices:
      - /dev/video0:/dev/video0
    privileged: true

  trackdlo-sam2:
    build:
      context: ..
      dockerfile: docker/Dockerfile.sam2
      args:
        ROS_DISTRO: humble
    network_mode: host
    environment:
      - RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
      - ROS_DOMAIN_ID=42
    profiles: ["sam2"]

  trackdlo-yolo:
    build:
      context: ..
      dockerfile: docker/Dockerfile.yolo
      args:
        ROS_DISTRO: humble
    network_mode: host
    environment:
      - RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
      - ROS_DOMAIN_ID=42
    profiles: ["yolo"]
```

### 使い方

```bash
# coreのみ（HSVセグメンテーション）
docker compose up trackdlo-core

# core + SAM2
docker compose --profile sam2 up

# core + YOLO
docker compose --profile yolo up

# Jazzyでビルド
docker compose build --build-arg ROS_DISTRO=jazzy
```

### GPU対応

`docker-compose.nvidia.yml` で NVIDIA GPU のデプロイ設定をオーバーライド:
```bash
docker compose -f docker-compose.yml -f docker-compose.nvidia.yml --profile sam2 up
```

## Humble / Jazzy 対応

### Dockerfileでの分岐

```dockerfile
ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-perception
```

### CMakeLists.txtでの分岐（必要な場合のみ）

```cmake
if($ENV{ROS_DISTRO} STREQUAL "jazzy")
  # Jazzy固有の依存やフラグ
endif()
```

### RMWの明示指定

両バージョンで `rmw_cyclonedds_cpp` を統一使用し、他のROS2プロジェクトとの互換性を確保。

## 外部連携インターフェース

### 公開トピック

| トピック | 型 | 用途 |
|---|---|---|
| `/trackdlo/results_pc` | PointCloud2 | 追跡済みDLOノード座標（メイン出力） |
| `/trackdlo/results_img` | Image | 追跡結果の可視化画像 |
| `/trackdlo/segmentation_mask` | Image | セグメンテーションマスク |
| `/trackdlo/init_nodes` | PointCloud2 | 初期化ノード（1回のみ） |

### 購読トピック

| トピック | 型 | 用途 |
|---|---|---|
| `/camera/color/image_raw` | Image | RGB画像 |
| `/camera/aligned_depth_to_color/image_raw` | Image | 深度画像 |
| `/camera/aligned_depth_to_color/camera_info` | CameraInfo | カメラ内部パラメータ |
| `/trackdlo/segmentation_mask` | Image | 外部セグメンテーション（オプション） |

### 連携方法

- `network_mode: host` + `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` + 同一 `ROS_DOMAIN_ID` で他コンテナと自動通信
- トピック名はYAMLパラメータでリマップ可能
- カスタムメッセージは使わず標準メッセージ型のみ（`trackdlo_msgs`は将来の拡張用に保持）

## RealSenseシリーズ対応

- `realsense2_camera` ドライバが D415/D435/D455 等を自動認識
- `camera_model` launchパラメータで機種固有の解像度・フィルタ設定を自動調整
- RealSenseプリセットJSON（`preset_decimation_4.0_depth_step_100.json`）を保持

## プレビューウィンドウ

現在の `composite_view_node.py` をそのまま使用:

```
┌──────────────────┬──────────────────┐
│ Camera (RGB)     │ Segmentation Mask│
├──────────────────┼──────────────────┤
│ Seg. Overlay     │ TrackDLO Results │
└──────────────────┴──────────────────┘
```

X11フォワーディング経由でDockerコンテナ内から表示。
