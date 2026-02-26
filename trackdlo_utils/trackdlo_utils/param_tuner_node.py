#!/usr/bin/env python3
"""CPD-LLE Parameter Tuner GUI Node.

Opens an OpenCV window with trackbars to dynamically adjust
TrackDLO tracking parameters at runtime via ROS2 parameter services.
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
import cv2
import numpy as np
from PIL import Image as PILImage, ImageDraw, ImageFont


# (name, type, slider_max, scale_divisor, offset, description)
# real_value = (slider_value + offset) / scale_divisor
PARAM_DEFS = [
    ('beta',       'double', 200,    100,   0,
     u'\u5f62\u72b6\u525b\u6027 (\u5c0f=\u67d4\u8edf, \u5927=\u76f4\u7dda\u7684)'),
    ('lambda',     'double', 100000, 1,     0,
     u'\u5927\u57df\u7684\u6ed1\u3089\u304b\u3055\u306e\u5f37\u5ea6'),
    ('alpha',      'double', 100,    10,    0,
     u'LLE\u6b63\u5247\u5316 (\u521d\u671f\u5f62\u72b6\u3078\u306e\u6574\u5408\u6027)'),
    ('mu',         'double', 50,     100,   1,
     u'\u30ce\u30a4\u30ba/\u5916\u308c\u5024\u6bd4\u7387 (0.01-0.5)'),
    ('max_iter',   'int',    100,    1,     1,
     u'EM\u6700\u5927\u53cd\u5fa9\u56de\u6570 (\u901f\u5ea6 vs \u7cbe\u5ea6)'),
    ('tol',        'double', 100,    10000, 1,
     u'\u53ce\u675f\u5224\u5b9a\u306e\u3057\u304d\u3044\u5024'),
    ('k_vis',      'double', 500,    1,     0,
     u'\u53ef\u8996\u6027\u9805\u306e\u91cd\u307f (\u96a0\u308c\u30ce\u30fc\u30c9\u306e\u4fe1\u983c\u5ea6)'),
    ('d_vis',      'double', 200,    1000,  1,
     u'\u30ae\u30e3\u30c3\u30d7\u88dc\u9593\u306e\u6700\u5927\u6e2c\u5730\u7dda\u8ddd\u96e2 [m]'),
    ('visibility_threshold', 'double', 50, 1000, 1,
     u'\u53ef\u8996\u5224\u5b9a\u306e\u8ddd\u96e2\u3057\u304d\u3044\u5024 [m]'),
    ('dlo_pixel_width',      'int',  100,   1,  5,
     u'DLO\u306e\u592a\u3055 (\u91cd\u306a\u308a\u5224\u5b9a\u7528) [px]'),
    ('downsample_leaf_size', 'double', 100, 1000, 1,
     u'\u30dc\u30af\u30bb\u30eb\u30b5\u30a4\u30ba [m] (\u5c0f=\u9ad8\u5bc6\u5ea6)'),
    ('lle_weight', 'double', 100,    1,     1,
     u'\u5c40\u6240\u7684\u306a\u5f62\u72b6\u4fdd\u6301\u306e\u5f37\u3055'),
]

WINDOW_NAME = 'CPD-LLE Param Tuner'
TARGET_NODE = '/trackdlo'


def real_to_slider(value, ptype, scale_divisor, offset):
    if ptype == 'int':
        return int(value)
    return int(round(value * scale_divisor)) - offset


def slider_to_real(slider_val, ptype, scale_divisor, offset):
    if ptype == 'int':
        return slider_val + offset
    return (slider_val + offset) / scale_divisor


class ParamTunerNode(Node):
    def __init__(self):
        super().__init__('param_tuner')

        self.set_cli = self.create_client(
            SetParameters, f'{TARGET_NODE}/set_parameters')
        self.get_cli = self.create_client(
            GetParameters, f'{TARGET_NODE}/get_parameters')

        self.prev_slider = {}

        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)

        # Create trackbars with default positions (will be updated once we
        # fetch current values from the target node)
        for name, ptype, slider_max, scale_div, offset, _desc in PARAM_DEFS:
            # start at minimum
            cv2.createTrackbar(name, WINDOW_NAME, 0, slider_max, lambda x: None)
            self.prev_slider[name] = 0

        # Try to fetch current parameter values from the target node
        self.initial_fetch_done = False
        self.fetch_timer = self.create_timer(1.0, self._try_fetch_initial)

        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
        self.get_logger().info(
            f'Param Tuner started. Waiting for {TARGET_NODE} node...')

    def _try_fetch_initial(self):
        if self.initial_fetch_done:
            self.fetch_timer.cancel()
            return

        if not self.get_cli.service_is_ready():
            self.get_logger().info(
                f'Waiting for {TARGET_NODE}/get_parameters service...',
                throttle_duration_sec=5.0)
            return

        req = GetParameters.Request()
        req.names = [d[0] for d in PARAM_DEFS]
        future = self.get_cli.call_async(req)
        future.add_done_callback(self._on_initial_params)

    def _on_initial_params(self, future):
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().warn(f'Failed to get parameters: {e}')
            return

        for i, (name, ptype, slider_max, scale_div, offset, _desc) in enumerate(PARAM_DEFS):
            pval = resp.values[i]
            if pval.type == ParameterType.PARAMETER_DOUBLE:
                real_val = pval.double_value
            elif pval.type == ParameterType.PARAMETER_INTEGER:
                real_val = pval.integer_value
            else:
                continue

            sv = real_to_slider(real_val, ptype, scale_div, offset)
            sv = max(0, min(sv, slider_max))
            cv2.setTrackbarPos(name, WINDOW_NAME, sv)
            self.prev_slider[name] = sv
            self.get_logger().info(f'  {name} = {real_val} (slider={sv})')

        self.initial_fetch_done = True
        self.fetch_timer.cancel()
        self.get_logger().info('Initial parameter values loaded.')

    def timer_callback(self):
        # Draw a blank info image
        info_img = self._draw_info()
        cv2.imshow(WINDOW_NAME, info_img)

        changed = []
        for name, ptype, slider_max, scale_div, offset, _desc in PARAM_DEFS:
            sv = cv2.getTrackbarPos(name, WINDOW_NAME)
            if sv != self.prev_slider.get(name):
                real_val = slider_to_real(sv, ptype, scale_div, offset)
                changed.append((name, ptype, real_val))
                self.prev_slider[name] = sv

        if changed and self.set_cli.service_is_ready():
            req = SetParameters.Request()
            for name, ptype, real_val in changed:
                p = Parameter()
                p.name = name
                pv = ParameterValue()
                if ptype == 'int':
                    pv.type = ParameterType.PARAMETER_INTEGER
                    pv.integer_value = int(real_val)
                else:
                    pv.type = ParameterType.PARAMETER_DOUBLE
                    pv.double_value = float(real_val)
                p.value = pv
                req.parameters.append(p)
                self.get_logger().info(f'Setting {name} = {real_val}')
            self.set_cli.call_async(req)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info('Param Tuner closing.')
            rclpy.shutdown()

    def _draw_info(self):
        """Draw an info image showing parameter values with Japanese descriptions."""
        row_h = 32
        h = 16 + len(PARAM_DEFS) * row_h
        w = 720

        pil_img = PILImage.new('RGB', (w, h), (40, 40, 40))
        draw = ImageDraw.Draw(pil_img)

        try:
            font_val = ImageFont.truetype(
                '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc', 16)
            font_desc = ImageFont.truetype(
                '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc', 14)
        except (IOError, OSError):
            font_val = ImageFont.load_default()
            font_desc = font_val

        y = 8
        for name, ptype, slider_max, scale_div, offset, desc in PARAM_DEFS:
            sv = cv2.getTrackbarPos(name, WINDOW_NAME)
            real_val = slider_to_real(sv, ptype, scale_div, offset)
            if ptype == 'int':
                val_text = f'{name}: {int(real_val)}'
            else:
                val_text = f'{name}: {real_val:.4f}'
            draw.text((10, y), val_text, fill=(220, 220, 220), font=font_val)
            draw.text((320, y + 1), desc, fill=(140, 180, 140), font=font_desc)
            y += row_h

        return np.array(pil_img)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ParamTunerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
