# trackdlo_ros2 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fork trackdlo2 into a standalone perception+preview project (trackdlo_ros2), removing robot/simulation dependencies, adding pluggable segmentation architecture, and supporting Humble+Jazzy.

**Architecture:** Fork-and-trim approach. Delete MoveIt/Gazebo/Description packages, create a new `trackdlo_segmentation` package with a base class + HSV implementation, restructure Docker to use profiles for segmentation selection, and parameterize ROS_DISTRO for Humble/Jazzy.

**Tech Stack:** ROS2 (Humble/Jazzy), C++17, Python 3, CUDA, OpenCV, Eigen3, PCL, Docker Compose with profiles

---

## File Structure

### Files to Delete
- `trackdlo_moveit/` (entire directory)
- `trackdlo_description/` (entire directory)
- `docker/Dockerfile.gazebo`
- `docker/Dockerfile.moveit`
- `docker/Dockerfile.perception` (replaced by Dockerfile.core)
- `docker/docker-compose.yml` (replaced by new compose)
- `docker/docker-compose.nvidia.yml` (replaced by new compose)
- `docker/docker-compose.realsense.nvidia.yml` (merged into new nvidia overlay)
- `trackdlo_bringup/launch/ur5_gazebo.launch.py`
- `trackdlo_bringup/launch/sim_full_pipeline.launch.py`
- `trackdlo_bringup/launch/full_pipeline.launch.py`
- `trackdlo_bringup/launch/visualize_output.launch.py`
- `trackdlo_bringup/launch/trackdlo.launch.py` (sim-only, replaced by renamed realsense_test)
- `trackdlo_bringup/config/trackdlo_params.yaml`
- `trackdlo_utils/trackdlo_utils/hsv_tuner_node.py` (moved to trackdlo_segmentation)

### Files to Create
- `trackdlo_segmentation/package.xml`
- `trackdlo_segmentation/setup.py`
- `trackdlo_segmentation/setup.cfg`
- `trackdlo_segmentation/resource/trackdlo_segmentation`
- `trackdlo_segmentation/trackdlo_segmentation/__init__.py`
- `trackdlo_segmentation/trackdlo_segmentation/base.py`
- `trackdlo_segmentation/trackdlo_segmentation/hsv_node.py`
- `trackdlo_segmentation/test/test_base.py`
- `docker/Dockerfile.core`
- `docker/Dockerfile.sam2`
- `docker/docker-compose.yml` (new)
- `docker/docker-compose.nvidia.yml` (new)

### Files to Modify
- `docker/Dockerfile.base` (parameterize ROS_DISTRO)
- `docker/Dockerfile.realsense` (remove, replaced by Dockerfile.core)
- `docker/build.sh` (remove sim targets, add profile-based builds)
- `docker/run.sh` (remove sim mode, add profile-based segmentation selection)
- `trackdlo_bringup/launch/realsense_test.launch.py` → rename to `trackdlo.launch.py`
- `trackdlo_utils/setup.py` (remove hsv_tuner entry point)
- `trackdlo_utils/package.xml` (no changes needed)
- `trackdlo_bringup/config/realsense_params.yaml` (add camera_model section)

---

## Task 1: Fork Repository and Initial Cleanup

**Files:**
- Delete: `trackdlo_moveit/`, `trackdlo_description/`
- Delete: `trackdlo_bringup/launch/ur5_gazebo.launch.py`
- Delete: `trackdlo_bringup/launch/sim_full_pipeline.launch.py`
- Delete: `trackdlo_bringup/launch/full_pipeline.launch.py`
- Delete: `trackdlo_bringup/launch/visualize_output.launch.py`
- Delete: `trackdlo_bringup/launch/trackdlo.launch.py` (sim version)
- Delete: `trackdlo_bringup/config/trackdlo_params.yaml`

- [ ] **Step 1: Fork the repository on GitHub**

Go to the trackdlo2 repository on GitHub and click "Fork" to create `trackdlo_ros2`. Then clone locally:

```bash
cd /home/user/repos/src
gh repo fork <OWNER>/trackdlo2 --clone --fork-name trackdlo_ros2
cd trackdlo_ros2
```

If you already have a local clone and just want to work from it:

```bash
cd /home/user/repos/src/trackdlo2
git checkout -b feat/trackdlo-ros2
```

- [ ] **Step 2: Delete robot control and simulation packages**

```bash
rm -rf trackdlo_moveit/
rm -rf trackdlo_description/
```

- [ ] **Step 3: Delete simulation-only launch files and config**

```bash
rm trackdlo_bringup/launch/ur5_gazebo.launch.py
rm trackdlo_bringup/launch/sim_full_pipeline.launch.py
rm trackdlo_bringup/launch/full_pipeline.launch.py
rm trackdlo_bringup/launch/visualize_output.launch.py
rm trackdlo_bringup/launch/trackdlo.launch.py
rm trackdlo_bringup/config/trackdlo_params.yaml
```

- [ ] **Step 4: Rename realsense_test.launch.py to trackdlo.launch.py**

```bash
mv trackdlo_bringup/launch/realsense_test.launch.py trackdlo_bringup/launch/trackdlo.launch.py
```

- [ ] **Step 5: Verify remaining file structure**

```bash
find . -name "*.py" -path "*/launch/*" | sort
# Expected:
# ./trackdlo_bringup/launch/camera.launch.py
# ./trackdlo_bringup/launch/evaluation.launch.py
# ./trackdlo_bringup/launch/trackdlo.launch.py

ls trackdlo_bringup/config/
# Expected:
# evaluation_params.yaml
# preset_decimation_4.0_depth_step_100.json
# realsense_params.yaml
```

- [ ] **Step 6: Commit**

```bash
git add -A
git commit -m "chore: remove simulation and robot control packages

Remove trackdlo_moveit, trackdlo_description, Gazebo launch files,
and simulation-only config. Rename realsense_test.launch.py to
trackdlo.launch.py as the main entry point."
```

---

## Task 2: Create trackdlo_segmentation Package — Base Class

**Files:**
- Create: `trackdlo_segmentation/package.xml`
- Create: `trackdlo_segmentation/setup.py`
- Create: `trackdlo_segmentation/setup.cfg`
- Create: `trackdlo_segmentation/resource/trackdlo_segmentation`
- Create: `trackdlo_segmentation/trackdlo_segmentation/__init__.py`
- Create: `trackdlo_segmentation/trackdlo_segmentation/base.py`
- Create: `trackdlo_segmentation/test/test_base.py`

- [ ] **Step 1: Create package directory structure**

```bash
mkdir -p trackdlo_segmentation/trackdlo_segmentation
mkdir -p trackdlo_segmentation/resource
mkdir -p trackdlo_segmentation/test
touch trackdlo_segmentation/resource/trackdlo_segmentation
```

- [ ] **Step 2: Write the test for SegmentationNodeBase**

Create `trackdlo_segmentation/test/test_base.py`:

```python
"""Tests for SegmentationNodeBase."""
import numpy as np
import pytest
import rclpy
from trackdlo_segmentation.base import SegmentationNodeBase


class DummySegNode(SegmentationNodeBase):
    """Concrete subclass for testing."""

    def __init__(self):
        super().__init__('dummy_seg')
        self.last_input = None

    def segment(self, cv_image: np.ndarray) -> np.ndarray:
        self.last_input = cv_image
        mask = np.zeros(cv_image.shape[:2], dtype=np.uint8)
        mask[cv_image[:, :, 0] > 128] = 255
        return mask


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_dummy_seg_instantiates():
    node = DummySegNode()
    assert node.get_name() == 'dummy_seg'
    node.destroy_node()


def test_segment_returns_correct_shape():
    node = DummySegNode()
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    mask = node.segment(img)
    assert mask.shape == (480, 640)
    assert mask.dtype == np.uint8
    node.destroy_node()


def test_segment_returns_binary_values():
    node = DummySegNode()
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    img[100:200, 100:200, 0] = 200  # Blue channel > 128
    mask = node.segment(img)
    assert set(np.unique(mask)).issubset({0, 255})
    assert mask[150, 150] == 255
    assert mask[0, 0] == 0
    node.destroy_node()


def test_has_publisher_and_subscriber():
    node = DummySegNode()
    pub_topics = [t[0] for t in node.get_publisher_names_and_types_by_node(
        'dummy_seg', '/')]
    sub_topics = [t[0] for t in node.get_subscriber_names_and_types_by_node(
        'dummy_seg', '/')]
    assert '/trackdlo/segmentation_mask' in pub_topics
    assert '/camera/color/image_raw' in sub_topics
    node.destroy_node()
```

- [ ] **Step 3: Write SegmentationNodeBase**

Create `trackdlo_segmentation/trackdlo_segmentation/base.py`:

```python
"""Base class for all segmentation nodes in trackdlo_ros2.

All segmentation implementations (HSV, SAM2, YOLO, DeepLab, etc.)
inherit from this class and implement the segment() method.
Output is published to /trackdlo/segmentation_mask (mono8).
"""
from abc import abstractmethod

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class SegmentationNodeBase(Node):
    """Base ROS2 node for DLO segmentation.

    Subscribes to an RGB image topic, calls the abstract segment() method,
    and publishes the resulting binary mask.
    """

    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.bridge = CvBridge()

        self.declare_parameter('rgb_topic', '/camera/color/image_raw')
        rgb_topic = self.get_parameter('rgb_topic').value

        self.mask_pub = self.create_publisher(
            Image, '/trackdlo/segmentation_mask', 10
        )
        self.image_sub = self.create_subscription(
            Image, rgb_topic, self._on_image, 10
        )

    def _on_image(self, msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        mask = self.segment(cv_image)
        mask_msg = self.bridge.cv2_to_imgmsg(mask, 'mono8')
        mask_msg.header = msg.header
        self.mask_pub.publish(mask_msg)

    @abstractmethod
    def segment(self, cv_image: np.ndarray) -> np.ndarray:
        """Segment a BGR image and return a binary mask.

        Args:
            cv_image: Input BGR image, shape (H, W, 3), dtype uint8.

        Returns:
            Binary mask, shape (H, W), dtype uint8, values 0 or 255.
        """
        ...
```

- [ ] **Step 4: Write __init__.py**

Create `trackdlo_segmentation/trackdlo_segmentation/__init__.py`:

```python
from trackdlo_segmentation.base import SegmentationNodeBase

__all__ = ['SegmentationNodeBase']
```

- [ ] **Step 5: Write package.xml**

Create `trackdlo_segmentation/package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>trackdlo_segmentation</name>
  <version>1.0.0</version>
  <description>Pluggable segmentation interface for trackdlo_ros2</description>
  <maintainer email="todo@todo.com">TODO</maintainer>
  <license>BSD-3-Clause</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>cv_bridge</exec_depend>
  <exec_depend>python3-opencv</exec_depend>
  <exec_depend>python3-numpy</exec_depend>

  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] **Step 6: Write setup.py and setup.cfg**

Create `trackdlo_segmentation/setup.py`:

```python
from setuptools import find_packages, setup

package_name = 'trackdlo_segmentation'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO',
    maintainer_email='todo@todo.com',
    description='Pluggable segmentation interface for trackdlo_ros2',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hsv_segmentation = trackdlo_segmentation.hsv_node:main',
        ],
    },
)
```

Create `trackdlo_segmentation/setup.cfg`:

```ini
[develop]
script_dir=$base/lib/trackdlo_segmentation
[install]
install_scripts=$base/lib/trackdlo_segmentation
```

- [ ] **Step 7: Run the tests**

```bash
cd /path/to/trackdlo_ros2
source /opt/ros/humble/setup.bash
colcon build --packages-select trackdlo_segmentation
source install/setup.bash
colcon test --packages-select trackdlo_segmentation --return-code-on-test-failure
colcon test-result --verbose
```

Expected: 4 tests pass.

- [ ] **Step 8: Commit**

```bash
git add trackdlo_segmentation/
git commit -m "feat: add trackdlo_segmentation package with SegmentationNodeBase

Introduces the pluggable segmentation interface. All segmentation
nodes (HSV, SAM2, YOLO, etc.) inherit from SegmentationNodeBase
and publish to /trackdlo/segmentation_mask."
```

---

## Task 3: HSV Segmentation Node

**Files:**
- Create: `trackdlo_segmentation/trackdlo_segmentation/hsv_node.py`
- Create: `trackdlo_segmentation/test/test_hsv_node.py`
- Modify: `trackdlo_utils/setup.py` (remove hsv_tuner entry)
- Delete: `trackdlo_utils/trackdlo_utils/hsv_tuner_node.py`

- [ ] **Step 1: Write the test for HsvSegmentationNode**

Create `trackdlo_segmentation/test/test_hsv_node.py`:

```python
"""Tests for HSV segmentation node."""
import numpy as np
import pytest
import rclpy
from trackdlo_segmentation.hsv_node import HsvSegmentationNode


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_hsv_node_instantiates():
    node = HsvSegmentationNode()
    assert node.get_name() == 'hsv_segmentation'
    node.destroy_node()


def test_segment_returns_binary_mask():
    node = HsvSegmentationNode()
    # Create a blue-ish image (H~100 in HSV, within default 85-135 range)
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    img[:, :] = [180, 100, 50]  # BGR: blue-dominant
    mask = node.segment(img)
    assert mask.shape == (480, 640)
    assert mask.dtype == np.uint8
    assert set(np.unique(mask)).issubset({0, 255})
    node.destroy_node()


def test_segment_detects_blue_object():
    node = HsvSegmentationNode()
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    # Paint a blue rectangle (H~100 in HSV)
    img[100:200, 100:300] = [200, 80, 30]  # BGR: strong blue
    mask = node.segment(img)
    # The blue region should be detected
    assert mask[150, 200] == 255
    # Non-blue region should be zero
    assert mask[0, 0] == 0
    node.destroy_node()


def test_has_hsv_parameters():
    node = HsvSegmentationNode()
    lower = node.get_parameter('hsv_threshold_lower_limit').value
    upper = node.get_parameter('hsv_threshold_upper_limit').value
    assert lower == '85 50 20'
    assert upper == '135 255 255'
    node.destroy_node()
```

- [ ] **Step 2: Run test to verify it fails**

```bash
colcon build --packages-select trackdlo_segmentation
source install/setup.bash
colcon test --packages-select trackdlo_segmentation --return-code-on-test-failure
```

Expected: FAIL — `hsv_node` module not found.

- [ ] **Step 3: Write HsvSegmentationNode**

Create `trackdlo_segmentation/trackdlo_segmentation/hsv_node.py`:

```python
"""HSV segmentation node with interactive GUI tuner.

Inherits from SegmentationNodeBase. Provides OpenCV trackbar GUI
for live HSV threshold adjustment.
"""
import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException

from trackdlo_segmentation.base import SegmentationNodeBase


class HsvSegmentationNode(SegmentationNodeBase):
    """HSV color-based segmentation with optional GUI tuner."""

    def __init__(self):
        super().__init__('hsv_segmentation')

        self.declare_parameter('hsv_threshold_upper_limit', '135 255 255')
        self.declare_parameter('hsv_threshold_lower_limit', '85 50 20')
        self.declare_parameter('enable_gui', True)

        upper_str = self.get_parameter('hsv_threshold_upper_limit').value
        lower_str = self.get_parameter('hsv_threshold_lower_limit').value
        self.enable_gui = self.get_parameter('enable_gui').value

        upper_vals = [int(x) for x in upper_str.split()]
        lower_vals = [int(x) for x in lower_str.split()]
        self.h_min, self.s_min, self.v_min = lower_vals
        self.h_max, self.s_max, self.v_max = upper_vals

        self.latest_image = None

        if self.enable_gui:
            cv2.namedWindow('HSV Tuner', cv2.WINDOW_AUTOSIZE)
            cv2.createTrackbar('H Min', 'HSV Tuner', self.h_min, 179, lambda x: None)
            cv2.createTrackbar('S Min', 'HSV Tuner', self.s_min, 255, lambda x: None)
            cv2.createTrackbar('V Min', 'HSV Tuner', self.v_min, 255, lambda x: None)
            cv2.createTrackbar('H Max', 'HSV Tuner', self.h_max, 179, lambda x: None)
            cv2.createTrackbar('S Max', 'HSV Tuner', self.s_max, 255, lambda x: None)
            cv2.createTrackbar('V Max', 'HSV Tuner', self.v_max, 255, lambda x: None)
            self.gui_timer = self.create_timer(1.0 / 30.0, self._gui_callback)
            self.get_logger().info(
                'HSV Tuner GUI started. Adjust sliders, press Q to save & quit.')

    def _on_image(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        mask = self.segment(self.latest_image)
        mask_msg = self.bridge.cv2_to_imgmsg(mask, 'mono8')
        mask_msg.header = msg.header
        self.mask_pub.publish(mask_msg)

    def segment(self, cv_image: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower = np.array([self.h_min, self.s_min, self.v_min])
        upper = np.array([self.h_max, self.s_max, self.v_max])
        return cv2.inRange(hsv, lower, upper)

    def _gui_callback(self):
        if self.latest_image is None:
            cv2.waitKey(1)
            return

        h_min = cv2.getTrackbarPos('H Min', 'HSV Tuner')
        s_min = cv2.getTrackbarPos('S Min', 'HSV Tuner')
        v_min = cv2.getTrackbarPos('V Min', 'HSV Tuner')
        h_max = cv2.getTrackbarPos('H Max', 'HSV Tuner')
        s_max = cv2.getTrackbarPos('S Max', 'HSV Tuner')
        v_max = cv2.getTrackbarPos('V Max', 'HSV Tuner')

        if (h_min != self.h_min or s_min != self.s_min or v_min != self.v_min or
                h_max != self.h_max or s_max != self.s_max or v_max != self.v_max):
            self.get_logger().info(
                f'HSV: lower="{h_min} {s_min} {v_min}" upper="{h_max} {s_max} {v_max}"')
            self.h_min, self.s_min, self.v_min = h_min, s_min, v_min
            self.h_max, self.s_max, self.v_max = h_max, s_max, v_max

        hsv = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv,
                           np.array([h_min, s_min, v_min]),
                           np.array([h_max, s_max, v_max]))
        masked = cv2.bitwise_and(self.latest_image, self.latest_image, mask=mask)

        display = np.hstack([
            cv2.resize(self.latest_image, (640, 480)),
            cv2.resize(masked, (640, 480)),
        ])
        cv2.imshow('HSV Tuner', display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info('Final HSV values for realsense_params.yaml:')
            self.get_logger().info(
                f'  hsv_threshold_lower_limit: "{self.h_min} {self.s_min} {self.v_min}"')
            self.get_logger().info(
                f'  hsv_threshold_upper_limit: "{self.h_max} {self.s_max} {self.v_max}"')
            rclpy.shutdown()

    def destroy_node(self):
        if self.enable_gui:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HsvSegmentationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
colcon build --packages-select trackdlo_segmentation
source install/setup.bash
colcon test --packages-select trackdlo_segmentation --return-code-on-test-failure
colcon test-result --verbose
```

Expected: All 8 tests pass (4 base + 4 hsv).

- [ ] **Step 5: Remove hsv_tuner from trackdlo_utils**

Delete `trackdlo_utils/trackdlo_utils/hsv_tuner_node.py`.

Edit `trackdlo_utils/setup.py` — remove the `hsv_tuner` entry point. Change:

```python
            'hsv_tuner = trackdlo_utils.hsv_tuner_node:main',
```

to nothing (delete the line).

- [ ] **Step 6: Commit**

```bash
git add trackdlo_segmentation/ trackdlo_utils/
git rm trackdlo_utils/trackdlo_utils/hsv_tuner_node.py
git commit -m "feat: add HSV segmentation node to trackdlo_segmentation

Migrate HSV tuner from trackdlo_utils to trackdlo_segmentation
package, inheriting from SegmentationNodeBase. Adds enable_gui
parameter to optionally disable GUI for headless operation."
```

---

## Task 4: Update Launch File for Segmentation Package

**Files:**
- Modify: `trackdlo_bringup/launch/trackdlo.launch.py` (renamed from realsense_test)
- Modify: `trackdlo_bringup/config/realsense_params.yaml`

- [ ] **Step 1: Update trackdlo.launch.py**

Replace the full content of `trackdlo_bringup/launch/trackdlo.launch.py`:

```python
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    bringup_dir = get_package_share_directory('trackdlo_bringup')
    params_file = os.path.join(bringup_dir, 'config', 'realsense_params.yaml')

    segmentation = LaunchConfiguration('segmentation')
    rviz = LaunchConfiguration('rviz')

    # use_external_mask is true when segmentation is not 'hsv'
    use_external_mask = PythonExpression(["'", segmentation, "' != 'hsv'"])

    # RealSense camera launch
    realsense_dir = get_package_share_directory('realsense2_camera')
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_dir, 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'camera_namespace': '',
            'depth_module.depth_profile': '640x480x15',
            'rgb_camera.color_profile': '640x480x15',
            'align_depth.enable': 'true',
            'pointcloud.enable': 'false',
            'temporal_filter.enable': 'true',
            'decimation_filter.enable': 'true',
        }.items(),
    )

    return [
        realsense_launch,

        # --- Static TF: base_link -> camera_link ---
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_camera_tf',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'base_link',
                '--child-frame-id', 'camera_link',
            ],
        ),

        # --- TrackDLO C++ tracking node ---
        Node(
            package='trackdlo_perception',
            executable='trackdlo',
            name='trackdlo',
            output='screen',
            parameters=[
                params_file,
                {'use_external_mask': use_external_mask},
            ],
            respawn=True,
            respawn_delay=3.0,
        ),

        # --- Python initialization node ---
        Node(
            package='trackdlo_perception',
            executable='init_tracker',
            name='init_tracker',
            output='screen',
            parameters=[
                params_file,
                {'use_external_mask': use_external_mask},
            ],
        ),

        # --- HSV Segmentation with GUI (from trackdlo_segmentation) ---
        Node(
            package='trackdlo_segmentation',
            executable='hsv_segmentation',
            name='hsv_segmentation',
            output='screen',
            parameters=[params_file],
            condition=LaunchConfigurationEquals('segmentation', 'hsv_tuner'),
        ),

        # --- SAM2 Segmentation (only when segmentation:=sam2) ---
        Node(
            package='trackdlo_utils',
            executable='sam2_segmentation',
            name='sam2_segmentation',
            output='screen',
            parameters=[params_file],
            condition=LaunchConfigurationEquals('segmentation', 'sam2'),
        ),

        # --- Composite View (4-panel display) ---
        Node(
            package='trackdlo_utils',
            executable='composite_view',
            name='composite_view',
            output='screen',
            parameters=[params_file],
        ),

        # --- CPD-LLE Parameter Tuner ---
        Node(
            package='trackdlo_utils',
            executable='param_tuner',
            name='param_tuner',
            output='screen',
        ),

        # --- RViz2 ---
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=[
                '-d', os.path.join(bringup_dir, 'rviz', 'tracking.rviz')
            ],
            condition=IfCondition(rviz),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'segmentation', default_value='hsv',
            description='Segmentation method: hsv (built-in), hsv_tuner (GUI), sam2',
            choices=['hsv', 'hsv_tuner', 'sam2'],
        ),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Launch RViz2 for visualization',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
```

- [ ] **Step 2: Add hsv_segmentation parameters to realsense_params.yaml**

Append to `trackdlo_bringup/config/realsense_params.yaml`:

```yaml
hsv_segmentation:
  ros__parameters:
    rgb_topic: /camera/color/image_raw
    hsv_threshold_upper_limit: "135 255 255"
    hsv_threshold_lower_limit: "85 50 20"
    enable_gui: true
```

- [ ] **Step 3: Verify launch file parses correctly**

```bash
colcon build --packages-select trackdlo_segmentation trackdlo_bringup
source install/setup.bash
ros2 launch trackdlo_bringup trackdlo.launch.py --show-args
```

Expected output should list `segmentation` and `rviz` arguments.

- [ ] **Step 4: Commit**

```bash
git add trackdlo_bringup/
git commit -m "feat: update launch file for pluggable segmentation

Replace hsv_tuner from trackdlo_utils with hsv_segmentation from
trackdlo_segmentation package. Add hsv_segmentation parameters
to realsense_params.yaml."
```

---

## Task 5: Docker — Parameterize Dockerfile.base for Humble/Jazzy

**Files:**
- Modify: `docker/Dockerfile.base`
- Modify: `docker/.env`

- [ ] **Step 1: Update Dockerfile.base**

Replace the full content of `docker/Dockerfile.base`:

```dockerfile
ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive

# Install common dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-message-filters \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-rosbag2-cpp \
    ros-${ROS_DISTRO}-rosbag2-storage-default-plugins \
    ros-${ROS_DISTRO}-sensor-msgs-py \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    libpcl-dev \
    libeigen3-dev \
    libopencv-dev \
    libgl1-mesa-dri \
    libglx-mesa0 \
    mesa-utils \
    libvulkan1 \
    mesa-vulkan-drivers \
    && rm -rf /var/lib/apt/lists/*

# Python dependencies
# Pin numpy<2 for compatibility with ros-humble-cv-bridge (compiled against numpy 1.x)
RUN pip3 install --no-cache-dir \
    "numpy<2" \
    scipy \
    scikit-image \
    open3d \
    Pillow

# Use CycloneDDS as default RMW
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=42

# Setup entrypoint
COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
```

- [ ] **Step 2: Update .env file**

Replace `docker/.env`:

```env
ROS_DOMAIN_ID=42
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ROS_DISTRO=humble
```

- [ ] **Step 3: Commit**

```bash
git add docker/Dockerfile.base docker/.env
git commit -m "feat: parameterize Dockerfile.base with ROS_DISTRO build arg

Supports both humble and jazzy via ARG ROS_DISTRO=humble.
Add ROS_DISTRO to .env for docker-compose."
```

---

## Task 6: Docker — Create Dockerfile.core

**Files:**
- Create: `docker/Dockerfile.core`
- Delete: `docker/Dockerfile.perception`
- Delete: `docker/Dockerfile.realsense`

- [ ] **Step 1: Create Dockerfile.core**

This replaces both `Dockerfile.perception` and `Dockerfile.realsense`, combining core perception with RealSense driver:

Create `docker/Dockerfile.core`:

```dockerfile
ARG ROS_DISTRO=humble
ARG BASE_IMAGE=trackdlo_ros2-base:latest
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive

# Install RealSense camera driver, RViz2, TF2, robot_state_publisher
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-realsense2-camera \
    ros-${ROS_DISTRO}-realsense2-description \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-xacro \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# Copy packages
COPY trackdlo_perception/ src/trackdlo_perception/
COPY trackdlo_segmentation/ src/trackdlo_segmentation/
COPY trackdlo_utils/ src/trackdlo_utils/
COPY trackdlo_bringup/ src/trackdlo_bringup/
COPY trackdlo_msgs/ src/trackdlo_msgs/

# Build workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf build log

# Source workspace on entry
RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

CMD ["bash"]
```

- [ ] **Step 2: Delete old Dockerfiles**

```bash
rm docker/Dockerfile.perception
rm docker/Dockerfile.realsense
```

- [ ] **Step 3: Commit**

```bash
git add docker/Dockerfile.core
git rm docker/Dockerfile.perception docker/Dockerfile.realsense
git commit -m "feat: add Dockerfile.core replacing perception and realsense images

Single core image with RealSense driver + perception + segmentation
+ utils. Includes trackdlo_segmentation package."
```

---

## Task 7: Docker — Create Dockerfile.sam2

**Files:**
- Create: `docker/Dockerfile.sam2`

- [ ] **Step 1: Create Dockerfile.sam2**

Create `docker/Dockerfile.sam2`:

```dockerfile
ARG ROS_DISTRO=humble
ARG BASE_IMAGE=trackdlo_ros2-base:latest
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive

# Create workspace
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# Copy only segmentation packages (base class + SAM2 node)
COPY trackdlo_segmentation/ src/trackdlo_segmentation/
COPY trackdlo_utils/ src/trackdlo_utils/
COPY trackdlo_msgs/ src/trackdlo_msgs/

# Build workspace (lightweight — only Python packages)
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --packages-select trackdlo_segmentation trackdlo_utils trackdlo_msgs && \
    rm -rf build log

# Install SAM2 dependencies
ARG SAM2_DEVICE=cpu

# Install CUDA runtime for GPU inference (only when SAM2_DEVICE=cuda)
RUN if [ "$SAM2_DEVICE" = "cuda" ]; then \
        apt-get update && apt-get install -y --no-install-recommends \
            wget gnupg2 && \
        wget -qO /tmp/cuda-keyring.deb \
            https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb && \
        dpkg -i /tmp/cuda-keyring.deb && rm /tmp/cuda-keyring.deb && \
        apt-get update && apt-get install -y --no-install-recommends \
            cuda-cudart-12-6 libcublas-12-6 libcusparse-12-6 \
            libcusolver-12-6 libcufft-12-6 libcurand-12-6 && \
        rm -rf /var/lib/apt/lists/* ; \
    fi
ENV PATH="/usr/local/cuda/bin:${PATH}"
ENV LD_LIBRARY_PATH="/usr/local/cuda/lib64:${LD_LIBRARY_PATH}"

# Install PyTorch + SAM2
RUN if [ "$SAM2_DEVICE" = "cpu" ]; then \
        pip3 install --no-cache-dir \
            torch torchvision --index-url https://download.pytorch.org/whl/cpu && \
        SAM2_BUILD_CUDA=0 pip3 install --no-cache-dir --no-build-isolation \
            "git+https://github.com/facebookresearch/sam2.git" && \
        pip3 install --no-cache-dir huggingface_hub ; \
    elif [ "$SAM2_DEVICE" = "cuda" ]; then \
        pip3 install --no-cache-dir torch torchvision && \
        pip3 install --no-cache-dir --no-build-isolation \
            "git+https://github.com/facebookresearch/sam2.git" && \
        pip3 install --no-cache-dir huggingface_hub ; \
    fi

# Pre-download SAM2 model weights
ARG SAM2_CHECKPOINT=facebook/sam2.1-hiera-small
RUN CUDA_VISIBLE_DEVICES="" python3 -c "\
from sam2.sam2_image_predictor import SAM2ImagePredictor; \
SAM2ImagePredictor.from_pretrained('${SAM2_CHECKPOINT}', device='cpu')"

# Source workspace on entry
RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

CMD ["bash", "-c", "source /ros2_ws/install/setup.bash && ros2 run trackdlo_utils sam2_segmentation"]
```

- [ ] **Step 2: Commit**

```bash
git add docker/Dockerfile.sam2
git commit -m "feat: add Dockerfile.sam2 for standalone SAM2 segmentation container

Separate container with PyTorch + SAM2 dependencies. Supports
cpu and cuda build variants via SAM2_DEVICE build arg."
```

---

## Task 8: Docker — Create New docker-compose Files

**Files:**
- Create: `docker/docker-compose.yml` (new)
- Create: `docker/docker-compose.nvidia.yml` (new)
- Delete: `docker/docker-compose.realsense.yml`
- Delete: `docker/docker-compose.realsense.nvidia.yml`
- Delete: old `docker/docker-compose.yml`
- Delete: old `docker/docker-compose.nvidia.yml`

- [ ] **Step 1: Delete old compose files**

```bash
rm docker/docker-compose.yml
rm docker/docker-compose.nvidia.yml
rm docker/docker-compose.realsense.yml
rm docker/docker-compose.realsense.nvidia.yml
```

- [ ] **Step 2: Create new docker-compose.yml**

Create `docker/docker-compose.yml`:

```yaml
services:
  # Base image (build dependency only)
  trackdlo-base:
    build:
      context: ..
      dockerfile: docker/Dockerfile.base
      args:
        ROS_DISTRO: ${ROS_DISTRO:-humble}
    image: trackdlo_ros2-base:latest
    command: "true"

  # Core: RealSense + perception + HSV segmentation + preview
  trackdlo-core:
    build:
      context: ..
      dockerfile: docker/Dockerfile.core
      args:
        ROS_DISTRO: ${ROS_DISTRO:-humble}
        BASE_IMAGE: trackdlo_ros2-base:latest
    image: trackdlo_ros2-core:latest
    container_name: trackdlo-core
    network_mode: host
    env_file: .env
    environment:
      - DISPLAY=${DISPLAY}
      - LIBGL_ALWAYS_SOFTWARE=0
      - SEGMENTATION=${SEGMENTATION:-hsv}
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - ${HOME}/.Xauthority:/root/.Xauthority:rw
      - /dev:/dev
      - ../trackdlo_bringup/rviz:/ros2_ws/install/trackdlo_bringup/share/trackdlo_bringup/rviz:ro
      - ../trackdlo_bringup/config:/ros2_ws/install/trackdlo_bringup/share/trackdlo_bringup/config:ro
      - ../trackdlo_bringup/launch:/ros2_ws/install/trackdlo_bringup/share/trackdlo_bringup/launch:ro
      - ../trackdlo_utils:/ros2_ws/src/trackdlo_utils
      - ../trackdlo_segmentation:/ros2_ws/src/trackdlo_segmentation
    privileged: true
    depends_on:
      trackdlo-base:
        condition: service_completed_successfully
    command: >
      bash -c "
        cd /ros2_ws &&
        source /opt/ros/$${ROS_DISTRO}/setup.sh &&
        colcon build --packages-select trackdlo_utils trackdlo_segmentation &&
        source install/setup.bash &&
        ros2 launch trackdlo_bringup trackdlo.launch.py segmentation:=$${SEGMENTATION:-hsv}
      "

  # SAM2 segmentation (separate container, optional)
  trackdlo-sam2:
    build:
      context: ..
      dockerfile: docker/Dockerfile.sam2
      args:
        ROS_DISTRO: ${ROS_DISTRO:-humble}
        BASE_IMAGE: trackdlo_ros2-base:latest
        SAM2_DEVICE: cpu
    image: trackdlo_ros2-sam2:latest
    container_name: trackdlo-sam2
    network_mode: host
    env_file: .env
    environment:
      - DISPLAY=${DISPLAY}
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - ${HOME}/.Xauthority:/root/.Xauthority:rw
    depends_on:
      trackdlo-base:
        condition: service_completed_successfully
    profiles: ["sam2"]
```

- [ ] **Step 3: Create new docker-compose.nvidia.yml**

Create `docker/docker-compose.nvidia.yml`:

```yaml
services:
  trackdlo-core:
    environment:
      - NVIDIA_VISIBLE_DEVICES=all
      - NVIDIA_DRIVER_CAPABILITIES=all
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: all
              capabilities: [gpu]

  trackdlo-sam2:
    environment:
      - NVIDIA_VISIBLE_DEVICES=all
      - NVIDIA_DRIVER_CAPABILITIES=all
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: all
              capabilities: [gpu]
```

- [ ] **Step 4: Commit**

```bash
git add docker/docker-compose.yml docker/docker-compose.nvidia.yml
git commit -m "feat: add new docker-compose with profile-based segmentation

Core service runs RealSense + perception + HSV. SAM2 runs in
separate container via --profile sam2. NVIDIA GPU support via
docker-compose.nvidia.yml overlay."
```

---

## Task 9: Docker — Update build.sh and run.sh

**Files:**
- Modify: `docker/build.sh`
- Modify: `docker/run.sh`
- Delete: `docker/Dockerfile.gazebo`
- Delete: `docker/Dockerfile.moveit`

- [ ] **Step 1: Delete simulation Dockerfiles**

```bash
rm docker/Dockerfile.gazebo
rm docker/Dockerfile.moveit
```

- [ ] **Step 2: Replace build.sh**

Replace the full content of `docker/build.sh`:

```bash
#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

TARGET="${1:-all}"
ROS_DISTRO="${ROS_DISTRO:-humble}"

echo "=== Building for ROS2 ${ROS_DISTRO} ==="

echo "=== Step 1: Building base image ==="
docker compose build --build-arg ROS_DISTRO="${ROS_DISTRO}" trackdlo-base

if [[ "$TARGET" == "all" || "$TARGET" == "core" ]]; then
  echo "=== Building core image ==="
  docker compose build --build-arg ROS_DISTRO="${ROS_DISTRO}" trackdlo-core
fi

if [[ "$TARGET" == "all" || "$TARGET" == "sam2" ]]; then
  echo "=== Building SAM2 image (CPU) ==="
  docker compose --profile sam2 build \
    --build-arg ROS_DISTRO="${ROS_DISTRO}" \
    --build-arg SAM2_DEVICE=cpu trackdlo-sam2
fi

if [[ "$TARGET" == "sam2-cuda" ]]; then
  echo "=== Building SAM2 image (CUDA) ==="
  docker compose --profile sam2 build \
    --build-arg ROS_DISTRO="${ROS_DISTRO}" \
    --build-arg SAM2_DEVICE=cuda trackdlo-sam2
fi

echo "=== Build complete ==="
```

- [ ] **Step 3: Replace run.sh**

Replace the full content of `docker/run.sh`:

```bash
#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

MODE="${1:-hsv}"

if [[ "$MODE" == "-h" || "$MODE" == "--help" ]]; then
  echo "Usage: $0 [hsv|hsv_tuner|sam2] [docker-compose options...]"
  echo ""
  echo "  Modes:"
  echo "    hsv           - HSV segmentation, no GUI (default)"
  echo "    hsv_tuner     - HSV segmentation with tuner GUI"
  echo "    sam2          - SAM2 segmentation (separate container)"
  echo ""
  echo "  Examples:"
  echo "    $0                            # HSV segmentation"
  echo "    $0 hsv_tuner                  # HSV with tuner GUI"
  echo "    $0 sam2                       # SAM2 segmentation"
  echo "    $0 hsv -d                     # Detached mode"
  echo ""
  echo "  Environment variables:"
  echo "    ROS_DISTRO=jazzy $0           # Use Jazzy"
  exit 0
fi

COMPOSE_FILES="-f docker-compose.yml"

# Add NVIDIA GPU support if available
if command -v nvidia-smi &> /dev/null; then
  COMPOSE_FILES="$COMPOSE_FILES -f docker-compose.nvidia.yml"
  echo "=== NVIDIA GPU detected ==="
fi

if [[ "$MODE" == "sam2" ]]; then
  echo "=== Starting core + SAM2 segmentation ==="
  SEGMENTATION=sam2 docker compose $COMPOSE_FILES --profile sam2 up "${@:2}"
else
  echo "=== Starting core (segmentation=${MODE}) ==="
  SEGMENTATION="$MODE" docker compose $COMPOSE_FILES up trackdlo-core "${@:2}"
fi
```

- [ ] **Step 4: Make scripts executable**

```bash
chmod +x docker/build.sh docker/run.sh
```

- [ ] **Step 5: Commit**

```bash
git rm docker/Dockerfile.gazebo docker/Dockerfile.moveit
git add docker/build.sh docker/run.sh
git commit -m "feat: update build.sh and run.sh for trackdlo_ros2

Remove simulation targets. Add profile-based segmentation
selection and auto-detect NVIDIA GPU. Support ROS_DISTRO env var."
```

---

## Task 10: Clean Up Remaining References and README

**Files:**
- Modify: `CLAUDE.md`
- Modify: `README.md`

- [ ] **Step 1: Update CLAUDE.md**

Update `CLAUDE.md` to reflect the new project scope. Key changes:

- Project name: trackdlo_ros2
- Remove all references to trackdlo_moveit, trackdlo_description, Gazebo, MoveIt, UR5 manipulation
- Remove simulation-related Docker commands (`bash build.sh sim`, `./run.sh amd sim`)
- Remove simulation launch commands (`full_pipeline.launch.py`, `sim_full_pipeline.launch.py`)
- Add trackdlo_segmentation to package table
- Update Docker commands to new format (`./run.sh hsv`, `./run.sh sam2`)
- Update build command to include `trackdlo_segmentation` in package list
- Remove `trackdlo_params.yaml` references
- Update Architecture section to remove Multi-Container Docker Architecture (4 containers → 1-2 containers)

The content should reflect that this is a perception+preview focused project, not a full robot control system.

- [ ] **Step 2: Update README.md**

Update `README.md` to describe trackdlo_ros2 as a standalone perception+preview project. Remove robot manipulation sections and update Docker usage examples.

- [ ] **Step 3: Commit**

```bash
git add CLAUDE.md README.md
git commit -m "docs: update documentation for trackdlo_ros2 project

Reflect new project scope: perception + preview only.
Remove simulation and robot control references."
```

---

## Task 11: Final Verification

**Files:** None (verification only)

- [ ] **Step 1: Verify no remaining references to deleted packages**

```bash
grep -r "trackdlo_moveit" --include="*.py" --include="*.xml" --include="*.yaml" --include="*.yml" --include="Dockerfile*" .
grep -r "trackdlo_description" --include="*.py" --include="*.xml" --include="*.yaml" --include="*.yml" --include="Dockerfile*" .
grep -r "ur5_gazebo" --include="*.py" --include="*.yaml" --include="*.yml" .
grep -r "Dockerfile.gazebo\|Dockerfile.moveit\|Dockerfile.perception\|Dockerfile.realsense" --include="*.sh" --include="*.yml" --include="*.yaml" .
```

Expected: No matches. If any found, remove the references.

- [ ] **Step 2: Verify colcon build succeeds**

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select trackdlo_msgs trackdlo_segmentation trackdlo_perception trackdlo_utils trackdlo_bringup --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Expected: All 5 packages build successfully.

- [ ] **Step 3: Run all tests**

```bash
source install/setup.bash
colcon test --packages-select trackdlo_segmentation trackdlo_perception trackdlo_utils trackdlo_bringup --return-code-on-test-failure
colcon test-result --verbose
```

Expected: All tests pass.

- [ ] **Step 4: Verify Docker build**

```bash
cd docker
bash build.sh core
```

Expected: Base and core images build successfully.

- [ ] **Step 5: Verify launch file**

```bash
source install/setup.bash
ros2 launch trackdlo_bringup trackdlo.launch.py --show-args
```

Expected: Shows `segmentation` and `rviz` arguments.

- [ ] **Step 6: Commit any remaining fixes**

If any issues found in verification, fix and commit:

```bash
git add -A
git commit -m "fix: address remaining cleanup issues from verification"
```
