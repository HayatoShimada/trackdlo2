# TrackDLO2

ROS2 Humble + Gazebo Fortress ベースの変形線状物体 (DLO: Deformable Linear Object) リアルタイム追跡・操作システム。
RGB-D カメラで DLO を認識・追跡し、UR5 ロボットアームで端点間を自律的に往復する。

## アーキテクチャ

Docker Compose で 4 つのコンテナに分離し、ROS2 トピック/サービスのみで通信する。

```
[Gazebo Container]            唯一のロボット状態権限
  robot_state_publisher     → /robot_description, /tf, /tf_static
  joint_state_broadcaster   → /joint_states
  joint_trajectory_controller ← FollowJointTrajectory action

[Perception Container]        トピックのみで通信
  init_tracker              → /trackdlo/init_nodes
  trackdlo_node             → /trackdlo/results_pc

[MoveIt Container]            robot_state_publisher なし
  move_group (直接起動)     → OMPL planner → joint_trajectory_controller
  dlo_manipulation_node:
    subscribes:  /trackdlo/results_pc
    publishes:   /trackdlo/endpoints (PoseArray)
    service:     ~/enable (SetBool)
    → MoveGroupInterface → move_group → FollowJointTrajectory

[Viz Container]               純粋な可視化
  rviz2 (RobotModel + MoveIt MotionPlanning display)
```

## パッケージ構成

```
trackdlo2/
├── trackdlo_bringup/        Launch ファイル・パラメータ設定・RViz 設定
├── trackdlo_description/    UR5 + RealSense D435 の URDF
├── trackdlo_perception/     DLO 追跡 (初期化 + CPD-LLE トラッキング)
├── trackdlo_moveit/         MoveIt2 経路計画 + DLO 操作ノード
├── trackdlo_msgs/           カスタムメッセージ定義 (将来用)
├── trackdlo_utils/          Depth 変換・テスト・可視化ユーティリティ
└── docker/                  Docker Compose + GPU 設定
    ├── docker-compose.yml        ベース構成 (GPU 非依存)
    ├── docker-compose.amd.yml    AMD GPU override
    ├── docker-compose.nvidia.yml NVIDIA GPU override
    └── run.sh                    GPU 自動選択起動スクリプト
```

## クイックスタート

### ビルド

```bash
cd docker/

# シミュレーション + RealSense テスト (全イメージ)
bash build.sh

# シミュレーションのみ
bash build.sh sim

# RealSense テストのみ
bash build.sh realsense

# RealSense + SAM2 (CPU)
bash build.sh realsense-sam2

# RealSense + SAM2 (CUDA)
bash build.sh realsense-sam2-cuda
```

### 起動 (シミュレーション)

```bash
# X11 転送を許可
xhost +local:docker

# AMD GPU (デフォルト)
./run.sh amd

# NVIDIA GPU
./run.sh nvidia

# バックグラウンド起動
./run.sh amd sim -d
```

### 起動 (RealSense 実機テスト)

```bash
xhost +local:docker

# HSV セグメンテーション (デフォルト)
./run.sh amd realsense
./run.sh nvidia realsense

# HSV チューナー GUI
./run.sh amd realsense:hsv_tuner

# SAM2 セグメンテーション (要: realsense-sam2 ビルド)
./run.sh nvidia realsense:sam2

# バックグラウンド起動
./run.sh amd realsense -d
```

### 動作確認

```bash
# move_group が起動しているか
docker exec trackdlo-moveit ros2 node list | grep move_group

# joint_trajectory_controller が active か
docker exec trackdlo-gazebo ros2 control list_controllers

# DLO 端点がパブリッシュされているか
docker exec trackdlo-moveit ros2 topic echo /trackdlo/endpoints --once

# 自律追従の有効/無効
docker exec trackdlo-moveit ros2 service call /dlo_manipulation/enable std_srvs/srv/SetBool "{data: false}"
docker exec trackdlo-moveit ros2 service call /dlo_manipulation/enable std_srvs/srv/SetBool "{data: true}"
```

### RViz からの操作

Viz コンテナの RViz2 に MoveIt MotionPlanning display が組み込まれている。

1. 自律追従を停止: `ros2 service call /dlo_manipulation/enable std_srvs/srv/SetBool "{data: false}"`
2. RViz の MotionPlanning パネルで Interactive Marker をドラッグしてゴール姿勢を設定
3. "Plan" ボタンで軌道を確認、"Plan & Execute" で実行

## RealSense 実機テスト (UR5/Gazebo 不要)

実機の RealSense D435 カメラのみでパーセプションパイプラインをテストする。
セグメンテーション方法は launch 引数で切替可能。

### 起動 (Docker)

```bash
cd docker/

# ビルド
bash build.sh realsense

# HSV (デフォルト)
./run.sh amd realsense

# HSV チューナー GUI
./run.sh amd realsense:hsv_tuner

# SAM2 (事前に bash build.sh realsense-sam2 が必要)
./run.sh nvidia realsense:sam2
```

### 起動 (ネイティブ)

```bash
# ビルド
colcon build --packages-select trackdlo_perception trackdlo_utils trackdlo_bringup
source install/setup.bash

# デフォルト: HSV しきい値によるセグメンテーション
ros2 launch trackdlo_bringup realsense_test.launch.py

# インタラクティブ HSV チューナー (スライダー GUI)
ros2 launch trackdlo_bringup realsense_test.launch.py segmentation:=hsv_tuner

# SAM2 セグメンテーション (クリックプロンプト)
ros2 launch trackdlo_bringup realsense_test.launch.py segmentation:=sam2

# RViz なしで起動
ros2 launch trackdlo_bringup realsense_test.launch.py rviz:=false
```

### セグメンテーションモード

| モード | 説明 | ユースケース |
|--------|------|-------------|
| `hsv` (デフォルト) | HSV しきい値処理 | DLO の色が既知、パラメータ調整済み |
| `hsv_tuner` | スライダー GUI でリアルタイム調整 | 新しい DLO の HSV 値を探す |
| `sam2` | Segment Anything Model 2 | 色に依存しない汎用セグメンテーション |

### HSV チューナーの使い方

OpenCV ウィンドウに H/S/V の最小・最大値スライダーが表示される。

- 左: 元画像、右: マスク適用結果
- スライダーを調整して DLO だけが抽出されるようにする
- マスクはリアルタイムで追跡パイプラインに送信される
- **Q キー**: 終了。最終値が YAML 互換形式でログに出力される

### SAM2 の使い方

起動すると最初のフレームが OpenCV ウィンドウに表示される。

- **左クリック**: DLO 上の点 (foreground)
- **右クリック**: 背景の点 (background)
- **R キー**: クリックをリセット
- **Enter**: 確定してセグメンテーション開始

SAM2 は毎フレームクリック点 + 前フレームのバウンディングボックスをプロンプトとして推論する。

```bash
# SAM2 のインストール
pip install git+https://github.com/facebookresearch/sam2.git

# CPU のみの場合 (推論が遅くなる)
# realsense_params.yaml の sam2_segmentation.ros__parameters.force_cpu を true に設定
```

### アーキテクチャ

```
[RealSense D435]
    │
    ├── segmentation:=hsv
    │   → trackdlo_node 内部で HSV しきい値処理 (既存動作)
    │
    ├── segmentation:=hsv_tuner
    │   → HSV Tuner GUI ノード ──→ /trackdlo/segmentation_mask
    │                                        ↓
    └── segmentation:=sam2                trackdlo_node
        → SAM2 ノード ──────────→ /trackdlo/segmentation_mask
                                   (use_external_mask=true)
```

## 処理フロー

### Phase 1: 初期化 (`init_tracker`)

**ノード:** `trackdlo_perception/trackdlo_perception/initialize.py`

最初の 1 フレームから DLO のスケルトンを抽出し、追跡用の初期ノード列を生成する。

```
RGB 画像 + Depth 画像
    │
    ├── 1. HSV 色空間でしきい値処理 → DLO のバイナリマスク生成
    ├── 2. Mode フィルタ (5x5) でノイズ除去
    ├── 3. Zhang のスケルトン化 → 1 ピクセル幅の骨格線
    ├── 4. 輪郭検出 + 曲率制約付きチェーン抽出 (30° 以内)
    ├── 5. カメラ内部パラメータと Depth で 3D 座標に変換
    ├── 6. スプライン補間で滑らかな曲線にフィッティング
    └── 7. 弧長に沿って等間隔に 45 ノードをサンプリング
            │
            v
    /trackdlo/init_nodes (PointCloud2, 1 回だけ publish)
```

### Phase 2: フレーム毎の追跡 (`tracker_node`)

**ノード:** `trackdlo_perception/src/trackdlo_node.cpp`
**アルゴリズム:** `trackdlo_perception/src/trackdlo.cpp`

毎フレームの RGB-D 入力に対して CPD-LLE アルゴリズムでノード位置を更新する。

#### 前処理

```
RGB + Depth (同期受信)
    │
    ├── 1. HSV しきい値処理 → バイナリマスク
    ├── 2. オクルージョンマスク適用 (オプション)
    ├── 3. マスク領域を Depth で 3D 点群に変換
    └── 4. ボクセルグリッドでダウンサンプリング (8mm)
            │
            v
    フィルタ済み点群 X (5,000 ~ 15,000 点)
```

#### 可視性推定

```
現在のノード列 Y + 点群 X
    │
    ├── 1. 各ノードを画像平面に射影
    ├── 2. 射影した辺を深度順に描画 (自己オクルージョン検出)
    ├── 3. 各ノードから最近傍点までの距離を計算
    │      → visibility_threshold (8mm) 以内なら「可視」
    └── 4. 可視ノードを測地線距離 d_vis (6cm) 以内に拡張
            │
            v
    可視ノードリスト + 自己オクルージョンフラグ
```

#### CPD-LLE トラッキング (コアアルゴリズム)

CPD (Coherent Point Drift) に LLE (Locally Linear Embedding) 正則化を加えた非剛体点群レジストレーション。

```
入力: 点群 X, 現在ノード Y, 可視性情報
    │
    ├── 1. 点群プルーニング: ノードから 10cm 以内の点のみ残す
    ├── 2. カーネル行列 G の計算 (β=0.35 でガウシアンカーネル)
    ├── 3. LLE 正則化行列 H の計算 (局所構造保存)
    │
    ├── 4. EM アルゴリズム (最大 50 回反復):
    │      │
    │      ├── E-step: ソフト対応行列 P を計算
    │      │           (可視性・測地線距離で重み付け)
    │      │
    │      ├── M-step: 正則化線形方程式を解く
    │      │   [2σ²P + λG + αH + J] Y_new = 2σ²PX + αY₀ + priors
    │      │     ├── λG : 大域的滑らかさ (λ=50000)
    │      │     ├── αH : 局所構造保存   (α=3.0)
    │      │     └── J  : 対応事前情報   (k_vis=50)
    │      │
    │      ├── σ² 更新 (ノイズパラメータ)
    │      └── 収束判定 (変化量 < 0.0002)
    │
    └── 5. 更新されたノード位置 Y を出力
            │
            v
    /trackdlo/results_pc          (PointCloud2: ノード位置)
    /trackdlo/results_marker      (MarkerArray: RViz 可視化)
    /trackdlo/results_img         (Image: アノテーション付き画像)
    /trackdlo/filtered_pointcloud (PointCloud2: 入力点群)
```

### Phase 3: ロボット操作 (`dlo_manipulation_node`)

**ノード:** `trackdlo_moveit/src/dlo_manipulation_node.cpp`

追跡結果を基に UR5 ロボットアームが DLO の端点間を自律往復する。

```
/trackdlo/results_pc (追跡結果)
    │
    ├── 1. 点群の先頭・末尾を端点 A, B として抽出
    ├── 2. TF2 でカメラ座標系 → planning frame (world) に変換
    ├── 3. /trackdlo/endpoints (PoseArray) として publish
    │
    └── 4. 状態機械 (2Hz, enabled_ && both_endpoints_valid_):
           ├── GOTO_A: 端点 A の上方 (approach_distance=0.3m) へ移動
           └── GOTO_B: 端点 B の上方へ移動
           (交互に切り替え、失敗 3 回で次の端点へ)
                │
                └── MoveGroupInterface → move_group (OMPL)
                        │
                        └── FollowJointTrajectory action
                                │
                                └── joint_trajectory_controller → Gazebo
```

## シミュレーション時の追加処理

Gazebo Fortress を使用する場合、カメラ出力の形式変換が必要:

```
Gazebo カメラ                  depth_format_converter             追跡ノードへ
  float32 (meters)  ────────>   uint16 (millimeters)  ────────>  tracker_node
  /gz/camera/depth_raw          正しいカメラ内部パラメータも再計算
```

## コンテナ起動シーケンス

```
t=0s   trackdlo-gazebo
       ├── Gazebo Fortress (DLO + テーブル + UR5)
       ├── robot_state_publisher → /robot_description, /tf
       ├── ros_gz_bridge (カメラトピック転送)
       ├── depth_format_converter
       ├── joint_state_broadcaster (t=3s)
       └── joint_trajectory_controller (t=5s)

t=10s  trackdlo-perception
       ├── init_tracker (初期化)
       └── trackdlo (追跡)

t=15s  trackdlo-moveit
       ├── move_group (OMPL planner, moveit_controllers.yaml)
       └── dlo_manipulation_node (操作)

t=20s  trackdlo-viz
       └── rviz2 (RobotModel + MotionPlanning display)
```

## ROS2 インターフェース

### トピック

| トピック | 型 | 発行元 | 説明 |
|---|---|---|---|
| `/camera/color/image_raw` | Image | Gazebo | RGB 画像 |
| `/camera/aligned_depth_to_color/image_raw` | Image | Gazebo | Depth 画像 |
| `/camera/aligned_depth_to_color/camera_info` | CameraInfo | Gazebo | カメラパラメータ |
| `/trackdlo/init_nodes` | PointCloud2 | Perception | 初期ノード (1 回) |
| `/trackdlo/results_pc` | PointCloud2 | Perception | 追跡結果ノード位置 |
| `/trackdlo/results_marker` | MarkerArray | Perception | RViz 用マーカー |
| `/trackdlo/results_img` | Image | Perception | アノテーション付き画像 |
| `/trackdlo/filtered_pointcloud` | PointCloud2 | Perception | フィルタ済み入力点群 |
| `/trackdlo/endpoints` | PoseArray | MoveIt | DLO 端点 (world 座標系) |
| `/robot_description` | String | Gazebo | URDF (latched) |
| `/joint_states` | JointState | Gazebo | 関節状態 |

### サービス

| サービス | 型 | ノード | 説明 |
|---|---|---|---|
| `/dlo_manipulation/enable` | SetBool | dlo_manipulation | 自律追従の ON/OFF |

### アクション

| アクション | 型 | ノード | 説明 |
|---|---|---|---|
| `/joint_trajectory_controller/follow_joint_trajectory` | FollowJointTrajectory | Gazebo | 軌道実行 |

## 主要パラメータ

### 追跡パラメータ (`trackdlo_bringup/config/trackdlo_params.yaml`)

| パラメータ | デフォルト値 | 説明 |
|---|---|---|
| `beta` | 0.35 | 形状剛性 (小さいほど柔軟) |
| `lambda` | 50000.0 | 大域的滑らかさの強度 |
| `alpha` | 3.0 | 初期形状への整合性 |
| `mu` | 0.1 | ノイズ比率 (外れ値 10%) |
| `max_iter` | 50 | EM 最大反復数 |
| `tol` | 0.0002 | 収束判定しきい値 |
| `k_vis` | 50.0 | 可視性項の重み |
| `d_vis` | 0.06 | ギャップ補間の最大測地線距離 (m) |
| `visibility_threshold` | 0.008 | 可視判定の距離しきい値 (m) |
| `num_of_nodes` | 45 | 追跡ノード数 |
| `downsample_leaf_size` | 0.008 | ボクセルサイズ (m) |
| `hsv_threshold_lower_limit` | "85 50 20" | HSV 下限 (H, S, V) |
| `hsv_threshold_upper_limit` | "135 255 255" | HSV 上限 (H, S, V) |

### 操作パラメータ (`moveit_planning.launch.py`)

| パラメータ | デフォルト値 | 説明 |
|---|---|---|
| `planning_group` | ur_manipulator | MoveIt planning group |
| `approach_distance` | 0.3 | 端点上方の接近距離 (m) |
| `grasp_offset_z` | 0.05 | 把持 Z オフセット (m) |
| `tracking_rate` | 2.0 | 追従ループ周波数 (Hz) |
| `position_tolerance` | 0.02 | 位置許容誤差 (m) |
| `max_consecutive_failures` | 3 | 端点切り替えまでの連続失敗数 |

## 依存関係

- ROS2 Humble
- Gazebo Fortress
- MoveIt2 (OMPL planner)
- Universal Robots ROS2 Driver (`ur_description`, `ur_moveit_config`)
- ros2_control / gz_ros2_control
- OpenCV, PCL, Eigen3
- scikit-image, scipy, Open3D

## ライセンス

BSD-3-Clause
