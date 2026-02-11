# TrackDLO2

ROS2 Humble ベースの変形線状物体 (DLO: Deformable Linear Object) リアルタイム追跡・操作システム。
RGB-Dカメラで DLO を認識・追跡し、UR5 ロボットアームで操作する。

## システム構成

```
trackdlo2/
├── trackdlo_bringup       # Launch ファイル・パラメータ設定・RViz 設定
├── trackdlo_description   # ロボット (UR5) とカメラ (RealSense D435) の URDF/SDF
├── trackdlo_perception    # DLO 追跡の中核 (初期化 + CPD-LLE トラッキング)
├── trackdlo_moveit        # MoveIt2 によるロボット操作
├── trackdlo_msgs          # カスタムメッセージ定義 (将来用)
├── trackdlo_utils         # Depth 変換・テスト・可視化ユーティリティ
└── docker/                # Docker Compose による実行環境
```

## 処理の全体フロー

```
┌─────────────────────────────────────────────────────────────────────┐
│                        入力 (RGB-D カメラ)                          │
│  /camera/color/image_raw (RGB)                                      │
│  /camera/aligned_depth_to_color/image_raw (Depth)                   │
│  /camera/aligned_depth_to_color/camera_info                         │
└──────────────┬──────────────────────────────────┬───────────────────┘
               │                                  │
               v                                  v
┌──────────────────────────┐       ┌──────────────────────────────┐
│  Phase 1: 初期化          │       │  Phase 2: フレーム毎の追跡     │
│  (init_tracker, Python)  │──────>│  (tracker_node, C++)         │
│  1回のみ実行              │ init  │  連続実行                     │
│                          │ nodes │                              │
└──────────────────────────┘       └──────────────┬───────────────┘
                                                  │
                                                  │ /trackdlo/results_pc
                                                  v
                                   ┌──────────────────────────────┐
                                   │  Phase 3: ロボット操作         │
                                   │  (dlo_manipulation_node, C++) │
                                   │  MoveIt2 による軌道計画・実行   │
                                   └──────────────────────────────┘
```

## Phase 1: 初期化 (`init_tracker`)

**ノード:** `trackdlo_perception/trackdlo_perception/initialize.py`

最初の1フレームから DLO のスケルトンを抽出し、追跡用の初期ノード列を生成する。

```
RGB画像 + Depth画像
    │
    ├── 1. HSV 色空間でしきい値処理 → DLO のバイナリマスク生成
    ├── 2. Mode フィルタ (5x5) でノイズ除去
    ├── 3. Zhang のスケルトン化 → 1ピクセル幅の骨格線
    ├── 4. 輪郭検出 + 曲率制約付きチェーン抽出 (30°以内)
    ├── 5. カメラ内部パラメータと Depth で 3D 座標に変換
    ├── 6. スプライン補間で滑らかな曲線にフィッティング
    └── 7. 弧長に沿って等間隔に 45 ノードをサンプリング
            │
            v
    /trackdlo/init_nodes (PointCloud2, 1回だけ publish)
```

## Phase 2: フレーム毎の追跡 (`tracker_node`)

**ノード:** `trackdlo_perception/src/trackdlo_node.cpp`
**アルゴリズム:** `trackdlo_perception/src/trackdlo.cpp`

毎フレームの RGB-D 入力に対して CPD-LLE アルゴリズムでノード位置を更新する。

### 2-1. 前処理

```
RGB + Depth (同期受信)
    │
    ├── 1. HSV しきい値処理 → バイナリマスク
    ├── 2. オクルージョンマスク適用 (オプション)
    ├── 3. マスク領域を Depth で 3D 点群に変換
    └── 4. ボクセルグリッドでダウンサンプリング (8mm)
            │
            v
    フィルタ済み点群 X (5,000〜15,000 点)
```

### 2-2. 可視性推定

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

### 2-3. CPD-LLE トラッキング (コアアルゴリズム)

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
    /trackdlo/results_pc       (PointCloud2: ノード位置)
    /trackdlo/results_marker   (MarkerArray: RViz 可視化)
    /trackdlo/results_img      (Image: アノテーション付き追跡画像)
    /trackdlo/filtered_pointcloud  (PointCloud2: 入力点群)
```

## Phase 3: ロボット操作 (`dlo_manipulation_node`)

**ノード:** `trackdlo_moveit/src/dlo_manipulation_node.cpp`

追跡結果を基に UR5 ロボットアームが DLO の端点に接近する。

```
/trackdlo/results_pc (追跡結果)
    │
    ├── 1. 点群の先頭・末尾を端点 A, B として抽出
    ├── 2. TF2 でカメラ座標系 → ワールド座標系に変換
    │
    ├── 3. 状態機械 (2Hz で実行):
    │      ├── GOTO_A: 端点 A の上方 (approach_distance=0.3m) へ移動
    │      └── GOTO_B: 端点 B の上方へ移動
    │      (交互に切り替え、失敗3回で次の端点へ)
    │
    └── 4. MoveIt2 で軌道計画 → UR5 実行
            │
            v
    UR5 関節コマンド → /joint_trajectory_controller
```

## シミュレーション時の追加処理

Gazebo Fortress を使用する場合、カメラ出力の形式変換が必要:

```
Gazebo カメラ                    depth_format_converter              追跡ノードへ
  float32 (meters)  ──────────>   uint16 (millimeters)   ──────────>  tracker_node
  /gz/camera/depth_raw            正しいカメラ内部パラメータも再計算
```

## Launch ファイル構成

### シミュレーション (`sim_full_pipeline.launch.py`)

```
t=0s   ur5_gazebo.launch.py
       ├── Gazebo Fortress (DLO + テーブル + UR5)
       ├── ros_gz_bridge (カメラトピック転送)
       ├── depth_format_converter
       ├── joint_state_broadcaster (t=3s)
       └── joint_trajectory_controller (t=5s)

t=10s  trackdlo.launch.py
       ├── init_tracker (初期化)
       └── trackdlo (追跡)

t=15s  moveit_planning.launch.py
       ├── MoveIt2 設定
       └── dlo_manipulation_node (操作)
```

### 実機 (`full_pipeline.launch.py`)

```
camera.launch.py        → RealSense D435 起動 + TF
trackdlo.launch.py      → 初期化 + 追跡
visualize_output.launch.py → RViz 可視化
```

## 主要パラメータ (`trackdlo_bringup/config/trackdlo_params.yaml`)

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

## ROS2 トピック一覧

| トピック | 型 | 方向 | 説明 |
|---|---|---|---|
| `/camera/color/image_raw` | Image | 入力 | RGB 画像 |
| `/camera/aligned_depth_to_color/image_raw` | Image | 入力 | Depth 画像 |
| `/camera/aligned_depth_to_color/camera_info` | CameraInfo | 入力 | カメラパラメータ |
| `/trackdlo/init_nodes` | PointCloud2 | 内部 | 初期ノード (1回) |
| `/trackdlo/results_pc` | PointCloud2 | 出力 | 追跡結果ノード位置 |
| `/trackdlo/results_marker` | MarkerArray | 出力 | RViz 用マーカー |
| `/trackdlo/results_img` | Image | 出力 | アノテーション付き画像 |
| `/trackdlo/filtered_pointcloud` | PointCloud2 | 出力 | フィルタ済み入力点群 |
| `/trackdlo/guide_nodes` | MarkerArray | 出力 | ガイドノード |
| `/trackdlo/self_occluded_pc` | PointCloud2 | 出力 | 自己オクルージョン点群 |

## Docker での実行

```bash
cd docker/

# ビルド
docker compose build

# 起動 (ホスト側で X11 許可が必要)
xhost +local:docker
docker compose up
```

## 依存関係

- ROS2 Humble
- Gazebo Fortress (シミュレーション)
- MoveIt2 (ロボット操作)
- OpenCV, PCL, Eigen3 (知覚処理)
- scikit-image, scipy (初期化)
