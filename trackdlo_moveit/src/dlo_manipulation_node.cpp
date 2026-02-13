/**
 * DLO (Deformable Linear Object) マニピュレーションノード
 *
 * 処理の流れ:
 * 1. 起動 → パラメータ・TF2 初期化 → deferred_init で MoveGroup/PlanningScene 構築
 * 2. TrackDLO の点群を results_callback で受信 → 計画フレームへ変換 → dlo_nodes_ 更新
 * 3. スタートアップ遅延後、tracking_timer_callback が定期実行
 * 4. SEARCHING: 最後の既知位置周辺を探索 → 検出で FORWARD へ
 * 5. FORWARD/BACKWARD: ノード列に沿ったカルテシアン経路で追従、成功で方向反転
 * 6. 検出が detection_timeout_ 以上途絶えたら SEARCHING に戻る
 */
#include "trackdlo_moveit/dlo_manipulation_node.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// 名前付き定数
static constexpr double kDownwardOrientationX = 1.0;
static constexpr double kDownwardOrientationY = 0.0;
static constexpr double kDownwardOrientationZ = 0.0;
static constexpr double kDownwardOrientationW = 0.0;
static constexpr double kCollisionObjectThickness = 0.02;

// ホームポジション関節角度（UR5/UR6）
static const std::vector<double> kHomeJoints = {
    0.0,       // shoulder_pan
    -M_PI_2,   // shoulder_lift  (-90°: 上腕水平)
    -M_PI_2,   // elbow          (-90°: 前腕上向き)
    -M_PI_2,   // wrist_1
     M_PI_2,   // wrist_2        (+90°: テーブルから離れる方向)
    0.0        // wrist_3
};

// =============================================================================
// コンストラクタ: パラメータ読み込み、TF2準備、遅延初期化タイマー登録
// MoveGroupInterface は shared_from_this() が必要なため、初回コールバックで初期化
// =============================================================================
DloManipulationNode::DloManipulationNode()
    : Node("dlo_manipulation"),
      last_detection_time_(0, 0, RCL_ROS_TIME)
{
    this->declare_parameter<std::string>("planning_group", "ur_manipulator");
    this->declare_parameter<std::string>("results_topic", "/trackdlo/results_pc");
    this->declare_parameter<double>("approach_distance", 0.1);
    this->declare_parameter<double>("tracking_rate", 2.0);
    this->declare_parameter<double>("position_tolerance", 0.02);
    this->declare_parameter<int>("max_consecutive_failures", 3);
    this->declare_parameter<double>("detection_timeout", 5.0);
    this->declare_parameter<double>("startup_delay", 40.0);
    this->declare_parameter<double>("tracking_velocity_scale", 0.05);

    // テーブルジオメトリ
    this->declare_parameter<double>("table_height", 0.75);
    this->declare_parameter<double>("table_center_x", 0.5);
    this->declare_parameter<double>("table_center_y", 0.0);
    this->declare_parameter<double>("table_size_x", 1.2);
    this->declare_parameter<double>("table_size_y", 0.8);

    // 速度・ポーズ
    this->declare_parameter<double>("search_pause_duration", 2.0);
    this->declare_parameter<double>("tracking_pause_duration", 1.0);
    this->declare_parameter<double>("search_velocity_scale", 0.05);
    this->declare_parameter<double>("approach_velocity_scale", 0.2);

    // 探索上限
    this->declare_parameter<int>("max_search_iterations", 30);

    planning_group_ = this->get_parameter("planning_group").as_string();
    results_topic_ = this->get_parameter("results_topic").as_string();
    approach_distance_ = this->get_parameter("approach_distance").as_double();
    tracking_rate_ = this->get_parameter("tracking_rate").as_double();
    position_tolerance_ = this->get_parameter("position_tolerance").as_double();
    max_consecutive_failures_ = this->get_parameter("max_consecutive_failures").as_int();
    detection_timeout_ = this->get_parameter("detection_timeout").as_double();
    startup_delay_ = this->get_parameter("startup_delay").as_double();
    tracking_velocity_scale_ = this->get_parameter("tracking_velocity_scale").as_double();

    table_height_ = this->get_parameter("table_height").as_double();
    table_center_x_ = this->get_parameter("table_center_x").as_double();
    table_center_y_ = this->get_parameter("table_center_y").as_double();
    table_size_x_ = this->get_parameter("table_size_x").as_double();
    table_size_y_ = this->get_parameter("table_size_y").as_double();

    search_pause_duration_ = this->get_parameter("search_pause_duration").as_double();
    tracking_pause_duration_ = this->get_parameter("tracking_pause_duration").as_double();
    search_velocity_scale_ = this->get_parameter("search_velocity_scale").as_double();
    approach_velocity_scale_ = this->get_parameter("approach_velocity_scale").as_double();

    max_search_iterations_ = this->get_parameter("max_search_iterations").as_int();

    // TF2 のセットアップ（バッファとリスナで座標変換を取得）
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // MoveGroupInterface は shared_from_this() が必要なため、初回コールバックで遅延初期化
    init_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(0),
        std::bind(&DloManipulationNode::deferred_init, this));
}

// =============================================================================
// 遅延初期化: MoveGroup / PlanningScene、プランナー設定、衝突オブジェクト追加、
// サブスクライバ・パブリッシャ・サービス登録。スタートアップ遅延後にトラッキング
// タイマーを本稼働させる（その間も results_callback で DLO 検出は受け付ける）
// =============================================================================
void DloManipulationNode::deferred_init()
{
    init_timer_->cancel();

    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), planning_group_);
    planning_scene_interface_ =
        std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

    move_group_->setPlanningPipelineId("ompl");
    move_group_->setPlannerId("RRTConnectkConfigDefault");
    move_group_->setPlanningTime(10.0);
    move_group_->setNumPlanningAttempts(20);
    move_group_->setMaxVelocityScalingFactor(0.3);
    move_group_->setMaxAccelerationScalingFactor(0.3);
    move_group_->setGoalPositionTolerance(0.02);
    move_group_->setWorkspace(-1.0, -1.0, 0.0, 1.0, 1.0, 2.0);

    last_detection_time_ = this->now();

    // 衝突オブジェクトとホームポジション移動は startup delay 後に実行
    // （コントローラー接続を待つ必要があるため）

    // スタートアップ遅延中も DLO を検出できるよう、サブスクライバは即座に開始
    results_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        results_topic_, 10,
        std::bind(&DloManipulationNode::results_callback, this, std::placeholders::_1));

    endpoints_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
        "/trackdlo/endpoints", 10);

    enable_srv_ = this->create_service<std_srvs::srv::SetBool>(
        "~/enable",
        std::bind(&DloManipulationNode::enable_callback, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(),
        "DLO following initialized: group=%s, rate=%.1fHz, tolerance=%.3fm, "
        "detection_timeout=%.1fs, startup_delay=%.1fs",
        planning_group_.c_str(), tracking_rate_, position_tolerance_,
        detection_timeout_, startup_delay_);

    // TrackDLO / RViz の初期化を待つため、トラッキングタイマー開始を遅延
    RCLCPP_INFO(this->get_logger(),
        "Waiting %.1fs for perception pipeline to initialize...", startup_delay_);

    tracking_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(startup_delay_ * 1000.0)),
        [this]() {
            // ワンショット: このスタートアップ用タイマーをキャンセルし、本番用タイマーに差し替え
            tracking_timer_->cancel();

            // 待機中に既に DLO が検出されていたか確認
            bool dlo_already_detected = false;
            {
                std::lock_guard<std::mutex> lock(endpoint_mutex_);
                dlo_already_detected = dlo_nodes_valid_;
            }

            if (dlo_already_detected) {
                // DLO 検出済み → ホームポジション移動をスキップし、直接追従開始
                RCLCPP_INFO(this->get_logger(),
                    "DLO already detected during startup delay! "
                    "Skipping home position move. Starting in FORWARD mode.");
                add_collision_objects();
                traversal_state_ = TraversalState::FORWARD;
            } else {
                // DLO 未検出 → ホームポジションへ移動後に探索開始
                RCLCPP_INFO(this->get_logger(),
                    "DLO not yet detected. Moving to home position first.");
                move_to_home_position();
                add_collision_objects();
            }

            double period_ms = 1000.0 / tracking_rate_;
            tracking_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(static_cast<int>(period_ms)),
                std::bind(&DloManipulationNode::tracking_timer_callback, this));

            RCLCPP_INFO(this->get_logger(),
                "Startup delay complete. Tracking timer started (%.1fHz).",
                tracking_rate_);
        });
}

// =============================================================================
// 検出結果コールバック: TrackDLO の点群 → 計画フレームへ変換 → ノード列と中心を計算
// → 共有データ更新（dlo_nodes_, last_known_dlo_center_ 等）→ 先端・末端を PoseArray で publish
//
// データフロー: trackdlo_node が Y_ (全ノード=可視+不可視推定位置) を
// /trackdlo/results_pc として ~15Hz で発行。visible_nodes < 3 時は発行されない。
// =============================================================================
void DloManipulationNode::results_callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
    pcl::PCLPointCloud2 pcl_cloud;
    pcl_conversions::toPCL(*msg, pcl_cloud);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromPCLPointCloud2(pcl_cloud, cloud);

    if (cloud.size() < 3) return;

    // カメラ座標の点群を計画フレーム（例: base_link）へ一括変換
    std::string planning_frame = move_group_->getPlanningFrame();
    geometry_msgs::msg::TransformStamped tf_stamped;
    try {
        tf_stamped = tf_buffer_->lookupTransform(
            planning_frame, msg->header.frame_id,
            tf2::TimePointZero);
    } catch (tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "TF transform failed: %s", ex.what());
        return;
    }

    // テーブル境界（ノードフィルタリング用。不可視ノードの飛び先を除外）
    double x_min = table_center_x_ - table_size_x_ / 2.0 - 0.1;
    double x_max = table_center_x_ + table_size_x_ / 2.0 + 0.1;
    double y_min = table_center_y_ - table_size_y_ / 2.0 - 0.1;
    double y_max = table_center_y_ + table_size_y_ / 2.0 + 0.1;
    double z_min = table_height_ - 0.1;
    double z_max = table_height_ + 1.0;

    std::vector<Eigen::Vector3d> nodes;
    nodes.reserve(cloud.size());

    for (auto & pt : cloud.points) {
        geometry_msgs::msg::PointStamped pt_cam, pt_world;
        pt_cam.header = msg->header;
        pt_cam.point.x = pt.x;
        pt_cam.point.y = pt.y;
        pt_cam.point.z = pt.z;
        tf2::doTransform(pt_cam, pt_world, tf_stamped);

        // テーブル外の異常なノードをフィルタリング
        double px = pt_world.point.x;
        double py = pt_world.point.y;
        double pz = pt_world.point.z;
        if (px < x_min || px > x_max || py < y_min || py > y_max ||
            pz < z_min || pz > z_max) {
            continue;
        }

        nodes.emplace_back(px, py, pz);
    }

    if (nodes.size() < 3) return;

    // 探索ウェイポイント用に DLO 中心を計算（喪失時の last_known_dlo_center_ に使用）
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    for (auto & n : nodes) center += n;
    center /= static_cast<double>(nodes.size());

    // ノードを move する前にエンドポイント用の先端・末端を取得
    Eigen::Vector3d front = nodes.front();
    Eigen::Vector3d back = nodes.back();

    {
        std::lock_guard<std::mutex> lock(endpoint_mutex_);
        dlo_nodes_ = std::move(nodes);
        latest_frame_id_ = planning_frame;
        dlo_nodes_valid_ = true;
        last_detection_time_ = this->now();
        last_known_dlo_center_ = center;
    }

    // エンドポイント（先端・末端ノード）を PoseArray で publish
    geometry_msgs::msg::PoseArray endpoints_msg;
    endpoints_msg.header.stamp = this->now();
    endpoints_msg.header.frame_id = planning_frame;

    geometry_msgs::msg::Pose pose_a;
    pose_a.position.x = front.x();
    pose_a.position.y = front.y();
    pose_a.position.z = front.z();
    pose_a.orientation.w = 1.0;
    endpoints_msg.poses.push_back(pose_a);

    geometry_msgs::msg::Pose pose_b;
    pose_b.position.x = back.x();
    pose_b.position.y = back.y();
    pose_b.position.z = back.z();
    pose_b.orientation.w = 1.0;
    endpoints_msg.poses.push_back(pose_b);

    endpoints_pub_->publish(endpoints_msg);
}

// =============================================================================
// トラッキング周期コールバック: 有効かつ非実行中の場合のみ処理。
// DLO が detection_timeout_ 以上検出されなければ SEARCHING に戻す。
// SEARCHING → search_for_dlo / それ以外 → track_along_dlo のどちらかを実行。
// =============================================================================
void DloManipulationNode::tracking_timer_callback()
{
    if (!enabled_ || executing_) return;

    // DLO 喪失: タイムアウト後に SEARCHING へ戻す
    {
        std::lock_guard<std::mutex> lock(endpoint_mutex_);
        if (traversal_state_ != TraversalState::SEARCHING && dlo_nodes_valid_) {
            double elapsed = (this->now() - last_detection_time_).seconds();
            if (elapsed > detection_timeout_) {
                RCLCPP_WARN(this->get_logger(),
                    "DLO lost for %.1fs, switching to SEARCHING", elapsed);
                traversal_state_ = TraversalState::SEARCHING;
                dlo_nodes_valid_ = false;
            }
        }
    }

    executing_ = true;

    if (traversal_state_ == TraversalState::SEARCHING) {
        search_for_dlo();
    } else {
        track_along_dlo();
    }

    executing_ = false;
}

// =============================================================================
// DLO 探索: 未検出時、最後の既知中心（last_known_dlo_center_）周辺のパトロール
// ウェイポイントを順に移動。移動中に results_callback で検出されれば FORWARD に切替。
// =============================================================================
void DloManipulationNode::search_for_dlo()
{
    Eigen::Vector3d center;
    // 待機中に DLO が検出されていないか確認
    {
        std::lock_guard<std::mutex> lock(endpoint_mutex_);
        if (dlo_nodes_valid_) {
            RCLCPP_INFO(this->get_logger(),
                "DLO detected! Switching to FORWARD");
            traversal_state_ = TraversalState::FORWARD;
            search_index_ = 0;
            return;
        }
        center = last_known_dlo_center_;
    }

    // テーブルパラメータから探索グリッドを動的生成
    double z = table_height_ + approach_distance_;
    double x_min = table_center_x_ - table_size_x_ / 2.0 + 0.1;
    double x_mid = table_center_x_;
    double x_max = table_center_x_ + table_size_x_ / 2.0 - 0.1;
    double y_min = table_center_y_ - table_size_y_ / 2.0 + 0.15;
    double y_max = table_center_y_ + table_size_y_ / 2.0 - 0.15;

    search_waypoints_ = {
        // まず最後の既知位置の近く
        Eigen::Vector3d(center.x(), center.y(), z),
        // テーブル全体をカバーするグリッド
        Eigen::Vector3d(x_min, 0.0, z),
        Eigen::Vector3d(x_mid, 0.0, z),
        Eigen::Vector3d(x_max, 0.0, z),
        Eigen::Vector3d(x_min, y_min, z),
        Eigen::Vector3d(x_mid, y_min, z),
        Eigen::Vector3d(x_max, y_min, z),
        Eigen::Vector3d(x_min, y_max, z),
        Eigen::Vector3d(x_mid, y_max, z),
        Eigen::Vector3d(x_max, y_max, z),
    };

    size_t wp_idx = search_index_ % search_waypoints_.size();
    auto & wp = search_waypoints_[wp_idx];

    // まず下向き姿勢（setPoseTarget）で試み、失敗したら姿勢制約なし
    // （setPositionTarget + 姿勢許容度を広げる）にフォールバック
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = wp.x();
    target_pose.position.y = wp.y();
    target_pose.position.z = wp.z();
    target_pose.orientation.x = kDownwardOrientationX;
    target_pose.orientation.y = kDownwardOrientationY;
    target_pose.orientation.z = kDownwardOrientationZ;
    target_pose.orientation.w = kDownwardOrientationW;

    move_group_->setMaxVelocityScalingFactor(search_velocity_scale_);
    move_group_->setMaxAccelerationScalingFactor(search_velocity_scale_);

    bool planned = false;
    auto plan_result = moveit::planning_interface::MoveGroupInterface::Plan{};

    // 試行1: 厳密な下向き姿勢
    move_group_->setPoseTarget(target_pose);
    if (move_group_->plan(plan_result) == moveit::core::MoveItErrorCode::SUCCESS) {
        planned = true;
    } else {
        // 試行2: 姿勢制約なし（位置のみ指定、姿勢は自由）
        RCLCPP_INFO(this->get_logger(),
            "Pose target failed for waypoint %zu, trying position-only", wp_idx);
        move_group_->setGoalOrientationTolerance(0.5);  // ~29度の許容
        move_group_->setPoseTarget(target_pose);
        if (move_group_->plan(plan_result) == moveit::core::MoveItErrorCode::SUCCESS) {
            planned = true;
        }
        // 許容度を戻す
        move_group_->setGoalOrientationTolerance(0.01);
    }

    if (planned) {
        RCLCPP_INFO(this->get_logger(),
            "Searching for DLO: moving to [%.3f, %.3f, %.3f] (waypoint %zu/%zu)",
            wp.x(), wp.y(), wp.z(), wp_idx + 1, search_waypoints_.size());
        move_group_->execute(plan_result);

        // 移動後に停止して TrackDLO に検知時間を与える
        RCLCPP_INFO(this->get_logger(),
            "Pausing %.1fs at waypoint %zu for DLO detection...",
            search_pause_duration_, wp_idx);
        rclcpp::sleep_for(std::chrono::milliseconds(
            static_cast<int>(search_pause_duration_ * 1000)));
    } else {
        RCLCPP_WARN(this->get_logger(),
            "Search waypoint %zu plan failed (all attempts), skipping", wp_idx);
    }

    search_index_++;

    // 探索上限チェック
    if (max_search_iterations_ > 0 &&
        static_cast<int>(search_index_) >= max_search_iterations_) {
        RCLCPP_ERROR(this->get_logger(),
            "DLO not found after %zu search iterations. "
            "Returning to home position.", search_index_);
        move_to_home_position();
        search_index_ = 0;
        return;
    }

    // 移動・停止後に再確認（この間に results_callback で検出されていれば FORWARD へ）
    std::lock_guard<std::mutex> lock(endpoint_mutex_);
    if (dlo_nodes_valid_) {
        RCLCPP_INFO(this->get_logger(),
            "DLO detected after search! Switching to FORWARD");
        traversal_state_ = TraversalState::FORWARD;
        search_index_ = 0;
    }
}

// =============================================================================
// ノード列からウェイポイントを生成（共通ヘルパー）
// 各ノードの approach_distance_ 上方に下向き姿勢のウェイポイントを配置
// =============================================================================
std::vector<geometry_msgs::msg::Pose> DloManipulationNode::generate_waypoints(
    const std::vector<Eigen::Vector3d> & nodes) const
{
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.reserve(nodes.size());
    for (const auto & n : nodes) {
        geometry_msgs::msg::Pose p;
        p.position.x = n.x();
        p.position.y = n.y();
        p.position.z = n.z() + approach_distance_;
        p.orientation.x = kDownwardOrientationX;
        p.orientation.y = kDownwardOrientationY;
        p.orientation.z = kDownwardOrientationZ;
        p.orientation.w = kDownwardOrientationW;
        waypoints.push_back(p);
    }
    return waypoints;
}

// =============================================================================
// DLO 沿い追従: 検出ノード列の各点の上方（approach_distance_）をウェイポイントにし、
// カルテシアン経路を計算・実行。成功で進行方向を反転（FWD↔BWD）。
// 経路達成率が低い場合は連続失敗カウントを増やし、上限で方向切替。
// =============================================================================
void DloManipulationNode::track_along_dlo()
{
    std::vector<Eigen::Vector3d> nodes;
    {
        std::lock_guard<std::mutex> lock(endpoint_mutex_);
        if (!dlo_nodes_valid_) return;
        nodes = dlo_nodes_;
    }

    auto waypoints = generate_waypoints(nodes);

    // BACKWARD 時はウェイポイントを逆順にして末端→先端方向に移動
    if (traversal_state_ == TraversalState::BACKWARD) {
        std::reverse(waypoints.begin(), waypoints.end());
    }

    const char * direction =
        (traversal_state_ == TraversalState::FORWARD) ? "FWD" : "BWD";

    // --- Step 1: 最初のウェイポイントへ関節空間プランニングで移動 ---
    // Cartesian path は現在位置から連続的に計算するため、
    // 開始点が遠いと経路達成率が極端に低くなる。
    // まず関節空間で最初のウェイポイントの近くへ移動する。
    auto & first_wp = waypoints.front();
    auto current_pose = move_group_->getCurrentPose().pose;
    double dx = first_wp.position.x - current_pose.position.x;
    double dy = first_wp.position.y - current_pose.position.y;
    double dz = first_wp.position.z - current_pose.position.z;
    double dist_to_start = std::sqrt(dx*dx + dy*dy + dz*dz);

    if (dist_to_start > position_tolerance_) {
        RCLCPP_INFO(this->get_logger(),
            "Moving to %s start point (dist=%.3fm) [%.3f, %.3f, %.3f]",
            direction, dist_to_start,
            first_wp.position.x, first_wp.position.y, first_wp.position.z);

        move_group_->setMaxVelocityScalingFactor(approach_velocity_scale_);
        move_group_->setMaxAccelerationScalingFactor(approach_velocity_scale_);
        move_group_->setPoseTarget(first_wp);

        auto approach_plan = moveit::planning_interface::MoveGroupInterface::Plan{};
        if (move_group_->plan(approach_plan) !=
            moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_WARN(this->get_logger(),
                "Failed to plan approach to %s start point", direction);
            consecutive_failures_++;
            if (consecutive_failures_ >= max_consecutive_failures_) {
                traversal_state_ = (traversal_state_ == TraversalState::FORWARD)
                    ? TraversalState::BACKWARD : TraversalState::FORWARD;
                consecutive_failures_ = 0;
            }
            return;
        }
        if (move_group_->execute(approach_plan) !=
            moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_WARN(this->get_logger(),
                "Failed to execute approach to %s start point", direction);
            consecutive_failures_++;
            if (consecutive_failures_ >= max_consecutive_failures_) {
                traversal_state_ = (traversal_state_ == TraversalState::FORWARD)
                    ? TraversalState::BACKWARD : TraversalState::FORWARD;
                consecutive_failures_ = 0;
            }
            return;
        }
    }

    // --- Step 2: 開始点で停止し TrackDLO の検知を安定させる ---
    RCLCPP_INFO(this->get_logger(),
        "Pausing %.1fs at %s start for detection to stabilize...",
        tracking_pause_duration_, direction);
    rclcpp::sleep_for(std::chrono::milliseconds(
        static_cast<int>(tracking_pause_duration_ * 1000)));

    // DLO ノードを最新に更新（停止中に新しい検出結果が来ている可能性）
    {
        std::lock_guard<std::mutex> lock(endpoint_mutex_);
        if (!dlo_nodes_valid_) return;
        nodes = dlo_nodes_;
    }
    // ウェイポイントを最新ノードで再生成
    waypoints = generate_waypoints(nodes);
    if (traversal_state_ == TraversalState::BACKWARD) {
        std::reverse(waypoints.begin(), waypoints.end());
    }

    // --- Step 3: Cartesian path で DLO に沿って追従 ---
    move_group_->setMaxVelocityScalingFactor(tracking_velocity_scale_);
    move_group_->setMaxAccelerationScalingFactor(tracking_velocity_scale_);

    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group_->computeCartesianPath(
        waypoints, 0.01, 0.0, trajectory, true);

    RCLCPP_INFO(this->get_logger(),
        "Cartesian path [%s]: %.1f%% achieved (%zu waypoints)",
        direction, fraction * 100.0, waypoints.size());

    if (fraction >= 0.3) {
        move_group_->execute(trajectory);

        // 追従完了後に停止して次の検知を安定させる
        RCLCPP_INFO(this->get_logger(),
            "Pausing %.1fs at %s end for detection to stabilize...",
            tracking_pause_duration_, direction);
        rclcpp::sleep_for(std::chrono::milliseconds(
            static_cast<int>(tracking_pause_duration_ * 1000)));

        // 成功時は進行方向を反転（往復追従）
        traversal_state_ = (traversal_state_ == TraversalState::FORWARD)
            ? TraversalState::BACKWARD : TraversalState::FORWARD;
        consecutive_failures_ = 0;
    } else {
        consecutive_failures_++;
        RCLCPP_WARN(this->get_logger(),
            "Cartesian path [%s] insufficient (%.1f%%), failure %d/%d",
            direction, fraction * 100.0,
            consecutive_failures_, max_consecutive_failures_);
        // 連続失敗が上限に達したら方向を切り替えて再試行
        if (consecutive_failures_ >= max_consecutive_failures_) {
            traversal_state_ = (traversal_state_ == TraversalState::FORWARD)
                ? TraversalState::BACKWARD : TraversalState::FORWARD;
            consecutive_failures_ = 0;
        }
    }
}

// 軌道の時間・速度・加速度を scale 倍にスケール（遅くする場合は scale < 1）
void DloManipulationNode::scale_trajectory_speed(
    moveit_msgs::msg::RobotTrajectory & trajectory, double scale)
{
    for (auto & point : trajectory.joint_trajectory.points) {
        // time_from_start を 1/scale 倍に延ばす（実時間を長くして遅くする）
        double t = point.time_from_start.sec +
                   point.time_from_start.nanosec * 1e-9;
        t /= scale;
        point.time_from_start.sec = static_cast<int32_t>(t);
        point.time_from_start.nanosec =
            static_cast<uint32_t>((t - point.time_from_start.sec) * 1e9);

        // 速度は scale 倍、加速度は scale^2 倍にスケール
        for (auto & v : point.velocities) v *= scale;
        for (auto & a : point.accelerations) a *= scale * scale;
    }
}

// 自律追従の有効/無効を SetBool サービスで切り替え
void DloManipulationNode::enable_callback(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response)
{
    enabled_ = request->data;
    response->success = true;
    response->message = enabled_ ? "Autonomous tracking enabled" : "Autonomous tracking disabled";
    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
}

// 衝突オブジェクト追加前にテーブル上方の安全な位置へ移動
// 初期関節状態では wrist_2_link がテーブルと衝突するため、
// 障害物なしの状態で関節空間プランニングにより安全位置へ退避する
void DloManipulationNode::move_to_home_position()
{
    move_group_->setMaxVelocityScalingFactor(0.3);
    move_group_->setMaxAccelerationScalingFactor(0.3);
    move_group_->setJointValueTarget(kHomeJoints);

    // コントローラー接続を待ちつつリトライ
    const int max_attempts = 15;
    for (int i = 0; i < max_attempts; ++i) {
        auto plan = moveit::planning_interface::MoveGroupInterface::Plan{};
        if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_WARN(this->get_logger(),
                "Home position plan failed (attempt %d/%d)", i + 1, max_attempts);
            rclcpp::sleep_for(std::chrono::seconds(2));
            continue;
        }
        if (move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Reached home position.");
            return;
        }
        RCLCPP_WARN(this->get_logger(),
            "Home position execute failed (attempt %d/%d), "
            "waiting for controller...", i + 1, max_attempts);
        rclcpp::sleep_for(std::chrono::seconds(2));
    }
    RCLCPP_ERROR(this->get_logger(),
        "Could not move to home position after %d attempts", max_attempts);
}

// 計画空間にテーブルを衝突オブジェクトとして追加（ロボットとの干渉回避）
void DloManipulationNode::add_collision_objects()
{
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

    // テーブル上面（薄い直方体。テーブル下のロボットリンクとの衝突を避ける）
    moveit_msgs::msg::CollisionObject table;
    table.header.frame_id = move_group_->getPlanningFrame();
    table.id = "table";

    shape_msgs::msg::SolidPrimitive table_shape;
    table_shape.type = shape_msgs::msg::SolidPrimitive::BOX;
    table_shape.dimensions = {table_size_x_, table_size_y_, kCollisionObjectThickness};

    geometry_msgs::msg::Pose table_pose;
    table_pose.position.x = table_center_x_;
    table_pose.position.y = table_center_y_;
    table_pose.position.z = table_height_ - kCollisionObjectThickness / 2.0;
    table_pose.orientation.w = 1.0;

    table.primitives.push_back(table_shape);
    table.primitive_poses.push_back(table_pose);
    table.operation = moveit_msgs::msg::CollisionObject::ADD;
    collision_objects.push_back(table);

    planning_scene_interface_->addCollisionObjects(collision_objects);
    RCLCPP_INFO(this->get_logger(), "Added %zu collision objects",
                collision_objects.size());
}

// エントリポイント: マルチスレッド Executor でノードを spin
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DloManipulationNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
