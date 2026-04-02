#include "trackdlo_perception/visualizer.hpp"

namespace trackdlo_perception {

cv::Mat Visualizer::draw_tracking_image(
    const cv::Mat& original_image, 
    const cv::Mat& processed_image, 
    const Eigen::MatrixXd& Y, 
    const Eigen::MatrixXd& proj_matrix, 
    const std::vector<int>& visible_nodes)
{
    // sort nodes based on how far away they are from the camera
    std::vector<double> averaged_node_camera_dists = {};
    std::vector<int> indices_vec = {};
    for (int i = 0; i < Y.rows() - 1; i++) {
        averaged_node_camera_dists.push_back(((Y.row(i) + Y.row(i + 1)) / 2).norm());
        indices_vec.push_back(i);
    }
    std::sort(indices_vec.begin(), indices_vec.end(),
        [&](const int& a, const int& b) {
            return (averaged_node_camera_dists[a] < averaged_node_camera_dists[b]);
        }
    );
    std::reverse(indices_vec.begin(), indices_vec.end());

    Eigen::MatrixXd nodes_h = Y.replicate(1, 1);
    nodes_h.conservativeResize(nodes_h.rows(), nodes_h.cols() + 1);
    nodes_h.col(nodes_h.cols() - 1) = Eigen::MatrixXd::Ones(nodes_h.rows(), 1);
    Eigen::MatrixXd image_coords = (proj_matrix * nodes_h.transpose()).transpose();

    cv::Mat tracking_img = 0.5 * original_image + 0.5 * processed_image;

    // draw points
    for (int idx : indices_vec) {

        int x = static_cast<int>(image_coords(idx, 0) / image_coords(idx, 2));
        int y = static_cast<int>(image_coords(idx, 1) / image_coords(idx, 2));

        cv::Scalar point_color;
        cv::Scalar line_color;

        if (std::find(visible_nodes.begin(), visible_nodes.end(), idx) != visible_nodes.end()) {
            point_color = cv::Scalar(0, 150, 255);
            line_color = cv::Scalar(0, 255, 0);
        }
        else {
            point_color = cv::Scalar(0, 0, 255);

            // line is colored red only when both bounding nodes are not visible
            if (std::find(visible_nodes.begin(), visible_nodes.end(), idx + 1) == visible_nodes.end()) {
                line_color = cv::Scalar(0, 0, 255);
            }
            else {
                line_color = cv::Scalar(0, 255, 0);
            }
        }

        cv::line(tracking_img, cv::Point(x, y),
                               cv::Point(static_cast<int>(image_coords(idx + 1, 0) / image_coords(idx + 1, 2)),
                                         static_cast<int>(image_coords(idx + 1, 1) / image_coords(idx + 1, 2))),
                               line_color, 5);

        cv::circle(tracking_img, cv::Point(x, y), 7, point_color, -1);

        if (std::find(visible_nodes.begin(), visible_nodes.end(), idx + 1) != visible_nodes.end()) {
            point_color = cv::Scalar(0, 150, 255);
        }
        else {
            point_color = cv::Scalar(0, 0, 255);
        }
        cv::circle(tracking_img, cv::Point(static_cast<int>(image_coords(idx + 1, 0) / image_coords(idx + 1, 2)),
                                            static_cast<int>(image_coords(idx + 1, 1) / image_coords(idx + 1, 2))),
                                            7, point_color, -1);
    }

    return tracking_img;
}

} // namespace trackdlo_perception
