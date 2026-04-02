#include "trackdlo_perception/visibility_checker.hpp"
#include <map>
#include <algorithm>

namespace trackdlo_perception {

VisibilityResult VisibilityChecker::check_visibility(
    const Eigen::MatrixXd& Y,
    const Eigen::MatrixXd& X,
    const Eigen::MatrixXd& proj_matrix,
    const cv::Mat& mask,
    double visibility_threshold,
    int dlo_pixel_width)
{
    VisibilityResult result;

    // 1. Calculate shortest distance from each node in Y to any point in X
    std::map<int, double> shortest_node_pt_dists;
    for (int m = 0; m < Y.rows(); m++) {
        double shortest_dist = 100000.0;
        // loop through all points in X
        for (int n = 0; n < X.rows(); n++) {
            double dist = (Y.row(m) - X.row(n)).norm();
            if (dist < shortest_dist) {
                shortest_dist = dist;
            }
        }
        shortest_node_pt_dists[m] = shortest_dist;
    }

    // 2. Sort nodes based on how far away they are from the camera
    std::vector<double> averaged_node_camera_dists;
    std::vector<int> indices_vec;
    for (int i = 0; i < Y.rows() - 1; i++) {
        averaged_node_camera_dists.push_back(((Y.row(i) + Y.row(i + 1)) / 2.0).norm());
        indices_vec.push_back(i);
    }
    
    // Sort from closest to furthest
    std::sort(indices_vec.begin(), indices_vec.end(),
        [&](const int& a, const int& b) {
            return (averaged_node_camera_dists[a] < averaged_node_camera_dists[b]);
        }
    );

    cv::Mat projected_edges = cv::Mat::zeros(mask.rows, mask.cols, CV_8U);

    // project Y onto projected_edges
    Eigen::MatrixXd Y_h = Y.replicate(1, 1);
    Y_h.conservativeResize(Y_h.rows(), Y_h.cols() + 1);
    Y_h.col(Y_h.cols() - 1) = Eigen::MatrixXd::Ones(Y_h.rows(), 1);
    Eigen::MatrixXd image_coords_mask = (proj_matrix * Y_h.transpose()).transpose();

    int img_rows = projected_edges.rows;
    int img_cols = projected_edges.cols;

    for (int idx : indices_vec) {
        // skip nodes behind the camera (z <= 0)
        if (image_coords_mask(idx, 2) <= 0 || image_coords_mask(idx + 1, 2) <= 0) {
            continue;
        }

        int col_1 = static_cast<int>(image_coords_mask(idx, 0) / image_coords_mask(idx, 2));
        int row_1 = static_cast<int>(image_coords_mask(idx, 1) / image_coords_mask(idx, 2));

        int col_2 = static_cast<int>(image_coords_mask(idx + 1, 0) / image_coords_mask(idx + 1, 2));
        int row_2 = static_cast<int>(image_coords_mask(idx + 1, 1) / image_coords_mask(idx + 1, 2));

        bool pt1_in_bounds = (row_1 >= 0 && row_1 < img_rows && col_1 >= 0 && col_1 < img_cols);
        bool pt2_in_bounds = (row_2 >= 0 && row_2 < img_rows && col_2 >= 0 && col_2 < img_cols);

        // 3. Check visibility based on projection
        if (pt1_in_bounds && projected_edges.at<uchar>(row_1, col_1) == 0) {
            if (shortest_node_pt_dists[idx] <= visibility_threshold) {
                if (std::find(result.visible_nodes.begin(), result.visible_nodes.end(), idx) == result.visible_nodes.end()) {
                    result.visible_nodes.push_back(idx);
                }
            }
            if (std::find(result.not_self_occluded_nodes.begin(), result.not_self_occluded_nodes.end(), idx) == result.not_self_occluded_nodes.end()) {
                result.not_self_occluded_nodes.push_back(idx);
            }
        }

        if (pt2_in_bounds && projected_edges.at<uchar>(row_2, col_2) == 0) {
            if (shortest_node_pt_dists[idx + 1] <= visibility_threshold) {
                if (std::find(result.visible_nodes.begin(), result.visible_nodes.end(), idx + 1) == result.visible_nodes.end()) {
                    result.visible_nodes.push_back(idx + 1);
                }
            }
            if (std::find(result.not_self_occluded_nodes.begin(), result.not_self_occluded_nodes.end(), idx + 1) == result.not_self_occluded_nodes.end()) {
                result.not_self_occluded_nodes.push_back(idx + 1);
            }
        }

        // add edges for checking overlap with upcoming nodes
        cv::line(projected_edges, cv::Point(col_1, row_1), cv::Point(col_2, row_2), cv::Scalar(255), dlo_pixel_width);
    }

    std::sort(result.visible_nodes.begin(), result.visible_nodes.end());

    return result;
}

} // namespace trackdlo_perception
