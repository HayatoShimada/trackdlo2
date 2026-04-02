#include "trackdlo_perception/image_preprocessor.hpp"

namespace trackdlo_perception {

ImagePreprocessor::ImagePreprocessor(bool use_external_mask, bool multi_color_dlo,
                                     const std::vector<int>& lower, const std::vector<int>& upper)
    : use_external_mask_(use_external_mask), multi_color_dlo_(multi_color_dlo),
      lower_(lower), upper_(upper) {}

void ImagePreprocessor::set_external_mask(const cv::Mat& mask) {
    external_mask_ = mask.clone();
    if (!external_mask_.empty()) {
        received_external_mask_ = true;
    }
}

void ImagePreprocessor::set_occlusion_mask(const cv::Mat& mask) {
    mask.copyTo(occlusion_mask_);
    if (!occlusion_mask_.empty()) {
        updated_opencv_mask_ = true;
    }
}

bool ImagePreprocessor::has_external_mask() const {
    return received_external_mask_;
}

cv::Mat ImagePreprocessor::color_thresholding(const cv::Mat& cur_image_hsv) {
    std::vector<int> lower_blue = {90, 90, 60};
    std::vector<int> upper_blue = {130, 255, 255};

    std::vector<int> lower_red_1 = {130, 60, 50};
    std::vector<int> upper_red_1 = {255, 255, 255};

    std::vector<int> lower_red_2 = {0, 60, 50};
    std::vector<int> upper_red_2 = {10, 255, 255};

    std::vector<int> lower_yellow = {15, 100, 80};
    std::vector<int> upper_yellow = {40, 255, 255};

    cv::Mat mask_blue, mask_red_1, mask_red_2, mask_red, mask_yellow, mask;
    // filter blue
    cv::inRange(cur_image_hsv, cv::Scalar(lower_blue[0], lower_blue[1], lower_blue[2]), cv::Scalar(upper_blue[0], upper_blue[1], upper_blue[2]), mask_blue);

    // filter red
    cv::inRange(cur_image_hsv, cv::Scalar(lower_red_1[0], lower_red_1[1], lower_red_1[2]), cv::Scalar(upper_red_1[0], upper_red_1[1], upper_red_1[2]), mask_red_1);
    cv::inRange(cur_image_hsv, cv::Scalar(lower_red_2[0], lower_red_2[1], lower_red_2[2]), cv::Scalar(upper_red_2[0], upper_red_2[1], upper_red_2[2]), mask_red_2);

    // filter yellow
    cv::inRange(cur_image_hsv, cv::Scalar(lower_yellow[0], lower_yellow[1], lower_yellow[2]), cv::Scalar(upper_yellow[0], upper_yellow[1], upper_yellow[2]), mask_yellow);

    // combine red mask
    cv::bitwise_or(mask_red_1, mask_red_2, mask_red);
    // combine overall mask
    cv::bitwise_or(mask_red, mask_blue, mask);
    cv::bitwise_or(mask_yellow, mask, mask);

    return mask;
}

bool ImagePreprocessor::process(const cv::Mat& cur_image_orig, cv::Mat& out_mask, cv::Mat& out_cur_image) {
    cv::Mat mask_without_occlusion_block;

    if (use_external_mask_) {
        // If external mask is required but not received yet, return false
        if (!received_external_mask_) {
            return false;
        }
        // Resize if needed to match the input image dimensions
        if (external_mask_.rows != cur_image_orig.rows || external_mask_.cols != cur_image_orig.cols) {
            cv::resize(external_mask_, mask_without_occlusion_block, cur_image_orig.size(), 0, 0, cv::INTER_NEAREST);
        } else {
            external_mask_.copyTo(mask_without_occlusion_block);
        }
    } else {
        cv::Mat cur_image_hsv;
        cv::cvtColor(cur_image_orig, cur_image_hsv, cv::COLOR_BGR2HSV);

        if (!multi_color_dlo_) {
            cv::inRange(cur_image_hsv, cv::Scalar(lower_[0], lower_[1], lower_[2]), 
                        cv::Scalar(upper_[0], upper_[1], upper_[2]), mask_without_occlusion_block);
        } else {
            mask_without_occlusion_block = color_thresholding(cur_image_hsv);
        }
    }

    // 1. Combine segmentation mask with occlusion mask if available
    if (updated_opencv_mask_) {
        cv::Mat occlusion_mask_gray;
        cv::cvtColor(occlusion_mask_, occlusion_mask_gray, cv::COLOR_BGR2GRAY);
        cv::bitwise_and(mask_without_occlusion_block, occlusion_mask_gray, out_mask);
        cv::bitwise_and(cur_image_orig, occlusion_mask_, out_cur_image);
    } else {
        mask_without_occlusion_block.copyTo(out_mask);
        cur_image_orig.copyTo(out_cur_image);
    }

    return true;
}

} // namespace trackdlo_perception
