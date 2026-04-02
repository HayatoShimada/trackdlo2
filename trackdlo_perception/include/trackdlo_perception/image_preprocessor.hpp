#ifndef TRACKDLO_PERCEPTION_IMAGE_PREPROCESSOR_HPP
#define TRACKDLO_PERCEPTION_IMAGE_PREPROCESSOR_HPP

#include <opencv2/opencv.hpp>
#include <vector>

namespace trackdlo_perception {

class ImagePreprocessor {
public:
    ImagePreprocessor(bool use_external_mask, bool multi_color_dlo,
                      const std::vector<int>& lower, const std::vector<int>& upper);

    void set_external_mask(const cv::Mat& mask);
    void set_occlusion_mask(const cv::Mat& mask);
    bool has_external_mask() const;

    // Returns true if mask is successfully generated.
    bool process(const cv::Mat& cur_image_orig, cv::Mat& out_mask, cv::Mat& out_cur_image);

private:
    cv::Mat color_thresholding(const cv::Mat& cur_image_hsv);

    bool use_external_mask_;
    bool multi_color_dlo_;
    std::vector<int> lower_;
    std::vector<int> upper_;

    cv::Mat external_mask_;
    cv::Mat occlusion_mask_;
    bool received_external_mask_{false};
    bool updated_opencv_mask_{false};
};

} // namespace trackdlo_perception

#endif // TRACKDLO_PERCEPTION_IMAGE_PREPROCESSOR_HPP
