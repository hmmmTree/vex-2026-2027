#pragma once

#include "robot/hardware.hpp"
#include "robot/odometry.hpp"

#include <vector>

namespace robot {

struct FieldTag {
    int    id;
    double x;
    double y;
};

struct VisionConfig {
    double horizontal_fov_deg = 73.0;
    double image_width_px     = 320.0;
    double tag_size_in        = 5.0;

    double min_tag_width_px = 15.0;

    double max_correction_in = 12.0;
};

class AprilTagLocalizer {
public:
    AprilTagLocalizer(Hardware& hardware, Odometry& odometry, VisionConfig config = {});

    void begin();

    void                         set_tag_map(std::vector<FieldTag> tags);
    const std::vector<FieldTag>& tag_map() const { return tags_; }

    VisionConfig config() const { return config_; }
    void         set_config(VisionConfig config) { config_ = config; }

    bool initial_fix(double start_heading, int max_attempts);

private:
    const FieldTag* find_tag(int id) const;
    double          focal_length_px() const;

    Hardware&             hw_;
    Odometry&             odometry_;
    VisionConfig          config_;
    std::vector<FieldTag> tags_;
};

}
