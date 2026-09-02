#include "robot/vision.hpp"
#include "robot/config.hpp"

#include <algorithm>
#include <cmath>

namespace robot {

namespace {

// Default field layout: one tag centred on each wall, 72 inches out.
const std::vector<FieldTag> DEFAULT_TAGS = {
    {1,   0.0,  72.0},
    {2,  72.0,   0.0},
    {3,   0.0, -72.0},
    {4, -72.0,   0.0},
};

}

AprilTagLocalizer::AprilTagLocalizer(Hardware& hardware, Odometry& odometry, VisionConfig config)
    : hw_(hardware), odometry_(odometry), config_(config), tags_(DEFAULT_TAGS) {}

void AprilTagLocalizer::begin() {
    hw_.aivision.enable_detection_types(pros::AivisionModeType::tags);
    hw_.aivision.set_tag_family(pros::AivisionTagFamily::tag_25H9);
    hw_.aivision.start_awb();
}

void AprilTagLocalizer::set_tag_map(std::vector<FieldTag> tags) {
    tags_ = std::move(tags);
}

const FieldTag* AprilTagLocalizer::find_tag(int id) const {
    for (const FieldTag& tag : tags_) {
        if (tag.id == id) return &tag;
    }
    return nullptr;
}

double AprilTagLocalizer::focal_length_px() const {
    return (config_.image_width_px / 2.0)
         / std::tan(config_.horizontal_fov_deg / 2.0 * deg2rad);
}

bool AprilTagLocalizer::initial_fix(double start_heading, int max_attempts) {
    const double focal  = focal_length_px();
    const double centre = config_.image_width_px / 2.0;

    // Anything more than max_correction_in away from where we already think we
    // are is measured against this, the pose as it stands before any fix.
    double reference_x = 0.0;
    double reference_y = 0.0;
    Pose   start;
    if (odometry_.read(start)) {
        reference_x = start.x;
        reference_y = start.y;
    }

    for (int attempt = 1; attempt <= max_attempts; attempt++) {
        int count = hw_.aivision.get_object_count();
        if (count < 0 || count == PROS_ERR)    count = 0;
        if (count > AIVISION_MAX_OBJECT_COUNT) count = AIVISION_MAX_OBJECT_COUNT;

        pros::lcd::print(1, "FIX try %d/%d objs:%d", attempt, max_attempts, count);

        // Of everything visible this frame, trust the widest tag: it is the
        // closest, so its distance estimate has the least error.
        double best_width = 0.0;
        double best_x     = 0.0;
        double best_y     = 0.0;
        int    best_id    = -1;

        for (int i = 0; i < count; i++) {
            pros::AIVision::Object object = hw_.aivision.get_object(i);
            if (!pros::AIVision::is_type(object, pros::AivisionDetectType::tag)) continue;

            const auto&  corners = object.object.tag;
            const double min_x = std::min(std::min(static_cast<double>(corners.x0),
                                                   static_cast<double>(corners.x1)),
                                          std::min(static_cast<double>(corners.x2),
                                                   static_cast<double>(corners.x3)));
            const double max_x = std::max(std::max(static_cast<double>(corners.x0),
                                                   static_cast<double>(corners.x1)),
                                          std::max(static_cast<double>(corners.x2),
                                                   static_cast<double>(corners.x3)));
            const double tag_width = max_x - min_x;

            pros::lcd::print(3, "saw T%d w:%.0f", object.id, tag_width);
            if (tag_width < config_.min_tag_width_px) continue;

            const FieldTag* tag = find_tag(object.id);
            if (tag == nullptr) {
                pros::lcd::print(4, "T%d not in map", object.id);
                continue;
            }

            // A tag of known size projects to a width that falls off with
            // distance, so width alone gives range.
            const double distance     = (config_.tag_size_in * focal) / tag_width;
            const double pixel_offset = ((min_x + max_x) * 0.5) - centre;
            const double angle_offset = std::atan2(pixel_offset, focal) * rad2deg;
            const double bearing_rad  = normalise_angle(start_heading + angle_offset) * deg2rad;

            const double estimated_x = tag->x - distance * std::sin(bearing_rad);
            const double estimated_y = tag->y - distance * std::cos(bearing_rad);

            if (std::hypot(estimated_x - reference_x, estimated_y - reference_y)
                > config_.max_correction_in) {
                pros::lcd::print(4, "T%d fix too far off", object.id);
                continue;
            }

            if (tag_width > best_width) {
                best_width = tag_width;
                best_x     = estimated_x;
                best_y     = estimated_y;
                best_id    = object.id;
            }
        }

        if (best_id >= 0) {
            odometry_.set_translation(best_x, best_y);
            pros::lcd::print(1, "FIX OK T%d (%.1f,%.1f)", best_id, best_x, best_y);
            return true;
        }

        pros::delay(50);
    }

    pros::lcd::print(1, "FIX: none - default pose");
    return false;
}

}
