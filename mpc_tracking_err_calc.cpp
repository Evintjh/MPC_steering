#include "common/ros_abstraction_layer/time.hpp"
#include "mpc_tracking_err_calc.hpp"


void MPCTrackingErrCalculator::SetVehicleState(const geometry_msgs::Point& pt, double yaw) {
  vehicle_pose_ = pt;
  vehicle_heading_ = yaw;
}

void MPCTrackingErrCalculator::SetRefLanes(const std::vector<double>& svg_ref_path, const std::vector<double>& middle_of_lane) {
  if (svg_ref_path.size() == 3) {
    svg_ref_path_ = svg_ref_path;
  }
  if (middle_of_lane.size() == 3) {
    middle_of_lane_ = middle_of_lane;
  }
}

void MPCTrackingErrCalculator::ClearAllLanes() {
  svg_ref_path_.reset();
  middle_of_lane_.reset();
}

std::vector<double> MPCTrackingErrCalculator::GetTrackingLane(uint8_t tracking_type) {
  std::vector<double> tracking_lane;
  switch (static_cast<int>(tracking_type)) {
    case 0:
      if (svg_ref_path_) {
        tracking_lane = svg_ref_path_.get();
      }
      break;
    case 1:
      if (middle_of_lane_) {
        tracking_lane = middle_of_lane_.get();
      }
      break;
    default:
      break;
  }
  return tracking_lane;
}

double MPCTrackingErrCalculator::computeLateralOffset(const std::vector<double>& lane, const geometry_msgs::Point& vehicle_pose, double vehicle_heading) {
  double lateral_offset = 10000.0;

  if (lane.size() == 3) {
    const double norm_sq = lane[0] * lane[0] + lane[1] * lane[1];
    if (norm_sq < 1e-10) {
      return lateral_offset;
    }
    lateral_offset = std::fabs(lane[0] * vehicle_pose.x + lane[1] * vehicle_pose.y + lane[2]) / sqrt(norm_sq);
  }
  return lateral_offset;
}

double MPCTrackingErrCalculator::computeHeadingOffset(const std::vector<double>& lane, const geometry_msgs::Point& vehicle_pose, double vehicle_heading) {
  double heading_offset = 10000.0;

  if (lane.size() == 3) {
    const double norm_sq = lane[0] * lane[0] + lane[1] * lane[1];
    if (norm_sq < 1e-10) {
      return heading_offset;
    }
    const double slope = atan(-lane[0] / lane[1]);
    double diff = std::fabs(slope - vehicle_heading);

    if (diff > 0.5 * M_PI) {
      diff = M_PI - diff;
    }
    heading_offset = diff;
  }
  return heading_offset;
}
