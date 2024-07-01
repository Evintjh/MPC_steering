#pragma once

#include <cmath>

#include <boost/optional.hpp>

#include "common/ros_abstraction_layer/geometry_msgs.hpp"

class MPCTrackingErrCalculator {
 public:
  void SetVehicleState(const geometry_msgs::Point& pt, double yaw);
  void SetRefLanes(const std::vector<double>& svg_ref_path = {}, const std::vector<double>& middle_of_lane = {});
  void ClearAllLanes();
  std::vector<double> GetTrackingLane(uint8_t tracking_type);
  double computeLateralOffset(const std::vector<double>& lane, const geometry_msgs::Point& vehicle_pose, double vehicle_heading);
  double computeHeadingOffset(const std::vector<double>& lane, const geometry_msgs::Point& vehicle_pose, double vehicle_heading);

 private:
  uint8_t target_mode_{0};

  boost::optional<std::vector<double>> svg_ref_path_;
  boost::optional<std::vector<double>> middle_of_lane_;
  boost::optional<geometry_msgs::Point> previous_closest_pt_;
  boost::optional<geometry_msgs::Point> current_lookahead_pt_;
  geometry_msgs::Point vehicle_pose_;
  double vehicle_heading_{0.0};
};
