#pragma once

#include <stdexcept>

#include <boost/thread/thread.hpp>
#include <fmutil/fm_filter.h>
#include <fmutil/fm_math.h>

#include "common/ros_abstraction_layer/geometry_msgs.hpp"
#include "common/ros_abstraction_layer/nav_msgs.hpp"
#include "common/ros_abstraction_layer/std_msgs.hpp"

#include "common/ros_abstraction_layer/subscriber.hpp"
#include "common/ros_abstraction_layer/publisher.hpp"
#include "common/ros_abstraction_layer/service_server.hpp"
#include "common/ros_abstraction_layer/time.hpp"
#include "common/ros_abstraction_layer/timer.hpp"
#include "common/ros_abstraction_layer/tf.hpp"

#include "mpc_tracking_err_calc.hpp"

#include "lib/generated/acado_auxiliary_functions.h"

namespace av {

class MPCSteering {
 public:

  bool CONTROL_VEL_;
  double TIME_INTERVAL_;
  int NUM_SQP_STEPS_;
  double ref_vel_;
  double min_lookahead_mpc_;
  double dist_to_terminal_pt_;

  bool feasible_;
  std::size_t path_hash_;
  bool initialized_;
  int tracking_point_;

  double dist_to_goal_;
  double lookahead_mpc_;
  geometry_msgs::Pose vehicle_base_;
  nav_msgs::Path predicted_path_;
  nav_msgs::Path mpc_reference_path_;
  double mpc_lateral_offset_;
  double mpc_heading_offset_;
  
  MPCSteering(bool CONTROL_VEL = false, double TIME_INTERVAL = 7.0, int NUM_SQP_STEPS = 10, std::string global_frameID = "golfcart/map", bool publish = true, double ref_vel = 0.0, double min_lookahead_mpc = 4.0, double dist_to_terminal_pt = 0.1);

  ~MPCSteering();

  bool steeringControl(double& steer_angle, float ego_vehicle_velocity);

  void setInitPose();

  void setLookahead(const float ego_vehicle_velocity);

  int setStepSize(const float ego_vehicle_velocity);

  void initAcadoVariables();

  double getAvgWaypointSpacing();

  void setRef(double ego_vel, int step_size);

  void runSQP(acado_timer& t);

  double getSteeringAngle();

  void updateStateFeedback();

  geometry_msgs::Pose computeOrthProj(geometry_msgs::Pose A, geometry_msgs::Pose B, geometry_msgs::Pose P);

  void updateTrackingPoint();

  void getPredictedPath();

  void printReferenceVariables();

  void printWaypoints();

  void getTrackingErrorMPC();

  geometry_msgs::PoseStamped convertToPose(const double x, const double y, const double heading);

  bool debug() const;

  std::size_t getPathHash() const;

  nav_msgs::Path getPath();

  std::vector<geometry_msgs::PoseStamped>* getPathPtr();

  void publishPathFromGeometryPoseStamped(std::vector<geometry_msgs::PoseStamped>& msg, ros::Publisher& pub);

  void setPoses(const std::vector<geometry_msgs::PoseStamped>& poses);

  void setPoses(const std::vector<geometry_msgs::PoseStamped>& poses, std::size_t hash_v);

  void setLocalPoses(const std::vector<geometry_msgs::PoseStamped>& poses);

  void setPath(nav_msgs::PathConstPtr& path);

  void setPath(nav_msgs::PathConstPtr& path, size_t hash_v);

  std::vector<geometry_msgs::PoseStamped> getPoses();

  void setMiddleOfLane(const std::vector<double>& line);

  geometry_msgs::Point getCollidedPt();

  template <typename T>
  void getLineBtwPoints(const T p1, const T p2, double* a, double* b, double* c);

 private:
  boost::mutex path_mutex_;

  std::string global_frameID_;
  int use_lane_detection_;
  bool debug_;
  bool verbose_;
  bool publish_;
  
  std::vector<geometry_msgs::PoseStamped> path_;
  geometry_msgs::Point current_point_;
  geometry_msgs::Point next_point_;
  int padded_pts_no_;
  std::vector<double> middle_of_lane_;
  geometry_msgs::Point collided_pt_;
  std::vector<double> accu_path_dist_;
  std::vector<geometry_msgs::PoseStamped> received_poses_;
  
  MPCTrackingErrCalculator mpc_tracking_err_calculator_;
};

}  // namespace av
