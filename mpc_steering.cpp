#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>
#include <cfloat>

#include "common/ros_abstraction_layer/init.hpp"
#include "common/ros_abstraction_layer/node.hpp"

#include "lib/generated/acado_common.h"

#include "mpc_steering.hpp"

#include "TF.h"


/* Global variables used by the solver. */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

namespace av {

MPCSteering::MPCSteering(bool CONTROL_VEL, double TIME_INTERVAL, int NUM_SQP_STEPS, std::string global_frameID, bool publish, double ref_vel, double min_lookahead_mpc, double dist_to_terminal_pt)
    : CONTROL_VEL_(CONTROL_VEL)
    , TIME_INTERVAL_(TIME_INTERVAL)
    , NUM_SQP_STEPS_(NUM_SQP_STEPS)
    , global_frameID_(global_frameID)
    , ref_vel_(ref_vel)
    , min_lookahead_mpc_(min_lookahead_mpc)
    , dist_to_terminal_pt_(dist_to_terminal_pt)
    , feasible_(true)
    , path_hash_(0)
    , initialized_(false)
    , tracking_point_(-1)
    , use_lane_detection_(-1)
    , padded_pts_no_(0)
    , debug_(false)
    , verbose_(false)
    , publish_(publish) {
}

MPCSteering::~MPCSteering() {};

bool MPCSteering::steeringControl(double& steer_angle, float ego_vehicle_velocity) {

  // ACADO timer
  acado_timer t; // Every time the MPC is called, the ACADO time starts at zero

  predicted_path_.header.frame_id = global_frameID_;
  predicted_path_.header.stamp = ros::Time::now();

  mpc_reference_path_.header.frame_id = global_frameID_;
  mpc_reference_path_.header.stamp = ros::Time::now();

  // If the route_plan is empty, return false
  if (path_.size() == 0) {
    return false;
  }

  // Compute the lookahead
  setLookahead(ego_vehicle_velocity);

  // Update the tracking point
  updateTrackingPoint();

  // Compute the step size
  int step_size = setStepSize(ego_vehicle_velocity);
  collided_pt_ = path_[tracking_point_ + ACADO_N * step_size].pose.position;

  // Print the waypoints
	if (verbose_ && debug_) printWaypoints(); // Mainly for debugging purposes

  /* Initialize the solver. */
  acado_initializeSolver();

  // Initialize states, controls, reference and the current state feedback
  initAcadoVariables();

  // Compute the reference for the current MPC
  setRef(ego_vehicle_velocity, step_size);

  // Run the real-time algorithm
  runSQP(t);

  // Print the reference path
  if (verbose_ && debug_) printReferenceVariables();

  // Get the predicted path
  getPredictedPath();

  // Get the steering angle from ACADO
  steer_angle = getSteeringAngle();

  // Compute the tracking error
  getTrackingErrorMPC();

  // If the problem is unfeasible, no path exists
  if (feasible_ == false) {
    return false;
  }

  return true;
}

void MPCSteering::setInitPose() {
  if (!initialized_) {
    current_point_ = vehicle_base_.position;
    next_point_ = path_[0].pose.position;
    initialized_ = true;
  }
}

void MPCSteering::setLookahead(const float ego_vehicle_velocity) {
  lookahead_mpc_ = std::max(min_lookahead_mpc_, ego_vehicle_velocity * TIME_INTERVAL_);
}

int MPCSteering::setStepSize(const float ego_vehicle_velocity) {
  double avg_waypoint_spacing = getAvgWaypointSpacing();
  return round(lookahead_mpc_ / (ACADO_N * avg_waypoint_spacing));
}

void MPCSteering::initAcadoVariables() {
  // Initialize the states and controls
  for (int i = 0; i < ACADO_NX * (ACADO_N + 1); ++i) {
    acadoVariables.x[i] = 0.0;
  }
  for (int i = 0; i < ACADO_NU * ACADO_N; ++i) {
    acadoVariables.u[i] = 0.0;
  }

  // Initialize the measurements/reference
  for (int i = 0; i < ACADO_NY * ACADO_N; ++i) {
    acadoVariables.y[i] = 0.0;
  }
  for (int i = 0; i < ACADO_NYN; ++i) {
    acadoVariables.yN[i] = 0.0;
  }

// MPC: initialize the current state feedback
#if ACADO_INITIAL_STATE_FIXED
  for (int i = 0; i < ACADO_NX; ++i) {
    acadoVariables.x0[0] = vehicle_base_.position.x;
    acadoVariables.x0[1] = vehicle_base_.position.y;
    acadoVariables.x0[2] = tf::getYaw(vehicle_base_.orientation);
  }
#endif

// Initialize the online data
#if ACADO_NOD > 0
  for (int i = 0; i < ACADO_NOD * (ACADO_N + 1); ++i) {
    acadoVariables.od[i] = 0.0;
  }
#endif
}

double MPCSteering::getAvgWaypointSpacing() {
  if (path_.size() < 2) {
    return 0.0;
  }

  double total_spacing = 0.0;

  auto it = path_.begin();
  auto next_it = std::next(it);

  while (next_it != path_.end() && total_spacing <= lookahead_mpc_) { // Only use the avg. spacing over the lookahead distance
    total_spacing += sqrt(pow(next_it->pose.position.x - it->pose.position.x, 2) + pow(next_it->pose.position.y - it->pose.position.y, 2));
    ++it;
    ++next_it;
  }

  return total_spacing / std::distance(path_.begin(), it);
}

void MPCSteering::setRef(double ego_vel, int step_size) {
  // Number of given waypoints
  int num_waypoints = path_.size();

  // Compute the reference
  int curr_idx = tracking_point_;
  for (int i = 0; i < ACADO_NY * ACADO_N; i = i + ACADO_NY) {
    if (curr_idx >= (num_waypoints - 1)) {
      acadoVariables.y[i] = 0.0;
      acadoVariables.y[i + 1] = 0.0;
      acadoVariables.y[i + 2] = 0.0;
      if (CONTROL_VEL_) {
        acadoVariables.y[i + 3] = ref_vel_;
        acadoVariables.y[i + 4] = 0.0;
        acadoVariables.y[i + 5] = 0.0;
      } else {
        acadoVariables.y[i + 3] = 0.0;
        acadoVariables.y[i + 4] = 0.0;
      }

    } else {
      if (curr_idx == 0) {
        acadoVariables.y[i] = 0.0;
        acadoVariables.y[i + 1] = 0.0;
      } else {
        acadoVariables.y[i] = 0.0;
        acadoVariables.y[i + 1] = 0.0;
      }
      acadoVariables.y[i + 2] = 0.0;
      if (CONTROL_VEL_) {
        acadoVariables.y[i + 3] = ref_vel_;
        acadoVariables.y[i + 4] = 0.0;
        acadoVariables.y[i + 5] = 0.0;
      } else {
        acadoVariables.y[i + 3] = 0.0;
        acadoVariables.y[i + 4] = 0.0;
      }
    }
    curr_idx += step_size;
  }

  // Compute the terminal reference
  if (curr_idx >= (num_waypoints - 1)) {
    acadoVariables.yN[0] = 0.0;
    acadoVariables.yN[1] = 0.0;
    acadoVariables.yN[2] = 0.0;
  } else {
    acadoVariables.yN[0] = 0.0;
    acadoVariables.yN[1] = 0.0;
    acadoVariables.yN[2] = 0.0;
  }

  // Compute the second reference used in the cost (online data)
  int curr_idx_2 = tracking_point_;
  nav_msgs::Path empty_msg;
  mpc_reference_path_.poses = empty_msg.poses;
  for (int i = 0; i < ACADO_NOD * (ACADO_N + 1); i = i + ACADO_NOD) {
    if (curr_idx_2 >= (num_waypoints - 1)) {
      acadoVariables.od[i] = path_[num_waypoints - 1].pose.position.x;
      acadoVariables.od[i + 1] = path_[num_waypoints - 1].pose.position.y;
      acadoVariables.od[i + 2] = atan2(path_[num_waypoints - 1].pose.position.y - path_[num_waypoints - step_size].pose.position.y,
                                       path_[num_waypoints - 1].pose.position.x - path_[num_waypoints - step_size].pose.position.x);
    } else {
      acadoVariables.od[i] = path_[curr_idx_2].pose.position.x;
      acadoVariables.od[i + 1] = path_[curr_idx_2].pose.position.y;
      // if (curr_idx_2 == tracking_point_) { // Uncomment to use the orthogonal projection instead of the tracking point
      //   geometry_msgs::Pose orth_projection = computeOrthProj(path_[curr_idx_2].pose, path_[curr_idx_2 + 1].pose, vehicle_base_);
      //   acadoVariables.od[i] = orth_projection.position.x;
      //   acadoVariables.od[i + 1] = orth_projection.position.y;
      // } else {
      //   acadoVariables.od[i] = path_[curr_idx_2].pose.position.x;
      //   acadoVariables.od[i + 1] = path_[curr_idx_2].pose.position.y;
      // }
      acadoVariables.od[i + 2] = atan2(path_[curr_idx_2 + step_size].pose.position.y - path_[curr_idx_2].pose.position.y,
                                       path_[curr_idx_2 + step_size].pose.position.x - path_[curr_idx_2].pose.position.x);
    }
    if (!CONTROL_VEL_) {
      acadoVariables.od[i + 3] = ego_vel;
    }
    geometry_msgs::PoseStamped tmp_pose = convertToPose(acadoVariables.od[i], acadoVariables.od[i + 1], acadoVariables.od[i + 2]);
    mpc_reference_path_.poses.push_back(tmp_pose);
    curr_idx_2 += step_size;
  }
}

void MPCSteering::runSQP(acado_timer& t) {
  if (verbose_) acado_printHeader();

  /* Prepare first step */
  acado_preparationStep();

  /* Get the time before start of the loop. */
  acado_tic(&t);

  /* The "real-time iterations" loop. */
  for (int iter = 0; iter < NUM_SQP_STEPS_; ++iter) {
    /* Perform the feedback step. */
    acado_feedbackStep();

    /* Apply the new control immediately to the process, first NU components. */
    if (verbose_)
      std::cout << "Real-Time Iteration " << iter << ": "
                << "KKT Tolerance = " << acado_getKKT() << std::endl
                << std::endl;
    if (verbose_)
      std::cout << "Real-Time Iteration " << iter << ": "
                << "Objective Value = " << acado_getObjective() << std::endl
                << std::endl;

    /* Prepare for the next step. */
    acado_preparationStep();

    // Check if the optimization problem is feasible
    feasible_ = true;
    if (iter == (NUM_SQP_STEPS_ - 1) && acado_getKKT() >= 1e-5) {
      feasible_ = false;
    }
  }
  /* Read the elapsed time. */
  real_t te = acado_toc(&t);

  if (verbose_) std::cout << std::endl << std::endl << "End of the RTI loop." << std::endl << std::endl << std::endl;

  /* Eye-candy. */

  if (!verbose_)
    std::cout << std::endl
              << std::endl
              << "Average time of one real-time iteration: " << 1e6 * te / NUM_SQP_STEPS_ << " microseconds" << std::endl
              << std::endl;

  if (verbose_ && debug_) acado_printDifferentialVariables();
  if (verbose_ && debug_) acado_printControlVariables();
}

double MPCSteering::getSteeringAngle() {
  if (CONTROL_VEL_) {
    return acadoVariables.u[1];
  } else {
    return acadoVariables.u[0];
  }
}

void MPCSteering::updateStateFeedback() {
  vehicle_base_.position.x = acadoVariables.x[ACADO_NX];
  vehicle_base_.position.y = acadoVariables.x[ACADO_NX + 1];
  vehicle_base_.orientation = tf::createQuaternionMsgFromYaw(acadoVariables.x[ACADO_NX + 2]);
}

geometry_msgs::Pose MPCSteering::computeOrthProj(geometry_msgs::Pose A, geometry_msgs::Pose B, geometry_msgs::Pose P) {
  // Define the relevant vectors
  geometry_msgs::Pose AB;
  AB.position.x = B.position.x - A.position.x;
  AB.position.y = B.position.y - A.position.y;
  geometry_msgs::Pose AP;
  AP.position.x = P.position.x - A.position.x;
  AP.position.y = P.position.y - A.position.y;

  // Compute the projections onto the line
  double dot_product = AP.position.x * AB.position.x + AB.position.y * AP.position.y;
  double projection_x = (dot_product / (pow(AB.position.x, 2) + pow(AB.position.y, 2))) * AB.position.x;
  double projection_y = (dot_product / (pow(AB.position.x, 2) + pow(AB.position.y, 2))) * AB.position.y;

  // Compute the projected point
  geometry_msgs::Pose projected_point;
  projected_point.position.x = A.position.x + projection_x;
  projected_point.position.y = A.position.y + projection_y;
  projected_point.orientation = tf::createQuaternionMsgFromYaw(atan2(B.position.y - A.position.y, B.position.x - A.position.x));

  return projected_point;
}

void MPCSteering::updateTrackingPoint() {
  // find the tracking_point_, which is the nearest path point to the vehicle.
  double dist_k1 = DBL_MAX;
  double accum_dist = 0.0;

  int original_path_length = path_.size() - padded_pts_no_;

  for (int i = (original_path_length - 1); i >= 0; i--) {
    const geometry_msgs::Point current_point = path_[i].pose.position;
    const double dist = fmutil::distance(current_point.x, current_point.y, vehicle_base_.position.x, vehicle_base_.position.y);

    if (dist < dist_k1) {
      tracking_point_ = i;
      dist_k1 = dist;
    }

  }

  if (tracking_point_ >= original_path_length) {
    tracking_point_ = original_path_length - 1;
  }
}

void MPCSteering::getPredictedPath() {
  nav_msgs::Path empty_msg;
  predicted_path_.poses = empty_msg.poses;
  for (int i = 0; i < ACADO_N * ACADO_NX + 1; i = i + ACADO_NX) {
    geometry_msgs::PoseStamped tmp_pose = convertToPose(acadoVariables.x[i], acadoVariables.x[i + 1], acadoVariables.x[i + 2]);
    predicted_path_.poses.push_back(tmp_pose);
	}
}

void MPCSteering::printReferenceVariables() { // This prints out the velocity instead of the zero reference for delta in the case that the velocity is not controlled
  std::cout << std::endl << "Reference variables:" << std::endl << "[" << std::endl;
  for (int i = 0; i < ACADO_N; ++i) {
    for (int j = 0; j < ACADO_NY; ++j) {
      if (j < ACADO_NOD) {
        std::cout << "\t" << acadoVariables.od[i * ACADO_NOD + j];
      } else if (j >= ACADO_NOD && j < ACADO_NY - 1) {
        std::cout << "\t" << acadoVariables.y[i * ACADO_NY + j];
      } else {
        std::cout << "\t" << acadoVariables.y[i * ACADO_NY + j] << std::endl;
      }
    }
  }
  for (int i = ACADO_NOD * ACADO_N; i < ACADO_NOD * (ACADO_N + 1); ++i) {
    if (i < ACADO_NOD * (ACADO_N + 1) - 1) {
      std::cout << "\t" << acadoVariables.od[i];
    } else {
      std::cout << "\t" << acadoVariables.od[i] << std::endl;
    }
  }
  std::cout << "]" << std::endl << std::endl;
}

void MPCSteering::printWaypoints() {
  std::cout << std::endl << "Waypoints:" << std::endl << "[" << std::endl;
  for (int i = 0; i < path_.size(); ++i) {
    std::cout << "\t" << path_[i].pose.position.x;
    std::cout << "\t" << path_[i].pose.position.y << std::endl;
  }
  std::cout << "[" << std::endl << std::endl;
}

void MPCSteering::getTrackingErrorMPC() {
  use_lane_detection_ = -1; // We only consider the case where we track the SVG path at the moment

  std::vector<double> svg_lane(3);

   if (tracking_point_ >= 0 || tracking_point_ < (static_cast<int>(path_.size()) - 1)) {
    getLineBtwPoints(path_[tracking_point_].pose.position, path_[tracking_point_ + 1].pose.position, svg_lane.data(),
                     svg_lane.data() + 1, svg_lane.data() + 2);
  } else {
    svg_lane = mpc_tracking_err_calculator_.GetTrackingLane(0);
  }

  mpc_tracking_err_calculator_.ClearAllLanes();

  geometry_msgs::Point closest_pt = path_[tracking_point_].pose.position;

  mpc_tracking_err_calculator_.SetRefLanes(svg_lane);

  mpc_lateral_offset_ = mpc_tracking_err_calculator_.computeLateralOffset(svg_lane, vehicle_base_.position, tf::getYaw(vehicle_base_.orientation));
  mpc_heading_offset_ = mpc_tracking_err_calculator_.computeHeadingOffset(svg_lane, vehicle_base_.position, tf::getYaw(vehicle_base_.orientation));
}

geometry_msgs::PoseStamped MPCSteering::convertToPose(const double x, const double y, const double heading) {
  geometry_msgs::PoseStamped tmp_pose;
  tmp_pose.header.frame_id = global_frameID_;
  tmp_pose.header.stamp = ros::Time::now();
  tmp_pose.pose.position.x = x;
  tmp_pose.pose.position.y = y;
  tmp_pose.pose.orientation = tf::createQuaternionMsgFromYaw(heading);
  return tmp_pose;
}

// All of the functions below are slightly adapated from the PurePursuit class

bool MPCSteering::debug() const { 
  return debug_;
}

std::size_t MPCSteering::getPathHash() const {
  return path_hash_;
}

nav_msgs::Path MPCSteering::getPath() {
  nav_msgs::Path path;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = global_frameID_;
  path.poses = path_;

  return path;
}

std::vector<geometry_msgs::PoseStamped>* MPCSteering::getPathPtr() {
  return &path_;
} 

void MPCSteering::publishPathFromGeometryPoseStamped(std::vector<geometry_msgs::PoseStamped>& msg, ros::Publisher& pub) {
  nav_msgs::Path path;
  path.header.frame_id = global_frameID_;
  path.header.stamp = ros::Time::now();
  path.poses = msg;
  pub.publish(path);
}

void MPCSteering::setPoses(const std::vector<geometry_msgs::PoseStamped>& poses) {
  std::size_t hash_v = 0;

  for (int i = static_cast<int>(poses.size()) - 1; i >= 0; i--) {
    const int x = static_cast<int>(poses[i].pose.position.x * 100.0);  // up to cm accuracy
    const int y = static_cast<int>(poses[i].pose.position.y * 100.0);
    const int z = static_cast<int>(poses[i].pose.position.z);  // unique segment id
    const std::hash<int> hasher;
    hash_v ^= hasher(x) + 0x9e3779b9 + (hash_v << 6) + (hash_v >> 2);  // copy from boost::hash_combine;
    hash_v ^= hasher(y) + 0x9e3779b9 + (hash_v << 6) + (hash_v >> 2);
    hash_v ^= hasher(z) + 0x9e3779b9 + (hash_v << 6) + (hash_v >> 2);
  }
  setPoses(poses, hash_v);
}

void MPCSteering::setPoses(const std::vector<geometry_msgs::PoseStamped>& poses, std::size_t hash_v) {
  if (poses.size() < 2) {
    return;
  }

  if (hash_v == path_hash_) {
    if (debug()) {
      std::cout << "Same pose received, returning..." << std::endl;
    }

    return;
  }
  received_poses_ = poses;
  path_hash_ = hash_v;

  setLocalPoses(received_poses_);
}

void MPCSteering::setLocalPoses(const std::vector<geometry_msgs::PoseStamped>& poses) {
  if (debug()) {
    std::cout << "Local poses size: " << poses.size() << std::endl;
  }

  if (poses.size() >= 2) {
    tracking_point_ = -1;
    std::vector<geometry_msgs::PoseStamped> smoothed_path_densified;
    smoothed_path_densified = poses;

    // No need to densify the path end for the MPC controller

    if (debug()) {
      std::cout << "smoothed_path_densified size: " << smoothed_path_densified.size() << std::endl;
    }

    // calulate dist to goal in advance
    accu_path_dist_.clear();
    accu_path_dist_.reserve(smoothed_path_densified.size());
    geometry_msgs::Point pt1 = smoothed_path_densified[smoothed_path_densified.size() - 1].pose.position;
    geometry_msgs::Point pt2 = smoothed_path_densified[smoothed_path_densified.size() - 1].pose.position;
    accu_path_dist_[smoothed_path_densified.size() - 1] = 0.0;
    dist_to_goal_ = 0.0;

    for (int idx = smoothed_path_densified.size() - 2; idx >= 0; idx--) {
      pt2 = smoothed_path_densified[idx].pose.position;
      dist_to_goal_ += fmutil::distance(pt1, pt2);
      accu_path_dist_[idx] = dist_to_goal_;
      pt1 = pt2;
    }

    path_ = smoothed_path_densified;

    // No need to publish, as this is already done by PPC
  }
}

void MPCSteering::setPath(nav_msgs::PathConstPtr& path) { 
  setPoses(path->poses); 
}

void MPCSteering::setPath(nav_msgs::PathConstPtr& path, size_t hash_v) {
  setPoses(path->poses, hash_v); 
}

std::vector<geometry_msgs::PoseStamped> MPCSteering::getPoses() {
  std::vector<geometry_msgs::PoseStamped> path;
  path_mutex_.lock();
  path = path_;
  path_mutex_.unlock();

  return path;
}

void MPCSteering::setMiddleOfLane(const std::vector<double>& line) { 
  middle_of_lane_ = line;
}

geometry_msgs::Point MPCSteering::getCollidedPt() { 
  return collided_pt_; 
}

template <typename T>
void MPCSteering::getLineBtwPoints(const T p1, const T p2, double* a, double* b, double* c) {
  const double dx = p2.x - p1.x;
  const double dy = p2.y - p1.y;

  if (fabs(dx) < fabs(dy)) {
    (*a) = 1.0;
    (*b) = -dx / dy;
    (*c) = -(*a) * p1.x - (*b) * p1.y;
  } else {
    (*a) = -dy / dx;
    (*b) = 1.0;
    (*c) = -(*a) * p1.x - (*b) * p1.y;
  }
}

}  // End of namespace av
