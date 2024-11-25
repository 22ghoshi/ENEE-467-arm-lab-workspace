#include "lab8/ur3e_move_interface.hpp"
#include <cmath>

void UR3eMoveInterface::drawCircleXY(double radius_meters)
{
  if (radius_meters <= 0)
    radius_meters = 0.1;

  /// TODO: Move the arm into a pose to draw the shape by setting a joint-space target

  RCLCPP_INFO(this->get_logger(), "Example 2");


  RCLCPP_INFO(this->get_logger(), "Moving out of up (it is a singularity)");

  std::vector<double> target_joint_positions {M_PI_2, -M_PI_4, M_PI_2, -3*M_PI_4, M_PI_2, 0};
  move_group_interface_->setJointValueTarget(target_joint_positions);

  moveit::planning_interface::MoveGroupInterface::Plan motion_plan_joints;
  bool joint_space_plan_success {planToJointSpaceGoal(target_joint_positions, motion_plan_joints)};

  if (joint_space_plan_success)
    move_group_interface_->execute(motion_plan_joints);

  std::vector<geometry_msgs::msg::Pose> waypoints;
  auto current_pose = move_group_interface_->getCurrentPose().pose;

  RCLCPP_INFO(this->get_logger(), "current pose(%f, %f, %f)", current_pose.position.x, current_pose.position.y, current_pose.position.z);
  RCLCPP_INFO(this->get_logger(), "current orientation (%f, %f, %f, %f)", current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);

  // rotate arm to be parallel to xy plane
  // auto pose1 = current_pose;
  // tf2::Quaternion current_orientation_tf;
  // tf2::convert(pose1.orientation, current_orientation_tf);

  // tf2::Quaternion desired_rotation_tf;
  // desired_rotation_tf.setRPY(0, M_PI_2, 0);

  // auto target_orientation_tf = desired_rotation_tf * current_orientation_tf;
  // tf2::convert(target_orientation_tf, pose1.orientation);

  // pose1.position.z -= 0.3;

  // RCLCPP_INFO(this->get_logger(), "desired pose (%f, %f, %f)", pose1.position.x, pose1.position.y, pose1.position.z);
  // RCLCPP_INFO(this->get_logger(), "desired orientation (%f, %f, %f, %f)", pose1.orientation.x, pose1.orientation.y, pose1.orientation.z, pose1.orientation.w);

  // move_group_interface_->setPoseTarget(pose1);
  // waypoints.push_back(pose1);
  // move_group_interface_->move();

  // move forward by r to start circle
  // auto pose2 = current_pose;
  // pose2.position.x -= 0.0;
  // move_group_interface_->setPoseTarget(pose2);
  // move_group_interface_->move();
  // waypoints.push_back(pose2);
  // RCLCPP_INFO(this->get_logger(), "desired pose (%f, %f, %f)", pose2.position.x, pose2.position.y, pose2.position.z);

  /// TODO: Set the points on the circle as waypoints and execute a Cartesian plan

  double points_to_generate = 36;

  for (double i = 0; i <= points_to_generate; i++) {
    auto pose = current_pose;
    pose.position.x = radius_meters*sin(2.0*M_PI*(i/points_to_generate));
    pose.position.y = radius_meters*cos(2.0*M_PI*(i/points_to_generate));
    RCLCPP_INFO(this->get_logger(), "(%f, %f)", pose.position.x, pose.position.y);
    waypoints.push_back(pose);
  }

  // auto mid_pose = start_pose;
  // mid_pose.position.x = -radius_meters;
  // move_group_interface_->setPoseTarget(mid_pose);
  // move_group_interface_->move();
  // move_group_interface_->setPoseTarget(start_pose);
  // move_group_interface_->move();
  // waypoints.push_back(mid_pose);
  // waypoints.push_back(start_pose);





  moveit_msgs::msg::RobotTrajectory cartesian_trajectory;
  bool cartesian_plan_success {planCartesianPath(waypoints, cartesian_trajectory)};

  auto track_request {std::make_shared<lab8::srv::TrackRequest::Request>()};

  track_request->tf_root_frame_name = move_group_interface_->getPlanningFrame();
  track_request->tf_tip_frame_name = move_group_interface_->getEndEffectorLink();

  track_request->plot_title = "A Circle in the horizontal plane";
  track_request->plot_axis_x = lab8::srv::TrackRequest::Request::X_AXIS;
  track_request->plot_axis_y = lab8::srv::TrackRequest::Request::Y_AXIS;

  track_request->status = lab8::srv::TrackRequest::Request::START;

  bool track_request_success {sendEEFTrackRequest(track_request)};

  if (cartesian_plan_success && track_request_success)
    move_group_interface_->execute(cartesian_trajectory);

  track_request->status = lab8::srv::TrackRequest::Request::STOP;
  sendEEFTrackRequest(track_request);

  /// TODO: Move the robot back to its home position by setting a named target
  move_group_interface_->setNamedTarget("up");
  move_group_interface_->move();
}

void UR3eMoveInterface::drawCircleYZ(double radius_meters)
{
  // if (radius_meters <= 0)
  //   radius_meters = 0.45 * 0.5;

  // /// TODO: Move the arm into a pose to draw the shape by setting a joint-space target
  // RCLCPP_INFO(this->get_logger(), "Moving out of up (it is a singularity)");

  // // std::vector<double> target_joint_positions {M_PI, -3*M_PI_4, 3*M_PI_4, -M_PI, 0, 0};
  // std::vector<double> target_joint_positions {51.47, -97.33, 118.59, -200.66, -51.05, 359.87};
  // std::transform(target_joint_positions.begin(), target_joint_positions.end(), target_joint_positions.begin(),
  //                 [this](double joint) {
  //                   RCLCPP_INFO(this->get_logger(), "joint %f", joint * (M_PI/180.0));
  //                   return joint * (M_PI/180.0);
  //                 });

  // move_group_interface_->setJointValueTarget(target_joint_positions);

  // moveit::planning_interface::MoveGroupInterface::Plan motion_plan_joints;
  // bool joint_space_plan_success {planToJointSpaceGoal(target_joint_positions, motion_plan_joints)};

  // if (joint_space_plan_success)
  //   move_group_interface_->execute(motion_plan_joints);
  // auto current_pose = move_group_interface_->getCurrentPose().pose;
  // auto pose1 = current_pose;
  // auto base_pose = u
  // pose1.position.y = 0;
  // pose1.position.z = 0;
  // move_group_interface_->setPoseTarget(pose1);
  // move_group_interface_->move();

  // /// TODO: Set the points on the circle as waypoints and execute a Cartesian plan
  // std::vector<geometry_msgs::msg::Pose> waypoints;
  // current_pose = move_group_interface_->getCurrentPose().pose;
  // RCLCPP_INFO(this->get_logger(), "current pose(%f, %f, %f)", current_pose.position.x, current_pose.position.y, current_pose.position.z);
  // double points_to_generate = 36;

  // for (double i = 0; i <= points_to_generate; i++) {
  //   auto pose = current_pose;
  //   pose.position.y += radius_meters*sin(2.0*M_PI*(i/points_to_generate));
  //   pose.position.z += radius_meters*cos(2.0*M_PI*(i/points_to_generate));
  //   RCLCPP_INFO(this->get_logger(), "(%f, %f)", pose.position.y, pose.position.z);
  //   waypoints.push_back(pose);
  // }
  // moveit_msgs::msg::RobotTrajectory cartesian_trajectory;
  // bool cartesian_plan_success {planCartesianPath(waypoints, cartesian_trajectory)};

  // auto track_request {std::make_shared<lab8::srv::TrackRequest::Request>()};


  // track_request->tf_root_frame_name = move_group_interface_->getPlanningFrame();
  // track_request->tf_tip_frame_name = move_group_interface_->getEndEffectorLink();

  // track_request->plot_title = "A Circle in the vertical plane";
  // track_request->plot_axis_x = lab8::srv::TrackRequest::Request::Y_AXIS;
  // track_request->plot_axis_y = lab8::srv::TrackRequest::Request::Z_AXIS;

  // track_request->status = lab8::srv::TrackRequest::Request::START;

  // bool track_request_success {sendEEFTrackRequest(track_request)};

  // if (cartesian_plan_success && track_request_success)
  //   move_group_interface_->execute(cartesian_trajectory);

  // track_request->status = lab8::srv::TrackRequest::Request::STOP;
  // sendEEFTrackRequest(track_request);


  // /// TODO: Move the robot back to its home position by setting a named target
  // move_group_interface_->setNamedTarget("up");
  // move_group_interface_->move();


  //ggg

  if (radius_meters <= 0)
    radius_meters = 0.45 * 0.5;

  /// TODO: Move the arm into a pose to draw the shape by setting a joint-space target
  {
    std::vector<double> target_joint_positions {M_PI_2, -M_PI_2, M_PI_2, -M_PI, -M_PI_2, 0};
    move_group_interface_->setJointValueTarget(target_joint_positions);

    moveit::planning_interface::MoveGroupInterface::Plan motion_plan;
    bool plan_success {planToJointSpaceGoal(target_joint_positions, motion_plan)};

    if (plan_success)
      move_group_interface_->execute(motion_plan);
  }

  {
    auto current_pose {move_group_interface_->getCurrentPose().pose};
    auto robot_base_pose {getRobotBasePose()};

    auto target_adjust_pose {current_pose};
    target_adjust_pose.position.y = robot_base_pose.position.y;

    double lower_z_bound {getRobotBasePose().position.z + 0.15};
    double upper_z_bound {lower_z_bound + 2 * radius_meters};
    target_adjust_pose.position.z = (lower_z_bound + upper_z_bound) / 2;

    std::vector<geometry_msgs::msg::Pose> waypoints {current_pose, target_adjust_pose};

    moveit_msgs::msg::RobotTrajectory trajectory;
    bool plan_success {planCartesianPath(waypoints, trajectory)};

    if (plan_success)
      move_group_interface_->execute(trajectory);
  }

  /// TODO: Set the points on the circle as waypoints and execute a Cartesian plan
  {
    std::vector<geometry_msgs::msg::Pose> waypoints;
    auto initial_pose {move_group_interface_->getCurrentPose().pose};
    waypoints.push_back(initial_pose);

    for (double theta {0}; theta < 2 * M_PI; theta += M_PI / 180) {
      geometry_msgs::msg::Pose point_on_shape {initial_pose};

      point_on_shape.position.y += radius_meters * std::cos(theta);
      point_on_shape.position.z += radius_meters * std::sin(theta);

      waypoints.push_back(point_on_shape);
    }

    moveit_msgs::msg::RobotTrajectory trajectory;
    bool plan_success {planCartesianPath(waypoints, trajectory)};
    auto track_request {std::make_shared<lab8::srv::TrackRequest::Request>()};


    track_request->tf_root_frame_name = move_group_interface_->getPlanningFrame();
    track_request->tf_tip_frame_name = move_group_interface_->getEndEffectorLink();

    track_request->plot_title = "A Circle in the vertical plane";
    track_request->plot_axis_x = lab8::srv::TrackRequest::Request::Y_AXIS;
    track_request->plot_axis_y = lab8::srv::TrackRequest::Request::Z_AXIS;

    track_request->status = lab8::srv::TrackRequest::Request::START;

    bool track_request_success {sendEEFTrackRequest(track_request)};

    if (plan_success && track_request_success)
      move_group_interface_->execute(trajectory);

    track_request->status = lab8::srv::TrackRequest::Request::STOP;
    sendEEFTrackRequest(track_request);

    if (plan_success)
      move_group_interface_->execute(trajectory);
  }

  /// TODO: Move the robot back to its home position by setting a named target
  {
    move_group_interface_->setNamedTarget("up");
    move_group_interface_->move();
  }
}

void UR3eMoveInterface::drawSquareXY(double side_meters)
{
  if (side_meters <= 0)
    side_meters = (0.45 * 2) * M_SQRT1_2;

  /// TODO: Move the arm into a pose to draw the shape by setting a joint-space target
  RCLCPP_INFO(this->get_logger(), "Example 2");

  RCLCPP_INFO(this->get_logger(), "Moving out of up (it is a singularity)");

  std::vector<double> target_joint_positions {-M_PI, -3*M_PI_4, -M_PI_2, -M_PI_4, M_PI_2, 0};
  move_group_interface_->setJointValueTarget(target_joint_positions);

  moveit::planning_interface::MoveGroupInterface::Plan motion_plan_joints;
  bool joint_space_plan_success {planToJointSpaceGoal(target_joint_positions, motion_plan_joints)};

  if (joint_space_plan_success)
    move_group_interface_->execute(motion_plan_joints);

  auto current_pose = move_group_interface_->getCurrentPose().pose;

  RCLCPP_INFO(this->get_logger(), "current pose(%f, %f, %f)", current_pose.position.x, current_pose.position.y, current_pose.position.z);
  RCLCPP_INFO(this->get_logger(), "current orientation (%f, %f, %f, %f)", current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);


  /// TODO: Set the corners of the square as waypoints and execute a Cartesian plan
  auto pose1 = current_pose;
  pose1.position.x = side_meters / 2;
  pose1.position.y = -side_meters / 2;
  auto pose2 = current_pose;
  pose2.position.x = side_meters / 2;
  pose2.position.y = side_meters / 2;
  auto pose3 = current_pose;
  pose3.position.x = -side_meters / 2;
  pose3.position.y = side_meters / 2;
  auto pose4 = current_pose;
  pose4.position.x = -side_meters / 2;
  pose4.position.y = -side_meters / 2;
  // auto pose5 = current_pose;
  // pose5.position.x -= 0.1;
  // auto pose5 = current_pose;
  // pose5.position.x = side_meters / 2;
  // pose5.position.y = -side_meters / 2;
  std::vector<geometry_msgs::msg::Pose> waypoints {pose1, pose2, pose3, pose4, current_pose};
  RCLCPP_INFO(this->get_logger(), "current pose(%f, %f, %f)", pose1.position.x, pose1.position.y, pose1.position.z);
  RCLCPP_INFO(this->get_logger(), "current pose(%f, %f, %f)", pose2.position.x, pose2.position.y, pose2.position.z);
  RCLCPP_INFO(this->get_logger(), "current pose(%f, %f, %f)", pose3.position.x, pose3.position.y, pose3.position.z);
  RCLCPP_INFO(this->get_logger(), "current pose(%f, %f, %f)", pose4.position.x, pose4.position.y, pose4.position.z);

  moveit_msgs::msg::RobotTrajectory cartesian_trajectory;
  bool cartesian_plan_success {planCartesianPath(waypoints, cartesian_trajectory)};

  auto track_request {std::make_shared<lab8::srv::TrackRequest::Request>()};

  track_request->tf_root_frame_name = move_group_interface_->getPlanningFrame();
  track_request->tf_tip_frame_name = move_group_interface_->getEndEffectorLink();

  track_request->plot_title = "A Square in the horizontal plane";
  track_request->plot_axis_x = lab8::srv::TrackRequest::Request::X_AXIS;
  track_request->plot_axis_y = lab8::srv::TrackRequest::Request::Y_AXIS;

  track_request->status = lab8::srv::TrackRequest::Request::START;

  bool track_request_success {sendEEFTrackRequest(track_request)};

  if (cartesian_plan_success && track_request_success)
    move_group_interface_->execute(cartesian_trajectory);

  track_request->status = lab8::srv::TrackRequest::Request::STOP;
  sendEEFTrackRequest(track_request);

  /// TODO: Move the robot back to its home position by setting a named target
  move_group_interface_->setNamedTarget("up");
  move_group_interface_->move();
}

void UR3eMoveInterface::drawSquareYZ(double side_meters)
{
  if (side_meters <= 0)
    side_meters = 0.425;

  /// TODO: Move the arm into a pose to draw the shape by setting a joint-space target

  /// TODO: Set the corners of the square as waypoints and execute a Cartesian plan

  /// TODO: Move the robot back to its home position by setting a named target
}
// https://enee467.readthedocs.io/en/latest/Lab8.html#writing-the-code
