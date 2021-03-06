/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
 ** Includes
 *****************************************************************************/
#include "../include/controller_switcher/qnode.hpp"
#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/ControllerState.h>
#include <std_srvs/Empty.h>

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace controller_switcher {

  /*****************************************************************************
   ** Implementation
   *****************************************************************************/

  QNode::QNode(int argc, char** argv ) :
    init_argc(argc),
    init_argv(argv)
  {}

  QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
    wait();
  }

  bool QNode::init() {
    ros::init(init_argc,init_argv,"controller_switcher");
    if ( ! ros::master::check() ) {
      return false;
    }
    ros::start();
    start();

    ros::NodeHandle n;
    sub_joints_state_ = n.subscribe("/" + robot_namespace_ + "/joint_states", 1000,\
    				    &controller_switcher::QNode::joints_state_callback, this);

    sub_joints_error_ = n.subscribe("/" + robot_namespace_ + "/cartesian_position_controller/error", 1000,\
				    &controller_switcher::QNode::joints_error_callback, this);

    sub_cartesian_error_ = n.subscribe("/" + robot_namespace_ + "/hybrid_impedance_controller/error", 1000,\
				       &controller_switcher::QNode::cartesian_error_callback, this);


    // sleep so that controller spawner can load all the controllers
    ros::Duration(3).sleep();

    // reset controller state
    is_cartpos_controller_active_ = false;
    is_hybrid_controller_active_ = false;

    return true;
  }

  void QNode::joints_state_callback(const sensor_msgs::JointState::ConstPtr& msg)
  {
    joints_state_mutex_.lock();
    joints_state_ = *msg;
    joints_state_mutex_.unlock();

    // Signal the UI that there is a new joints state message
    Q_EMIT jointsStateArrived();
  }

  void QNode::get_joints_state(std::vector<double>& positions)
  {
    joints_state_mutex_.lock();
    positions = joints_state_.position;
    joints_state_mutex_.unlock();
  }

  void QNode::joints_error_callback(const lwr_force_position_controllers::CartesianPositionJointsMsg::ConstPtr& msg)
  {
    joints_error_mutex_.lock();
    joints_error_ = *msg;
    joints_error_mutex_.unlock();

    // Signal the UI that there is a joint position error message
    // only if Cartesian Position Controller is active
    if (is_cartpos_controller_active_)
      Q_EMIT jointsErrorArrived();
  }

  void QNode::cartesian_error_callback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
  {
    cartesian_error_mutex_.lock();
    cartesian_error_ = *msg;
    cartesian_error_mutex_.unlock();

    // Signal the UI that there is a cartesian position error message
    // only if Cartesian Position Controller is active
    if (is_hybrid_controller_active_)
      Q_EMIT cartesianErrorArrived();
  }

  void QNode::get_joints_error(std::vector<double>& errors)
  {
    joints_error_mutex_.lock();
    errors.push_back(joints_error_.a1);
    errors.push_back(joints_error_.a2);
    errors.push_back(joints_error_.e1);
    errors.push_back(joints_error_.a3);
    errors.push_back(joints_error_.a4);
    errors.push_back(joints_error_.a5);
    errors.push_back(joints_error_.a6);
    joints_error_mutex_.unlock();
  }

  void QNode::get_cartesian_error(std::vector<double>& errors)
  {
    cartesian_error_mutex_.lock();
    errors.push_back(cartesian_error_.wrench.force.x);
    errors.push_back(cartesian_error_.wrench.force.y);
    errors.push_back(cartesian_error_.wrench.force.z);
    errors.push_back(cartesian_error_.wrench.torque.x);
    errors.push_back(cartesian_error_.wrench.torque.y);
    errors.push_back(cartesian_error_.wrench.torque.z);
    cartesian_error_mutex_.unlock();
  }

  bool QNode::get_controllers_list(std::vector<std::string>& running_list, std::vector<std::string>& stopped_list)
  {
    ros::NodeHandle n;
    ros::ServiceClient client;
    controller_manager_msgs::ListControllers service;
    std::vector<controller_manager_msgs::ControllerState> controller_list;

    ros::service::waitForService("/" + robot_namespace_ + "/controller_manager/list_controllers");

    client = n.serviceClient<controller_manager_msgs::ListControllers>("/" + robot_namespace_ + "/controller_manager/list_controllers");
    if(!client.call(service))
      return false;
    controller_list = service.response.controller;

    for (std::vector<controller_manager_msgs::ControllerState>::iterator it = controller_list.begin();
	 it != controller_list.end(); ++it)
      {
	if(it->state == "running")
	  running_list.push_back(it->name);
	else if (it->state == "stopped")
	  stopped_list.push_back(it->name);
      }

    return true;
  }

  bool QNode::switch_controllers(const std::string start_controller, const std::string stop_controller, bool& switch_ok)
  {
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<controller_manager_msgs::SwitchController>("/" + robot_namespace_ + "/controller_manager/switch_controller");
    controller_manager_msgs::SwitchController service;
    std::vector<std::string> start_controllers;
    std::vector<std::string> stop_controllers;
    start_controllers.push_back(start_controller);
    stop_controllers.push_back(stop_controller);
    service.request.start_controllers = start_controllers;
    service.request.stop_controllers = stop_controllers;
    service.request.strictness = 2;

    if(!client.call(service))
      return false;

    switch_ok = service.response.ok;

    return true;
  }

  void QNode::set_robot_namespace(std::string name)
  {
    robot_namespace_ = name;
  }

  bool QNode::request_ftsensor_calibration()
  {
    ros::NodeHandle n;
    ros::ServiceClient client;
    std_srvs::Empty service;

    client = n.serviceClient<std_srvs::Empty>("/my_sensor/ft_sensor_hw/calibrate");

    return client.call(service);
  }

  bool QNode::request_ftsensor_home()
  {
    ros::NodeHandle n;
    ros::ServiceClient client;
    std_srvs::Empty service;

    client = n.serviceClient<std_srvs::Empty>("/lwr/ft_sensor_calib_controller/move_home_pose");

    return client.call(service);
  }

  bool QNode::request_ftsensor_next_pose()
  {
    ros::NodeHandle n;
    ros::ServiceClient client;
    std_srvs::Empty service;

    client = n.serviceClient<std_srvs::Empty>("/lwr/ft_sensor_calib_controller/move_next_calib_pose");

    return client.call(service);
  }

  bool QNode::request_ftsensor_start_compensation()
  {
    ros::NodeHandle n;
    ros::ServiceClient client;
    std_srvs::Empty service;

    client = n.serviceClient<std_srvs::Empty>("/lwr/ft_sensor_calib_controller/start_compensation");

    return client.call(service);
  }

  bool QNode::request_ftsensor_estimate()
  {
    ros::NodeHandle n;
    ros::ServiceClient client;
    std_srvs::Empty service;

    client = n.serviceClient<std_srvs::Empty>("/lwr/ft_sensor_calib_controller/do_estimation_step");

    return client.call(service);
  }

  bool QNode::request_ftsensor_save()
  {
    ros::NodeHandle n;
    ros::ServiceClient client;
    std_srvs::Empty service;

    client = n.serviceClient<std_srvs::Empty>("/lwr/ft_sensor_calib_controller/save_calib_data");

    return client.call(service);
  }

  void QNode::get_trajectories_progress()
  {
    get_current_cmd<lwr_force_position_controllers::CartesianPositionCommandTraj,\
		    lwr_force_position_controllers::CartesianPositionCommandTrajMsg>(progress_cartpos_);
    get_current_cmd<lwr_force_position_controllers::HybridImpedanceCommandTrajPos,\
		    lwr_force_position_controllers::HybridImpedanceCommandTrajPosMsg>(progress_hybrid_pos_);
    get_current_cmd<lwr_force_position_controllers::HybridImpedanceCommandTrajForce,\
		    lwr_force_position_controllers::HybridImpedanceCommandTrajForceMsg>(progress_hybrid_force_);
  }

  void QNode::get_progress_cartpos(double& elapsed, double& duration)
  {
    elapsed = progress_cartpos_.elapsed_time;
    duration = progress_cartpos_.p2p_traj_duration;
  }
  void QNode::get_progress_hybrid_pos(double& elapsed, double& duration)
  {
    elapsed = progress_hybrid_pos_.elapsed_time;
    duration = progress_hybrid_pos_.p2p_traj_duration;
  }
  void QNode::get_progress_hybrid_force(double& elapsed, double& duration)
  {
    elapsed = progress_hybrid_force_.elapsed_time;
    duration = progress_hybrid_force_.force_ref_duration;
  }

  void QNode::run()
  {
    double frequency = 100;
    double step = 1.0 / frequency;

    ros::Rate loop_rate(33);
    double elapsed = 0;
    while ( ros::ok() ) 
      {
	ros::spinOnce();
	loop_rate.sleep();

	// get progress of trajectories at about 1Hz
	elapsed += step;
	if (elapsed > 1)
	  {
	    elapsed = 0;
	    get_trajectories_progress();
	    Q_EMIT progressDataArrived();
	  }
      }
    //ros::spin();
    //Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
  }

}  // namespace controller_switcher
