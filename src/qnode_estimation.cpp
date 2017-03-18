/**
 * @file /src/qnode_estimation.cpp
 *
 **/

/*****************************************************************************
 ** Includes
 *****************************************************************************/
#include "../include/controller_switcher/qnode_estimation.hpp"
#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include <std_srvs/Empty.h>

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace controller_switcher {

  /*****************************************************************************
   ** Implementation
   *****************************************************************************/

  QNodeEstimation::QNodeEstimation(int argc, char** argv ) :
    init_argc(argc),
    init_argv(argv)
  {}

  QNodeEstimation::~QNodeEstimation() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
    wait();
  }

  bool QNodeEstimation::init() {
    ros::init(init_argc,init_argv,"controller_switcher");
    if ( ! ros::master::check() ) {
      return false;
    }

    start_autonomus_estimation_ = false;

    ros::start();
    start();

    return true;
  }

  void QNodeEstimation::request_ftsensor_autonomus_est()
  {
    start_autonomus_estimation_ = true;
  }

  bool QNodeEstimation::ftsensor_autonomus_est()
  {
    ros::NodeHandle n;
    ros::ServiceClient client;
    std_srvs::Empty service;

    client = n.serviceClient<std_srvs::Empty>("/lwr/ft_sensor_calib_controller/start_autonomus_estimation");

    return client.call(service);
  }

  void QNodeEstimation::run()
  {
    ros::Rate loop_rate(1);

    start_autonomus_estimation_ = false;

    while ( ros::ok() ) 
      {	
	if(start_autonomus_estimation_)
	  {
	    ftsensor_autonomus_est();
	    start_autonomus_estimation_ = false;
	  }
	ros::spinOnce();
	loop_rate.sleep();
      }
  }

}  // namespace controller_switcher
