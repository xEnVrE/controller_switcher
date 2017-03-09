/**
 * @file /include/controller_switcher/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef controller_switcher_QNODE_HPP_
#define controller_switcher_QNODE_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <QMutex>

#include <lwr_force_position_controllers/CartesianInverseCommand.h>
#include <lwr_force_position_controllers/CartesianPositionCommandGains.h>
#include <lwr_force_position_controllers/CartesianPositionCommandTraj.h>
#include <lwr_force_position_controllers/HybridImpedanceCommandTrajPos.h>
#include <lwr_force_position_controllers/HybridImpedanceCommandTrajForce.h>
#include <lwr_force_position_controllers/HybridImpedanceCommandGains.h>
#include <lwr_force_position_controllers/HybridImpedanceSwitchForcePos.h>
#include <lwr_force_position_controllers/CartesianPositionJointsMsg.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace controller_switcher {

  /*****************************************************************************
   ** Class
   *****************************************************************************/

  class QNode : public QThread {
    Q_OBJECT
  public:
    QNode(int argc, char** argv );
    virtual ~QNode();
    bool init();
    void run();

    template <class ServiceType, class ServiceMessageType>
    bool set_command(ServiceMessageType command, ServiceMessageType& response);
    template <class ServiceType, class ServiceMessageType>
    bool get_current_cmd(ServiceMessageType& current_command);
    bool request_ftsensor_calibration();
    bool get_controllers_list(std::vector<std::string>& running_list, std::vector<std::string>& stopped_list);
    bool switch_controllers(const std::string start_controller, const std::string stop_controller, bool& switch_ok);
    void set_robot_namespace(std::string name);
    void get_joints_state(std::vector<double>& positions);
    void get_joints_error(std::vector<double>& errors);
    void get_cartesian_error(std::vector<double>& errors);
    void get_progress_cartpos(double& elapsed, double& duration);
    void get_progress_hybrid_pos(double& elapsed, double& duration);
    void get_progress_hybrid_force(double& elapsed, double& duration);
    void set_cartpos_controller_state(bool state) {is_cartpos_controller_active_ = state;}
    void set_hybrid_controller_state(bool state) {is_hybrid_controller_active_ = state;}
    bool get_cartpos_controller_state() {return is_cartpos_controller_active_;}
    bool get_hybrid_controller_state() {return is_hybrid_controller_active_;}

  Q_SIGNALS:
    void rosShutdown();
    void jointsStateArrived();
    void jointsErrorArrived();
    void cartesianErrorArrived();
    void progressDataArrived();

  private:
    void joints_state_callback(const sensor_msgs::JointState::ConstPtr& msg);
    void joints_error_callback(const lwr_force_position_controllers::CartesianPositionJointsMsg::ConstPtr& msg);
    void cartesian_error_callback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
    void get_trajectories_progress();

    int init_argc;
    char** init_argv;
    bool is_cartpos_controller_active_;
    bool is_hybrid_controller_active_;
    std::string robot_namespace_;
    ros::Subscriber sub_joints_state_;
    ros::Subscriber sub_joints_error_;
    ros::Subscriber sub_cartesian_error_;
    sensor_msgs::JointState joints_state_;
    lwr_force_position_controllers::CartesianPositionJointsMsg joints_error_;
    lwr_force_position_controllers::CartesianPositionCommandTrajMsg progress_cartpos_;
    lwr_force_position_controllers::HybridImpedanceCommandTrajPosMsg progress_hybrid_pos_;
    lwr_force_position_controllers::HybridImpedanceCommandTrajForceMsg progress_hybrid_force_;
    geometry_msgs::WrenchStamped cartesian_error_;

    QMutex joints_state_mutex_;
    QMutex joints_error_mutex_;
    QMutex cartesian_error_mutex_;
  };

  template <class ServiceType, class ServiceMessageType>
  bool QNode::set_command(ServiceMessageType command, ServiceMessageType& response)
  {
    ros::NodeHandle n;
    ros::ServiceClient client;
    ServiceType service;
    std::string service_name;

    // Choose the service name depending on the ServiceType type
    if(std::is_same<ServiceType, lwr_force_position_controllers::CartesianPositionCommandTraj>::value)
      service_name = "/" + robot_namespace_ + "/cartesian_position_controller/set_cartpos_traj_cmd";
    else if(std::is_same<ServiceType, lwr_force_position_controllers::CartesianPositionCommandGains>::value)
      service_name = "/" + robot_namespace_ + "/cartesian_position_controller/set_cartpos_gains_cmd";
    else if (std::is_same<ServiceType, lwr_force_position_controllers::HybridImpedanceCommandTrajPos>::value)
      service_name = "/" + robot_namespace_ + "/hybrid_impedance_controller/set_hybrid_traj_pos_cmd";
    else if (std::is_same<ServiceType, lwr_force_position_controllers::HybridImpedanceCommandTrajForce>::value)
      service_name = "/" + robot_namespace_ + "/hybrid_impedance_controller/set_hybrid_traj_force_cmd";
    else if (std::is_same<ServiceType, lwr_force_position_controllers::HybridImpedanceCommandGains>::value)
      service_name = "/" + robot_namespace_ + "/hybrid_impedance_controller/set_hybrid_gains_cmd";
    else if (std::is_same<ServiceType, lwr_force_position_controllers::HybridImpedanceSwitchForcePos>::value)
      service_name = "/" + robot_namespace_ + "/hybrid_impedance_controller/switch_force_position_z";
    client = n.serviceClient<ServiceType>(service_name);

    service.request.command = command;

    if (client.call(service))
      {
	response = service.response.command;
	return true;
      }
    else
      return false;
  }

  template <class ServiceType, class ServiceMessageType>
  bool QNode::get_current_cmd(ServiceMessageType& current_command)
  {
    ros::NodeHandle n;
    ros::ServiceClient client;
    ServiceType service;
    std::string service_name;
    double outcome;

    // Choose the service name depending on the ServiceType type
    if(std::is_same<ServiceType, lwr_force_position_controllers::CartesianPositionCommandTraj>::value)
      service_name = "/" + robot_namespace_ + "/cartesian_position_controller/get_cartpos_traj_cmd";
    else if(std::is_same<ServiceType, lwr_force_position_controllers::CartesianPositionCommandGains>::value)
      service_name = "/" + robot_namespace_ + "/cartesian_position_controller/get_cartpos_gains_cmd";
    else if (std::is_same<ServiceType, lwr_force_position_controllers::HybridImpedanceCommandTrajPos>::value)
      service_name = "/" + robot_namespace_ + "/hybrid_impedance_controller/get_hybrid_traj_pos_cmd";
    else if (std::is_same<ServiceType, lwr_force_position_controllers::HybridImpedanceCommandTrajForce>::value)
      service_name = "/" + robot_namespace_ + "/hybrid_impedance_controller/get_hybrid_traj_force_cmd";
    else if (std::is_same<ServiceType, lwr_force_position_controllers::HybridImpedanceCommandGains>::value)
      service_name = "/" + robot_namespace_ + "/hybrid_impedance_controller/get_hybrid_gains_cmd";
    client = n.serviceClient<ServiceType>(service_name);
    
    outcome = client.call(service);

    if (outcome)
      current_command = service.response.command;
    
    return outcome;
  }

}  // namespace controller_switcher

#endif /* controller_switcher_QNODE_HPP_ */
