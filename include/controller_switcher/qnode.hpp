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
#include <lwr_force_position_controllers/CartesianPositionCommand.h>
#include <lwr_force_position_controllers/HybridImpedanceCommand.h>
#include <lwr_force_position_controllers/FtSensorToolEstimationMsg.h>
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
    bool set_command(ServiceMessageType command);
    template <class ServiceType, class ServiceMessageType>
    bool get_current_cmd(ServiceMessageType& current_command);
    bool request_ftsensor_calibration();
    bool get_controllers_list(std::vector<std::string>& running_list, std::vector<std::string>& stopped_list);
    bool switch_controllers(const std::string start_controller, const std::string stop_controller);
    void set_robot_namespace(std::string name);
    void get_joints_state(std::vector<double>& positions);
    void get_joints_error(std::vector<double>& errors);
    void get_cartesian_error(std::vector<double>& errors);
    void set_cartpos_controller_state(bool state) {is_cartpos_controller_active_ = state;}
    void set_hybrid_controller_state(bool state) {is_hybrid_controller_active_ = state;}

  Q_SIGNALS:
    void rosShutdown();
    void jointsStateArrived();
    void jointsErrorArrived();
    void cartesianErrorArrived();

  private:
    int init_argc;
    char** init_argv;
    bool is_cartpos_controller_active_;
    bool is_hybrid_controller_active_;
    std::string robot_namespace_;
    ros::Subscriber sub_joints_state_;
    ros::Subscriber sub_joints_error_;
    ros::Subscriber sub_cartesian_error_;
    void joints_state_callback(const sensor_msgs::JointState::ConstPtr& msg);
    void joints_error_callback(const lwr_force_position_controllers::CartesianPositionJointsMsg::ConstPtr& msg);
    void cartesian_error_callback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
    sensor_msgs::JointState joints_state_;
    lwr_force_position_controllers::CartesianPositionJointsMsg joints_error_;
    geometry_msgs::WrenchStamped cartesian_error_;

    QMutex joints_state_mutex_;
    QMutex joints_error_mutex_;
    QMutex cartesian_error_mutex_;
  };

  template <class ServiceType, class ServiceMessageType>
  bool QNode::set_command(ServiceMessageType command)
  {
    ros::NodeHandle n;
    ros::ServiceClient client;
    ServiceType service;
    std::string service_name;

    // Choose the service name depending on the ServiceType type
    if(std::is_same<ServiceType, lwr_force_position_controllers::CartesianPositionCommand>::value)
      service_name = "/" + robot_namespace_ + "/cartesian_position_controller/set_cartesian_position_command";
    else if (std::is_same<ServiceType, lwr_force_position_controllers::HybridImpedanceCommand>::value)
      service_name = "/" + robot_namespace_ + "/hybrid_impedance_controller/set_hybrid_impedance_command";
    else if(std::is_same<ServiceType, lwr_force_position_controllers::CartesianInverseCommand>::value)
      service_name= "/" + robot_namespace_ + "/hybrid_impedance_controller/set_cartesian_inverse_command";
    client = n.serviceClient<ServiceType>(service_name);

    service.request.command = command;

    if (client.call(service))
      return true;
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
    if(std::is_same<ServiceType, lwr_force_position_controllers::CartesianPositionCommand>::value)
      service_name = "/" + robot_namespace_ + "/cartesian_position_controller/get_cartesian_position_command";
    else if (std::is_same<ServiceType, lwr_force_position_controllers::HybridImpedanceCommand>::value)
      service_name = "/" + robot_namespace_ + "/hybrid_impedance_controller/get_hybrid_impedance_command";
    else if(std::is_same<ServiceType, lwr_force_position_controllers::CartesianInverseCommand>::value)
      service_name= "/" + robot_namespace_ + "/hybrid_impedance_controller/get_cartesian_inverse_command";
    client = n.serviceClient<ServiceType>(service_name);
    
    outcome = client.call(service);

    if (outcome)
      current_command = service.response.command;
    
    return outcome;
  }

}  // namespace controller_switcher

#endif /* controller_switcher_QNODE_HPP_ */
