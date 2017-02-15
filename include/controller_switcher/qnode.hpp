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

#include <lwr_force_position_controllers/CartesianPositionCommand.h>
#include <lwr_force_position_controllers/CartesianPositionCommandMsg.h>
#include <lwr_force_position_controllers/HybridImpedanceCommand.h>
#include <lwr_force_position_controllers/HybridImpedanceCommandMsg.h>

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
    bool set_ftsensor();
    bool get_controllers_list(std::vector<std::string>& running_list, std::vector<std::string>& stopped_list);
    bool switch_controllers(const std::string start_controller, const std::string stop_controller);
    void set_robot_namespace(std::string name);

  Q_SIGNALS:
    void rosShutdown();

  private:
    int init_argc;
    char** init_argv;
    std::string robot_namespace_;
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
    client = n.serviceClient<ServiceType>(service_name);

    service.request.command = command;

    if (client.call(service))
      return true;
    else
      return false;
  }


}  // namespace controller_switcher

#endif /* controller_switcher_QNODE_HPP_ */
