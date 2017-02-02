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

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>


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

    //QStringListModel* loggingModel() { return &logging_model; }
    //void log( const LogLevel &level, const std::string &msg);
    bool set_command (float position_x, float position_y, float force_z);
    bool get_controllers_list(std::vector<std::string>& running_list, std::vector<std::string>& stopped_list);
    bool switch_controllers(const std::string start_controller, const std::string stop_controller);

  Q_SIGNALS:
    //void loggingUpdated();
    void rosShutdown();

  private:
    int init_argc;
    char** init_argv;
  };

}  // namespace controller_switcher

#endif /* controller_switcher_QNODE_HPP_ */
