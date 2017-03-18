/**
 * @file /include/controller_switcher/qnode_estimation.hpp
 *
 **/
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef controller_switcher_QNODE_ESTIMATION_HPP_
#define controller_switcher_QNODE_ESTIMATION_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif

#include <string>
#include <QThread>

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace controller_switcher {

  /*****************************************************************************
   ** Class
   *****************************************************************************/

  class QNodeEstimation : public QThread {
    Q_OBJECT
  public:
    QNodeEstimation(int argc, char** argv );
    virtual ~QNodeEstimation();
    bool init();
    void run();

    void request_ftsensor_autonomus_est();
					 
  private:
    bool ftsensor_autonomus_est();

    bool start_autonomus_estimation_;
    int init_argc;
    char** init_argv;
  };

}  // namespace controller_switcher

#endif /* controller_switcher_QNODE_HPP_ */
