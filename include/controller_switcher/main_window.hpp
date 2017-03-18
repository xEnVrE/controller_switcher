/**
 * @file /include/controller_switcher/main_window.hpp
 *
 * @brief Qt based gui for controller_switcher.
 *
 * @date November 2010
 **/
#ifndef controller_switcher_MAIN_WINDOW_H
#define controller_switcher_MAIN_WINDOW_H

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <QtGui/QMainWindow>

#ifndef Q_MOC_RUN
#include "ui_main_window.h"
#include "qnode.hpp"
#include "qnode_estimation.hpp"
#endif

/*****************************************************************************
 ** Namespace
 *****************************************************************************/

namespace controller_switcher {

  /*****************************************************************************
   ** Interface [MainWindow]
   *****************************************************************************/
  /**
   * @brief Qt central, all operations relating to the view part here.
   */
  class MainWindow : public QMainWindow {
    Q_OBJECT

  public:
    MainWindow(int argc, char** argv, QWidget *parent = 0);
    ~MainWindow();

  private:
    void switch_tab(int index);
    void fill_controllers_list();
    void fill_cartpos_traj_fields();
    void fill_cartpos_gains_fields();
    void fill_hybrid_traj_pos_fields();
    void fill_hybrid_traj_force_fields();
    void fill_hybrid_gains_fields();
    void change_error_z_label(std::string label_text);
    void field_error_msg_box(std::string field_name);
    void service_error_msg_box(std::string controller_name);
    void set_send_state_label(QLabel* label, bool accepted, double elapsed, double duration);
    void set_progress_bar(QProgressBar* bar, double elapsed_time, double total_time);

  public Q_SLOTS:
    /******************************************
     ** Auto-connections (connectSlotsByName())
     *******************************************/
    void on_buttonQuit_clicked(bool check);
    void on_checkBoxEnableForce_hybrid_stateChanged(int state);
    void on_buttonCalibrate_ftsensor_clicked(bool check);
    void on_buttonSwitch_clicked(bool check);
    void on_buttonSetTraj_cartpos_clicked(bool check);
    void on_buttonSetGains_cartpos_clicked(bool check);
    void on_buttonSetTrajPos_hybrid_clicked(bool check);
    void on_buttonSetTrajForce_hybrid_clicked(bool check);
    void on_buttonSetGains_hybrid_clicked(bool check);
    void on_buttonHome_ftsensor_clicked(bool check);
    void on_buttonNextPose_ftsensor_clicked(bool check);
    void on_buttonEstimate_ftsensor_clicked(bool check);
    void on_buttonAutonomusEst_ftsensor_clicked(bool check);
    void on_buttonStart_compensation_ftsensor_clicked(bool check);
    void on_buttonSave_ftsensor_clicked(bool check);

    /******************************************
     ** manual connection
     *******************************************/
    void update_joints_state();
    void update_joints_error();
    void update_cartesian_error();
    void update_progress_data();

  private:
    Ui::MainWindowDesign ui;
    QNode qnode;
    QNodeEstimation qnode_estimation;
  };

}  // namespace controller_switcher

#endif // controller_switcher_MAIN_WINDOW_H
