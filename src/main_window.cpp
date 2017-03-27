/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/controller_switcher/main_window.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace controller_switcher {

  using namespace Qt;

  /*****************************************************************************
   ** Implementation [MainWindow]
   *****************************************************************************/

  MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , qnode(argc,argv)
    , qnode_estimation(argc,argv)
  {
    // setup the ui and
    // connect all ui's triggers to on_...() callbacks
    ui.setupUi(this); 

    // init ros nodes
    qnode.set_robot_namespace(argv[1]);
    qnode.init();
    qnode_estimation.init();

    // move the window to the center
    setGeometry(QStyle::alignedRect(Qt::LeftToRight,
				    Qt::AlignCenter,
				    size(),
				    qApp->desktop()->availableGeometry()));

    // force size of the window
    //setFixedSize(706, 681);

    // fill controller lists from robot_namespace_/controller_manager/ListControllers
    fill_controllers_list();

    // fill controllers gains fields with default gains 
    fill_cartpos_gains_fields();
    fill_hybrid_gains_fields();

    // connect joints state view update to joints state topic callback
    QObject::connect(&qnode, SIGNAL(jointsStateArrived()), this, SLOT(update_joints_state()));

    // connect joints error view update to joints error topic callback
    QObject::connect(&qnode, SIGNAL(jointsErrorArrived()), this, SLOT(update_joints_error()));

    // connect cartesian error view update to joints error topic callback
    QObject::connect(&qnode, SIGNAL(cartesianErrorArrived()), this, SLOT(update_cartesian_error()));

    // connect progress bars and send state labels update to progress data service call
    // service call is done in the run() method of qnode
    QObject::connect(&qnode, SIGNAL(progressDataArrived()), this, SLOT(update_progress_data()));

  }

  MainWindow::~MainWindow() {}

  /*****************************************************************************
   ** Implementation [Slots][manually connected]
   *****************************************************************************/
  
  void MainWindow::update_joints_state()
  {
    std::vector<QLabel*> labels_list;
    std::vector<double> joints_position;

    // get new joints state
    qnode.get_joints_state(joints_position);

    labels_list.push_back(ui.textJoint_a1);
    labels_list.push_back(ui.textJoint_a2);
    labels_list.push_back(ui.textJoint_a3);
    labels_list.push_back(ui.textJoint_a4);
    labels_list.push_back(ui.textJoint_a5);
    labels_list.push_back(ui.textJoint_a6);
    labels_list.push_back(ui.textJoint_e1); // <-- ORDERING HERE IS IMPORTANT

    for (int i=0; i<7; i++)
      labels_list.at(i)->setText(QString::number(180.0/3.14 * joints_position.at(i), 'f', 3));
  }

  void MainWindow::update_joints_error()
  {
    std::vector<QLabel*> labels_list;
    std::vector<double> joints_error;

    // get new joints error
    qnode.get_joints_error(joints_error);

    labels_list.push_back(ui.textJoint_a1_err);
    labels_list.push_back(ui.textJoint_a2_err);
    labels_list.push_back(ui.textJoint_e1_err); // <-- ORDERING HERE IS IMPORTANT
    labels_list.push_back(ui.textJoint_a3_err);
    labels_list.push_back(ui.textJoint_a4_err);
    labels_list.push_back(ui.textJoint_a5_err);
    labels_list.push_back(ui.textJoint_a6_err);

    for (int i=0; i<7; i++)
      labels_list.at(i)->setText(QString::number(180.0/3.14 * joints_error.at(i), 'f', 3));
  }

  void MainWindow::update_cartesian_error()
  {
    std::vector<QLabel*> labels_list;
    std::vector<double> cartesian_error;

    // get new cartesian error
    qnode.get_cartesian_error(cartesian_error);

    labels_list.push_back(ui.textCart_x_err);
    labels_list.push_back(ui.textCart_y_err);
    labels_list.push_back(ui.textCart_z_err);
    labels_list.push_back(ui.textCart_alpha_err);
    labels_list.push_back(ui.textCart_beta_err);
    labels_list.push_back(ui.textCart_gamma_err);

    // position error
    for (int i=0; i<6; i++)
      labels_list.at(i)->setText(QString::number(cartesian_error.at(i), 'f', 3));
  }

  void MainWindow::update_progress_data()
  { 
    double cartpos_elapsed, hybrid_pos_elapsed, hybrid_force_elapsed;
    double cartpos_duration, hybrid_pos_duration, hybrid_force_duration;

    void get_progress_cartpos(double& elapsed, double& duration);
    void get_progress_hybrid_pos(double& elapsed, double& duration);
    void get_progress_hybrid_force(double& elapsed, double& duration);

    if (qnode.get_cartpos_controller_state() == true)
      {
	qnode.get_progress_cartpos(cartpos_elapsed, cartpos_duration);

	set_progress_bar(ui.progressBar_cartpos, cartpos_elapsed, cartpos_duration);

	set_send_state_label(ui.textSendState_cartpos, false, cartpos_elapsed, cartpos_duration);	
      }
    else if(qnode.get_hybrid_controller_state() == true)
      {
	qnode.get_progress_hybrid_pos(hybrid_pos_elapsed, hybrid_pos_duration);
	qnode.get_progress_hybrid_force(hybrid_force_elapsed, hybrid_force_duration);

	set_progress_bar(ui.progressBar_hybrid_pos, hybrid_pos_elapsed, hybrid_pos_duration);
	set_progress_bar(ui.progressBar_hybrid_force, hybrid_force_elapsed, hybrid_force_duration);

	set_send_state_label(ui.textSendTrajPosState_hybrid, false, hybrid_pos_elapsed, hybrid_pos_duration);
	set_send_state_label(ui.textSendTrajForceState_hybrid, false, hybrid_force_elapsed, hybrid_force_duration);
      }

  }

  void MainWindow::set_progress_bar(QProgressBar* bar, double elapsed_time, double total_time)
  {
    bar->setValue(elapsed_time / total_time * 100.0);
  }
    
  /*****************************************************************************
   ** Implementation [Slots]
   *****************************************************************************/
  void MainWindow::on_buttonQuit_clicked(bool check)
  {
    close();    
  }

  void MainWindow::on_checkBoxEnableForce_hybrid_stateChanged(int state)
  {
    bool enable_force;
    bool outcome;
    double forcez0;

    enable_force = ui.checkBoxEnableForce_hybrid->isChecked();

    forcez0 = ui.textForceZ0_hybrid->text().toDouble(&outcome);
    if (!outcome)
      {
	field_error_msg_box("ForceZ0");
	return;
      }


    lwr_force_position_controllers::HybridImpedanceSwitchForcePosMsg cmd, response;
    cmd.enable_force_z = enable_force;
    cmd.forcez0 = forcez0;
					      
    outcome = qnode.set_command<lwr_force_position_controllers::HybridImpedanceSwitchForcePos,\
    				lwr_force_position_controllers::HybridImpedanceSwitchForcePosMsg>(cmd, response);
    if(!outcome)
      {
	service_error_msg_box("HybridImpedanceController(Switch Force/Position Z)");
	return;
      }

    // change the UI depending on the enable force condition
    if (enable_force)
      {
	change_error_z_label("Fz (N)");
	ui.textForceZ_hybrid->setEnabled(true);
	ui.buttonSetTrajForce_hybrid->setEnabled(true);
	ui.textPositionZ_hybrid->setEnabled(false);
	
      }
    else
      {
	change_error_z_label("z (m)");
	ui.textPositionZ_hybrid->setEnabled(true);
	ui.textForceZ_hybrid->setEnabled(false);
	ui.buttonSetTrajForce_hybrid->setEnabled(false);
      }

    // wait to avoid filling with old data
    sleep(1);
    fill_hybrid_traj_pos_fields();
    fill_hybrid_traj_force_fields();
  }

  void MainWindow::on_buttonCalibrate_ftsensor_clicked(bool check)
  {
    if(!qnode.request_ftsensor_calibration())
      service_error_msg_box("FtSensorCalibration");
  }

  void MainWindow::on_buttonSetTrajPos_hybrid_clicked(bool check)
  {
    double position_x, position_y, position_z;
    double alpha, beta, gamma;
    double p2p_traj_duration;
    bool outcome;

    position_x = ui.textPositionX_hybrid->text().toDouble(&outcome);
    if (!outcome)
      {
	field_error_msg_box("PositionX");
	return;
      }

    position_y = ui.textPositionY_hybrid->text().toDouble(&outcome);
    if (!outcome)
      {
	field_error_msg_box("PositionY");
	return;
      }

    position_z = ui.textPositionZ_hybrid->text().toDouble(&outcome);
    if (!outcome)
      {
	field_error_msg_box("PositionZ");
	return;
      }

    alpha = ui.textAlpha_hybrid->text().toDouble(&outcome);
    if (!outcome)
      {
	field_error_msg_box("Alpha");
	return;
      }

    beta = ui.textBeta_hybrid->text().toDouble(&outcome);
    if (!outcome)
      {
	field_error_msg_box("Beta");
	return;
      }

    gamma = ui.textGamma_hybrid->text().toDouble(&outcome);
    if (!outcome)
      {
	field_error_msg_box("Gamma");
	return;
      }

    p2p_traj_duration = ui.textTraj_duration_hybrid->text().toDouble(&outcome);
    if (!outcome || (p2p_traj_duration <= 0))
      {
	field_error_msg_box("Trajectory Duration");
	return;
      }

    lwr_force_position_controllers::HybridImpedanceCommandTrajPosMsg cmd, response;
    cmd.x = position_x;
    cmd.y = position_y;
    cmd.z = position_z;
    cmd.alpha = alpha;
    cmd.beta = beta;
    cmd.gamma = gamma;
    cmd.p2p_traj_duration = p2p_traj_duration;
					      
    outcome = qnode.set_command<lwr_force_position_controllers::HybridImpedanceCommandTrajPos,\
    				lwr_force_position_controllers::HybridImpedanceCommandTrajPosMsg>(cmd, response);
    if(!outcome)
      service_error_msg_box("HybridImpedanceController(Set Position Trajectory)");
    else
      set_send_state_label(ui.textSendTrajPosState_hybrid, response.accepted, response.elapsed_time, response.p2p_traj_duration);
  }

  void MainWindow::on_buttonSetTrajForce_hybrid_clicked(bool check)
  {
    double force_z;
    double force_ref_duration;
    bool outcome;

    force_z = ui.textForceZ_hybrid->text().toDouble(&outcome);
    if (!outcome)
      {
	field_error_msg_box("ForceZ");
	return;
      }

    force_ref_duration = ui.textForce_duration_hybrid->text().toDouble(&outcome);
    if (!outcome || (force_ref_duration <= 0))
      {
	field_error_msg_box("Force reference Duration");
	return;
      }

    lwr_force_position_controllers::HybridImpedanceCommandTrajForceMsg cmd, response;
    cmd.forcez = force_z;
    cmd.force_ref_duration = force_ref_duration;

    outcome = qnode.set_command<lwr_force_position_controllers::HybridImpedanceCommandTrajForce,\
    				lwr_force_position_controllers::HybridImpedanceCommandTrajForceMsg>(cmd, response);
    if(!outcome)
      service_error_msg_box("HybridImpedanceController(Set Force Trajectory)");
    else
      set_send_state_label(ui.textSendTrajForceState_hybrid, response.accepted, response.elapsed_time, response.force_ref_duration);
  }

  void MainWindow::on_buttonSetGains_hybrid_clicked(bool check)
  {
    double kp_pos, kp_att, kp_gamma;
    double kd_pos, kd_att, kd_gamma;
    double km_f, kd_f;
    double kp_z_im, kp_gamma_im, kd_pos_im, kd_att_im;
    bool outcome;

    /////////////// proportional //////////////////
    kp_pos = ui.textKp_pos_hybrid->text().toDouble(&outcome);
    if (!outcome || (kp_pos <= 0))
      {
	field_error_msg_box("Kp (pos)");
	return;
      }

    kp_att = ui.textKp_att_hybrid->text().toDouble(&outcome);
    if (!outcome || (kp_att <= 0))
      {
	field_error_msg_box("Kp (att)");
	return;
      }

    kp_gamma = ui.textKp_gamma_hybrid->text().toDouble(&outcome);
    if (!outcome || (kp_gamma <= 0))
      {
	field_error_msg_box("Kp (gamma)");
	return;
      }

    /////////////// derivative //////////////////
    kd_pos = ui.textKd_pos_hybrid->text().toDouble(&outcome);
    if (!outcome || (kd_pos <= 0))
      {
	field_error_msg_box("Kd (pos)");
	return;
      }

    kd_att = ui.textKd_att_hybrid->text().toDouble(&outcome);
    if (!outcome || (kd_att <= 0))
      {
	field_error_msg_box("Kd (att)");
	return;
      }

    kd_gamma = ui.textKd_gamma_hybrid->text().toDouble(&outcome);
    if (!outcome || (kd_gamma <= 0))
      {
	field_error_msg_box("Kd (gamma)");
	return;
      }

    /////////////// force //////////////////
    km_f = ui.textKmf_hybrid->text().toDouble(&outcome);
    if (!outcome || (km_f <= 0))
      {
	field_error_msg_box("Km Force");
	return;
      }

    kd_f = ui.textKdf_hybrid->text().toDouble(&outcome);
    if (!outcome || (kd_f <= 0))
      {
	field_error_msg_box("Kd Force");
	return;
      }

    /////////////// internal motion  //////////////////
    kp_z_im = ui.textKp_null_z_hybrid->text().toDouble(&outcome);
    if (!outcome || (kp_z_im <= 0))
      {
	field_error_msg_box("Kp Null (z)");
	return;
      }

    kp_gamma_im = ui.textKp_null_gamma_hybrid->text().toDouble(&outcome);
    if (!outcome || (kp_gamma_im <= 0))
      {
	field_error_msg_box("Kp Null (gamma)");
	return;
      }

    kd_pos_im = ui.textKd_null_pos_hybrid->text().toDouble(&outcome);
    if (!outcome || (kd_pos_im <= 0))
      {
	field_error_msg_box("Kd Null (pos)");
	return;
      }

    kd_att_im = ui.textKd_null_att_hybrid->text().toDouble(&outcome);
    if (!outcome || (kd_att_im <= 0))
      {
	field_error_msg_box("Kd Null (att)");
	return;
      }

    lwr_force_position_controllers::HybridImpedanceCommandGainsMsg cmd, response;
    cmd.kp_pos = kp_pos;
    cmd.kp_att = kp_att;
    cmd.kp_gamma = kp_gamma;
    cmd.kd_pos = kd_pos;
    cmd.kd_att = kd_att;
    cmd.kd_gamma = kd_gamma;
    cmd.km_f = km_f;
    cmd.kd_f = kd_f;
    cmd.kp_z_im = kp_z_im;
    cmd.kp_gamma_im = kp_gamma_im;
    cmd.kd_pos_im = kd_pos_im;
    cmd.kd_att_im = kd_att_im;

    outcome = qnode.set_command<lwr_force_position_controllers::HybridImpedanceCommandGains,\
    				lwr_force_position_controllers::HybridImpedanceCommandGainsMsg>(cmd, response);
    if(!outcome)
      service_error_msg_box("HybridImpedanceController(Set Gains)");
  }
    
  void MainWindow::on_buttonSetTraj_cartpos_clicked(bool check)
  {
    double position_x, position_y, position_z;
    double yaw, pitch, roll;
    double p2p_traj_duration;
    bool hold_last_qdes_found;
    bool outcome;

    position_x = ui.textPositionX_cartpos->text().toDouble(&outcome);
    if (!outcome)
      {
	field_error_msg_box("PositionX");
	return;
      }

    position_y = ui.textPositionY_cartpos->text().toDouble(&outcome);
    if (!outcome)
      {
	field_error_msg_box("PositionY");
	return;
      }

    position_z = ui.textPositionZ_cartpos->text().toDouble(&outcome);
    if (!outcome)
      {
	field_error_msg_box("PositionZ");
	return;
      }

    yaw = ui.textYaw_cartpos->text().toDouble(&outcome);
    if (!outcome)
      {
	field_error_msg_box("Yaw");
	return;
      }

    pitch = ui.textPitch_cartpos->text().toDouble(&outcome);
    if (!outcome)
      {
	field_error_msg_box("Pitch");
	return;
      }

    roll = ui.textRoll_cartpos->text().toDouble(&outcome);
    if (!outcome)
      {
	field_error_msg_box("Roll");
	return;
      }

    p2p_traj_duration = ui.textDuration_cartpos->text().toDouble(&outcome);
    if (!outcome || (p2p_traj_duration <= 0))
      {
	field_error_msg_box("Duration");
	return;
      }

    lwr_force_position_controllers::CartesianPositionCommandTrajMsg command, response;
    command.x = position_x;
    command.y = position_y;
    command.z = position_z;
    command.yaw = yaw;
    command.pitch = pitch;
    command.roll = roll;
    command.p2p_traj_duration = p2p_traj_duration;

    outcome = qnode.set_command<lwr_force_position_controllers::CartesianPositionCommandTraj,\
    				lwr_force_position_controllers::CartesianPositionCommandTrajMsg>(command, response);
    if(!outcome)
      service_error_msg_box("CartesianPositionController(Set Traj)");
    else
      set_send_state_label(ui.textSendState_cartpos, response.accepted, response.elapsed_time, response.p2p_traj_duration);
  }

  void MainWindow::on_buttonSetGains_cartpos_clicked(bool check)
  {
    bool outcome;
    double kp, kp_a4, kp_a5, kp_a6;
    double kd, kd_a4, kd_a5, kd_a6;

    kp = ui.textKp_cartpos->text().toDouble(&outcome);
    if (!outcome || (kp <= 0))
      {
	field_error_msg_box("kp");
	return;
      }

    kp_a4 = ui.textKp_a4_cartpos->text().toDouble(&outcome);
    if (!outcome || (kp_a4 <= 0))
      {
	field_error_msg_box("kp (a4)");
	return;
      }
    
    kp_a5 = ui.textKp_a5_cartpos->text().toDouble(&outcome);
    if (!outcome || (kp_a5 <= 0))
      {
	field_error_msg_box("kp (a5)");
	return;
      }

    kp_a6 = ui.textKp_a6_cartpos->text().toDouble(&outcome);
    if (!outcome || (kp_a6 <= 0))
      {
	field_error_msg_box("kp (a6)");
	return;
      }

    kd = ui.textKd_cartpos->text().toDouble(&outcome);
    if (!outcome || (kd <= 0))
      {
	field_error_msg_box("kd");
	return;
      }

    kd_a4 = ui.textKd_a4_cartpos->text().toDouble(&outcome);
    if (!outcome || (kd_a4 <= 0))
      {
	field_error_msg_box("kd (a4)");
	return;
      }
    
    kd_a5 = ui.textKd_a5_cartpos->text().toDouble(&outcome);
    if (!outcome || (kd_a5 <= 0))
      {
	field_error_msg_box("kd (a5)");
	return;
      }

    kd_a6 = ui.textKd_a6_cartpos->text().toDouble(&outcome);
    if (!outcome || (kd_a6 <= 0))
      {
	field_error_msg_box("kd (a6)");
	return;
      }

    
    lwr_force_position_controllers::CartesianPositionCommandGainsMsg command, response;
    command.kp = kp;
    command.kp_a4 = kp_a4;
    command.kp_a5 = kp_a5;
    command.kp_a6 = kp_a6;
    command.kd = kd;
    command.kd_a4 = kd_a4;
    command.kd_a5 = kd_a5;
    command.kd_a6 = kd_a6;

    
    outcome = qnode.set_command<lwr_force_position_controllers::CartesianPositionCommandGains,\
    				lwr_force_position_controllers::CartesianPositionCommandGainsMsg>(command, response);
    if(!outcome)
      service_error_msg_box("CartesianPositionController(Set Gains)");
  }

  void MainWindow::on_buttonSwitch_clicked(bool check)
  {
    std::string stop_controller = ui.comboRunningCtl->currentText().toStdString();
    std::string start_controller = ui.comboStoppedCtl->currentText().toStdString();

    bool switch_succeded;
    if(qnode.switch_controllers(start_controller, stop_controller, switch_succeded) && switch_succeded)
      {
	// wait before reloading controllers because after the service call
	// it is not sure that controller list provided by controller manager is updated
	sleep(1);
	// if controller switch succeded reload controller list in combo boxes
	fill_controllers_list();
	// Let the ros node knows that cartesian position controller is started
	// so that it can signal when new errors are available on each topic callback
	qnode.set_cartpos_controller_state(start_controller == "cartesian_position_controller");
	qnode.set_hybrid_controller_state(start_controller == "hybrid_impedance_controller");

	if(start_controller == "cartesian_position_controller")
	  {
	    fill_cartpos_traj_fields();
	    fill_cartpos_gains_fields();
	    switch_tab(0);
	  }
	if(start_controller == "hybrid_impedance_controller")
	  {
	    fill_hybrid_traj_pos_fields();
	    fill_hybrid_traj_force_fields();
	    fill_hybrid_gains_fields();
	    switch_tab(1);
	  }

      }
  }

  void MainWindow::on_buttonHome_ftsensor_clicked(bool check)
  {
    if(!qnode.request_ftsensor_home())
      service_error_msg_box("FtSensorHome");
  }

  void MainWindow::on_buttonNextPose_ftsensor_clicked(bool check)
  {
    if(!qnode.request_ftsensor_next_pose())
      service_error_msg_box("FtSensorNextPose");
  }

  void MainWindow::on_buttonEstimate_ftsensor_clicked(bool check)
  {
    if(!qnode.request_ftsensor_estimate())
      service_error_msg_box("FtSensorEstimate");
  }

  void MainWindow::on_buttonAutonomusEst_ftsensor_clicked(bool check)
  {
    qnode_estimation.request_ftsensor_autonomus_est();
  }

  void MainWindow::on_buttonStart_compensation_ftsensor_clicked(bool check)
  {
    if(!qnode.request_ftsensor_start_compensation())
      service_error_msg_box("FtStartCompensation");
  }

  void MainWindow::on_buttonSave_ftsensor_clicked(bool check)
  {
    if(!qnode.request_ftsensor_save())
      service_error_msg_box("FtSensorSave");
  }

  /*****************************************************************************
   ** Implementation
   *****************************************************************************/

  void MainWindow::switch_tab(int index)
  {
    ui.ErrorsTabWidget->setCurrentIndex(index);
    ui.ControllersTabWidget->setCurrentIndex(index);
  }

  void MainWindow::fill_controllers_list()
  {
    
    // remove all items from combobox
    while(ui.comboRunningCtl->count() != 0)
      ui.comboRunningCtl->removeItem(0);
    while(ui.comboStoppedCtl->count() != 0)
      ui.comboStoppedCtl->removeItem(0);

    std::vector<std::string> running_controllers, stopped_controllers;
    if(!qnode.get_controllers_list(running_controllers, stopped_controllers))
      std::cout << "Error while fetching controllers list." << std::endl;

    for (std::vector<std::string>::iterator it = running_controllers.begin();
	 it != running_controllers.end(); ++it)
      ui.comboRunningCtl->addItem(QString::fromStdString(*it));

    for (std::vector<std::string>::iterator it = stopped_controllers.begin();
	 it != stopped_controllers.end(); ++it)
      ui.comboStoppedCtl->addItem(QString::fromStdString(*it));
  }

  void MainWindow::fill_cartpos_traj_fields()
  {
    bool outcome;

    // Cartesian Position Controller
    lwr_force_position_controllers::CartesianPositionCommandTrajMsg cartpos_current_traj;
						      
    outcome = qnode.get_current_cmd<lwr_force_position_controllers::CartesianPositionCommandTraj,\
    				    lwr_force_position_controllers::CartesianPositionCommandTrajMsg>(cartpos_current_traj);
    if(!outcome)
      service_error_msg_box("CartesianPositionController(Get Current Traj)");

    ui.textPositionX_cartpos->setText(QString::number(cartpos_current_traj.x,'f', 3));
    ui.textPositionY_cartpos->setText(QString::number(cartpos_current_traj.y,'f', 3));
    ui.textPositionZ_cartpos->setText(QString::number(cartpos_current_traj.z,'f', 3));
    ui.textYaw_cartpos->setText(QString::number(cartpos_current_traj.yaw,'f', 3));
    ui.textPitch_cartpos->setText(QString::number(cartpos_current_traj.pitch,'f', 3));
    ui.textRoll_cartpos->setText(QString::number(cartpos_current_traj.roll,'f', 3));
    ui.textDuration_cartpos->setText(QString::number(cartpos_current_traj.p2p_traj_duration,'f', 3));
  }

  void MainWindow::fill_cartpos_gains_fields()
  {
    bool outcome;

    lwr_force_position_controllers::CartesianPositionCommandGainsMsg cartpos_current_gains;
						      
    outcome = qnode.get_current_cmd<lwr_force_position_controllers::CartesianPositionCommandGains,\
    				    lwr_force_position_controllers::CartesianPositionCommandGainsMsg>(cartpos_current_gains);
    if(!outcome)
      service_error_msg_box("CartesianPositionController(Get Current Gains)");

    ui.textKp_cartpos->setText(QString::number(cartpos_current_gains.kp,'f', 0));
    ui.textKp_a4_cartpos->setText(QString::number(cartpos_current_gains.kp_a4,'f', 0));
    ui.textKp_a5_cartpos->setText(QString::number(cartpos_current_gains.kp_a5,'f', 0));
    ui.textKp_a6_cartpos->setText(QString::number(cartpos_current_gains.kp_a6,'f', 0));
    ui.textKd_cartpos->setText(QString::number(cartpos_current_gains.kd,'f', 0));
    ui.textKd_a4_cartpos->setText(QString::number(cartpos_current_gains.kd_a4,'f', 0));
    ui.textKd_a5_cartpos->setText(QString::number(cartpos_current_gains.kd_a5,'f', 0));
    ui.textKd_a6_cartpos->setText(QString::number(cartpos_current_gains.kd_a6,'f', 0));

  }


  void MainWindow::fill_hybrid_traj_pos_fields()
  {
    bool outcome;

    lwr_force_position_controllers::HybridImpedanceCommandTrajPosMsg current_cmd;

    outcome = qnode.get_current_cmd<lwr_force_position_controllers::HybridImpedanceCommandTrajPos,\
    				    lwr_force_position_controllers::HybridImpedanceCommandTrajPosMsg>(current_cmd);
    if(!outcome)
      service_error_msg_box("HybridImpedanceController(Get Position Trajectory)");

    ui.textPositionX_hybrid->setText(QString::number(current_cmd.x,'f', 3));
    ui.textPositionY_hybrid->setText(QString::number(current_cmd.y,'f', 3));
    ui.textPositionZ_hybrid->setText(QString::number(current_cmd.z,'f', 3));
    ui.textAlpha_hybrid->setText(QString::number(current_cmd.alpha,'f', 3));
    ui.textBeta_hybrid->setText(QString::number(current_cmd.beta,'f', 3));
    ui.textGamma_hybrid->setText(QString::number(current_cmd.gamma,'f', 3));
    ui.textTraj_duration_hybrid->setText(QString::number(current_cmd.p2p_traj_duration));
  }

  void MainWindow::fill_hybrid_traj_force_fields()
  {
    bool outcome;

    lwr_force_position_controllers::HybridImpedanceCommandTrajForceMsg current_cmd;

    outcome = qnode.get_current_cmd<lwr_force_position_controllers::HybridImpedanceCommandTrajForce,\
    				    lwr_force_position_controllers::HybridImpedanceCommandTrajForceMsg>(current_cmd);
    if(!outcome)
      service_error_msg_box("HybridImpedanceController(Get Force Trajectory)");

    ui.textForceZ_hybrid->setText(QString::number(current_cmd.forcez,'f', 3));
    ui.textForceZ0_hybrid->setText(QString::number(current_cmd.forcez0));
    ui.textForce_duration_hybrid->setText(QString::number(current_cmd.force_ref_duration));

  }

  void MainWindow::fill_hybrid_gains_fields()
  {
    bool outcome;

    lwr_force_position_controllers::HybridImpedanceCommandGainsMsg current_cmd;

    outcome = qnode.get_current_cmd<lwr_force_position_controllers::HybridImpedanceCommandGains,\
    				    lwr_force_position_controllers::HybridImpedanceCommandGainsMsg>(current_cmd);
    if(!outcome)
      service_error_msg_box("HybridImpedanceController(Get Gains)");

    ui.textKp_pos_hybrid->setText(QString::number(current_cmd.kp_pos,'f', 0));
    ui.textKp_att_hybrid->setText(QString::number(current_cmd.kp_att,'f', 0));
    ui.textKp_gamma_hybrid->setText(QString::number(current_cmd.kp_gamma,'f', 0));

    ui.textKd_pos_hybrid->setText(QString::number(current_cmd.kd_pos,'f', 0));
    ui.textKd_att_hybrid->setText(QString::number(current_cmd.kd_att,'f', 0));
    ui.textKd_gamma_hybrid->setText(QString::number(current_cmd.kd_gamma,'f', 0));

    ui.textKmf_hybrid->setText(QString::number(current_cmd.km_f,'f', 0));
    ui.textKdf_hybrid->setText(QString::number(current_cmd.kd_f,'f', 0));

    ui.textKp_null_z_hybrid->setText(QString::number(current_cmd.kp_z_im, 'f', 3));
    ui.textKp_null_gamma_hybrid->setText(QString::number(current_cmd.kp_gamma_im, 'f', 3));
    ui.textKd_null_pos_hybrid->setText(QString::number(current_cmd.kd_pos_im, 'f', 0));
    ui.textKd_null_att_hybrid->setText(QString::number(current_cmd.kd_att_im, 'f', 0));
  }

  void MainWindow::change_error_z_label(std::string label_text)
  {
    ui.HybridZLabel->setText(QString::fromStdString(label_text));
  }

  void MainWindow::set_send_state_label(QLabel* label, bool accepted, double elapsed, double duration)
  {
    QString label_text;

    if(accepted)
      label_text = QString::fromStdString("Accepted");
    else 
      {
	if (elapsed >= duration)
	  label_text = QString::fromStdString("Completed ");
	else
	  label_text = QString::fromStdString("Wait ") +\
	    QString::number(duration - elapsed, 'f', 0) +\
	    QString::fromStdString(" s");
      }

    label->setText(label_text);
  }

  void MainWindow::field_error_msg_box(std::string field_name)
  {
    QMessageBox msg_box;
    msg_box.setGeometry(QStyle::alignedRect(Qt::LeftToRight,
					    Qt::AlignCenter,
					    msg_box.size(),
					    qApp->desktop()->availableGeometry()));
    QString error_pre = "The value inserted in the field ";
    QString error_post = " is invalid.";
    
    msg_box.setText(error_pre + QString::fromStdString(field_name) + error_post);
    msg_box.exec();
  }

  void MainWindow::service_error_msg_box(std::string controller_name)
  {
    QMessageBox msg_box;
    msg_box.setGeometry(QStyle::alignedRect(Qt::LeftToRight,
					    Qt::AlignCenter,
					    msg_box.size(),
					    qApp->desktop()->availableGeometry()));
    QString error_pre = "Service execution related to ";
    QString error_post = " failed!";
    
    msg_box.setText(error_pre + QString::fromStdString(controller_name) + error_post);
    msg_box.exec();
  }

}  // namespace controller_switcher

