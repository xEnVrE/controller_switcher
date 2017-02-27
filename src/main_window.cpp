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
#include <lwr_force_position_controllers/FtSensorToolEstimationMsg.h>

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
  {
    // setup the ui and
    // connect all ui's triggers to on_...() callbacks
    ui.setupUi(this); 

    // init ros node
    qnode.set_robot_namespace(argv[1]);
    qnode.init();

    // move the window to the center
    setGeometry(QStyle::alignedRect(Qt::LeftToRight,
				    Qt::AlignCenter,
				    size(),
				    qApp->desktop()->availableGeometry()));

    // force size of the window
    setFixedSize(965, 720);

    // fill controller lists from robot_namespace_/controller_manager/ListControllers
    fill_controllers_list();

    // fill controllers command fields with current commands
    fill_controllers_command_fields();

    // fill sensor configuration
    fill_sensor_configuration();

    // connect joints state view update to joints state topic callback
    QObject::connect(&qnode, SIGNAL(jointsStateArrived()), this, SLOT(updateJointsState()));

    // connect joints error view update to joints error topic callback
    QObject::connect(&qnode, SIGNAL(jointsErrorArrived()), this, SLOT(updateJointsError()));

    // connect cartesian error view update to joints error topic callback
    QObject::connect(&qnode, SIGNAL(cartesianErrorArrived()), this, SLOT(updateCartesianError()));
  }

  MainWindow::~MainWindow() {}

  /*****************************************************************************
   ** Implementation [Slots][manually connected]
   *****************************************************************************/
  
  void MainWindow::updateJointsState()
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

  void MainWindow::updateJointsError()
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

  void MainWindow::updateCartesianError()
  {
    std::vector<QLabel*> labels_list;
    std::vector<double> cartesian_error;

    // get new cartesian error
    qnode.get_cartesian_error(cartesian_error);

    labels_list.push_back(ui.textCart_x_err);
    labels_list.push_back(ui.textCart_y_err);
    labels_list.push_back(ui.textCart_Fz_err);
    labels_list.push_back(ui.textCart_yaw_err);
    labels_list.push_back(ui.textCart_pitch_err);
    labels_list.push_back(ui.textCart_roll_err);

    // position error
    for (int i=0; i<6; i++)
      labels_list.at(i)->setText(QString::number(cartesian_error.at(i), 'f', 3));
  }

  /*****************************************************************************
   ** Implementation [Slots]
   *****************************************************************************/

  /*
   * These triggers whenever the button is clicked, regardless of whether it
   * is already checked or not.
   */
 
  void MainWindow::on_buttonQuit_clicked(bool check)
  {
    close();    
  }

  void MainWindow::on_buttonCalibrate_ftsensor_clicked(bool check)
  {
    if(!qnode.request_ftsensor_calibration())
      service_error_msg_box("FtSensorCalibration");
  }

  void MainWindow::on_buttonEstimateTool_ftsensor_clicked(bool check)
  {
    lwr_force_position_controllers::FtSensorToolEstimationMsg response;

    if(!qnode.request_ftsensor_tool_estimation(response))
      service_error_msg_box("FtSensorToolEstimation");
    
    // Update with the new estimation
    ui.labelWristToolComX->setText(QString::number(response.arm_x));
    ui.labelWristToolComY->setText(QString::number(response.arm_y));
    ui.labelWristToolComZ->setText(QString::number(response.arm_z));
    ui.labelToolMass->setText(QString::number(response.mass));
  }

  void MainWindow::on_buttonSetBias_ftsensor_clicked(bool check)
  {
    if(!qnode.request_ftsensor_bias_setup())
      service_error_msg_box("FtSensorBiasSetup");
  }

  void MainWindow::on_buttonReload_cartpos_clicked(bool check)
  {
    fill_cartpos_command_fields();
  }

  void MainWindow::on_buttonSet_hybrid_clicked(bool check)
  {
    double position_x, position_y, force_z;
    double yaw, pitch, roll;
    double kp, kd, km_f, kd_f, kp_im, kd_im;
    double frequency, radius;
    double center_x, center_y;
    bool circle_trj;
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

    yaw = ui.textYaw_hybrid->text().toDouble(&outcome);
    if (!outcome)
      {
	field_error_msg_box("Yaw");
	return;
      }

    pitch = ui.textPitch_hybrid->text().toDouble(&outcome);
    if (!outcome)
      {
	field_error_msg_box("Pitch");
	return;
      }

    roll = ui.textRoll_hybrid->text().toDouble(&outcome);
    if (!outcome)
      {
	field_error_msg_box("Roll");
	return;
      }

    force_z = ui.textForceZ_hybrid->text().toDouble(&outcome);
    if (!outcome)
      {
	field_error_msg_box("ForceZ");
	return;
      }

    kp = ui.textKp_hybrid->text().toDouble(&outcome);
    if (!outcome)
      {
	field_error_msg_box("Kp");
	return;
      }

    kd = ui.textKd_hybrid->text().toDouble(&outcome);
    if (!outcome)
      {
	field_error_msg_box("Kd");
	return;
      }

    km_f = ui.textKmf_hybrid->text().toDouble(&outcome);
    if (!outcome)
      {
	field_error_msg_box("Km Force");
	return;
      }

    kd_f = ui.textKdf_hybrid->text().toDouble(&outcome);
    if (!outcome)
      {
	field_error_msg_box("Kd Force");
	return;
      }

    kp_im = ui.textKp_null_hybrid->text().toDouble(&outcome);
    if (!outcome)
      {
	field_error_msg_box("Kp Null");
	return;
      }

    kd_im = ui.textKd_null_hybrid->text().toDouble(&outcome);
    if (!outcome)
      {
	field_error_msg_box("Kd Null");
	return;
      }

    center_x = ui.textCenterX_hybrid->text().toDouble(&outcome);
    if (!outcome)
      {
	field_error_msg_box("Center X");
	return;
      }

    center_y = ui.textCenterY_hybrid->text().toDouble(&outcome);
    if (!outcome)
      {
	field_error_msg_box("Center Y");
	return;
      }

    frequency = ui.textFrequency_hybrid->text().toDouble(&outcome);
    if (!outcome)
      {
	field_error_msg_box("Frequency");
	return;
      }

    radius = ui.textRadius_hybrid->text().toDouble(&outcome);
    if (!outcome)
      {
	field_error_msg_box("Radius");
	return;
      }

    circle_trj = ui.checkBoxEnableTraj_hybrid->isChecked();

    lwr_force_position_controllers::HybridImpedanceCommandMsg command_hybrid;
    command_hybrid.x = position_x;
    command_hybrid.y = position_y;
    command_hybrid.yaw = yaw;
    command_hybrid.pitch = pitch;
    command_hybrid.roll = roll;
    command_hybrid.forcez = force_z;
    command_hybrid.kp = kp;
    command_hybrid.kd = kd;
    command_hybrid.km_f = km_f;
    command_hybrid.kd_f = kd_f;
    command_hybrid.circle_trj = circle_trj;
    command_hybrid.center_x = center_x;
    command_hybrid.center_y = center_y;
    command_hybrid.frequency = frequency;
    command_hybrid.radius = radius;	

    lwr_force_position_controllers::CartesianInverseCommandMsg command_cartesian_inverse;
    command_cartesian_inverse.kp_im = kp_im;
    command_cartesian_inverse.kd_im = kd_im;
					      
    outcome = qnode.set_command<lwr_force_position_controllers::HybridImpedanceCommand,\
    				lwr_force_position_controllers::HybridImpedanceCommandMsg>(command_hybrid);
    if(!outcome)
      service_error_msg_box("HybridImpedanceController(set)");

    outcome = qnode.set_command<lwr_force_position_controllers::CartesianInverseCommand,\
    				lwr_force_position_controllers::CartesianInverseCommandMsg>(command_cartesian_inverse);
    // if(!outcome)
    //   service_error_msg_box("CartesianInverseController(set)");
    
  }

  void MainWindow::on_buttonSet_cartpos_clicked(bool check)
  {
    double position_x, position_y, position_z;
    double yaw, pitch, roll;
    double kp, kd;
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

    kp = ui.textKp_cartpos->text().toDouble(&outcome);
    if (!outcome)
      {
	field_error_msg_box("kp");
	return;
      }

    kd = ui.textKd_cartpos->text().toDouble(&outcome);
    if (!outcome)
      {
	field_error_msg_box("kd");
	return;
      }

    p2p_traj_duration = ui.textDuration_cartpos->text().toDouble(&outcome);
    if (!outcome)
      {
	field_error_msg_box("Duration");
	return;
      }


    hold_last_qdes_found = ui.checkBoxUseLastQ_cartpos->isChecked();

    lwr_force_position_controllers::CartesianPositionCommandMsg command;
    command.x = position_x;
    command.y = position_y;
    command.z = position_z;
    command.yaw = yaw;
    command.pitch = pitch;
    command.roll = roll;
    command.kp = kp;
    command.kd = kd;
    command.p2p_traj_duration = p2p_traj_duration;
    command.hold_last_qdes_found = hold_last_qdes_found;

    outcome = qnode.set_command<lwr_force_position_controllers::CartesianPositionCommand,\
    				lwr_force_position_controllers::CartesianPositionCommandMsg>(command);
    if(!outcome)
      service_error_msg_box("CartesianPositionController(set)");
  }

  void MainWindow::on_buttonSwitch_clicked(bool check)
  {
    std::string stop_controller = ui.comboRunningCtl->currentText().toStdString();
    std::string start_controller = ui.comboStoppedCtl->currentText().toStdString();

    if(qnode.switch_controllers(start_controller, stop_controller))
      {
	// if controller switch succeded reload controller list in combo boxes
	fill_controllers_list();
	// Let the ros node knows that cartesian position controller is started
	// so that it can signal when new errors are available on each topic callback
	qnode.set_cartpos_controller_state(start_controller == "cartesian_position_controller");
	qnode.set_hybrid_controller_state(start_controller == "hybrid_impedance_controller");
      }
  }
    
  /*****************************************************************************
   ** Implementation
   *****************************************************************************/

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

  void MainWindow::fill_cartpos_command_fields()
  {
    bool outcome;

    // Cartesian Position Controller
    lwr_force_position_controllers::CartesianPositionCommandMsg cartpos_current_cmd;
						      
    outcome = qnode.get_current_cmd<lwr_force_position_controllers::CartesianPositionCommand,\
    				    lwr_force_position_controllers::CartesianPositionCommandMsg>(cartpos_current_cmd);
    if(!outcome)
      service_error_msg_box("CartesianPositionController(get)");

    ui.textPositionX_cartpos->setText(QString::number(cartpos_current_cmd.x,'f', 3));
    ui.textPositionY_cartpos->setText(QString::number(cartpos_current_cmd.y,'f', 3));
    ui.textPositionZ_cartpos->setText(QString::number(cartpos_current_cmd.z,'f', 3));
    ui.textYaw_cartpos->setText(QString::number(cartpos_current_cmd.yaw,'f', 3));
    ui.textPitch_cartpos->setText(QString::number(cartpos_current_cmd.pitch,'f', 3));
    ui.textRoll_cartpos->setText(QString::number(cartpos_current_cmd.roll,'f', 3));
    ui.textKp_cartpos->setText(QString::number(cartpos_current_cmd.kp,'f', 3));
    ui.textKd_cartpos->setText(QString::number(cartpos_current_cmd.kd,'f', 3));
    ui.textDuration_cartpos->setText(QString::number(cartpos_current_cmd.p2p_traj_duration,'f', 3));
  }

  void MainWindow::fill_hybrid_command_fields()
  {

    bool outcome;

    lwr_force_position_controllers::HybridImpedanceCommandMsg hybrid_curr_cmd;
    lwr_force_position_controllers::CartesianInverseCommandMsg cartesian_inverse_curr_cmd;

    outcome = qnode.get_current_cmd<lwr_force_position_controllers::HybridImpedanceCommand,\
    				    lwr_force_position_controllers::HybridImpedanceCommandMsg>(hybrid_curr_cmd);
    if(!outcome)
      service_error_msg_box("HybridImpedanceController(get)");

    outcome = qnode.get_current_cmd<lwr_force_position_controllers::CartesianInverseCommand,\
    				    lwr_force_position_controllers::CartesianInverseCommandMsg>(cartesian_inverse_curr_cmd);

    if(!outcome)
      service_error_msg_box("CartesianInverseController(get)");

    ui.textPositionX_hybrid->setText(QString::number(hybrid_curr_cmd.x,'f', 3));
    ui.textPositionY_hybrid->setText(QString::number(hybrid_curr_cmd.y,'f', 3));
    ui.textYaw_hybrid->setText(QString::number(hybrid_curr_cmd.yaw,'f', 3));
    ui.textPitch_hybrid->setText(QString::number(hybrid_curr_cmd.pitch,'f', 3));
    ui.textRoll_hybrid->setText(QString::number(hybrid_curr_cmd.roll,'f', 3));
    ui.textForceZ_hybrid->setText(QString::number(hybrid_curr_cmd.forcez,'f', 3));
    ui.textKp_hybrid->setText(QString::number(hybrid_curr_cmd.kp,'f', 3));
    ui.textKd_hybrid->setText(QString::number(hybrid_curr_cmd.kd,'f', 3));
    ui.textKmf_hybrid->setText(QString::number(hybrid_curr_cmd.km_f,'f', 3));
    ui.textKdf_hybrid->setText(QString::number(hybrid_curr_cmd.kd_f,'f', 3));
    ui.textCenterX_hybrid->setText(QString::number(hybrid_curr_cmd.center_x,'f', 3));
    ui.textCenterY_hybrid->setText(QString::number(hybrid_curr_cmd.center_y,'f', 3));
    ui.textFrequency_hybrid->setText(QString::number(hybrid_curr_cmd.frequency,'f', 3));
    ui.textRadius_hybrid->setText(QString::number(hybrid_curr_cmd.radius,'f', 3));
    ui.checkBoxEnableTraj_hybrid->setChecked(hybrid_curr_cmd.circle_trj);
    ui.textKp_null_hybrid->setText(QString::number(cartesian_inverse_curr_cmd.kp_im));
    ui.textKd_null_hybrid->setText(QString::number(cartesian_inverse_curr_cmd.kd_im));

  }

  void MainWindow::fill_controllers_command_fields()
  {
    // Cartesian Position Controller
    fill_cartpos_command_fields();

    // Hybrid Impedance Controller
    fill_hybrid_command_fields();

  }

  void MainWindow::fill_sensor_configuration()
  {
    lwr_force_position_controllers::FtSensorToolEstimationMsg response;

    if(!qnode.get_ftsensor_estimated_tool(response))
      service_error_msg_box("FtSensorController(get)");

    // Show the last tool estimation
    ui.labelWristToolComX->setText(QString::number(response.arm_x));
    ui.labelWristToolComY->setText(QString::number(response.arm_y));
    ui.labelWristToolComZ->setText(QString::number(response.arm_z));
    ui.labelToolMass->setText(QString::number(response.mass));
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

