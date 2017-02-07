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
#include <lwr_controllers/SetCartesianPositionCommand.h>

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
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    qnode.init();
    setGeometry(QStyle::alignedRect(Qt::LeftToRight,
				    Qt::AlignCenter,
				    size(),
				    qApp->desktop()->availableGeometry()));

    // Fill controller lists from lwr/controller_manager/ListControllers
    fill_controllers_list();
  }

  MainWindow::~MainWindow() {}

  /*****************************************************************************
   ** Implementation [Slots]
   *****************************************************************************/

  // void MainWindow::showNoMasterMessage() {
  //   QMessageBox msgBox;
  //   msgBox.setText("Couldn't find the ros master.");
  //   msgBox.exec();
  //   close();
  // }

  /*
   * These triggers whenever the button is clicked, regardless of whether it
   * is already checked or not.
   */
 
  void MainWindow::on_buttonQuit_clicked(bool check)
  {
    close();    
  }

  void MainWindow::on_buttonSet_hybrid_clicked(bool check)
  {
    double position_x, position_y, force_z;
    double kp, kd, km_f, kd_f;
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

    force_z = ui.textForceZ_hybrid->text().toDouble(&outcome);
    if (!outcome)
      {
	field_error_msg_box("ForceZ");
	return;
      }

    kp = ui.textKp_hybrid->text().toDouble(&outcome);
    if (!outcome)
      {
	field_error_msg_box("kp");
	return;
      }

    kd = ui.textKd_hybrid->text().toDouble(&outcome);
    if (!outcome)
      {
	field_error_msg_box("kd");
	return;
      }

    km_f = ui.textKmf_hybrid->text().toDouble(&outcome);
    if (!outcome)
      {
	field_error_msg_box("km_f");
	return;
      }

    kd_f = ui.textKdf_hybrid->text().toDouble(&outcome);
    if (!outcome)
      {
	field_error_msg_box("kd_f");
	return;
      }
    lwr_controllers::HybridImpedanceCommand command;
    command.x = position_x;
    command.y = position_y;
    command.z = force_z;
    command.kp = kp;
    command.kd = kd;
    command.km_f = km_f;
    command.kd_f = kd_f;
						      
    outcome = qnode.set_command<lwr_controllers::SetHybridImpedanceCommand,\
    				lwr_controllers::HybridImpedanceCommand>(command);
    if(!outcome)
      service_error_msg_box("HybridImpedanceController");
    
  }

  void MainWindow::on_buttonSet_cartpos_clicked(bool check)
  {
    double position_x, position_y, position_z;
    double yaw, pitch, roll;
    double kp, kd;
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

    lwr_controllers::CartesianPositionCommand command;
    command.x = position_x;
    command.y = position_y;
    command.z = position_z;
    command.yaw = yaw;
    command.pitch = pitch;
    command.roll = roll;
    command.kp = kp;
    command.kd = kd;
						      
    outcome = qnode.set_command<lwr_controllers::SetCartesianPositionCommand,\
    				lwr_controllers::CartesianPositionCommand>(command);
    if(!outcome)
      service_error_msg_box("CartesianPositionController");
  }

  void MainWindow::on_buttonSwitch_clicked(bool check)
  {
    std::string stop_controller = ui.comboRunningCtl->currentText().toStdString();
    std::string start_controller = ui.comboStoppedCtl->currentText().toStdString();

    if(qnode.switch_controllers(start_controller, stop_controller))
      {
	// if controller switch succeded reload controller list in combo boxes
	fill_controllers_list();
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
    QString error_pre = "Service execution for controller ";
    QString error_post = " failed!.";
    
    msg_box.setText(error_pre + QString::fromStdString(controller_name) + error_post);
    msg_box.exec();
  }

}  // namespace controller_switcher

