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

  void MainWindow::on_buttonSet_clicked(bool check)
  {
    QMessageBox msg_box;
    msg_box.setGeometry(QStyle::alignedRect(Qt::LeftToRight,
					    Qt::AlignCenter,
					    msg_box.size(),
					    qApp->desktop()->availableGeometry()));
    float position_x, position_y, force_z;
    bool outcome;
    QString error_pre = "The value inserted in the field ";
    QString error_post = " is invalid.";

    position_x = ui.textPositionX->text().toFloat(&outcome);
    if (!outcome)
      {
	msg_box.setText(error_pre + "Position X" + error_post);
	msg_box.exec();
	return;
      }

    position_y = ui.textPositionY->text().toFloat(&outcome);
    if (!outcome)
      {
	msg_box.setText(error_pre + "Position Y" + error_post);
	msg_box.exec();
	return;
      }

    force_z = ui.textForceZ->text().toFloat(&outcome);
    if (!outcome)
      {
	msg_box.setText(error_pre + "Force Z" + error_post);
	msg_box.exec();
	return;
      }

    outcome = qnode.set_command(position_x, position_y, force_z);
    if(!outcome)
      {
	msg_box.setText("Service execution failed!");
	msg_box.exec();
      }    
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
  
  void MainWindow::enable_hybrid_controller_pane(bool state)
  {
    ui.textPositionX->setEnabled(state);
    ui.textPositionY->setEnabled(state);
    ui.textForceZ->setEnabled(state);
    ui.buttonSet->setEnabled(state);
  }
}  // namespace controller_switcher

