#!/usr/bin/env python3

# Copyright 2024 TIER IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import sys
import rclpy
import webbrowser
from rclpy.node import Node
from PyQt5.QtWidgets import QMainWindow, QApplication, QMenu, QAction, QStyle, qApp
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import Qt, QTimer
from ui_monitor import Ui_MainWindow
from kistler_driver_msgs.msg import E0Status


class MainWindow(QMainWindow):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)

        self.kistler_velocity = 0
        self.vehicle_velocity = 0
        self.diff = 0
        self.error_rate = 0

        self.icon_path = os.path.join(os.path.dirname(__file__), 'vehicle.png')
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.setWindowIcon(QIcon(self.icon_path))
        self.create_menubars()
        self.create_timer()
        self.show()
        self.ui.vehicle_model.setItemData(0, 0, Qt.UserRole - 1)
        self.ui.vehicle_model.currentIndexChanged.connect(self.on_vehicle_model_changed)

        self.ui.textBrowser_kistler_data.setText(str(self.timer_update))
        self.ui.textBrowser_vehicle_data.setText(str(self.timer_update))
        self.ui.textBrowser_diff_data.setText(str(self.timer_update))
        self.ui.textBrowser_error_data.setText(str(self.timer_update))

        # ROS2 init
        rclpy.init(args=None)
        self.node = Node('Qt_view_node')
        self.sub_kistler_velocity = self.node.create_subscription(E0Status, '/kistler/e0_status', self.on_kistler_velocity, 10)
        rclpy.spin_once(self.node)

    def on_vehicle_model_changed(self):
        selected_model = self.ui.vehicle_model.currentText()
        self.ui.show_vehicle_model.setText(f"Selected Vehicle Model: {selected_model}")

        if selected_model != "- - - - Select Vehicle Model - - - -":
            if selected_model == "BYD J6 Gen1":
                from j6_interface_msgs.msg import M0Status
                self.sub_vehicle_velocity = self.node.create_subscription(M0Status, '/j6/can/status/m0_status', self.on_vehicle_velocity_j6, 10)
            elif selected_model == "BYD J6 Gen2":
                from j6_interface_msgs.msg import M0Status
                self.sub_vehicle_velocity = self.node.create_subscription(M0Status, '/j6/can/status/m0_status', self.on_vehicle_velocity_j6, 10)
            elif selected_model == "PIX RoboBus":
                from pix_robobus_driver_msgs.msg import WheelSpeedReport
                self.sub_vehicle_velocity = self.node.create_subscription(WheelSpeedReport, '/pix_robobus/wheel_speed_report', self.on_vehicle_velocity_pix, 10)
            else:
                print("Select a valid vehicle model, please.")
            self.ui.vehicle_model.setDisabled(True)

    def __del__(self):
        self.node.destroy_node()

    def create_timer(self):
        # create timer
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.timer_update)
        self.timer.start(10)

    def timer_update(self):
        rclpy.spin_once(self.node)
        self.update_label()
        self.calcError()
        self.show()

    def update_label(self):
        self.ui.textBrowser_kistler_data.setText(str("{:.2f}".format(self.kistler_velocity)))
        self.ui.textBrowser_vehicle_data.setText(str("{:.2f}".format(self.vehicle_velocity)))
        self.ui.textBrowser_diff_data.setText(str("{:.2f}".format(self.diff)))
        self.ui.textBrowser_error_data.setText(str("{:.2f}".format(self.error_rate)))
        self.show()

    ### ROS2 Data Updater
    def on_kistler_velocity(self, msg):
        self.kistler_velocity = msg.velocity_x
        self.update_label()

    def on_vehicle_velocity_j6(self, msg):
        self.front_left_wheel_speed = msg.front_left_wheel_speed
        self.front_right_wheel_speed = msg.front_right_wheel_speed
        self.vehicle_velocity = (self.front_left_wheel_speed + self.front_right_wheel_speed) / 2
        self.update_label()

    def on_vehicle_velocity_pix(self, msg):
        self.vehicle_velocity = (msg.wheel_speed_fr + msg.wheel_speed_fl + msg.wheel_speed_rr + msg.wheel_speed_rl) / 4 * 3.6
        self.update_label()

    def calcError(self):
        if self.kistler_velocity == 0:
            pass
        else:
            self.diff = self.vehicle_velocity - self.kistler_velocity
            self.error_rate = abs(self.diff / self.kistler_velocity) * 100
            if self.error_rate >= 3:
                self.ui.textBrowser_error_data.setStyleSheet("background-color: rgb(255,0,0); color: white;")
                self.ui.label_error_data_color.setText("誤差率が3%を超えてます")
                self.ui.label_error_data_color.setFixedWidth(240)
                self.ui.label_error_data_color.setStyleSheet(
                    "color: rgb(255,255,255);"
                    "background-color: rgb(255,0,0);"
                    "border-radius:5px;")
            else:
                self.ui.textBrowser_error_data.setStyleSheet("background-color: rgb(0,255,0); color: white;")
                self.ui.label_error_data_color.setText("誤差率が3%以下になってます")
                self.ui.label_error_data_color.setFixedWidth(280)
                self.ui.label_error_data_color.setStyleSheet(
                    "color: rgb(255,255,255);"
                    "background-color: rgb(0,255,0);"
                    "border-radius:5px;")

    ### QMenu
    def create_menubars(self):
        menuBar = self.menuBar()
        # Creating menus using a QMenu object
        fileMenu = QMenu("&File", self)
        fileMenu.addAction(self.exit_action())
        fileMenu.addMenu(self.prefer_action())

        menuBar.addMenu(fileMenu)
        # Creating menus using a title
        # editMenu = menuBar.addMenu("&Edit")
        # editMenu.addMenu("TODO")

        helpMenu = menuBar.addMenu("&Help")
        helpAction1 = QAction("User Guide", self)
        helpAction1.triggered.connect(lambda: self.open_help_link("https://tier4.atlassian.net/wiki/spaces/KB4FAE/pages/3097592313"))
        helpMenu.addAction(helpAction1)

        helpAction2 = QAction("Readme", self)
        helpAction2.triggered.connect(lambda: self.open_help_link("https://github.com/tier4/kistler_driver/blob/main/README.md"))
        helpMenu.addAction(helpAction2)

    def prefer_action(self):
        preferMenu = QMenu('Preferences', self)
        return preferMenu

    def exit_action(self):
        # Exit Action, connect
        exitAction = QAction(self.style().standardIcon(QStyle.SP_DialogCancelButton),'&Exit', self)
        exitAction.setShortcut('Ctrl+Q')
        exitAction.setStatusTip('Exit application')
        exitAction.triggered.connect(qApp.quit)
        self.statusBar()
        return exitAction

    def open_help_link(self, url):
        webbrowser.open(url)


if __name__=="__main__":
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())
