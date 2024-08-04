import sys
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import QMainWindow, QApplication, QMenu, QAction, QStyle, qApp
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import Qt, QTimer
from ui_monitor import Ui_MainWindow
from kistler_driver_msgs.msg import E0Status
from j6_interface_msgs.msg import M0Status

class MainWindow(QMainWindow):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)

        self.kistler_velocity = 0
        self.vehicle_velocity = 0
        self.diff = 0
        self.error_rate = 0

        self.icon_path = "vehicle.png"
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.setWindowIcon(QIcon(self.icon_path))
        self.create_menubars()
        self.create_timer()
        self.show()

        self.ui.textBrowser_kistler_data.setText(str(self.timer_update))
        self.ui.textBrowser_vehicle_data.setText(str(self.timer_update))
        self.ui.textBrowser_diff_data.setText(str(self.timer_update))
        self.ui.textBrowser_error_data.setText(str(self.timer_update))

        # ROS2 init
        rclpy.init(args=None)
        self.node = Node('Qt_view_node')
        self.sub_kistler_velocity = self.node.create_subscription(E0Status, '/kistler/e0_status', self.on_kistler_velocity, 10)
        self.sub_vehicle_velocity = self.node.create_subscription(M0Status, '/can/status/m0_status', self.on_vehicle_velocity, 10)
        rclpy.spin_once(self.node)

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
        self.timer.start(10)

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

    def on_vehicle_velocity(self, msg):
        self.front_left_wheel_speed = msg.front_left_wheel_speed
        self.front_right_wheel_speed = msg.front_right_wheel_speed
        self.vehicle_velocity = (self.front_left_wheel_speed + self.front_right_wheel_speed) / 2
        self.update_label()

    def calcError(self):
        if self.kistler_velocity == 0:
            pass
        else:
            self.diff = self.vehicle_velocity - self.kistler_velocity
            self.error_rate = abs(self.diff / self.kistler_velocity) * 100
            if self.error_rate >= 3:
                self.ui.textBrowser_error_data.setTextBackgroundColor(Qt.GlobalColor.red)
                self.ui.label_error_data_color.setText("誤差率が3%を超えてます")
                self.ui.label_error_data_color.setFixedWidth(240)
                self.ui.label_error_data_color.setStyleSheet(
                    "color: rgb(255,255,255);"
                    "background-color: rgb(255,0,51);"
                    "border-radius:5px;")
            else:
                self.ui.textBrowser_error_data.setTextBackgroundColor(Qt.GlobalColor.green)
                self.ui.label_error_data_color.setText("誤差率が3%以下になってます")
                self.ui.label_error_data_color.setFixedWidth(280)
                self.ui.label_error_data_color.setStyleSheet(
                    "color: rgb(255,255,255);"
                    "background-color: rgb(18,230,95);"
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
        editMenu = menuBar.addMenu("&Edit")
        editMenu.addMenu("Undo")
        helpMenu = menuBar.addMenu("&Help")
        helpMenu.addMenu("Get Started")

    def prefer_action(self):
        preferMenu = QMenu('Preferences', self)
        preferAct = QAction(QIcon('image/setting.jpg'),'Setting', self)
        preferMenu.addAction(preferAct)

        return preferMenu

    def exit_action(self):
       # Exit Action, connect
        exitAction = QAction(self.style().standardIcon(QStyle.SP_DialogCancelButton),'&Exit', self)
        exitAction.setShortcut('Ctrl+Q')
        exitAction.setStatusTip('Exit application')
        exitAction.triggered.connect(qApp.quit)
        self.statusBar()
        return exitAction


if __name__=="__main__":
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())