# -*- coding:utf-8 -*-
import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from cv_bridge import CvBridge
import cv2
from PyQt5 import uic
import PyQt5.QtGui as qg
import PyQt5.QtCore as qc
import PyQt5.QtWidgets as qw
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionClient
from sensor_msgs.msg import Imu, Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# Replace with your actual action type
from irobot_create_msgs.action import Dock, Undock

# path to the image
# /home/jeremy/ros_ws/install/full_gui/lib/python3.10/site-packages/full_gui/images.png

plt.ion()


class SimpleDockClient(Node):
    def __init__(self):
        super().__init__('simple_dock_client')
        self._client = ActionClient(self, Dock, '/rpi_10/dock')

    def send_goal(self):
        self._client.wait_for_server()
        goal_msg = Dock.Goal()
        self.get_logger().info('Sent dock request...')
        self._client.send_goal_async(goal_msg)

class SimpleUndockClient(Node):
    def __init__(self):
        super().__init__('simple_undock_client')
        self._client = ActionClient(self, Undock, '/rpi_10/undock')

    def send_goal(self):
        self._client.wait_for_server()
        goal_msg = Undock.Goal()
        self.get_logger().info('Sent undock request...')
        self._client.send_goal_async(goal_msg)

class GraphView(qw.QWidget):
    def __init__(self, name='Name', title='Title', graph_title='Graph Title', node=None, parent=None):
        super(GraphView, self).__init__(parent)
        self.name = name
        self.graph_title = graph_title
        self.node = node
        self.dpi = 100
        self.fig = Figure((5.0, 3.0), dpi=self.dpi, facecolor=(1, 1, 1), edgecolor=(0, 0, 0))
        self.axes = self.fig.add_subplot(111)
        self.canvas = FigureCanvas(self.fig)
        self.canvas.setParent(self)
        self.toolbar = NavigationToolbar(self.canvas, self)

        self.layout = qw.QVBoxLayout()
        self.layout.addWidget(self.toolbar)
        self.layout.addWidget(self.canvas)
        self.layout.setStretchFactor(self.canvas, 1)
        self.setLayout(self.layout)
        self.canvas.show()

        self.timer_period = 100  # ms
        self.timer = qc.QTimer()
        self.timer.setTimerType(qc.Qt.PreciseTimer)
        self.timer.timeout.connect(self.do_something)
        self.timer.start(self.timer_period)
        print("Plot initialized...")

    def clear(self):
        self.axes.clear()

    def plot(self, *args, **kwargs):
        self.axes.plot(*args, **kwargs)

    def draw(self):
        self.canvas.draw()

    def add_patch(self, patch):
        self.axes.add_patch(patch)

    def scatter(self, *args, **kwargs):
        self.axes.scatter(*args, **kwargs)

    def text(self, *args, **kwargs):
        self.axes.text(*args, **kwargs)

    def do_something(self, *args, **kwargs):
        self.clear()
        self.plot(self.node.history[:,0],self.node.history[:,1])
        self.draw()
        # rclpy.spin_once(self.node)

# self constructed main window from qtdesigner
class MainWindow(qw.QMainWindow):
    def __init__(self, speedPub, inputPub, imuPub, d2pickup,JCTClient):
        super(MainWindow, self).__init__()

        # variables
        self.speedCoeff = 1
        self.speedPublisher = speedPub
        self.inputPublisher = inputPub
        self.imuPublisher = imuPub
        self.dockClient = SimpleDockClient()
        self.undockClient = SimpleUndockClient()
        self.d2pickup = d2pickup
        self.jctClient = JCTClient
        
        # Load the .ui file dynamically
        uic.loadUi("/home/jeremy/ros_ws/install/full_gui/lib/python3.10/site-packages/full_gui/mwindow.ui", self)
        
        # Connect the button
        self.backButton.clicked.connect(self.back_Button)
        self.backLeftButton.clicked.connect(self.back_Left_Button)
        self.backRightButton.clicked.connect(self.back_Right_Button)
        self.frontButton.clicked.connect(self.front_Button)
        self.frontLeftButton.clicked.connect(self.front_Left_Button)
        self.frontRightButton.clicked.connect(self.front_Right_Button)
        self.leftButton.clicked.connect(self.left_Button)
        self.rightButton.clicked.connect(self.right_Button)
        self.stopButton.clicked.connect(self.stop_Button)
        self.dockButton.clicked.connect(self.dockClient.send_goal)
        self.undockButton.clicked.connect(self.undockClient.send_goal)
        self.speedSlider.valueChanged.connect(self.update_Speed) # range 1-10
        self.plotButton.clicked.connect(lambda: self.stackedWidget.setCurrentIndex(0))
        # self.cameraButton.clicked.connect(lambda: self.stackedWidget.setCurrentIndex(1))
        self.saveDataButton.clicked.connect(self.imuPublisher.saveFileDialog)

        self.toPickupButton.clicked.connect(lambda: self.d2pickup.startMoving(colorNum = 0))
        self.toGreenButton.clicked.connect(lambda: self.d2pickup.startMoving(colorNum = 1))
        self.toBlueButton.clicked.connect(lambda: self.d2pickup.startMoving(colorNum = 2))

        self.ur5StartButton.clicked.connect(self.jctClient.execute_next_trajectory)

    def back_Button(self):
        self.speedPublisher.publish(direction=5, speedCoeff=self.speedCoeff)
        self.debugLabel.setText("back_button")
        self.inputPublisher.publish("Back")

    def back_Left_Button(self):
        self.speedPublisher.publish(direction=6, speedCoeff=self.speedCoeff)
        self.debugLabel.setText("back_Left_Button")
        self.inputPublisher.publish("Back Left")

    def back_Right_Button(self):
        self.speedPublisher.publish(direction=4, speedCoeff=self.speedCoeff)
        self.debugLabel.setText("back_Right_Button")
        self.inputPublisher.publish("Back Right")

    def front_Button(self):
        self.speedPublisher.publish(direction=1, speedCoeff=self.speedCoeff)
        self.debugLabel.setText("front_Button")
        self.inputPublisher.publish("Front")

    def front_Left_Button(self):
        self.speedPublisher.publish(direction=8, speedCoeff=self.speedCoeff)
        self.debugLabel.setText("front_Left_Button")
        self.inputPublisher.publish("Front Left")

    def front_Right_Button(self):
        self.speedPublisher.publish(direction=2, speedCoeff=self.speedCoeff)
        self.debugLabel.setText("front_Right_Button")
        self.inputPublisher.publish("Front right")

    def left_Button(self):
        self.speedPublisher.publish(direction=7, speedCoeff=self.speedCoeff)
        self.debugLabel.setText("left_Button")
        self.inputPublisher.publish("Left")

    def right_Button(self):
        self.speedPublisher.publish(direction=3, speedCoeff=self.speedCoeff)
        self.debugLabel.setText("right_Button")
        self.inputPublisher.publish("Right")

    def stop_Button(self):
        self.speedPublisher.publish(direction=0, speedCoeff=self.speedCoeff)
        self.debugLabel.setText("stop_Button")
        self.inputPublisher.publish("Stop")

    def dock_Button(self):
        self.inputPublisher.publish("Dock")
        self.debugLabel.setText("dock_Button")

    def undock_Button(self):
        self.inputPublisher.publish("Undock")
        self.debugLabel.setText("undock_Button")
        
    def update_Speed(self):
        self.speedCoeff = self.speedSlider.value()
        self.speedLabel.setText(str(self.speedSlider.value())+"/10")
        data = "Speed updated to" + str(self.speedCoeff)
        self.inputPublisher.publish(data)

class DriveToPickup(Node):
    def __init__(self, speedPub):
        super().__init__('subscriber')
        self.br = CvBridge()
        self.arrived = 0
        self.colorNum = 0

        # Publisher for sending direction_n_speed_data
        self.speedPublisher = speedPub
        self.get_logger().info("DriveToPickup Node initialized")

    def startMoving(self, colorNum):
        self.colorNum = colorNum #RGB
        self.get_logger().info("Start Moving")
        self.search_threshold = 200
        self.imageSubscription = self.create_subscription(Image,
                                                          '/rpi_10/oakd/rgb/preview/image_raw',
                                                          self.my_callback, 10)
        if (self.arrived):
            return

    def my_callback(self,data):
        self.get_logger().info('Receiving video frame')
        current_frame = self.br.imgmsg_to_cv2(data)
        hsv_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)
        height, width, _ = current_frame.shape 

        # Color mask
        color_ranges = {
            "Red": [(130, 150, 150), (180, 255, 255)],
            "Green": [(32, 80, 80), (80, 255, 255)],
            "Blue": [(75, 95, 47), (157, 255, 255)],
        }

        # # Color mask
        # color_ranges = {
        #     "Red": [(130, 150, 150), (180, 255, 255)],
        #     "Green": [(32, 80, 80), (80, 255, 255)],
        #     "Blue": [(75, 89, 47), (157, 255, 255)],
        # }

        kernel = np.ones((15, 15), np.uint8)
        color_names = list(color_ranges.keys())
        first_color_name = color_names[self.colorNum]
        lower, upper = color_ranges[first_color_name]

        # Create mask for the current color
        mask = cv2.inRange(hsv_frame, np.array(lower), np.array(upper))
        
        # Apply morphological operations to clean the mask
        mask_op = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask_cl = cv2.morphologyEx(mask_op, cv2.MORPH_CLOSE, kernel)
        binary_matrix = (mask_cl // 255).astype(np.uint8)
        total_ones_area = np.sum(binary_matrix)

        # Find contours of blobs
        contours, _ = cv2.findContours(mask_cl, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


        for contour in contours:
            # Get bounding box of the contour
            x, y, w, h = cv2.boundingRect(contour)

            C_x = x + w // 2
            C_y = y + h // 2

        # Get contour of detected color
        contours, _ = cv2.findContours(mask_cl, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            sum_x, sum_y, count = 0, 0, 0
            for contour in contours:
                x, y, w, h = cv2.boundingRect(contour)  # Get bounding box
                C_x = x + w // 2  # Middle X
                sum_x += C_x
                count += 1 

            # Compute the middle point
            avg_C_x = int(sum_x / count)

            # Direction transformation
            dir_horizontal = (avg_C_x / width) * 69  # Normalize to 69° Field Of Vision
            dir_result = dir_horizontal - 34.5
        else:
            avg_C_x, dir_result = 0, 0  # If no detection

        # Compute total 1
        total_ones = np.sum(binary_matrix)
        if (total_ones < 5 and self.search_threshold):
            self.get_logger().info("Searching...")
            self.speedPublisher.publishDistDir(0,20)
            self.search_threshold = self.search_threshold - 1
            return
        elif (self.search_threshold <= 0):
            self.get_logger().info("Unable to find location.")
            self.arrived = 1
            return

        speed_threshold = width * height * 0.2
        stop_threshold = width * height * 0.4
        speed_multiplier = 1
        if(total_ones > stop_threshold):
            speed_multiplier = 0
            dir_result = 0
        elif (total_ones > speed_threshold):
            speed_multiplier = 1 - (total_ones /(width * height))


        # self.get_logger().info("Publishing Data: distance: %f, direction: %f" % (speed_multiplier, dir_result))
        self.speedPublisher.publishDistDir(speed_multiplier,dir_result)
        if(abs(speed_multiplier) < 0.05):
            self.get_logger().info('Arrived')
            self.destroy_subscription(self.imageSubscription)
            return
        return

        


class IMUDataLogger(Node):
    labels = ['ax', 'ay', 'az']

    def __init__(self):
        super().__init__('imu_data_logger')
        self.subscription = self.create_subscription(
            Imu, '/rpi_10/imu', self.listener_callback, qos_profile_sensor_data)
        self.t0 = None
        self.history = np.zeros((0, 4))
        self.fig = None
        print("Subscriber created!")

    def listener_callback(self, msg):
        # self.get_logger().info("received message")
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.t0 is None:
            self.t0 = current_time
        t = current_time - self.t0
        # print(t)
        new_data = np.array([[t, msg.linear_acceleration.x,
                                 msg.linear_acceleration.y,
                                 msg.linear_acceleration.z]])
        self.history = np.vstack([self.history, new_data])

    def saveFileDialog(self):
        file_dialog = qw.QFileDialog()
        file_dialog.setWindowTitle("Select Directory")
        file_dialog.setFileMode(qw.QFileDialog.FileMode.Directory)
        file_dialog.setViewMode(qw.QFileDialog.ViewMode.List)

        if file_dialog.exec():
            selected_directory = file_dialog.selectedFiles()[0]
            print("Selected Directory:", os.path.join(selected_directory,"imu_log.csv"))
            print(np.shape(self.history))
            np.savetxt(os.path.join(selected_directory,"imu_log.csv"), self.history, delimiter=',')


class inputLogger(Node):
    def __init__(self):
        super().__init__('inputLogger')
        self.publisher_ = self.create_publisher(String, '/input_logger', 10)

    def publish(self, inputString):
        msg = String()
        msg.data = inputString
        self.publisher_.publish(msg)


class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('CmdVelPublisher')
        self.publisher_ = self.create_publisher(Twist, '/rpi_10/cmd_vel', 10)


    def publish(self, direction,speedCoeff):
        msg = Twist()

        # 1 = front, 2 = front right, 3 = right, 4 = back right
        # 5 = back,  6 = back left,   7 = left,  8 = front left, 0 = stop
        match direction:
            case 1: # front
                msg.linear.x = 1.0
                msg.angular.z = 0.0
            case 2: # front right
                msg.linear.x = 0.8
                msg.angular.z = -1.0
            case 3: # right
                msg.linear.x = 0.0
                msg.angular.z = -1.0
            case 4: # back right 
                msg.linear.x = -0.8
                msg.angular.z = 1.0
            case 5: # back
                msg.linear.x = -1.0
                msg.angular.z = 0.0
            case 6: # back left
                msg.linear.x = -0.8
                msg.angular.z = -1.0
            case 7: # left
                msg.linear.x = 0.0
                msg.angular.z = 1.0
            case 8: # front left
                msg.linear.x = 0.8 
                msg.angular.z = 1.0 
            case 0: # stop
                msg.linear.x = 0.0
                msg.angular.z = 0.0
            case default:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
        
        speed = speedCoeff/5.0
        msg.linear.x = msg.linear.x * speed
        msg.angular.z = msg.angular.z * speed
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg}"')

    def publishDistDir(self,distance,direction): #,speedCoeff):
        msg = Twist()
        # speed = speedCoeff/5.0
        msg.linear.x = distance * 0.1 #* speed
        msg.angular.z = direction * -0.015 #* speed
        self.publisher_.publish(msg)
        # self.get_logger().info(f'Publishing: "{msg}"')


from builtin_interfaces.msg import Duration
from action_msgs.msg import GoalStatus
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
import time

TRAJECTORIES = {
    "traj0": [
        {
            "positions": [0.043128, -1.28824, 1.37179, -1.82208, -1.63632, -0.18],
            "velocities": [0.001, 0.001, 0.001, 0.001, 0.001, 0.001],
            "time_from_start": Duration(sec=4, nanosec=0),
        },
        {
            "positions": [-0.195016, -1.70093, 0.902027, -0.944217, -1.52982, -0.195171],
            "velocities": [0.001, 0.001, 0.001, 0.001, 0.001, 0.001],
            "time_from_start": Duration(sec=8, nanosec=0),
        },
    ],
    "traj1": [
        {
            "positions": [-0.195016, -1.70094, 0.902027, -0.944217, -1.52982, -0.195171],
            "velocities": [0.001, 0.001, 0.001, 0.001, 0.001, 0.001],
            "time_from_start": Duration(sec=0, nanosec=0),
        },
        {
            "positions": [0.30493, -0.982258, 0.955637, -1.48215, -1.72737, 0.204445],
            "velocities": [0.001, 0.001, 0.001, 0.001, 0.001, 0.001],
            "time_from_start": Duration(sec=8, nanosec=0),
        },
    ],
}

class JTCClient(rclpy.node.Node):
    """Small test client for the jtc."""

    def __init__(self):
        super().__init__("jtc_client")
        self.declare_parameter("controller_name", "scaled_joint_trajectory_controller")
        self.declare_parameter(
            "joints",
            [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ],
        )

        controller_name = self.get_parameter("controller_name").value + "/follow_joint_trajectory"
        self.joints = self.get_parameter("joints").value

        if self.joints is None or len(self.joints) == 0:
            raise Exception('"joints" parameter is required')

        self._action_client = ActionClient(self, FollowJointTrajectory, controller_name)
        self.get_logger().info(f"Waiting for action server on {controller_name}")
        self._action_client.wait_for_server()

        self.parse_trajectories()
        self.i = 0
        self._send_goal_future = None
        self._get_result_future = None
        # self.execute_next_trajectory()

    def parse_trajectories(self):
        self.goals = {}

        for traj_name in TRAJECTORIES:
            goal = JointTrajectory()
            goal.joint_names = self.joints
            for pt in TRAJECTORIES[traj_name]:
                point = JointTrajectoryPoint()
                point.positions = pt["positions"]
                point.velocities = pt["velocities"]
                point.time_from_start = pt["time_from_start"]
                goal.points.append(point)

            self.goals[traj_name] = goal

    def execute_next_trajectory(self):
        if self.i >= len(self.goals):
            self.get_logger().info("Done with all trajectories")
            raise SystemExit
        traj_name = list(self.goals)[self.i]
        self.i = self.i + 1
        if traj_name:
            self.execute_trajectory(traj_name)

    def execute_trajectory(self, traj_name):
        self.get_logger().info(f"Executing trajectory {traj_name}")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = self.goals[traj_name]

        goal.goal_time_tolerance = Duration(sec=0, nanosec=500000000)
        goal.goal_tolerance = [
            JointTolerance(position=0.01, velocity=0.01, name=self.joints[i]) for i in range(6)
        ]

        self._send_goal_future = self._action_client.send_goal_async(goal)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected :(")
            raise RuntimeError("Goal rejected :(")

        self.get_logger().debug("Goal accepted :)")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        self.get_logger().info(f"Done with result: {self.status_to_str(status)}")
        if status == GoalStatus.STATUS_SUCCEEDED:
            time.sleep(2)
            self.execute_next_trajectory()
        else:
            if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
                self.get_logger().error(
                    f"Done with result: {self.error_code_to_str(result.error_code)}"
                )
            raise RuntimeError("Executing trajectory failed. " + result.error_string)

    @staticmethod
    def error_code_to_str(error_code):
        if error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            return "SUCCESSFUL"
        if error_code == FollowJointTrajectory.Result.INVALID_GOAL:
            return "INVALID_GOAL"
        if error_code == FollowJointTrajectory.Result.INVALID_JOINTS:
            return "INVALID_JOINTS"
        if error_code == FollowJointTrajectory.Result.OLD_HEADER_TIMESTAMP:
            return "OLD_HEADER_TIMESTAMP"
        if error_code == FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED:
            return "PATH_TOLERANCE_VIOLATED"
        if error_code == FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED:
            return "GOAL_TOLERANCE_VIOLATED"

    @staticmethod

    def status_to_str(error_code):
        if error_code == GoalStatus.STATUS_UNKNOWN:
            return "UNKNOWN"
        if error_code == GoalStatus.STATUS_ACCEPTED:
            return "ACCEPTED"
        if error_code == GoalStatus.STATUS_EXECUTING:
            return "EXECUTING"
        if error_code == GoalStatus.STATUS_CANCELING:
            return "CANCELING"
        if error_code == GoalStatus.STATUS_SUCCEEDED:
            return "SUCCEEDED"
        if error_code == GoalStatus.STATUS_CANCELED:
            return "CANCELED"
        if error_code == GoalStatus.STATUS_ABORTED:
            return "ABORTED"


def main(args=None):
    app = qw.QApplication(sys.argv)
    rclpy.init(args=args)

    imu_data_logger = IMUDataLogger()
    cmd_vel_publisher = CmdVelPublisher()
    input_logger = inputLogger()
    drive_to_pickup = DriveToPickup(speedPub=cmd_vel_publisher)
    jtc_client = JTCClient()
    main_window = MainWindow(speedPub=cmd_vel_publisher, 
                             inputPub=input_logger, 
                             imuPub = imu_data_logger,
                             d2pickup = drive_to_pickup,
                             JCTClient = jtc_client)

    gv = GraphView(node=imu_data_logger)
    layout_idx0 = qw.QVBoxLayout(main_window.stackedWidget.widget(0))  # Get page at index 0
    layout_idx0.addWidget(gv)

    label = qw.QLabel()
    label.setText("About: \n Intentionally left empty, just to see try multi-page function")
    layout_idx1 = qw.QVBoxLayout(main_window.stackedWidget.widget(1))  # Get page at index 1
    layout_idx1.addWidget(label)
    main_window.show()


    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(imu_data_logger)
    executor.add_node(drive_to_pickup)
    executor.add_node(input_logger)
    executor.add_node(cmd_vel_publisher)
    executor.add_node(jtc_client)

    # Spin in a thread
    import threading
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()


    app.exec_()

    jtc_client.destroy_node()
    imu_data_logger.destroy_node()
    input_logger.destroy_node()
    cmd_vel_publisher.destroy_node()
    drive_to_pickup.destroy_node()
    executor.shutdown()
    rclpy.shutdown()
    sys.exit()


if __name__ == '__main__':
    main()
