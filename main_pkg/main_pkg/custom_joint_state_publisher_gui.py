#!/usr/bin/env python3
import argparse
import math
import random
import signal
import sys
import threading

import rclpy
from joint_state_publisher.joint_state_publisher import JointStatePublisher
from joint_state_publisher_gui.flow_layout import FlowLayout
from python_qt_binding.QtCore import Qt, Signal, pyqtSlot
from python_qt_binding.QtGui import QFont
from python_qt_binding.QtWidgets import (
    QApplication,
    QFormLayout,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QPushButton,
    QScrollArea,
    QSlider,
    QVBoxLayout,
    QWidget,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

from custom_srv_pkg.srv import JointPose

RANGE = 10000
LINE_EDIT_WIDTH = 45
SLIDER_WIDTH = 200
INIT_NUM_SLIDERS = 7  # Initial number of sliders to show in window

# Defined by style - currently using the default style
DEFAULT_WINDOW_MARGIN = 11
DEFAULT_CHILD_MARGIN = 9
DEFAULT_BTN_HEIGHT = 25
DEFAULT_SLIDER_HEIGHT = 64  # Is the combination of default heights in Slider

# Calculate default minimums for window sizing
MIN_WIDTH = SLIDER_WIDTH + DEFAULT_CHILD_MARGIN * 4 + DEFAULT_WINDOW_MARGIN * 2
MIN_HEIGHT = (
    DEFAULT_BTN_HEIGHT * 2 + DEFAULT_WINDOW_MARGIN * 2 + DEFAULT_CHILD_MARGIN * 2
)


class Slider(QWidget):
    def __init__(self, name):
        super().__init__()

        self.joint_layout = QVBoxLayout()
        self.row_layout = QHBoxLayout()

        font = QFont("Helvetica", 9, QFont.Bold)
        self.label = QLabel(name)
        self.label.setFont(font)
        self.row_layout.addWidget(self.label)

        self.display = QLineEdit("0.00")
        self.display.setAlignment(Qt.AlignRight)
        self.display.setFont(font)
        self.display.setReadOnly(True)
        self.display.setFixedWidth(LINE_EDIT_WIDTH)
        self.row_layout.addWidget(self.display)

        self.joint_layout.addLayout(self.row_layout)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setFont(font)
        self.slider.setRange(0, RANGE)
        self.slider.setValue(int(RANGE / 2))
        self.slider.setFixedWidth(SLIDER_WIDTH)

        self.joint_layout.addWidget(self.slider)

        self.setLayout(self.joint_layout)

    def remove(self):
        self.joint_layout.removeWidget(self.slider)
        self.slider.setParent(None)

        self.row_layout.removeWidget(self.display)
        self.display.setParent(None)

        self.row_layout.removeWidget(self.label)
        self.label.setParent(None)

        self.row_layout.setParent(None)


class JointStatePublisherGui(QMainWindow):
    sliderUpdateTrigger = Signal()
    initialize = Signal()

    def __init__(self, title, jsp):
        super(JointStatePublisherGui, self).__init__()

        self.joint_map = {}

        self.setWindowTitle(title)

        # Button for randomizing the sliders
        self.rand_button = QPushButton("Randomize", self)
        self.rand_button.clicked.connect(self.randomizeEvent)

        # Button for centering the sliders
        self.ctr_button = QPushButton("Center", self)
        self.ctr_button.clicked.connect(self.centerEvent)

        # Clone Button
        self.clone_button = QPushButton("Clone Real", self)
        self.clone_button.clicked.connect(self.clone_real_joint_states)

        # Jitter Button
        self.jitter_button = QPushButton("Jitter", self)
        self.jitter_button.clicked.connect(self.jitterEvent)

        # Move Button
        self.move_button = QPushButton("Move!!!", self)
        self.move_button.clicked.connect(self.move_joint_state)

        # Scroll area widget contents - layout
        self.scroll_layout = FlowLayout()

        # Scroll area widget contents
        self.scroll_widget = QWidget()
        self.scroll_widget.setLayout(self.scroll_layout)

        # Scroll area for sliders
        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)
        self.scroll_area.setWidget(self.scroll_widget)

        # Main layout
        self.main_layout = QVBoxLayout()

        # Add buttons and scroll area to main layout
        # self.main_layout.addWidget(self.rand_button)
        # self.main_layout.addWidget(self.ctr_button)
        self.main_layout.addWidget(self.clone_button)
        self.main_layout.addWidget(self.jitter_button)
        self.main_layout.addWidget(self.move_button)

        self.main_layout.addWidget(self.scroll_area)


        # central widget
        self.central_widget = QWidget()
        self.central_widget.setLayout(self.main_layout)
        self.setCentralWidget(self.central_widget)

        self.jsp = jsp
        self.jsp.set_source_update_cb(self.sliderUpdateCb)
        self.jsp.set_robot_description_update_cb(self.initializeCb)

        self.move_client = self.jsp.create_client(JointPose, '/move_joint_pose')


        self.running = True
        self.sliders = {}

        # Setup signal for initializing the window
        self.initialize.connect(self.initializeSliders)
        # Set up a signal for updating the sliders based on external joint info
        self.sliderUpdateTrigger.connect(self.updateSliders)

        # Tell self to draw sliders in case the JointStatePublisher already has a robot_description
        self.initialize.emit()

    def initializeSliders(self):
        self.joint_map = {}

        for sl, _ in self.sliders.items():
            self.scroll_layout.removeWidget(sl)
            sl.remove()

        ### Generate sliders ###
        for name in self.jsp.joint_list:
            if name not in self.jsp.free_joints:
                continue
            joint = self.jsp.free_joints[name]

            if joint["min"] == joint["max"]:
                continue

            slider = Slider(name)

            self.joint_map[name] = {
                "display": slider.display,
                "slider": slider.slider,
                "joint": joint,
            }

            self.scroll_layout.addWidget(slider)
            # Connect to the signal provided by QSignal
            slider.slider.valueChanged.connect(
                lambda event, name=name: self.onSliderValueChangedOne(name)
            )

            self.sliders[slider] = slider

        # Set zero positions read from parameters
        # self.centerEvent(None)
        self.clone_real_joint_states()

        # Set size of min size of window based on number of sliders.
        if (
            len(self.sliders) >= INIT_NUM_SLIDERS
        ):  # Limits min size to show INIT_NUM_SLIDERS
            num_sliders = INIT_NUM_SLIDERS
        else:
            num_sliders = len(self.sliders)
        scroll_layout_height = num_sliders * DEFAULT_SLIDER_HEIGHT
        scroll_layout_height += (num_sliders + 1) * DEFAULT_CHILD_MARGIN
        self.setMinimumSize(MIN_WIDTH, scroll_layout_height + MIN_HEIGHT)

        self.sliderUpdateTrigger.emit()

    def sliderUpdateCb(self):
        self.sliderUpdateTrigger.emit()

    def initializeCb(self):
        self.initialize.emit()

    def onSliderValueChangedOne(self, name):
        # A slider value was changed, but we need to change the joint_info metadata.
        joint_info = self.joint_map[name]
        slidervalue = joint_info["slider"].value()
        joint = joint_info["joint"]
        joint["position"] = self.sliderToValue(slidervalue, joint)
        joint_info["display"].setText("%.3f" % joint["position"])

    @pyqtSlot()
    def updateSliders(self):
        for name, joint_info in self.joint_map.items():
            joint = joint_info["joint"]
            slidervalue = self.valueToSlider(joint["position"], joint)
            joint_info["slider"].setValue(slidervalue)

    def centerEvent(self, event):
        self.jsp.get_logger().info("Centering")
        for name, joint_info in self.joint_map.items():
            joint = joint_info["joint"]
            joint_info["slider"].setValue(self.valueToSlider(joint["zero"], joint))

    def randomizeEvent(self, event):
        self.jsp.get_logger().info("Randomizing")
        for name, joint_info in self.joint_map.items():
            joint = joint_info["joint"]
            joint_info["slider"].setValue(
                self.valueToSlider(random.uniform(joint["min"], joint["max"]), joint)
            )

    def jitterEvent(self, event):
        self.jsp.get_logger().info("Jittering ±10 degrees")
        delta = math.radians(10)  # ±10 degrees in radians
        self.jsp.get_logger().info("Randomizing each joint by ±10°")
        for name, joint_info in self.joint_map.items():
            if "hande" in name:
                continue
            joint = joint_info["joint"]
            current = joint["position"]
            # compute perturbation bounds, clamped to joint limits
            low = max(joint["min"], current - delta)
            high = min(joint["max"], current + delta)
            new_pos = random.uniform(low, high)
            # update the joint metadata
            joint["position"] = new_pos
            # update GUI slider and display
            slider_val = self.valueToSlider(new_pos, joint)
            joint_info["slider"].setValue(slider_val)
            joint_info["display"].setText(f"{new_pos:.3f}")

    def valueToSlider(self, value, joint):
        return int(
            (value - joint["min"]) * float(RANGE) / (joint["max"] - joint["min"])
        )

    def sliderToValue(self, slider, joint):
        pctvalue = slider / float(RANGE)
        return joint["min"] + (joint["max"] - joint["min"]) * pctvalue

    def closeEvent(self, event):
        self.running = False

    def loop(self):
        while self.running:
            rclpy.spin_once(self.jsp, timeout_sec=0.1)

    def clone_real_joint_states(self):
        self.jsp.get_logger().info("Clone From Real")
        if not rclpy.ok():
            rclpy.init()
        temp_node = rclpy.create_node("clone_real_listener")
        received = {}

        def cb(msg: JointState):
            received["msg"] = msg
            temp_node.destroy_node()

        sub = temp_node.create_subscription(
            JointState, "/joint_states", cb, qos_profile_sensor_data
        )
        rclpy.spin_once(temp_node, timeout_sec=1.0)

        if "msg" in received:
            js = received["msg"]

            for name, pos in zip(js.name, js.position):
                name = "fake_" + name
                if name in self.joint_map:
                    info = self.joint_map[name]
                    j = info["joint"]
                    clamped = max(min(pos, j["max"]), j["min"])
                    val = self.valueToSlider(clamped, j)
                    info["slider"].setValue(val)
                    info["display"].setText(f"{clamped:.3f}")
        else:
            self.jsp.get_logger().warn("No /joint_states received.")

    def move_joint_state(self):
        # 1. Build a JointState msg from current sliders
        js = JointState()
        js.header.stamp = self.jsp.get_clock().now().to_msg()
        js.name = []
        js.position = []

        for name, info in self.joint_map.items():
            js.name.append(name.removeprefix("fake_"))
            js.position.append(info['joint']['position'])

        # 2. Wait for the service, send the request
        if not self.move_client.wait_for_service(timeout_sec=1.0):
            self.jsp.get_logger().error('move_joint_state service not available')
            return

        req = JointPose.Request()
        req.joint_state = js

        # 3. Call async and handle the response
        future = self.move_client.call_async(req)
        future.add_done_callback(self._on_move_response)

    def _on_move_response(self, future):
        try:
            res = future.result()
            if res.success:
                self.jsp.get_logger().info('Move succeeded')
            else:
                self.jsp.get_logger().warn('Move reported failure')
        except Exception as e:
            self.jsp.get_logger().error(f'Move service call failed: {e}')


def main():
    # Initialize rclpy with the command-line arguments
    rclpy.init()

    # Strip off the ROS 2-specific command-line arguments
    stripped_args = rclpy.utilities.remove_ros_args(args=sys.argv)
    parser = argparse.ArgumentParser()
    parser.add_argument("urdf_file", help="URDF file to use", nargs="?", default=None)

    # Parse the remaining arguments, noting that the passed-in args must *not*
    # contain the name of the program.
    parsed_args = parser.parse_args(args=stripped_args[1:])

    app = QApplication(sys.argv)
    jsp = JointStatePublisher(parsed_args.urdf_file)
    gui = JointStatePublisherGui("Joint State Publisher", jsp)
    gui.show()

    executor = MultiThreadedExecutor()
    executor.add_node(jsp)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    signal.signal(signal.SIGINT, signal.SIG_DFL)
    exit_code = app.exec_()

    jsp.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
