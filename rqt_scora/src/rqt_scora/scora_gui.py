import os
from ament_index_python import get_resource
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtWidgets import QWidget
from rclpy.qos import QoSProfile
from rqt_gui_py.plugin import Plugin
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.node import Node

class Trajectory_publisher(Node):
        def __init__(self):
            super().__init__('trajectory_publsiher_node')
            publish_topic = "/joint_trajectory_controller/joint_trajectory"
            self.trajectory_publihser = self.create_publisher(JointTrajectory, publish_topic, 10)
            timer_period = 1
            self.joints = ['link_1_to_base_link', 'link_2_to_link_1', 'link_3_to_link_2',
                        'link_4_to_link_3', 'gripper_left_to_link_4', 'gripper_right_to_link_4']
            self.goal_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

class RqtScora(Plugin):
    j5 = 0.0
    j6 = 0.0

    def __init__(self, context):
        super(RqtScora, self).__init__(context)
        self.setObjectName('RqtScora')
        self._node = context.node
        self._publisher = None
        self._widget = QWidget()
        _, package_path = get_resource('packages', 'rqt_scora')
        ui_file = os.path.join(package_path, 'share', 'rqt_scora', 'resource', 'RqtScora.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('RqtScoraUi')
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)
    
        #Button clear
        self._widget.homeButton.pressed.connect(self._on_home_btn)
        #Button Grasp
        self._widget.GraspButton.pressed.connect(self._on_grasp_btn)
        #Button Relase
        self._widget.ReleaseButton.pressed.connect(self._on_relase_btn)
        #Slider joint 1
        self._widget.j1_slider.valueChanged.connect(self._on_j1_slider_changed)
        self._widget.j1_slider.setMaximum(2500)
        self._widget.j1_slider.setMinimum(-2500)
        #Slider joint 2
        self._widget.j2_slider.valueChanged.connect(self._on_j2_slider_changed)
        self._widget.j2_slider.setMaximum(2000)
        self._widget.j2_slider.setMinimum(-2000)
        #Slider joint 3
        self._widget.j3_slider.valueChanged.connect(self._on_j3_slider_changed)
        self._widget.j3_slider.setMaximum(2500)
        self._widget.j3_slider.setMinimum(-2500)
        #Slider joint 4
        self._widget.j4_slider.valueChanged.connect(self._on_j4_slider_changed)
        self._widget.j4_slider.setMaximum(2000)
        self._widget.j4_slider.setMinimum(-2000)

        self._update_parameter_timer = QTimer(self)
        self._update_parameter_timer.timeout.connect(self._on_parameter_changed)
        self._update_parameter_timer.start(100)
        self.zero_cmd_sent = False

        self._publisher = self._node.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', qos_profile=QoSProfile(depth=10))

    def _on_grasp_btn(self):
        RqtScora.j5=0.03
        RqtScora.j6=0.03
        self._on_parameter_changed()

    def _on_relase_btn(self):
        RqtScora.j5=0.0
        RqtScora.j6=0.0
        self._on_parameter_changed()

    def _on_home_btn(self):
        self._widget.j1_slider.setValue(0)
        self._widget.j2_slider.setValue(0)
        self._widget.j3_slider.setValue(0)
        self._widget.j4_slider.setValue(0)
    
    def _on_j1_slider_changed(self):
        self._widget.current_j1_label.setText(
            '%0.2f rad' % (self._widget.j1_slider.value() / 1000))
        self._on_parameter_changed()

    def _on_j2_slider_changed(self):
        self._widget.current_j2_label.setText(
            '%0.2f rad' % (self._widget.j2_slider.value() / 1000))
        self._on_parameter_changed()

    def _on_j3_slider_changed(self):
        self._widget.current_j3_label.setText(
            '%0.2f rad' % (self._widget.j3_slider.value() / 1000))
        self._on_parameter_changed()

    def _on_j4_slider_changed(self):
        self._widget.current_j4_label.setText(
            '%0.2f rad' % (self._widget.j4_slider.value() / 1000))
        self._on_parameter_changed()

    def _on_parameter_changed(self):
        self._send_trajectory(
            self._widget.j1_slider.value() / 1000 ,
            self._widget.j2_slider.value() / 1000 ,
            self._widget.j3_slider.value() / 1000 ,
            self._widget.j4_slider.value() / 1000 ,
            RqtScora.j5,
            RqtScora.j6)

    def _send_trajectory(self, j1, j2, j3, j4, j5 ,j6):
        scora_trajectory = Trajectory_publisher()
        scora_trajectory.goal_positions = [j1, j2, j3, j4, j5, j6]
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = scora_trajectory.joints
        point = JointTrajectoryPoint()
        point.positions = scora_trajectory.goal_positions
        point.time_from_start = Duration(sec=1)
        trajectory_msg.points.append(point)
        scora_trajectory.trajectory_publihser.publish(trajectory_msg)