# SimpleRobotSteeringPlugin plugin based on rqt_robot_steering.
# Original copyright notice below.
#
# Copyright (c) 2011, Dirk Thomas, TU Darmstadt
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the TU Darmstadt nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import division

from functools import partial
import math
import os
import rospkg
import rospy
import time

from python_qt_binding import loadUi
from python_qt_binding.QtCore import QCoreApplication, QEvent, QObject, Qt, QTimer, Signal, Slot
from python_qt_binding.QtGui import QFont
from python_qt_binding.QtWidgets import QDialog, QGridLayout, QLabel, QLineEdit, \
                                        QPushButton, QTextBrowser, QTextEdit, QVBoxLayout, QWidget
from qt_gui.plugin import Plugin

from bwi_msgs.srv import QuestionDialog, QuestionDialogResponse, QuestionDialogRequest
from geometry_msgs.msg import Twist


class QuestionDialogPlugin(Plugin):

    def __init__(self, context):
        super(QuestionDialogPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('QuestionDialogPlugin')

        font_size = rospy.get_param("~font_size", 40)

        # Create QWidget
        self.widget = QWidget()
        self.widget.setFont(QFont("Times", font_size, QFont.Bold))
        self._layout = QVBoxLayout(self.widget)
        self._text_browser = QTextBrowser(self.widget)
        self._layout.addWidget(self._text_browser)
        self._button_layout = QGridLayout()
        self._layout.addLayout(self._button_layout)

        # layout = QVBoxLayout(self.widget)
        # layout.addWidget(self.button)
        self.widget.setObjectName('QuestionDialogPluginUI')
        if context.serial_number() > 1:
            self.widget.setWindowTitle(self.widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self.widget)

        # Setup service provider
        self.service = rospy.Service('question_dialog', QuestionDialog,
                                     self.service_callback)
        self.response_ready = False
        self.response = None
        self.buttons = []
        self.text_label = None
        self.text_input = None

        self.connect(self.widget, Signal("update"), self.update)
        self.connect(self.widget, Signal("timeout"), self.timeout)

    def shutdown_plugin(self):
        self.service.shutdown()

    def service_callback(self, req):
        self.response_ready = False
        self.request = req
        self.widget.emit(Signal("update"))
        # Start timer against wall clock here instead of the ros clock.
        start_time = time.time()
        while not self.response_ready:
            if self.request != req:
                # The request got preempted by a new request.
                return QuestionDialogResponse(QuestionDialogRequest.PREEMPTED, "")
            if req.timeout != QuestionDialogRequest.NO_TIMEOUT:
                current_time = time.time()
                if current_time - start_time > req.timeout:
                    self.widget.emit(Signal("timeout"))
                    return QuestionDialogResponse(
                            QuestionDialogRequest.TIMED_OUT, "")
            time.sleep(0.2)
        return self.response

    def update(self):
        self.clean()
        req = self.request
        self._text_browser.setText(req.message)
        if req.type == QuestionDialogRequest.DISPLAY:
            # All done, nothing more too see here.
            self.response = QuestionDialogResponse(
                    QuestionDialogRequest.NO_RESPONSE, "")
            self.response_ready = True
        elif req.type == QuestionDialogRequest.CHOICE_QUESTION:
            for index, options in enumerate(req.options):
                button = QPushButton(options, self.widget)
                button.clicked.connect(partial(self.handle_button, index))
                row = index / 3
                col = index % 3
                self._button_layout.addWidget(button, row, col)
                self.buttons.append(button)
        elif req.type == QuestionDialogRequest.TEXT_QUESTION:
            self.text_label = QLabel("Enter here: ", self.widget)
            self._button_layout.addWidget(self.text_label, 0, 0)
            self.text_input = QLineEdit(self.widget)
            self.text_input.editingFinished.connect(self.handle_text)
            self._button_layout.addWidget(self.text_input, 0, 1)

    def timeout(self):
        self._text_browser.setText("Oh no! The request timed out.")
        self.clean()

    def clean(self):
        while self._button_layout.count():
            item = self._button_layout.takeAt(0)
            item.widget().deleteLater()
        self.buttons = []
        self.text_input = None
        self.text_label = None

    def handle_button(self, index):
        self.response = QuestionDialogResponse(index, "")
        self.clean()
        self.response_ready = True

    def handle_text(self):
        self.response = QuestionDialogResponse(
            QuestionDialogRequest.TEXT_RESPONSE,
            self.text_input.text())
        self.clean()
        self.response_ready = True

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    # def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog


class KeyEventFilter(QObject):

    def __init__(self, context):
        super(KeyEventFilter, self).__init__(context)
        self.parent = context

    def eventFilter(self, receiver, event):

        # If the currently focused widget is a QLineEdit or a QTextEdit, don't capture.
        focusedWidget = QCoreApplication.instance().focusWidget()
        if (isinstance(focusedWidget, QLineEdit) or isinstance(focusedWidget, QTextEdit)):
            return super(KeyEventFilter, self).eventFilter(receiver, event)

        # If keyboard is diabled, don't capture.
        try:
            if self.parent.widget.disable_keyboard_button.isChecked():
                return super(KeyEventFilter, self).eventFilter(receiver, event)
        except RuntimeError:
            # This error happens when the widget is being destroyed.
            pass

        if event.type() == QEvent.KeyPress and not event.isAutoRepeat():
            if event.key() == Qt.Key_W:
                self.parent.widget.w_button.setChecked(True)
                return True
            elif event.key() == Qt.Key_S:
                self.parent.widget.s_button.setChecked(True)
                return True
            elif event.key() == Qt.Key_A:
                self.parent.widget.a_button.setChecked(True)
                return True
            elif event.key() == Qt.Key_D:
                self.parent.widget.d_button.setChecked(True)
                return True
            elif event.key() == Qt.Key_Q:
                self.parent.widget.q_button.setChecked(True)
                return True
            elif event.key() == Qt.Key_E:
                self.parent.widget.e_button.setChecked(True)
                return True

        if event.type() == QEvent.KeyRelease and not event.isAutoRepeat():
            if event.key() == Qt.Key_W:
                self.parent.widget.w_button.setChecked(False)
                return True
            elif event.key() == Qt.Key_S:
                self.parent.widget.s_button.setChecked(False)
                return True
            elif event.key() == Qt.Key_A:
                self.parent.widget.a_button.setChecked(False)
                return True
            elif event.key() == Qt.Key_D:
                self.parent.widget.d_button.setChecked(False)
                return True
            elif event.key() == Qt.Key_Q:
                self.parent.widget.q_button.setChecked(False)
                return True
            elif event.key() == Qt.Key_E:
                self.parent.widget.e_button.setChecked(False)
                return True

        return super(KeyEventFilter, self).eventFilter(receiver, event)


class SettingsDialog(QDialog):

    def __init__(self):
        super(SettingsDialog, self).__init__()
        self.setWindowTitle('Options')

        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('bwi_rqt_plugins'), 'resources', 'SimpleRobotSteeringSettings.ui')
        loadUi(ui_file, self)


class SimpleRobotSteeringPlugin(Plugin):

    DEFAULT_LINEAR_X_VELOCITY = 1.0
    DEFAULT_LINEAR_Y_VELOCITY = 0.5
    DEFAULT_ANGULAR_VELOCITY = math.pi / 2

    def __init__(self, context):
        super(SimpleRobotSteeringPlugin, self).__init__(context)
        self.setObjectName('SimpleRobotSteeringPlugin')

        self.publisher = None

        self.widget = QWidget()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('bwi_rqt_plugins'), 'resources', 'SimpleRobotSteering.ui')
        loadUi(ui_file, self.widget)
        self.widget.setObjectName('SimpleRobotSteeringUI')
        if context.serial_number() > 1:
            self.widget.setWindowTitle(self.widget.windowTitle() + (' (%d)' % context.serial_number()))

        # Add Key Capture and disable functionality.
        self.disable_capture = False
        self.key_event_filter = KeyEventFilter(self)
        QCoreApplication.instance().installEventFilter(self.key_event_filter)
        context.add_widget(self.widget)

        self.widget.topic_line_edit.textChanged.connect(self.on_topic_changed)

        self.target_linear_x_vel = SimpleRobotSteeringPlugin.DEFAULT_LINEAR_X_VELOCITY
        self.target_linear_y_vel = SimpleRobotSteeringPlugin.DEFAULT_LINEAR_Y_VELOCITY
        self.target_angular_vel = SimpleRobotSteeringPlugin.DEFAULT_ANGULAR_VELOCITY

        self.linear_x_vel = 0
        self.linear_y_vel = 0
        self.angular_vel = 0

        # After doing so, key press events seem to work ok.
        # self.widget.w_button.setFocus()

        # timer to consecutively send twist messages.
        self.update_parameter_timer = QTimer(self)
        self.update_parameter_timer.timeout.connect(self.on_parameter_changed)
        self.update_parameter_timer.start(100)

        self.widget.w_button.toggled.connect(self.toggle_w_button)
        self.widget.s_button.toggled.connect(self.toggle_s_button)
        self.widget.a_button.toggled.connect(self.toggle_a_button)
        self.widget.d_button.toggled.connect(self.toggle_d_button)
        self.widget.q_button.toggled.connect(self.toggle_q_button)
        self.widget.e_button.toggled.connect(self.toggle_e_button)

        # Setup dialog to change settings.
        self.dialog = SettingsDialog()
        self.dialog.accepted.connect(self.update_target_vel)
        self.dialog.rejected.connect(partial(self.widget.settings_button.setChecked, False))
        self.widget.settings_button.clicked.connect(self.exec_dialog)

    @Slot(str)
    def on_topic_changed(self, topic):
        topic = str(topic)
        self.unregister_publisher()
        try:
            self.publisher = rospy.Publisher(topic, Twist, queue_size=1)
        except TypeError:
            self.publisher = rospy.Publisher(topic, Twist)

    def unregister_publisher(self):
        if self.publisher is not None:
            self.publisher.unregister()
            self.publisher = None

    def toggle_w_button(self, bool):
        if bool:
            self.linear_x_vel = self.target_linear_x_vel
            self.widget.s_button.setChecked(False)
        else:
            self.linear_x_vel = 0 if self.linear_x_vel > 0 else self.linear_x_vel

    def toggle_s_button(self, bool):
        if bool:
            self.linear_x_vel = -self.target_linear_x_vel
            self.widget.w_button.setChecked(False)
        else:
            self.linear_x_vel = 0 if self.linear_x_vel < 0 else self.linear_x_vel

    def toggle_a_button(self, bool):
        if bool:
            self.linear_y_vel = self.target_linear_y_vel
            self.widget.d_button.setChecked(False)
        else:
            self.linear_y_vel = 0 if self.linear_y_vel > 0 else self.linear_y_vel

    def toggle_d_button(self, bool):
        if bool:
            self.linear_y_vel = -self.target_linear_y_vel
            self.widget.a_button.setChecked(False)
        else:
            self.linear_y_vel = 0 if self.linear_y_vel < 0 else self.linear_y_vel

    def toggle_q_button(self, bool):
        if bool:
            self.angular_vel = self.target_angular_vel
            self.widget.e_button.setChecked(False)
        else:
            self.angular_vel = 0 if self.angular_vel > 0 else self.angular_vel

    def toggle_e_button(self, bool):
        if bool:
            self.angular_vel = -self.target_angular_vel
            self.widget.q_button.setChecked(False)
        else:
            self.angular_vel = 0 if self.angular_vel < 0 else self.angular_vel

    def on_parameter_changed(self):
        self.widget.linear_x_vel.setText("%.2f" % self.linear_x_vel)
        self.widget.linear_y_vel.setText("%.2f" % self.linear_y_vel)
        self.widget.angular_vel.setText("%.2f" % self.angular_vel)
        self.send_twist(self.linear_x_vel, self.linear_y_vel, self.angular_vel)

    def send_twist(self, x_linear, y_linear, z_angular):
        if self.publisher is None:
            return
        twist = Twist()
        twist.linear.x = x_linear
        twist.linear.y = y_linear
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = z_angular

        self.publisher.publish(twist)

    def exec_dialog(self):
        self.dialog.target_linear_x_vel.setValue(self.target_linear_x_vel)
        self.dialog.target_linear_y_vel.setValue(self.target_linear_y_vel)
        self.dialog.target_angular_vel.setValue(self.target_angular_vel)
        self.dialog.exec_()

    def update_target_vel(self):
        self.target_linear_x_vel = self.dialog.target_linear_x_vel.value()
        self.target_linear_y_vel = self.dialog.target_linear_y_vel.value()
        self.target_angular_vel = self.dialog.target_angular_vel.value()
        self.widget.settings_button.setChecked(False)

    def shutdown_plugin(self):
        self.update_parameter_timer.stop()
        self.unregister_publisher()

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value('topic', self.widget.topic_line_edit.text())
        instance_settings.set_value('target_linear_x_vel', self.target_linear_x_vel)
        instance_settings.set_value('target_linear_y_vel', self.target_linear_y_vel)
        instance_settings.set_value('target_angular_vel', self.target_angular_vel)

    def restore_settings(self, plugin_settings, instance_settings):
        topic = instance_settings.value('topic', "/cmd_vel")
        topic = rospy.get_param("~default_topic", topic)
        self.widget.topic_line_edit.setText(topic)

        self.target_linear_x_vel = instance_settings.value('target_linear_x_vel',
                                                           SimpleRobotSteeringPlugin.DEFAULT_LINEAR_X_VELOCITY)
        self.target_linear_x_vel = float(self.target_linear_x_vel)
        self.target_linear_x_vel = rospy.get_param("~default_target_x_vel", self.target_linear_x_vel)

        self.target_linear_y_vel = instance_settings.value('target_linear_y_vel',
                                                           SimpleRobotSteeringPlugin.DEFAULT_LINEAR_Y_VELOCITY)
        self.target_linear_y_vel = rospy.get_param("~default_target_y_vel", self.target_linear_y_vel)
        self.target_linear_y_vel = float(self.target_linear_y_vel)

        self.target_angular_vel = instance_settings.value('target_angular_vel',
                                                          SimpleRobotSteeringPlugin.DEFAULT_ANGULAR_VELOCITY)
        self.target_angular_vel = rospy.get_param("~default_target_x_vel", self.target_angular_vel)
        self.target_angular_vel = float(self.target_angular_vel)
