import os
import time
import re
import ast

import rospy
import rospkg
import moveit_commander
import moveit_msgs.msg

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, qWarning, Signal
from python_qt_binding.QtGui import QWidget, QGridLayout, QPushButton, QButtonGroup, QVBoxLayout, \
    QFrame, QLineEdit, QTabWidget


class MoveitCommanderWidget(QWidget):
    """
    Provides a UI for Moveit Commander
    - parses rosparam 'robot_description_semantic' to get content of robot srdf
    - there are tabs for each move_group group
    - for each group, there is a set of buttons with the named targets
    - for each group, there is a button to get current joint values and a button to move to specific joint values
    """
    def __init__(self, context):
        super(MoveitCommanderWidget, self).__init__()
        self.setObjectName('MoveitCommanderWidget')

        self.groups = self.get_groups()
        self.commanders = [moveit_commander.MoveGroupCommander(group) for group in self.groups]

        # list of QLineEdit fields used to display joint values
        self.text_joint_values = []

        self.MAX_COLUMNS = 4

        self.tab_widget = QTabWidget()
        for g in self.groups:
            frame = self.setup_group_frame(g)
            self.tab_widget.addTab(frame, g)

        widget_layout = QVBoxLayout()
        widget_layout.addWidget(self.tab_widget)
        self.setLayout(widget_layout)

    def setup_group_frame(self, group):
        layout = QVBoxLayout()

        # grid for buttons for named targets
        grid = QGridLayout()
        grid.setSpacing(1)

        self.button_group = QButtonGroup(self)

        row = 0
        column = 0
        named_targets = self.get_named_targets(group)
        for target in named_targets:
            button = QPushButton(target)
            self.button_group.addButton(button)
            grid.addWidget(button, row, column)
            column += 1
            if column >= self.MAX_COLUMNS:
                row += 1
                column = 0

        self.button_group.buttonClicked.connect(self._handle_button_clicked)

        # grid for show joint value and move arm buttons/text field
        joint_values_grid = QGridLayout()
        joint_values_grid.setSpacing(1)

        button_show_joint_values = QPushButton('Current Joint Values')
        button_show_joint_values.clicked[bool].connect(self._handle_show_joint_values_clicked)

        line_edit = QLineEdit()
        self.text_joint_values.append(line_edit)

        button_move_to = QPushButton('Move to')
        button_move_to.clicked[bool].connect(self._handle_move_to_clicked)

        joint_values_grid.addWidget(button_show_joint_values, 0, 1)
        joint_values_grid.addWidget(line_edit, 0, 2)
        joint_values_grid.addWidget(button_move_to, 0, 3)

        layout.addLayout(grid)
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        layout.addWidget(line)
        layout.addLayout(joint_values_grid)

        frame = QFrame()
        frame.setLayout(layout)
        return frame

    def _handle_button_clicked(self, button):
        commander = self.commanders[self.tab_widget.currentIndex()]
        rospy.loginfo('Moving ' + commander.get_name() + ' to: ' + button.text())
        try:
            commander.set_named_target(str(button.text()))
        except Exception as e:
            rospy.logerr('Unable to set target: %s' % (str(e)))
            return

        error_code = commander.go(wait=True)
        if error_code == moveit_msgs.msg.MoveItErrorCodes.SUCCESS:
            rospy.loginfo("Succeeded")
        else:
            rospy.logerr("Movement failed with error code: %d", error_code)

    def _handle_show_joint_values_clicked(self, checked):
        commander = self.commanders[self.tab_widget.currentIndex()]
        l = commander.get_current_joint_values()
        l = [round(ll, 6) for ll in l]
        self.text_joint_values[self.tab_widget.currentIndex()].setText(str(l))

    def _handle_move_to_clicked(self, checked):
        commander = self.commanders[self.tab_widget.currentIndex()]
        # parse string as list
        l = ast.literal_eval(str(self.text_joint_values[self.tab_widget.currentIndex()].text()))

        if all(isinstance(x, float) for x in l):
            try:
                commander.set_joint_value_target(l)
            except Exception as e:
                rospy.logerr('Unable to set target position: %s' % (str(e)))
                return
            error_code = commander.go(wait=True)
            if error_code == moveit_msgs.msg.MoveItErrorCodes.SUCCESS:
                rospy.loginfo("Succeeded")
            else:
                rospy.logerr("Movement failed with error code: %d", error_code)

        else:
            rospy.logerr("Malformed joint angle list")

    def get_named_targets(self, group_name):
        s = rospy.get_param('/robot_description_semantic')
        rex = "group_state[ \\\\\n\r\f\v\t]*name=\"[a-zA-Z0-9_/\n\r ]*\"[ \\\\\n\r\f\v\t]*group=\"[ \\\\\n\r\f\v\t]*" \
              + group_name + "\""
        p = re.compile(rex)

        # returns a list of 'group_state name="named_target"' entries
        l = p.findall(s)

        pp = re.compile("name=\"[a-zA-Z0-9_/\n\r ]*\"")
        named_targets = [pp.findall(ll)[0][6:-1].strip() for ll in l]

        return named_targets

    def get_groups(self):
        s = rospy.get_param('/robot_description_semantic')
        p = re.compile("group[ \\\\\n\r\f\v\t]*name=\"[a-zA-Z0-9_/\n\r ]*\"")

        # returns a list of 'group name="group_name"' entries
        l = p.findall(s)

        pp = re.compile("name=\"[a-zA-Z0-9_/\n\r ]*\"")
        groups = [pp.findall(ll)[0][6:-1].strip() for ll in l]

        return groups
