import os
import time
import sys
import yaml
import signal

import rospy
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, qWarning, Signal
from python_qt_binding.QtGui import QWidget, QGridLayout, QPushButton, QApplication

from node_events_widget import NodeEventsWidget


class NodeWidgetsContainer(QWidget):
    """
    This widget is a container for multiple NodeEventsWidgets
    in a 2 column grid
    """

    def __init__(self):
        super(NodeWidgetsContainer, self).__init__()

        yaml_file = rospy.get_param("~config_file")

        stream = open(yaml_file, "r")
        nodes = yaml.load(stream)

        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('mcr_event_gui'), 'ros', 'resources', 'NodeWidgetsContainer.ui')
        loadUi(ui_file, self)

        grid = QGridLayout()
        grid.setSpacing(1)

        MAX_COLUMNS = 2

        self.setObjectName('NodeWidgetsContainer')

        self.clear_button = QPushButton("Clear all")
        self.clear_button.clicked[bool].connect(self._handle_clear_clicked)
        self.stop_button = QPushButton("Stop all")
        self.stop_button.clicked[bool].connect(self._handle_stop_clicked)
        grid.addWidget(self.clear_button, 0, 0)
        grid.addWidget(self.stop_button, 0, 1)

        self.widgets = []

        row = 1
        column = 0
        for k in sorted(nodes.keys()):
            node_name = k
            widget = NodeEventsWidget(node_name, nodes[k])
            width = widget.width()
            height = widget.height()
            grid.addWidget(widget, row, column)
            grid.setColumnMinimumWidth(column, width)
            grid.setRowMinimumHeight(row, height)
            self.widgets.append(widget)
            column += 1
            if column >= MAX_COLUMNS:
                row += 1
                column = 0

        self.setLayout(grid)

    def _handle_clear_clicked(self, checked):
        for w in self.widgets:
            w.clear_event_out()

    def _handle_stop_clicked(self, checked):
        for w in self.widgets:
            w.send_stop_event()


def main():
    rospy.init_node('event_gui', anonymous=True)
    app = QApplication(sys.argv)
    event_gui = NodeWidgetsContainer()
    event_gui.show()
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.exit(app.exec_())
