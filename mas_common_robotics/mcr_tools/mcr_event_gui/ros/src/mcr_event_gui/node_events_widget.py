import os
import time

import rospy
import rospkg

import std_msgs.msg

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, qWarning, Signal
from python_qt_binding.QtGui import QGraphicsView, QWidget, QSizePolicy


class NodeEventsGraphicsView(QGraphicsView):
    def __init__(self, parent=None):
        super(NodeEventsGraphicsView, self).__init__()


class NodeEventsWidget(QWidget):
    """
    This widget contains buttons for publishing to event_in and event_out for a node.
    It also subscribes and displays event_in and event_out for the node.
    event_in string is expected to be any of 'e_start', 'e_stop' or 'e_trigger'
    """
    def __init__(self, node_name, topic_dict):
        super(NodeEventsWidget, self).__init__()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('mcr_event_gui'), 'ros', 'resources', 'NodeEventsWidget.ui')
        loadUi(ui_file, self, {'NodeEventsGraphicsView': NodeEventsGraphicsView})

        self.setObjectName('NodeEventsWidget')

        self.event_out_sub = rospy.Subscriber(topic_dict['event_out_topic'], std_msgs.msg.String, self.event_out_cb)
        self.event_out_pub = rospy.Publisher(topic_dict['event_out_topic'], std_msgs.msg.String, queue_size=1)
        self.event_in_pub = rospy.Publisher(topic_dict['event_in_topic'], std_msgs.msg.String, queue_size=1)
        self.event_in_sub = rospy.Subscriber(topic_dict['event_in_topic'], std_msgs.msg.String, self.event_in_cb)

        self.label_node_name.setText(node_name)
        self.label_node_name.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.label_node_name.setAlignment(Qt.AlignCenter)

        self.label_event_out.setToolTip(topic_dict['event_out_topic'])
        self.label_event_in.setToolTip(topic_dict['event_in_topic'])

        self.button_e_start.clicked[bool].connect(self._handle_e_start_clicked)
        self.button_e_trigger.clicked[bool].connect(self._handle_e_trigger_clicked)
        self.button_e_stop.clicked[bool].connect(self._handle_e_stop_clicked)
        self.button_event_out.clicked[bool].connect(self._handle_event_out_clicked)

        if "e_start" not in topic_dict['event_in_strings']:
            self.button_e_start.setEnabled(False)

        if "e_trigger" not in topic_dict['event_in_strings']:
            self.button_e_trigger.setEnabled(False)

        if "e_stop" not in topic_dict['event_in_strings']:
            self.button_e_stop.setEnabled(False)

    def handle_close(self, event):
        self.shutdown_all()
        self.event_in_pub.unregister()
        self.event_out_sub.unregister()

    def _handle_e_start_clicked(self, checked):
        self.event_in_pub.publish("e_start")

    def _handle_e_trigger_clicked(self, checked):
        self.event_in_pub.publish("e_trigger")

    def _handle_e_stop_clicked(self, checked):
        self.event_in_pub.publish("e_stop")

    def _handle_clear_clicked(self, checked):
        self.clear_event_out()

    def _handle_event_out_clicked(self, checked):
        self.event_out_pub.publish(self.text_event_out.text())

    def event_out_cb(self, msg):
        self.text_event_out_show.setText(msg.data)

    def event_in_cb(self, msg):
        self.text_event_in.setText(msg.data)

    def clear_event_out(self):
        self.text_event_out_show.setText("")
        self.text_event_in.setText("")

    def send_stop_event(self):
        self.event_in_pub.publish("e_stop")
