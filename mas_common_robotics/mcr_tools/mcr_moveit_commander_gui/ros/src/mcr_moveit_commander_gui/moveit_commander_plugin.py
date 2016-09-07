import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from moveit_commander_widget import MoveitCommanderWidget

import yaml


class MoveitCommanderPlugin(Plugin):

    def __init__(self, context):
        super(MoveitCommanderPlugin, self).__init__(context)
        self.setObjectName('MoveitCommanderPlugin')

        self._widget = MoveitCommanderWidget(context)
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('mcr_moveit_commander_gui'), 'ros', 'resources', 'MoveitCommanderPlugin.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('MoveitCommanderPluginUi')
