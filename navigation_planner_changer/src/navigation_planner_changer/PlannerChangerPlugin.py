import os
import rospy
import rospkg
import sys

from dynamic_reconfigure.client import Client, DynamicReconfigureCallbackException
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
import PyQt5.QtCore
#from python_qt_binding.QtGui import QLabel, QFont, QHBoxLayout, QVBoxLayout
from python_qt_binding.QtGui import *
from python_qt_binding.QtWidgets import *
from navigation_planner_changer.ROSPlannerGetter import ROSPlannerGetter

class PlannerChangerPlugin(Plugin):

    def __init__(self, context):
        super(PlannerChangerPlugin, self).__init__(context)
        # Give QObjects reasonable names
        #print type(context)
        self.setObjectName('PlannerChangerPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        #parser.add_argument("-q", "--quiet", action="store_true",
        #              dest="quiet",
        #              help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        #if not args.quiet:
        #    print 'arguments: ', args
        #    print 'unknowns: ', unknowns

        self.gui_enable = False
        # Create QWidget
        self._widget = QWidget()
        self._widget.setFixedSize(300,300)
        # Get path to UI file which should be in the "resource" folder of this package
        #ui_file = os.path.join(rospkg.RosPack().get_path('collision_visualizer'), 'resource', 'MyPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        #loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('MyPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        #if context.serial_number() > 1:
        self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        #context.add_widget(self._widget)
        layout = QVBoxLayout(self._widget)
        layout.setAlignment(PyQt5.QtCore.Qt.AlignJustify)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSizeConstraint(2)

        self.planner_getter = ROSPlannerGetter()

        global_list = QComboBox()
        [global_list.addItem(i) for i in self.planner_getter.getGlobalPlanners()]
        global_list.currentTextChanged.connect(self.globalTextCB)
        #list.addItem("option two")
        global_list.move(120,70)
        layout.addWidget(global_list)

        local_list = QComboBox()
        [local_list.addItem(i) for i in self.planner_getter.getLocalPlanners()]
        local_list.currentTextChanged.connect(self.localTextCB)

        #list.addItem("option two")
        local_list.move(130,70)
        layout.addWidget(local_list)

        button = QPushButton("Update Planners")
        button.move(100,70)
        button.clicked.connect(self.handleButton)
        layout.addWidget(button)
        #context.add_widget(self._widget)
        #context.remove_widget(self._widget)

        #text.setText("experiment")
        context.add_widget(self._widget)

        #context.add_widget(gridLayout)

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        self.trigger_configuration()
        self.dyn_client = Client("/navigation/move_base_flex", None)
        self.new_config = dict()
        self.new_config["global_planner"] = self.planner_getter.getGlobalPlanners()[0]
        self.new_config["local_planner"] = self.planner_getter.getLocalPlanners()[0]

    def globalTextCB(self,text):
        self.new_config["global_planner"] = text

    def localTextCB(self,text):
        self.new_config["local_planner"] = text

    def handleButton(self):
        try:
            self.dyn_client.update_configuration(self.new_config)
        except DynamicReconfigureCallbackException:
            rospy.logerr("Something goes wrong")

    def handleCheckBox(self,state):
        self.gui_enable = state

    def shutdown_plugin(self):
        #del self._widget
        sys.exit()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        #print "save ", plugin_settings,instance_settings
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        #print "restore ", plugin_settings.all_keys(),instance_settings
        # v = instance_settings.value(k)
        pass

    def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
        #print "trigger", type(self)
        pass
