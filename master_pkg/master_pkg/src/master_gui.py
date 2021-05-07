#!/usr/bin/env python

from PyQt5 import QtWidgets, uic
from PyQt5.QtWidgets import QFileDialog,QMainWindow,QMessageBox,QAction
from PyQt5.QtCore import QTimer, QTime, Qt
from threading import Thread
import sys
import time
import rospy


from Master import Master

class MasterUi(QMainWindow):

    def __init__(self):
        # Call the inherited classes __init__ method
        super(MasterUi, self).__init__()

    def setupUI(self):

        # loads the ui file
        uic.loadUi('resource/master_gui.ui', self)

        # creates the master node to control start and test ID
        self.master = Master()

        # connects the button events to the handle methods
        self.button_start.clicked[bool].connect(self._handle_start_clicked)
        self.button_test_ID.clicked[bool].connect(self._handle_test_ID_clicked)

        # keeps disabled until the test IDs are setted
        self.enable_button(self.button_start, False, "lightgrey", "START")

        # creating the timer objects
        self.timer_acquisition = QTimer(self)

        # adding action to timers
        self.timer_acquisition.timeout.connect(self._show_time_acquisition)

        # connets the method to handle the close event
        quit = QAction("Quit", self)
        quit.triggered.connect(self.close)

    def _handle_start_clicked(self, checked):
        """
        Handles the ACQUIRE button events.
        """
        if checked:
            # keeps checked the START button
            self.button_start.setChecked(True)
            self.enable_button(self.button_test_ID, False, "lightgrey")
            self.enable_button(self.button_start, True, "red", "STOP")

            self.time_start = rospy.Time.now().to_sec()
            # update the timer every 10 msec
            self.timer_acquisition.start(10)

            # starts the acquisition
            self.master.start(True)

            # disables spin boxes
            self.spinBox_num_subject.setEnabled(False)
            self.spinBox_num_cond.setEnabled(False)
            self.spinBox_num_run.setEnabled(False)
            self.button_test_ID.setEnabled(False)

        else:
            # releases the LAUNCH_NODES button
            self.button_start.setChecked(False)
            self.enable_button(self.button_start, False, "lightgrey", "START")
            self.enable_button(self.button_test_ID, True, "blue")

            # stops the timer
            self.timer_acquisition.stop()

            # stops the acquisition
            # the acquire flags are changed by the master method
            self.master.start(False)

            # enables the spin boxes
            self.spinBox_num_subject.setEnabled(True)
            self.spinBox_num_cond.setEnabled(True)
            self.spinBox_num_run.setEnabled(True)
            self.enable_button(self.button_start, True, "lightgrey")

            # increments the run number for the next acquisition
            self.spinBox_num_run.setValue(self.spinBox_num_run.value() + 1)

    def _handle_test_ID_clicked(self):

        self.enable_button(self.button_start, True, "green", "START")

        # reads the current numSubject, numCondition and numRun
        n_sub  = self.spinBox_num_subject.value()
        n_cond = self.spinBox_num_cond.value()
        n_run  = self.spinBox_num_run.value()

        # updates the current ID test
        self.master.set_test_ID(n_sub,n_cond,n_run)

    def _show_time_acquisition(self):
        # method called by acquisition timer
        # showing acquisition time to the label
        self.label_time_acquisition.setText("Time: {0:.3f} [s]".format(rospy.Time.now().to_sec() - self.time_start))

    def closeEvent(self, event):
        """
        Handles the close event.
        """
        close = QMessageBox.question(self,"Quit","Are you sure want to stop process?", QMessageBox.Yes | QMessageBox.No)
        if close == QMessageBox.Yes:
            # sends the command to stop the acquisition
            self.master.start(False)

            #closes
            event.accept()
        else:
            #cancels
            event.ignore()

    def enable_button(self, button, enabled, color, text=''):
        if text=='':
            text = button.text()

        button.setText(text)
        button.setEnabled(enabled)
        button.setStyleSheet("color: " + color)

def closing_hook():
    '''Closing hook when node has been shutdown '''
    rospy.logwarn("Master GUI closed")

def main():

    app = QtWidgets.QApplication(sys.argv)
    window = MasterUi()
    window.setupUI()
    window.show()

    rospy.on_shutdown(closing_hook)
    sys.exit(app.exec_())

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
