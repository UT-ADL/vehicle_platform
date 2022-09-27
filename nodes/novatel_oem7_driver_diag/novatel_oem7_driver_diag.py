#!/usr/bin/env python

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from novatel_oem7_msgs.msg import BESTPOS, INSPVA
import diagnostic_updater
from diagnostic_updater._diagnostic_status_wrapper import DiagnosticStatusWrapper
import diagnostic_msgs


class NovatelDriverDiag:
    updater = None
    bestpos = BESTPOS()  # type: BESTPOS
    inspva = INSPVA()  # type: INSPVA

    def __init__(self):
        self.updater = diagnostic_updater.Updater()
        self.updater.setHardwareID("Novatel_driver")
        self.updater.add("Status", self.diag_task)

        rospy.Subscriber('bestpos', BESTPOS, self.bestpos_callback, queue_size=1)
        rospy.Subscriber('inspva', INSPVA, self.inspva_callback, queue_size=1)

        pass

    def bestpos_callback(self, bestpos):
        self.bestpos = bestpos
        self.updater.update()
        pass

    def inspva_callback(self, inspva):
        self.inspva = inspva
        self.updater.update()
        pass

    def diag_task(self, stat):
        """
        :type stat : DiagnosticStatusWrapper
        :param stat:
        :return:
        """
        stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "OK")

        if self.bestpos.diff_age > 10.:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, "Diff age high")
        if self.bestpos.sol_age > 2.:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, "Sol age high")
        if self.bestpos.sol_status.status != 0: # See https://github.com/novatel/novatel_oem7_driver/blob/master/src/novatel_oem7_msgs/msg/SolutionStatus.msg
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Solution not computed")
        if self.bestpos.pos_type.type not in [56, 55, 40, 34]:  # See https://github.com/novatel/novatel_oem7_driver/blob/master/src/novatel_oem7_msgs/msg/PositionOrVelocityType.msg
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, "RTK not computed")
        if self.bestpos.num_svs <= 5:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, "Satellites tracked low")
        if self.bestpos.num_sol_svs <= 5:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, "Satellites used low")
        if self.bestpos.lon_stdev >= 0.5:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, "Lon stdev (m) high")
        if self.bestpos.lat_stdev >= 0.5:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, "Lat stdev (m) high")
        if self.inspva.status.status != 3:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, "Inertial Solution lost")

        stat.add("RTK diff age", self.bestpos.diff_age)
        stat.add("Solution age", self.bestpos.sol_age)
        stat.add("Solution status", self.bestpos.sol_status.status)
        stat.add("Position or velocity type", self.bestpos.pos_type.type)
        stat.add("Satellites tracked", self.bestpos.num_svs)
        stat.add("Satellites used", self.bestpos.num_sol_svs)
        stat.add("Lon stdev (m)", self.bestpos.lon_stdev)
        stat.add("Lat stdev (m)", self.bestpos.lat_stdev)
        stat.add("Inertial Solution Status", self.inspva.status.status)

        return stat

    @staticmethod
    def run():
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('novatel_oem7_driver_diag', anonymous=True, log_level=rospy.INFO)
    node = NovatelDriverDiag()
    node.run()
