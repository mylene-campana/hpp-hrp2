#!/usr/bin/env python
# Copyright (c) 2014 CNRS
# Author: Florent Lamiraux
#
# This file is part of hpp-hrp2.
# hpp-hrp2 is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-hrp2 is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-hrp2.  If not, see
# <http://www.gnu.org/licenses/>.

from hpp.corbaserver.manipulation.robot import Robot as Parent

class Robot (Parent):
    packageName = "hrp2_14_description"
    urdfName = "hrp2_14"
    urdfSuffix = ""
    srdfSuffix = ""
    rootJointType = "freeflyer"
    halfSitting = \
        {"base_joint_xyz": (0.0, 0.0, 0.648702),
         "base_joint_SO3": (1.0, 0.0, 0.0, 0.0),
         "CHEST_JOINT0": 0.0,
         "CHEST_JOINT1": 0.0,
         "HEAD_JOINT0": 0.0,
         "HEAD_JOINT1": 0.0,
         "LARM_JOINT0": 0.261799,
         "LARM_JOINT1": 0.17453,
         "LARM_JOINT2": 0.0,
         "LARM_JOINT3": -0.523599,
         "LARM_JOINT4": 0.0,
         "LARM_JOINT5": 0.0,
         "LARM_JOINT6": 0.1,
         "LHAND_JOINT0": 0.0,
         "LHAND_JOINT1": 0.0,
         "LHAND_JOINT2": 0.0,
         "LHAND_JOINT3": 0.0,
         "LHAND_JOINT4": 0.0,
         "RARM_JOINT0": 0.261799,
         "RARM_JOINT1": -0.17453,
         "RARM_JOINT2": 0.0,
         "RARM_JOINT3": -0.523599,
         "RARM_JOINT4": 0.0,
         "RARM_JOINT5": 0.0,
         "RARM_JOINT6": 0.1,
         "RHAND_JOINT0": 0.0,
         "RHAND_JOINT1": 0.0,
         "RHAND_JOINT2": 0.0,
         "RHAND_JOINT3": 0.0,
         "RHAND_JOINT4": 0.0,
         "LLEG_JOINT0": 0.0,
         "LLEG_JOINT1": 0.0,
         "LLEG_JOINT2": -0.453786,
         "LLEG_JOINT3": 0.872665,
         "LLEG_JOINT4": -0.418879,
         "LLEG_JOINT5": 0.0,
         "RLEG_JOINT0": 0.0,
         "RLEG_JOINT1": 0.0,
         "RLEG_JOINT2": -0.453786,
         "RLEG_JOINT3": 0.872665,
         "RLEG_JOINT4": -0.418879,
         "RLEG_JOINT5": 0.0
         }

    ## Constructor
    # \param compositeName name of the composite robot that will be built later,
    # \param robotName name of the first robot that is loaded now,
    def __init__ (self, compositeName, robotName):
        Parent.__init__ (self, compositeName, robotName, self.rootJointType)
        hs = self.halfSitting.copy ()
        for joint, value in hs.iteritems ():
            self.halfSitting [self.displayName + "/" + joint] = value
        self.rightWrist = self.displayName + "/RARM_JOINT5"
        self.leftWrist  = self.displayName + "/LARM_JOINT5"
        self.rightAnkle = self.displayName + "/RLEG_JOINT5"
        self.leftAnkle  = self.displayName + "/LLEG_JOINT5"
        self.waist = self.displayName + "/base_joint_SO3"
        self.chest = self.displayName + "/CHEST_JOINT1"
        self.gaze = self.displayName + "/HEAD_JOINT1"

    ## Virtual function to load the robot model
    def loadModel (self, robotName, rootJointType):
        self.loadHumanoidModel \
            (robotName, rootJointType, self.packageName, self.urdfName,
             self.urdfSuffix, self.srdfSuffix)

    def getInitialConfig (self):
        q = []
        for n in self.jointNames:
            if self.halfSitting.has_key (n):
                dof = self.halfSitting [n]
                if type (dof) is tuple:
                    q += dof
                else:
                    q.append (dof)
            else:
                size = self.client.basic.robot.getJointConfigSize (n)
                q += size * [None,]
        return q

    def leftHandClosed (self) :
        dofs = {self.displayName + "/LARM_JOINT6": 0.1,
                self.displayName + "/LHAND_JOINT0": 0.0,
                self.displayName + "/LHAND_JOINT1": 0.0,
                self.displayName + "/LHAND_JOINT2": 0.0,
                self.displayName + "/LHAND_JOINT3": 0.0,
                self.displayName + "/LHAND_JOINT4": 0.0}
        return dofs

    def rightHandClosed (self) :
        dofs = {self.displayName + "/RARM_JOINT6": 0.1,
                self.displayName + "/RHAND_JOINT0": 0.0,
                self.displayName + "/RHAND_JOINT1": 0.0,
                self.displayName + "/RHAND_JOINT2": 0.0,
                self.displayName + "/RHAND_JOINT3": 0.0,
                self.displayName + "/RHAND_JOINT4": 0.0}
        return dofs

    def leftHandOpened (self) :
        dofs = {self.displayName + "/LARM_JOINT6": 0.75,
                self.displayName + "/LHAND_JOINT0": -0.75,
                self.displayName + "/LHAND_JOINT1": 0.75,
                self.displayName + "/LHAND_JOINT2": -0.75,
                self.displayName + "/LHAND_JOINT3": 0.75,
                self.displayName + "/LHAND_JOINT4": -0.75}
        return dofs

    def rightHandOpened (self) :
        dofs = {self.displayName + "/RARM_JOINT6": 0.75,
                self.displayName + "/RHAND_JOINT0": -0.75,
                self.displayName + "/RHAND_JOINT1": 0.75,
                self.displayName + "/RHAND_JOINT2": -0.75,
                self.displayName + "/RHAND_JOINT3": 0.75,
                self.displayName + "/RHAND_JOINT4": -0.75}
        return dofs
