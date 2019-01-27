import time
import openravepy
import numpy as np
import h5py
from pyquaternion import Quaternion as pyQuat

if not __openravepy_build_doc__:
    from openravepy import *  # matrixFromQuat included
    from numpy import *


class x_arm:
    currentJointError = np.zeros([7])  # joint1 -- joint7

    graspOrientation = np.zeros([4])  # quaternion(w,x,y,z)
    graspJointsAngle = np.zeros([7])  # joint1 -- joint7
    preGraspJointsAngle = np.zeros([7])  # joint1 -- joint7

    def __init__(self,
                 robotPath="/home/liquanlin/workspace/catkin_ws/src/x_arm/robots",
                 robotName="x_arm",
                 robotManipName="x_arm",
                 calibRobotName="x_arm_calib",
                 calibRobotManipName="x_arm_calib",
                 reachabilityFilePath="/home/liquanlin/workspace/python/pynb"):

        self.env = Environment()
        # robot ikmodel
        robotFile = robotPath + "/" + robotName + ".xml"
        self.env.Load(robotFile)
        self.robot = self.env.GetRobots()[0]
        self.robotManip = self.robot.SetActiveManipulator(robotManipName)
        self.robotIkmodel = databases.inversekinematics.InverseKinematicsModel(self.robot,
                                                                               iktype=IkParameterization.Type.Transform6D)
        if not self.robotIkmodel.load():
            self.robotIkmodel.autogenerate()

        # robot calib ikmodel
        robotCalibFile = robotPath + "/" + calibRobotName + ".xml"
        self.env.Load(robotCalibFile)
        self.robotCalib = self.env.GetRobots()[1]
        self.robotCalibManip = self.robotCalib.SetActiveManipulator(calibRobotManipName)
        self.robotCalibIkmodel = databases.inversekinematics.InverseKinematicsModel(self.robotCalib,
                                                                                    iktype=IkParameterization.Type.Transform6D)
        if not self.robotCalibIkmodel.load():
            self.robotCalibIkmodel.autogenerate()

        self.newrobot = RaveCreateRobot(self.env, self.robot.GetXMLId())
        self.newrobot.Clone(self.robot, 0)
        for link in self.newrobot.GetLinks():
            for geom in link.GetGeometries():
                geom.SetTransparency(0.7)
        self.env.Add(self.newrobot, True)
        self.newrobot.SetTransform(self.robot.GetTransform())
        self.newrobot.SetDOFValues([0, 0, 0, 0, 0, 0, 0], self.robotIkmodel.manip.GetArmIndices())

        hdf5File = reachabilityFilePath + "/pose-1364698_576_4.h5"
        h5f = h5py.File(hdf5File, 'r')
        self.poseSolutionsIndexInSerialization = h5f['poseSolutionsIndexInSerialization']
        self.reachabilitySet = h5f['reachabilitySet']
        self.center = h5f['center']

        self.jointsUpLimit = np.array([120.0, 120.0, 90.0, 115.0, 110.0, 75.0, 20.0]) / 180.0 * 3.1415926
        self.jointsDownLimit = np.array([-30.0, -20.0, -90.0, -10.0, -120.0, -25.0, -20.0]) / 180.0 * 3.1415926
        self.jointsCenter = (self.jointsUpLimit + self.jointsDownLimit) / 2.0
        self.jointsRange = (self.jointsUpLimit - self.jointsDownLimit) / 2.0

        print "initialization completed"

    def grapsPoseGen(self, targetPosition, objectOrientation):  # position: x,y,z + objectOrientation: w,x,y,z

        planedOrientation = np.zeros([4])

        positionInDatabase = (np.round(targetPosition, 2) * 100 + self.center).astype(int)
        [numSolution, solutionStartIndex] = self.poseSolutionsIndexInSerialization[positionInDatabase[0],
                                            positionInDatabase[1],
                                            positionInDatabase[2], :]

        # if grasp object with orientation, the y axis of object aligned with major axis of the object
        if (np.abs(np.linalg.norm(objectOrientation) - 1) < 1e-1):

            # turn around the y axis of the object to select the best orientation for grasp
            step = 1  # degree
            angleLoss = 10000
            for i in range(0, 360, step):
                with self.env:
                    while True:
                        # if not self.robot.CheckSelfCollision():
                        if True:
                            T = matrixFromQuat(quatMult(objectOrientation, quatFromAxisAngle([0, 1, 0],
                                                                                             i / 180.0 * 3.14)))  # matrixFromQuat(np.array([w,x,y,z]))
                            T[0:3, 3] = targetPosition
                            # solutions = self.robotIkmodel.manip.FindIKSolutions(T,IkFilterOptions.CheckEnvCollisions)
                            solutions = self.robotIkmodel.manip.FindIKSolutions(T, 0)

                            if solutions is not None and len(solutions) > 0:
                                for j in range(len(solutions)):
                                    tempLoss = np.array([1, 1, 1, 1, 1, 1, 1]).dot(
                                        np.abs((solutions[j, :] - self.jointsCenter) / self.jointsRange))
                                    if (np.sum(solutions[j, :] > self.jointsUpLimit) == 0 and
                                            np.sum(solutions[j, :] < self.jointsDownLimit) == 0 and
                                            tempLoss < angleLoss):
                                        angleLoss = tempLoss
                                        planedOrientation = quatMult(objectOrientation,
                                                                     quatFromAxisAngle([0, 1, 0], i / 180.0 * 3.14))

                        break

            # if not find the best grasp orientation along the y axis of the object,
            # then select an orientation closest to the object from the database
            if angleLoss > 9999:
                orientationLoss = -10000
                yAxisOfObjectFromQuaterion = matrixFromQuat(objectOrientation)[0:3,
                                             1]  # w,x,y,z of list, tuple, numpy array
                for i in range(solutionStartIndex.astype(int), (solutionStartIndex + numSolution).astype(int), 1):
                    with self.env:
                        while True:
                            # if not self.robot.CheckSelfCollision():
                            if True:
                                T = matrixFromQuat(self.reachabilitySet[i, :])  # matrixFromQuat(np.array([w,x,y,z]))
                                T[0:3, 3] = targetPosition
                                # solutions = self.robotIkmodel.manip.FindIKSolutions(T,IkFilterOptions.CheckEnvCollisions)
                                solutions = self.robotIkmodel.manip.FindIKSolutions(T, 0)
                                if solutions is not None and len(solutions) > 0:
                                    yAxisOfOrientationCandidate = matrixFromQuat(self.reachabilitySet[i, :])[0:3, 1]
                                    tempLoss = yAxisOfOrientationCandidate.dot(yAxisOfObjectFromQuaterion)

                                    if (tempLoss > orientationLoss):
                                        orientationLoss = tempLoss
                                        planedOrientation = self.reachabilitySet[i, :]
                            break


        # grasp ball without orientation
        else:
            mixLoss = 10000
            for i in range(solutionStartIndex.astype(int), (solutionStartIndex + numSolution).astype(int), 1):
                with self.env:
                    while True:
                        # if not self.robot.CheckSelfCollision():
                        if True:
                            T = matrixFromQuat(self.reachabilitySet[i, :])
                            T[0:3, 3] = targetPosition
                            # solutions = self.robotIkmodel.manip.FindIKSolutions(T,IkFilterOptions.CheckEnvCollisions)
                            solutions = self.robotIkmodel.manip.FindIKSolutions(T, 0)
                            if solutions is not None and len(solutions) > 0:
                                for j in range(len(solutions)):
                                    tempLoss = np.array([1, 1, 1, 1, 1, 1, 1]).dot(
                                        np.abs((solutions[j, :] - self.jointsCenter) / self.jointsRange))
                                    tempLoss += np.array([0, 1, 0]).dot(
                                        matrixFromQuat(self.reachabilitySet[i, :])[0:3, 2])

                                    if (np.sum(solutions[j, :] > self.jointsUpLimit) == 0 and
                                            np.sum(solutions[j, :] < self.jointsDownLimit) == 0 and
                                            tempLoss < mixLoss):
                                        mixLoss = tempLoss
                                        planedOrientation = self.reachabilitySet[i, :]
                        break

        # calculate the grasp position with offset
        graspOffset = 0.02  # unit: m
        planedOrientationWithOffset = np.zeros([4])
        planedJointsAngleWithOffset = np.zeros([7])  # grasp joints angle

        graspPosition = targetPosition + matrixFromQuat(planedOrientation)[0:3, 2] * graspOffset -  matrixFromQuat(planedOrientation)[0:3, 0] * graspOffset

        self.graspPositionWithOffset = graspPosition

        positionInDatabase = (np.round(graspPosition, 2) * 100 + self.center).astype(int)
        [numSolution, solutionStartIndex] = self.poseSolutionsIndexInSerialization[positionInDatabase[0],
                                            positionInDatabase[1],
                                            positionInDatabase[2], :]

        # if grasp object with orientation, the y axis of object aligned with major axis of the object
        if (np.abs(np.linalg.norm(objectOrientation) - 1) < 1e-1):
            # turn around the y axis of the object to select the best orientation for grasp
            step = 1  # degree
            angleLoss = 10000
            for i in range(0, 360, step):
                with self.env:
                    while True:
                        # if not self.robot.CheckSelfCollision():
                        if True:
                            T = matrixFromQuat(quatMult(objectOrientation, quatFromAxisAngle([0, 1, 0],
                                                                                             i / 180.0 * 3.14)))  # matrixFromQuat(np.array([w,x,y,z]))
                            T[0:3, 3] = graspPosition
                            # solutions = self.robotIkmodel.manip.FindIKSolutions(T,IkFilterOptions.CheckEnvCollisions)
                            solutions = self.robotIkmodel.manip.FindIKSolutions(T, 0)

                            if solutions is not None and len(solutions) > 0:
                                for j in range(len(solutions)):
                                    tempLoss = np.array([1, 1, 1, 1, 1, 1, 1]).dot(
                                        np.abs((solutions[j, :] - self.jointsCenter) / self.jointsRange))
                                    if (np.sum(solutions[j, :] > self.jointsUpLimit) == 0 and
                                            np.sum(solutions[j, :] < self.jointsDownLimit) == 0 and
                                            tempLoss < angleLoss):
                                        angleLoss = tempLoss
                                        planedOrientationWithOffset = quatMult(objectOrientation,
                                                                               quatFromAxisAngle([0, 1, 0],
                                                                                                 i / 180.0 * 3.14))
                                        planedJointsAngleWithOffset = solutions[j, :]

                        break

            # if not find the best grasp orientation along the y axis of the object,
            # then select an orientation closest to the object from the database
            if angleLoss > 9999:
                orientationLoss = -10000
                yAxisOfObjectFromQuaterion = matrixFromQuat(objectOrientation)[0:3,
                                             1]  # w,x,y,z of list, tuple, numpy array

                for i in range(solutionStartIndex.astype(int), (solutionStartIndex + numSolution).astype(int), 1):
                    with self.env:
                        while True:
                            # if not self.robot.CheckSelfCollision():
                            if True:
                                T = matrixFromQuat(self.reachabilitySet[i, :])  # matrixFromQuat(np.array([w,x,y,z]))
                                T[0:3, 3] = targetPosition
                                # solutions = self.robotIkmodel.manip.FindIKSolutions(T,IkFilterOptions.CheckEnvCollisions)
                                solutions = self.robotIkmodel.manip.FindIKSolutions(T, 0)
                                if solutions is not None and len(solutions) > 0:
                                    yAxisOfOrientationCandidate = matrixFromQuat(self.reachabilitySet[i, :])[0:3, 1]
                                    tempLoss = yAxisOfOrientationCandidate.dot(yAxisOfObjectFromQuaterion)
                                    if (tempLoss > orientationLoss):
                                        orientationLoss = tempLoss
                                        planedOrientationWithOffset = self.reachabilitySet[i, :]
                                        angleLoss1 = 10000
                                        for j in range(len(solutions)):
                                            tempLoss1 = np.array([1, 1, 1, 1, 1, 1, 1]).dot(
                                                np.abs((solutions[j, :] - self.jointsCenter) / self.jointsRange))
                                            if (np.sum(solutions[j, :] > self.jointsUpLimit) == 0 and
                                                    np.sum(solutions[j, :] < self.jointsDownLimit) == 0 and
                                                    tempLoss1 < angleLoss1):
                                                angleLoss1 = tempLoss
                                                planedJointsAngleWithOffset = solutions[j, :]

                            break


        # grasp ball without orientation
        else:
            mixLoss = 10000
            planedOrientationQuat = pyQuat(planedOrientation)
            for i in range(solutionStartIndex.astype(int), (solutionStartIndex + numSolution).astype(int), 1):
                with self.env:
                    while True:
                        # if not self.robot.CheckSelfCollision():
                        if True:
                            T = matrixFromQuat(self.reachabilitySet[i, :])
                            T[0:3, 3] = graspPosition
                            # solutions = self.robotIkmodel.manip.FindIKSolutions(T,IkFilterOptions.CheckEnvCollisions)
                            solutions = self.robotIkmodel.manip.FindIKSolutions(T, 0)
                            if solutions is not None and len(solutions) > 0:
                                for j in range(len(solutions)):
                                    tempQuat = pyQuat(self.reachabilitySet[i, :])
                                    tempLoss = pyQuat.absolute_distance(tempQuat, planedOrientationQuat)
                                    if (np.sum(solutions[j, :] > self.jointsUpLimit) == 0 and
                                            np.sum(solutions[j, :] < self.jointsDownLimit) == 0 and
                                            tempLoss < mixLoss):
                                        mixLoss = tempLoss
                                        planedOrientationWithOffset = self.reachabilitySet[i, :]
                                        planedJointsAngleWithOffset = solutions[j, :]
                        break

        self.graspOrientation = planedOrientationWithOffset
        self.graspJointsAngle = planedJointsAngleWithOffset
        return planedOrientationWithOffset

    def preGrasp(self, targetPose):  # position: x,y,z + objectOrientation: w,x,y,z

        targetPosition = targetPose[0:3]
        objectOrientation = targetPose[3:7]

        calibOffSet = 0.12  # unit: m

        planedOrientation = self.grapsPoseGen(targetPosition, objectOrientation)
        T = matrixFromQuat(planedOrientation)

        if (np.abs(np.linalg.norm(objectOrientation) - 1) < 1e-1):
            calibPosition = self.graspPositionWithOffset + T[0:3, 2] * calibOffSet * 0.6 - T[0:3,
                                                                                           0] * calibOffSet * 0.6  # grasp object with orientation
        else:
            calibPosition = self.graspPositionWithOffset + T[0:3, 2] * calibOffSet  # grasp ball

        positionInDatabase = (np.round(calibPosition, 2) * 100 + self.center).astype(int)
        [numSolution, solutionStartIndex] = self.poseSolutionsIndexInSerialization[positionInDatabase[0],
                                            positionInDatabase[1],
                                            positionInDatabase[2], :]
        preGraspOrientation = np.zeros([1, 4])
        preGraspOrientationLoss = 10000
        preGraspAnglesLoss = 10000

        planedOrientationQuat = pyQuat(planedOrientation)
        for i in range(solutionStartIndex.astype(int), (solutionStartIndex + numSolution).astype(int), 1):
            with self.env:
                while True:
                    # if not self.robot.CheckSelfCollision():
                    if True:
                        T = matrixFromQuat(self.reachabilitySet[i, :])  # matrixFromQuat(np.array([w,x,y,z]))
                        T[0:3, 3] = calibPosition
                        # solutions = self.robotIkmodel.manip.FindIKSolutions(T,IkFilterOptions.CheckEnvCollisions)
                        solutions = self.robotIkmodel.manip.FindIKSolutions(T, 0)
                        if solutions is not None and len(solutions) > 0:
                            calibOrientationQuat = pyQuat(self.reachabilitySet[i, :])
                            tempLoss = pyQuat.absolute_distance(planedOrientationQuat, calibOrientationQuat)
                            if (tempLoss < preGraspOrientationLoss):
                                preGraspOrientationLoss = tempLoss
                                preGraspOrientation = self.reachabilitySet[i, :]
                    break

        with self.env:
            while True:
                # if not self.robot.CheckSelfCollision():
                if True:
                    T = matrixFromQuat(preGraspOrientation)  # matrixFromQuat(np.array([w,x,y,z]))
                    T[0:3, 3] = calibPosition
                    # solutions = self.robotIkmodel.manip.FindIKSolutions(T,IkFilterOptions.CheckEnvCollisions)
                    solutions = self.robotIkmodel.manip.FindIKSolutions(T, 0)
                    if solutions is not None and len(solutions) > 0:
                        for j in range(len(solutions)):
                            tempLoss = np.array([0, 0, 0, 0, 1, 1, 1]).dot(
                                np.abs((solutions[j, :] - self.jointsCenter) / self.jointsRange))
                            if (np.sum(solutions[j, :] > self.jointsUpLimit) == 0 and
                                    np.sum(solutions[j, :] < self.jointsDownLimit) == 0 and
                                    tempLoss < preGraspAnglesLoss):
                                preGraspAnglesLoss = tempLoss
                                preGraspAngles = solutions[j, :]
                break

        self.preGraspJointsAngle = preGraspAngles

        tempAngle = self.currentJointError + self.preGraspJointsAngle
        if (np.sum(tempAngle > self.jointsUpLimit) == 0 and
                np.sum(tempAngle < self.jointsDownLimit) == 0):
            return self.currentJointError + self.preGraspJointsAngle
        else:
            print "compensation out of joint limit"
            return self.preGraspJointsAngle

    def calibCompensation(self, trackerPose):  # pose: position(x,y,z) + orientation(w,x,y,z)
        angleLoss = 10000
        with self.env:
            while True:
                # if not self.robotCalib.CheckSelfCollision():
                if True:
                    T = matrixFromQuat(trackerPose[3:7])
                    T[0:3, 3] = trackerPose[0:3]
                    # solutions = self.robotCalibIkmodel.manip.FindIKSolutions(T,IkFilterOptions.CheckEnvCollisions)
                    solutions = self.robotCalibIkmodel.manip.FindIKSolutions(T, 0)
                    if solutions is not None and len(solutions) > 0:
                        for j in range(len(solutions)):
                            tempLoss = np.array([0, 0, 0, 0, 0, 1, 1]).dot(np.abs(solutions[j, :]))
                            if (np.sum((solutions[j, :] > self.jointsUpLimit)[0:5]) == 0 and
                                    np.sum((solutions[j, :] < self.jointsDownLimit)[0:5]) == 0 and
                                    tempLoss < angleLoss):
                                angleLoss = tempLoss
                                actualJointsAngle = solutions[j, 0:5]
                break

        self.currentJointError[0:5] = self.preGraspJointsAngle[0:5] + self.currentJointError[0:5] - actualJointsAngle

        print "currentJointError"
        print self.currentJointError[0:5]

        tempAngle = self.currentJointError + self.preGraspJointsAngle
        if (np.sum(tempAngle > self.jointsUpLimit) == 0 and
                np.sum(tempAngle < self.jointsDownLimit) == 0):
            self.robotCalib.SetDOFValues(self.currentJointError + self.preGraspJointsAngle,
                                         self.robotCalibIkmodel.manip.GetArmIndices())
            return self.currentJointError + self.preGraspJointsAngle
        else:
            print "compensation out of joint limit"
            self.robotCalib.SetDOFValues(self.preGraspJointsAngle, self.robotCalibIkmodel.manip.GetArmIndices())
            return self.preGraspJointsAngle

    def grasp(self):
        tempAngle = self.currentJointError + self.graspJointsAngle
        if (np.sum(tempAngle > self.jointsUpLimit) == 0 and
                np.sum(tempAngle < self.jointsDownLimit) == 0):
            self.robotCalib.SetDOFValues(self.currentJointError + self.graspJointsAngle,
                                         self.robotCalibIkmodel.manip.GetArmIndices())
            return self.currentJointError + self.graspJointsAngle
        else:
            print "compensation out of joint limit"
            self.robotCalib.SetDOFValues(self.graspJointsAngle, self.robotCalibIkmodel.manip.GetArmIndices())
            return self.graspJointsAngle

    def moveBack(self):

        tempAngle = self.currentJointError + self.preGraspJointsAngle
        if (np.sum(tempAngle > self.jointsUpLimit) == 0 and
                np.sum(tempAngle < self.jointsDownLimit) == 0):
            return self.currentJointError + self.preGraspJointsAngle
        else:
            print "compensation out of joint limit"
            return self.preGraspJointsAngle

    def move(self, deltaPosition):
        print 1

    def motionPlan(self, startPose, targetPose):
        print 2

    def showGUI(self):
        self.env.SetViewer('qtcoin')

    def robotUpdate(self):
        self.robot.SetDOFValues(self.graspJointsAngle, self.robotIkmodel.manip.GetArmIndices())

        self.newrobot.SetDOFValues(self.preGraspJointsAngle, self.robotIkmodel.manip.GetArmIndices())

    def test(self):
        print self.jointsUpLimit
        print self.jointsDownLimit
        print self.robot.GetLinkTransformations()[7]
        print self.robot.GetLinks()

    def pseuCalib(self):
        calibPose = np.zeros([7])  # position(x,y,z) + quaternion(w,x,y,z)

        angle = self.preGraspJointsAngle
        angle[5] = 0
        angle[6] = 0
        self.robotCalib.SetDOFValues(angle, self.robotCalibIkmodel.manip.GetArmIndices())
        calibPose[0:3] = self.robotCalib.GetLinkTransformations()[8][0:3, 3]
        calibPose[3:7] = quatFromRotationMatrix(self.robotCalib.GetLinkTransformations()[8][0:3, 0:3])
        self.calibAndGrasp(calibPose)

if __name__ == "__main__":
    x_arm0 = x_arm()
    x_arm0.showGUI()
    x_arm0.preGrasp([-0.2,0,0.45,0,0,0,0])
    x_arm0.pseuCalib()
    x_arm0.robotUpdate()
    x_arm0.preGrasp([-0.2,0,0.45,0.5,0.5,-0.5,0.5])
    x_arm0.pseuCalib()
    x_arm0.robotUpdate()
