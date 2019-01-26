#!/usr/bin/env python
from IK_fast_solver.srv import *
import rospy
import x_arm

x_arm_plan = x_arm.x_arm(robotPath="/home/x-arm/macaca/src/x_arm/robots",
                         robotName="x_arm",
                         robotManipName="x_arm",
                         calibRobotName="x_arm_calib",
                         calibRobotManipName="x_arm_calib",
                         reachabilityFilePath="/home/x-arm/macaca/data")
x_arm_plan.showGUI()

def handle_query_solution(req):  
    if req.flag==0:#pre_grasp
        angle = x_arm_plan.preGrasp([req.x, req.y, req.z, req.q_w, req.q_x, req.q_y, req.q_z])
        x_arm_plan.robotUpdate()
        print('pre_grasp_angle: %f %f %f %f %f %f %f'%(angle[0],angle[1],angle[2],angle[3],angle[4],angle[5],angle[6]))
        return angle[0],angle[1],angle[2],angle[3],angle[4],angle[5],angle[6]
    if req.flag==1:#calib
        angle = x_arm_plan.calibCompensation([req.x, req.y, req.z, req.q_w, req.q_x, req.q_y, req.q_z])
        x_arm_plan.robotUpdate()
        print('calib_angle: %f %f %f %f %f %f %f'%(angle[0],angle[1],angle[2],angle[3],angle[4],angle[5],angle[6]))
        return angle[0],angle[1],angle[2],angle[3],angle[4],angle[5],angle[6]
    if req.flag==2:#grasp
        angle = x_arm_plan.grasp()
        x_arm_plan.robotUpdate()
        print('grasp_angle: %f %f %f %f %f %f %f'%(angle[0],angle[1],angle[2],angle[3],angle[4],angle[5],angle[6]))
        return angle[0],angle[1],angle[2],angle[3],angle[4],angle[5],angle[6]

def query_solution_server():
    rospy.init_node('query_solution_server')
    s=rospy.Service('grasp',grasp,handle_query_solution)
    print("Ready to query the solution")
    rospy.spin()

if __name__=="__main__":
    query_solution_server()



