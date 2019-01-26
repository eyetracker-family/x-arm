#!/usr/bin/env python

from reverse_kinematics.srv import *
from reverse_kinematics.msg import solution,solutions
import rospy
import h5py


fileName="/home/x-arm/macaca/data/pose-1364698_576_4.h5"
#fileName="../../../data/pose-1364698_576_4.h5"
f=h5py.File(fileName,'r')

poseSolutionsIndexInSerialization = f['poseSolutionsIndexInSerialization']
reachabilitySet = f['reachabilitySet']
center = f['center']
                                                                                      

def handle_query_solution(req):  
    poses=solutions()
    num = int(poseSolutionsIndexInSerialization[req.x+center.value,req.y+center.value,req.z+center.value,0])
    startIndex =int(poseSolutionsIndexInSerialization[req.x+center.value,req.y+center.value,req.z+center.value,1])

    print ("num:%f startIndex: %f" %(num,startIndex))
    print ("x:%f y:%f z:%f" %(req.x,req.y,req.z)) 


    for i in range(num):
        pose=solution()
        pose.w = reachabilitySet[startIndex+i,0]
        pose.x = reachabilitySet[startIndex+i,1]
        pose.y = reachabilitySet[startIndex+i,2]
        pose.z = reachabilitySet[startIndex+i,3]

        poses.solutions.append(pose)
        print ("w:%f x:%f y:%f z:%f" %(pose.w,pose.x,pose.y,pose.z))
    
    for j in range(num):
        print ("w:%f x:%f y:%f z:%f" %(poses.solutions[j].w,poses.solutions[j].x,poses.solutions[j].y,poses.solutions[j].z))

    print(len(poses.solutions))
 
    return querySolutionsResponse(poses)

def query_solution_server():
    rospy.init_node('query_solution_server')
    s=rospy.Service('query_solution',querySolutions,handle_query_solution)
    print("Ready to query the solution")
    rospy.spin()

if __name__=="__main__":
    query_solution_server()
