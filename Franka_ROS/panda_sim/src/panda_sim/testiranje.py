#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose 

print('test')
from panda_sim.srv import CartesianGoal, CartesianGoalResponse, JointsGoal, JointsGoalResponse, NamedGoal, \
    NamedGoalResponse, VacuumGripperControl, VacuumGripperControlRequest


def named_positions(value):
    
    rospy.wait_for_service('move_arm_named_pose')

    try:
        ret = rospy.ServiceProxy('move_arm_named_pose', NamedGoal)
        resp1 = ret(value)
        return True
    except rospy.ServiceException as e:
        print('Service call failed: %s'%e)

def move_gripper(pos, frm):

    rospy.wait_for_service('move_arm_cartesian_pose')

    try:
        ret = rospy.ServiceProxy('move_arm_cartesian_pose', CartesianGoal)
        resp1 = ret(pos, frm)
        return True
    except rospy.ServiceException as e:
        print('Service call failed: %s'%e)

def vac_grip(enable):
        try:
            rospy.wait_for_service('gripper/control', 1)
            control = rospy.ServiceProxy('gripper/control', VacuumGripperControl)
            req = VacuumGripperControlRequest()
            req.enable = enable
            control(req)
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr('Service call failed with error: %s', e)


if __name__ == '__main__':
    print('test')

    pos_dict = {
        'white':[0.0, 0.5, 0.024, 0.0, 1.0, 0.0, 0.0], 
        'black':[0.5, 0.0, 0.024, 0.0, 1.0, 0.0, 0.0], 
        'coke' :[0.0, -0.5, 0.2, 0.0, 1.0, 0.0, 0.0], 
        'beer' :[-0.5, 0.0, 0.35, 0.0, 1.0, 0.0, 0.0]
    }
    
    p = Pose()

    control = 'home'
    srv_res = named_positions(control)

    srv_res = vac_grip(False)

    d = pos_dict['white']
    p.position.x = d[0]
    p.position.y = d[1]
    p.position.z = d[2]
    p.orientation.x = d[3] 
    p.orientation.y = d[4] 
    p.orientation.z = d[5] 
    p.orientation.w = d[6] 
    
    frame = 'local'
    srv_res = move_gripper(p, frame)
    srv_res = vac_grip(True)

    p.position.z = d[2] + 0.5
    srv_res = move_gripper(p, frame)


    d = pos_dict['coke']
    p.position.x = d[0]
    p.position.y = d[1]
    p.position.z = d[2]
    p.orientation.x = d[3] 
    p.orientation.y = d[4] 
    p.orientation.z = d[5] 
    p.orientation.w = d[6] 
    
    frame = 'local'
    srv_res = move_gripper(p, frame)
    srv_res = vac_grip(False)

    d = pos_dict['black']
    p.position.x = d[0]
    p.position.y = d[1]
    p.position.z = d[2]
    p.orientation.x = d[3] 
    p.orientation.y = d[4] 
    p.orientation.z = d[5] 
    p.orientation.w = d[6] 
    
    frame = 'local'
    srv_res = move_gripper(p, frame)
    srv_res = vac_grip(True)

    p.position.z = d[2] + 0.5
    srv_res = move_gripper(p, frame)


    d = pos_dict['beer']
    p.position.x = d[0]
    p.position.y = d[1]
    p.position.z = d[2]
    p.orientation.x = d[3] 
    p.orientation.y = d[4] 
    p.orientation.z = d[5] 
    p.orientation.w = d[6] 
    
    frame = 'local'
    srv_res = move_gripper(p, frame)
    srv_res = vac_grip(False)