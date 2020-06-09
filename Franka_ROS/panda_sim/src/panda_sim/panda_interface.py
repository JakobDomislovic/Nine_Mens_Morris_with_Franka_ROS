#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys

import rospy
import tf2_ros
import tf2_geometry_msgs
import moveit_commander
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from moveit_msgs.msg import DisplayTrajectory, Constraints, OrientationConstraint

from panda_sim.srv import CartesianGoal, CartesianGoalResponse, JointsGoal, JointsGoalResponse, NamedGoal, \
    NamedGoalResponse, VacuumGripperControl, VacuumGripperControlRequest


class ArmInterface(object):
    
    def __init__(self):
        arm_group_name = 'panda_arm'
        self.arm_move_group = moveit_commander.MoveGroupCommander(arm_group_name)

        self.ns = rospy.get_namespace()

        # Load parameters from ROS param server.
        self.base_frame = rospy.get_param('/environment_setup/CONFIG/base_frame', 'world')
        self.fixed_frame = rospy.get_param('/environment_setup/CONFIG/global_frame', 'world')

        # Create a transform listener.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Create services.
        rospy.Service('move_arm_joints_pose', JointsGoal, self.move_arm_joints_pose_srv)
        rospy.Service('move_arm_cartesian_pose', CartesianGoal, self.move_arm_cartesian_pose_srv)
        rospy.Service('move_arm_named_pose', NamedGoal, self.move_arm_named_pose_srv)
        #rospy.Service('move_arm_cartesian_path', CartesianPath, self.move_arm_cartesian_path_srv)


        # Make a simple planning scene to avoid collision with the ground.
        rospy.sleep(5)
        rospy.loginfo('Planing scene loaded.')
        self.planning_scene = moveit_commander.PlanningSceneInterface(self.ns, synchronous=True)
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'world'
        pose.pose.position = Point(0, 0, -0.5)
        pose.pose.orientation = Quaternion(0, 0, 0, 1)
        self.planning_scene.add_cylinder('ground_plane', pose, radius=1, height=1)

        self.dummy = True
        self.last_move = 0
        # Everything is ready. Just keep the node from exiting.
        rospy.spin()

    def move_arm_cartesian_pose(self, goal_pose):

        rospy.loginfo('Got new goal:\n%s', goal_pose)

        # Set the orientations constraint for end effector link. We don't care about the yaw of the final link so it is
        # an unnecessary constraint on the path planning. Here we allow for any yaw value by specifying a large goal 
        # tolerance for that axis. This also helps to avoid unnatural movement.
        
        print(goal_pose.position.z)
        
        if goal_pose.position.z == 0.024 or self.last_move == 0.024:
            print('Turning constrants: TRUE')
            self.yaw_on_off(True, goal_pose)
        else:
            print('Turning constrants: FALSE')
            self.yaw_on_off(False, goal_pose)

        self.last_move = goal_pose.position.z
        
        self.arm_move_group.set_pose_target(goal_pose)
        success = self.arm_move_group.go(wait=True)
        self.arm_move_group.stop()  # Ensure there is no residual movement.
        self.arm_move_group.clear_pose_targets()  # It is recommended to clear goal when planning with poses.
        return success

    def move_arm_cartesian_path(self, waypoints, eef_step=0.01, jump_threshold=0.0):
        (plan, fraction) = self.arm_move_group.compute_cartesian_path(waypoints, eef_step, jump_threshold)
        success = self.arm_move_group.execute(plan, wait=True)
        self.arm_move_group.stop()
        return success

    def transform_pose(self, frame, pose):
        if frame == 'local':
            return True, frame, pose
        else:
            if frame == 'global':
                fixed_frame = self.fixed_frame
            elif frame[0] == '/':
                fixed_frame = frame[1:]
            else:
                fixed_frame = self.ns.split('/')[1] + '/' + frame

            # TODO: Deal with transformation of EE orientation.
            # When commanding pick and place poses, we always want the orientation of end effector to look downwards,
            # regardless of the frame in which we specified the position. Therefore, orientation should not be
            # transformed. In other cases, we want to be able to specify orientation in any frame. Transforming the
            # orientation could lead to unexpected poses (it makes sense to specify orientation locally), but it isn't
            # intuitive to specify a pose in two different frames.

            # >  # Save the end effector's orientation and use only desired position when calculating transform.
            # >  ee_orientation = deepcopy(pose.orientation)
            # >  pose.orientation = Quaternion(0, 0, 0, 1)
            try:
                transform = self.tf_buffer.lookup_transform(self.base_frame, fixed_frame, rospy.Time(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                return False, fixed_frame, None
            pose_stamped = PoseStamped()
            pose_stamped.pose = pose
            local_pose = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
            # >  local_pose.pose.orientation = ee_orientation
            return True, fixed_frame, local_pose.pose

    def move_arm_cartesian_pose_srv(self, req):
        resp = CartesianGoalResponse()

        tf_success, frame, local_pose = self.transform_pose(req.frame, req.pose)
        if not tf_success:
            resp.success = False
            resp.error_msg = "Couldn't find transformation from '{}' to '{}'.".format(frame, self.base_frame)
            return resp

        print('Trenutna pozicija z: {}'.format(local_pose.position.z))
        print('Stara pozicija z: {}'.format(self.last_move))

        success = self.move_arm_cartesian_pose(local_pose)
        resp.success = success
        resp.error_msg = "" if success else "Moving to specified position failed."

        return resp

    def yaw_on_off(self, flag, goal_pose):

        if flag: print('CONSTRAINTS ON')
        if goal_pose != 'jump':
            arm_constraints = Constraints()
            arm_constraints.name = 'default'
            yaw_constraint = OrientationConstraint()
            yaw_constraint.header.frame_id = 'world'
            yaw_constraint.link_name = 'panda_link7'
            yaw_constraint.orientation = goal_pose.orientation
            yaw_constraint.absolute_x_axis_tolerance = 0.35
            yaw_constraint.absolute_y_axis_tolerance = 0.35
            yaw_constraint.absolute_z_axis_tolerance = 0.05
            yaw_constraint.weight = 1
            arm_constraints.orientation_constraints.append(yaw_constraint)
            self.arm_move_group.set_path_constraints(arm_constraints)

        if not flag:
            print('CONSTRAINTS OFF')
            self.arm_move_group.clear_path_constraints()


    def move_arm_cartesian_path_srv(self, req):
        resp = CartesianGoalResponse()
        local_pose_list = []

        for req_i in req: # buduci da je sad req lista Pose poruka idem provjeravat jednu po jednu je li ok
            tf_success, frame, local_pose = self.transform_pose(req_i.frame, req_i.pose)
            if not tf_success:
                resp.success = False
                resp.error_msg = "Couldn't find transformation from '{}' to '{}'.".format(frame, self.base_frame)
                return resp
            
            local_pose_list.append(local_pose)

        success = self.move_arm_cartesian_path(local_pose_list)
        resp.success = success
        resp.error_msg = "" if success else "Moving to specified position failed."
        
        return resp

    def move_arm_joints_pose_srv(self, req):
        resp = JointsGoalResponse()

        joints_goal = self.arm_move_group.get_current_joint_values()
        if len(joints_goal) != len(req.joint_positions):
            resp.success = False
            resp.error_msg = "Wrong number of joints specified. Should be {}, but I got {}.".format(
                len(joints_goal), len(req.joint_positions))
        else:
            for i in range(len(joints_goal)):
                joints_goal[i] = req.joint_positions[i]
            success = self.arm_move_group.go(joints_goal, wait=True)
            self.arm_move_group.stop()  # Ensure there is no residual movement.
            resp.success = success
            resp.error_msg = "" if success else "Moving to specified position failed."

        return resp

    def move_arm_named_pose_srv(self, req):
        # gasimo ogranicenja kad radimo named_positions
        self.yaw_on_off(False, 'jump')

        resp = NamedGoalResponse()

        if req.pose_name in self.arm_move_group.get_named_targets():
            self.arm_move_group.set_named_target(req.pose_name)
            success = self.arm_move_group.go(wait=True)
            self.arm_move_group.stop()  # Ensure there is no residual movement.
            self.arm_move_group.clear_pose_targets()  # It is recommended to clear goal when planning with poses.
            resp.success = success
            resp.error_msg = "" if success else "Moving to specified position failed."
        else:
            resp.success = False
            resp.error_msg = "Unknown pose named '{}'".format(req.pose_name)

        return resp


    def gripper_control(self, enable):
        try:
            rospy.wait_for_service('gripper/control', 1)
            control = rospy.ServiceProxy('gripper/control', VacuumGripperControl)
            req = VacuumGripperControlRequest()
            req.enable = enable
            control(req)
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr('Service call failed with error: %s', e)


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('arm_interface')

    try:
        node = ArmInterface()
    except rospy.ROSInterruptException:
        pass
