#!/usr/bin/env python

import numpy
import random
import sys

import geometry_msgs.msg
import moveit_msgs.msg
import moveit_msgs.srv
import rospy
import tf
import moveit_commander
from urdf_parser_py.urdf import URDF
from std_msgs.msg import String
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Transform
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import copy
from moveit_commander.conversions import pose_to_list

def convert_to_message(T):
    t = geometry_msgs.msg.Pose()
    position = tf.transformations.translation_from_matrix(T)
    orientation = tf.transformations.quaternion_from_matrix(T)
    t.position.x = position[0]
    t.position.y = position[1]
    t.position.z = position[2]
    t.orientation.x = orientation[0]
    t.orientation.y = orientation[1]
    t.orientation.z = orientation[2]
    t.orientation.w = orientation[3]        
    return t

class MoveArm(object):

    def __init__(self):

	self.step_size=0.25

	#Loads the ur5 moveit command
	self.ur5_robot = moveit_commander.RobotCommander()
	self.group_name = "ur5"
	self.group = moveit_commander.MoveGroupCommander(self.group_name)

	#setup display trajectory
	self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

        #Loads the robot model, which contains the robot's kinematics information
	self.num_joints = 0
        self.joint_names = []
        self.joint_axes = []
        self.robot = URDF.from_parameter_server()
        self.base = self.robot.get_root()
        self.get_joint_info()

        # Wait for moveit IK service
        rospy.wait_for_service("compute_ik")
        self.ik_service = rospy.ServiceProxy('compute_ik',  moveit_msgs.srv.GetPositionIK)
        print "IK service ready"

	# Wait for moveit FK service
	self.joint_state_current = None
	rospy.wait_for_service("compute_fk")
	self.fk_service = rospy.ServiceProxy('compute_fk',  moveit_msgs.srv.GetPositionFK)
	print "FK service ready"
	

        # Wait for validity check service
        rospy.wait_for_service("check_state_validity")
        self.state_valid_service = rospy.ServiceProxy('check_state_validity',  
                                                      moveit_msgs.srv.GetStateValidity)
        print "State validity service ready"

        # MoveIt parameter
        robot_moveit = moveit_commander.RobotCommander()
        self.group_name = robot_moveit.get_group_names()[1]

	#Subscribe to topics
	rospy.Subscriber('/joint_states', JointState, self.get_joint_state)
	rospy.Subscriber('/motion_planning_command', String, self.motion_planning)
	rospy.Subscriber('/move_z', Float32, self.move_z)
	rospy.Subscriber('/move_x', Float32, self.move_x)
	rospy.Subscriber('/move_y', Float32, self.move_y)
	rospy.Subscriber('/go_home', Float32, self.go_home)
        self.current_obstacle = "None"
        rospy.Subscriber('/obstacle', String, self.get_obstacle)

	#Set up publisher
	self.pub = rospy.Publisher('/joint_trajectory', JointTrajectory, queue_size=10)

    '''This callback provides you with the current joint positions of the robot 
     in member variable q_current.'''
    def get_joint_state(self, msg):
	self.joint_state_current = msg
        self.q_current = []
        for name in self.joint_names:
            self.q_current.append(msg.position[msg.name.index(name)])

    '''This callback provides you with the name of the current obstacle which
    exists in the RVIZ environment. Options are "None", "Simple", "Hard",
    or "Super". '''
    def get_obstacle(self, msg):
        self.current_obstacle = msg.data

    '''This is the callback which will implement your RRT motion planning.
    You are welcome to add other functions to this class (i.e. an
    "is_segment_valid" function will likely come in handy multiple times 
    in the motion planning process and it will be easiest to make this a 
    seperate function and then call it from motion planning). You may also
    create trajectory shortcut and trajectory sample functions if you wish, 
    which will also be called from the motion planning function.'''    
    def motion_planning(self, mp_cmd):
        print "Starting motion planning"
	########INSERT YOUR RRT MOTION PLANNING HERE##########
	print(self.q_current)
	#home_goal=[0.05997617915272713, -1.5376494566546839, 1.8350682258605957, -0.29061824480165654, 0.1237255185842514, 0.03281692788004875]
	#home_goal=[-0.022920442795482998, -1.6324915783467269, 2.2074940637732396, -2.1458263788344274, -1.5707715844594965, -1.5936662208306265]
	#home_goal = [-0.8710439840899866, -1.268482510243551, 1.7783546447753906, -2.0807698408709925, -1.5705984274493616, -2.4418392817126673]


	#home_plan=self.group.plan(home_goal)
	#self.group.stop()
	#self.group.execute(home_plan)
	#print(self.FK())

	"""
	pose_goal = geometry_msgs.msg.Pose()
	q_f_e = tf.transformations.quaternion_from_euler(-numpy.pi/2, 0, 0)
	T_home2 = numpy.dot(tf.transformations.translation_matrix((0.4, 0.1, 0.3)), tf.transformations.quaternion_matrix(q_f_e))
	q_home2 = self.IK(T_home2)
	print(q_home2)
	home_plan=self.group.plan(q_home2)
	self.group.stop()
	self.group.execute(home_plan)
	print(self.FK())
	"""

	"""
	waypoints = []
	scale = 1.0
	wpose = self.group.get_current_pose().pose
	wpose.position.z -= scale * 0.1  
	waypoints.append(copy.deepcopy(wpose))

	wpose.position.x += scale * 0.1  
	waypoints.append(copy.deepcopy(wpose))

	wpose.position.z += scale * 0.1  
	waypoints.append(copy.deepcopy(wpose))

	wpose.position.x -= scale * 0.1  
	waypoints.append(copy.deepcopy(wpose))

	wpose.position.y += scale * 0.1  
	waypoints.append(copy.deepcopy(wpose))

	wpose.position.y -= scale * 0.1  
	waypoints.append(copy.deepcopy(wpose))

	# We want the Cartesian path to be interpolated at a resolution of 1 cm
	# which is why we will specify 0.01 as the eef_step in Cartesian
	# translation.  We will disable the jump threshold by setting it to 0.0 disabling:
	(plan, fraction) = self.group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.001,        # eef_step
                                   0.0)         # jump_threshold
	self.group.execute(plan, wait=True)
	"""

	x_m = []
	y_m = []
	z_m = []
	linenumber = 0
	with open("/home/max/ur5test/src/motion_planning/src/fu.txt") as f:
   		for line in f:
        		data = line.split()
			x_m.append(int(data[0]))
        		y_m.append(int(data[1]))
        		z_m.append(int(data[2]))

	#print(x_m)
	waypoints = []
	scale = 1.5
	zscale = 1.0
	org_pose = self.group.get_current_pose().pose
	start_z = org_pose.position.z
	for i in range(len(x_m)):
		wpose = self.group.get_current_pose().pose
		org_pose.position.x += scale * x_m[i] * 0.001
		org_pose.position.y += scale * y_m[i] * 0.001
		org_pose.position.z = start_z - scale * z_m[i] * 0.001
		wpose.position.x = org_pose.position.x
		wpose.position.y = org_pose.position.y
		wpose.position.z = org_pose.position.z
		waypoints.append(copy.deepcopy(wpose))

	(plan, fraction) = self.group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.001,        # eef_step
                                   0.0)         # jump_threshold
	self.group.execute(plan, wait=True)
	

    def compute_distance(self, q1, q2):
	q1=numpy.array(q1)
	q2=numpy.array(q2)
	q1-=q2
	return numpy.sqrt(q1.dot(q1))

    def is_to_goal(self, q, qgoal):
	dis=self.compute_distance(q,qgoal)
	num_of_a=int(dis/self.step_size)
	if (num_of_a>0):
		num_of_a+=1
		head=numpy.array(q)
		tail=numpy.array(qgoal)
		for j in range(num_of_a):
			elem=head+((j+1)*1.0/num_of_a)*(tail-head)
			if (self.is_state_valid(elem.tolist())==False):
				return False
	else:
		return True
	return True

    def go_home(self, msg):
	home_goal = [-0.8710439840899866, -1.268482510243551, 1.7783546447753906, -2.0807698408709925, -1.5705984274493616, -2.4418392817126673]


	home_plan=self.group.plan(home_goal)
	self.group.stop()
	self.group.execute(home_plan)

    def move_z(self, msg):
	waypoints = []
	scale = 1.0
	wpose = self.group.get_current_pose().pose
	wpose.position.z += scale * msg.data  
	waypoints.append(copy.deepcopy(wpose))
	(plan, fraction) = self.group.compute_cartesian_path(
                           waypoints,   # waypoints to follow
                           0.001,        # eef_step
                           0.0)         # jump_threshold
	self.group.execute(plan, wait=True)

    def move_x(self, msg):
	waypoints = []
	scale = 1.0
	wpose = self.group.get_current_pose().pose
	wpose.position.x += scale * msg.data  
	waypoints.append(copy.deepcopy(wpose))
	(plan, fraction) = self.group.compute_cartesian_path(
                           waypoints,   # waypoints to follow
                           0.001,        # eef_step
                           0.0)         # jump_threshold
	self.group.execute(plan, wait=True)

    def move_y(self, msg):
	waypoints = []
	scale = 1.0
	wpose = self.group.get_current_pose().pose
	wpose.position.y += scale * msg.data  
	waypoints.append(copy.deepcopy(wpose))
	(plan, fraction) = self.group.compute_cartesian_path(
                           waypoints,   # waypoints to follow
                           0.001,        # eef_step
                           0.0)         # jump_threshold
	self.group.execute(plan, wait=True)

    """ This function will perform IK for a given transform T of the end-effector.
    It returns a list q[] of values, which are the result positions for the 
    joints of the robot arm, ordered from proximal to distal. If no IK solution 
    is found, it returns an empy list.
    """
    def IK(self, T_goal):
        req = moveit_msgs.srv.GetPositionIKRequest()
        req.ik_request.group_name = self.group_name
        req.ik_request.robot_state = moveit_msgs.msg.RobotState()
        req.ik_request.robot_state.joint_state.name = self.joint_names
        req.ik_request.robot_state.joint_state.position = numpy.zeros(self.num_joints)
        req.ik_request.robot_state.joint_state.velocity = numpy.zeros(self.num_joints)
        req.ik_request.robot_state.joint_state.effort = numpy.zeros(self.num_joints)
        req.ik_request.robot_state.joint_state.header.stamp = rospy.get_rostime()
        req.ik_request.avoid_collisions = True
        req.ik_request.pose_stamped = geometry_msgs.msg.PoseStamped()
        req.ik_request.pose_stamped.header.frame_id = self.base
        req.ik_request.pose_stamped.header.stamp = rospy.get_rostime()
        req.ik_request.pose_stamped.pose = convert_to_message(T_goal)
        req.ik_request.timeout = rospy.Duration(3.0)
        res = self.ik_service(req)
        q = []
        if res.error_code.val == res.error_code.SUCCESS:
            q = res.solution.joint_state.position
        return q

    def FK(self):
	while not rospy.is_shutdown() and self.joint_state_current is None:
           	rospy.logwarn("Waiting for a /joint_states message...")
		rospy.sleep(0.1)
	req = moveit_msgs.srv.GetPositionFKRequest()
	req.header.frame_id = 'base_link'
	req.fk_link_names = ['wrist_3_link']
	req.robot_state.joint_state = self.joint_state_current
	resp = self.fk_service.call(req)
	return resp

    '''This is a function which will collect information about the robot which
       has been loaded from the parameter server. It will populate the variables
       self.num_joints (the number of joints), self.joint_names and
       self.joint_axes (the axes around which the joints rotate)'''
    def get_joint_info(self):
        link = self.robot.get_root()
        while True:
            if link not in self.robot.child_map: break
            (joint_name, next_link) = self.robot.child_map[link][0]
            current_joint = self.robot.joint_map[joint_name]
            if current_joint.type != 'fixed':
                self.num_joints = self.num_joints + 1
                self.joint_names.append(current_joint.name)
                self.joint_axes.append(current_joint.axis)
            link = next_link


    """ This function checks if a set of joint angles q[] creates a valid state,
    or one that is free of collisions. The values in q[] are assumed to be values
    for the joints of the KUKA arm, ordered from proximal to distal. 
    """
    def is_state_valid(self, q):
        req = moveit_msgs.srv.GetStateValidityRequest()
        req.group_name = self.group_name
        req.robot_state = moveit_msgs.msg.RobotState()
        req.robot_state.joint_state.name = self.joint_names
        req.robot_state.joint_state.position = q
        req.robot_state.joint_state.velocity = numpy.zeros(self.num_joints)
        req.robot_state.joint_state.effort = numpy.zeros(self.num_joints)
        req.robot_state.joint_state.header.stamp = rospy.get_rostime()
        res = self.state_valid_service(req)
        return res.valid


'''This is a class which you can use to keep track of your tree branches.
It is easiest to do this by appending instances of this class to a list 
(your 'tree'). The class has a parent field and a joint position field (q). 
You can initialize a new branch like this:
RRTBranch(parent, q)
Feel free to keep track of your branches in whatever way you want - this
is just one of many options available to you.'''
class RRTBranch(object):
    def __init__(self, parent, q):
	self.parent = parent
	self.q = q


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_arm', anonymous=True)
    ma = MoveArm()
    rospy.spin()

