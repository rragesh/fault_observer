#!/usr/bin/python
import rospy
import actionlib
import numpy
from sensor_msgs.msg import JointState
from sensor_msgs.msg import CameraInfo
from ensenso_camera_msgs.msg import RequestDataAction, RequestDataActionGoal, RequestDataActionResult
# Automated planner
import sys
import pprint

from fault_observer.AutomatedPlanner.pddl_parser import *
from fault_observer.AutomatedPlanner.planner import *

class Observer(object):

    def __init__(self):
        self.trigger_goal  = False
        self.trigger_result = False
        self.status = False
        self.compare = False
        self.count = 0
        self.init_state = []
        self.vel = []
        self.goal_sub = rospy.Subscriber("request_data/goal", RequestDataActionGoal, self.goal_cb)
        self.result_sub = rospy.Subscriber("request_data/result", RequestDataActionResult, self.result_cb)
        self.joint_state_sub = None

    def goal_cb(self,msg):
        # callback to subscribe for the action goal
        self.trigger_goal = True

    def result_cb(self,msg):
        # callback to subscribe for the action result
        self.trigger_result = True

    def joint_state_cb(self,msg):
        # callback to subscribe for the joint state position
        # print("\n")
        # print(msg.position)
        self.vel.append(msg.velocity)

    def cycle(self):
        # Black box
        if self.trigger_goal == True:
            print("\nCamera started capturing")
            print("I am monitoring the joints")
            self.js_sub = rospy.Subscriber("/prbt/joint_states",JointState, self.joint_state_cb)
            self.trigger_goal  = False

        if self.trigger_result == True:
            self.js_sub.unregister()
            # count number of captures
            self.count = self.count+1
            # check for the vibration
            self.compare = all(x == self.vel[0] for x in self.vel)
            self.init_state.append(self.compare)
            print("Not moving = %r"%self.compare)
            self.compare = False
            print("camera stopped capturing\n")
            self.trigger_result = False
            self.vel = []
        self.create_problem()


    def create_problem(self):
        if self.count == 4:
            with open("/home/Automatica2018/snp_demo_ws/src/fault_observer/config/problem.pddl",'w') as self.pddl:
                self.pddl.write("(define (problem SCAN-4-1)\n")
                self.pddl.write("(:domain SCAN)\n")
                self.pddl.write("(:objects  P0 - SP0 P1 - SP1 P2 - SP2 P3 - SP3)\n")
                self.pddl.write("(:init " )
                # the initial conditions are updated here based on observation
                self.create_init()
                # Finish updating of initial conditions
                self.pddl.write("(:goal (continue)))\n")
            self.count = 0
            self.init_state = []
            self.status = self.create_plan()
            if self.status:
                pass
                # if the status is True then execute a service call to the snp_demo_gui
                # request sent with "fault_detected" bool and response in "fault_isolated" bool
                # based on the request the state machine with procedd or stop

    def create_init(self):
        # Initial conditions are updated here based on observation
        new_index = []
        states  = self.init_state
        lookFor = True
        i = 0
        index = 0
        try:
            while i < len(states):
                index = states.index(lookFor,i)
                i = index + 1
                new_index.append(index)
        except:pass
        for i in new_index:
            print >>self.pddl, '(stable P{})'.format(i),
        print >>self.pddl, ')'

    def create_plan(self):
        # Automated planners are used to generate plan
        domain = "/home/Automatica2018/snp_demo_ws/src/fault_observer/config/domain.pddl"   #contains init and goal states
        problem = "/home/Automatica2018/snp_demo_ws/src/fault_observer/config/problem.pddl"       #contains problme description
        self.parser = PDDL_Parser(domain_file = domain, problem_file = problem)
        self.plan = planner(self.parser,verbose=True)
        # error here
        print("\nDiagnosis steps\n")
        for action in self.plan:
            print(action)
        if self.plan:
            return True
        else:
            return False
