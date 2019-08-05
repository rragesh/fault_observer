#!/usr/bin/python
import rospy
import actionlib
import numpy
from sensor_msgs.msg import JointState
from sensor_msgs.msg import CameraInfo
from ensenso_camera_msgs.msg import RequestDataAction, RequestDataActionGoal, RequestDataActionResult
from fault_observer.srv import fault_detect, fault_detectResponse
# Automated planner modules
import sys
import pprint
from fault_observer.AutomatedPlanner.pddl_parser import *
from fault_observer.AutomatedPlanner.planner import *

class Observer(object):

    def __init__(self):
        self.scan_running  = False
        self.trigger_result = False
        self.status = False
        self.compare = False
        self.count = 0
        self.init_state = []
        self.vel = []
        self.goal_sub = rospy.Subscriber("request_data/goal", RequestDataActionGoal, self.goal_cb)
        self.result_sub = rospy.Subscriber("request_data/result", RequestDataActionResult, self.result_cb)
        faultServer = rospy.Service('fault_detect', fault_detect, self.create_problem)
        # self.joint_state_sub = rospy.Subscriber("/prbt/joint_states", JointState, self.joint_state_cb)

    def goal_cb(self,msg):
        # callback to subscribe for the action goal
        self.scan_running = True
        print('\nMonitoring the joint velocity')
        self.js_sub = rospy.Subscriber("/prbt/joint_states",JointState, self.joint_state_cb)

    def result_cb(self,msg):
        # callback to subscribe for the action result
        self.scan_running = False
        self.trigger_result = True
        # count number of captures
        self.js_sub.unregister()
        self.count = self.count+1
        # check for the vibration
        self.compare = all(x == self.vel[0] for x in self.vel)
        self.init_state.append(self.compare)
        rospy.loginfo("Robot stopped = %r"%self.compare)
        self.compare = False
        self.trigger_result = False
        self.vel = []

    def joint_state_cb(self,msg):
        # callback to subscribe for the joint state position
        if self.scan_running:
            self.vel.append(msg.velocity)

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
        rospy.loginfo("Diagnosis steps")
        if bool(self.plan):
            for action in self.plan:
                rospy.loginfo(action)

    def create_problem(self, req):
        # Create PDDL problem statement
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
            self.create_plan()
            # Service server response
            # return fault_detectResponse(False, "plan not found")
            if bool(self.plan)== True:
                resp = False
                msg_resp = "No faults detected in Scan"
            else:
                resp = True
                msg_resp = "Fault detected and Isolated "

            return fault_detectResponse(resp, msg_resp)


# godel_bridge.cpp *--Modifications--*

  #
  # #include "fault_observer/fault_detect.h"
  #
  # fault_observer::fault_detect srv_fault_detect;
  # if (!ros::service::call("fault_detect", srv_fault_detect)) return makeError("Unable to call fault detection service");
  # log(srv_fault_detect.response.fault_message);
  # if (srv_fault_detect.response.fault_detected) return makeError("Fault detected:" + srv_fault_detect.response.fault_message);
