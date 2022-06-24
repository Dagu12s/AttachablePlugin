#!/usr/bin/env python3
from mimetypes import init
from re import S
import time
from urllib import request

from numpy import empty

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from rm2_simulation.action import AttachModel
from std_msgs.msg import String
from std_msgs.msg import Empty
from std_msgs.msg import Bool
import attachable_pkg.mergeROSModels as merge

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterValue

class AttachableJointActionServer(Node):

    def __init__(self):
        self.contact = False
        super().__init__('AttachableJointActionServer')
        self._action_server = ActionServer(
            self,
            AttachModel,
            'AttachableJoint',
            self.execute_callback)
        self.robotStatePublisherClient = self.create_client(SetParameters, '/robot_state_publisher/set_parameters')
        self.contactPublisherTopic = "/AttacherContact/contact"
        self.contactSubscriberTopic = "/AttacherContact/touched"
        self.contactPublisher = self.create_publisher(String, self.contactPublisherTopic,10)
        self.contactSubscriber = self.create_subscription(Bool, self.contactSubscriberTopic, self.contactSubscriber_callack, 10)

        self.filename = "/myfirst"
        self.msgToPublish = String()

        self.error = False
            

    def contactSubscriber_callack(self, msg):
        self.contact = msg.data        


    def waitToResponse(self, timeout):
        initTime = time.time()
        while rclpy.ok():
            rclpy.spin_once(self)
            if ((time.time() - initTime) > timeout):
                break


    def attachModelIgnition(self):
        attachableJointPublisher = self.create_publisher(String, "/" + self.parentLink + "/attach" ,10)
        parentSplit = self.parentLink.split("_")
        childSplit = self.childLink.split("_")
        parentModel = parentSplit[2] + "_" + parentSplit[3]
        childModel =   childSplit[2] + "_" + childSplit[3]
        self.msgToPublish.data = 'data: "[{}][{}][{}][{}]" '.format(parentModel, self.parentLink, childModel, self.childLink)
        attachableJointPublisher.publish(self.msgToPublish)


    def dettachModelIgnition(self):
        attachableJointPublisher = self.create_publisher(Empty, "/" + self.parentLink + "/detach" ,10)
        emptyMsg = Empty()
        attachableJointPublisher.publish(emptyMsg)


    def restartStatePublisher(self):
        self.get_logger().info('Restart State Publisher...')
        
        req = SetParameters.Request()

        parametervalue = ParameterValue()
        parameter = Parameter()

        parametervalue.type=4
        parametervalue.bool_value=False
        parametervalue.integer_value=0
        parametervalue.double_value=0.0
        
        parametervalue.byte_array_value=[]
        parametervalue.bool_array_value=[]
        parametervalue.integer_array_value=[]
        parametervalue.double_array_value=[]
        parametervalue.string_array_value=[]

        file = open(self.filename+".urdf")
        parametervalue.string_value = file.read()
        file.close()

        parameter.name = 'robot_description'
        parameter.value = parametervalue

        req.parameters = [parameter]

        future = self.robotStatePublisherClient.call_async(req)

        # robot_description_config = xacro.process_file(self.filename)
        # robot_desc = robot_description_config.toxml()
  
        
#self.parametervalue.string_value='<?xml version="1.0" ?><robot name="test"><link name="bodyBase_body_1"><visual><origin rpy="0 0 0" xyz="0 0 0"/><geometry><mesh filename="package://rm2_simulation/models/rm2_body/meshes/b_simple.dae"/></geometry></visual><collision><geometry><mesh filename="package://rm2_simulation/models/rm2_body/meshes/body_collision.stl"/></geometry></collision><inertial><mass value="1.8"/><inertia ixx="2.501" ixy="0" ixz="0" iyy="2.501" iyz="0.0" izz="5"/></inertial></link><link name="AttachableLink_1_body_1"/><joint name="AttachableLinkJoint_1_body_1" type="fixed"><parent link="bodyBase_body_1"/><child link="AttachableLink_1_body_1"/><origin rpy="0 0 0" xyz="0.0 0 0.085"/></joint></robot>'
#self.parametervalue.string_value='<?xml version="1.0" ?><robot name="test"><link name="bodyBase_body_1"><visual><origin rpy="0 0 0" xyz="0 0 0"/><geometry><mesh filename="package://rm2_simulation/models/rm2_body/meshes/b_simple.dae"/></geometry></visual><collision><origin rpy="0 0 0" xyz="0 0 0"/><geometry><mesh filename="package://rm2_simulation/models/rm2_body/meshes/body_collision.stl"/></geometry></collision><inertial><mass value="1.8"/><inertia ixx="2.501" ixy="0" ixz="0" iyy="2.501" iyz="0.0" izz="5"/></inertial></link><link name="AttachableLink_1_body_1"/><joint name="AttachableLinkJoint_1_body_1" type="fixed"><parent link="bodyBase_body_1"/><child link="AttachableLink_1_body_1"/><origin rpy="0 0 0" xyz="0.0 0 0.085"/></joint><link name="AttachableLink_2_body_1"/><joint name="AttachableLinkJoint_2_body_1" type="fixed"><parent link="bodyBase_body_1"/><child link="AttachableLink_2_body_1"/><origin rpy="-2.0944 0 0" xyz="0.0 0.0736 -0.0425"/></joint><link name="AttachableLink_3_body_1"/><joint name="AttachableLinkJoint_3_body_1" type="fixed"><parent link="bodyBase_body_1"/><child link="AttachableLink_3_body_1"/><origin rpy="2.0944 0 0" xyz="0.0 -0.0736 -0.0425"/></joint></robot>'


    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        result = AttachModel.Result()

        feedback_msg = AttachModel.Feedback()
        self.childLink = goal_handle.request.child
        self.parentLink = goal_handle.request.parent

        self.get_logger().info("Executing Goal...")

        if True == goal_handle.request.attach:
                
            #Check if the linsk are touching       
            self.msgToPublish.data = 'data: "[{}][{}]" '.format(self.parentLink, self.childLink)
            self.contactPublisher.publish(self.msgToPublish)

            self.waitToResponse(0.5)

            if self.contact:

                self.get_logger().info("Links are in contact, Attaching models...")

                #Add model in Gazebo Ingition
                self.attachModelIgnition()

                #Add model in URDF
                merge.addModel(self.filename, "AttachableLink_1_body_1", "AttachableLink_1_leg_1" )#self.parentLink, self.childLink)

                #Reestart State Publisher with the new URDF
                self.restartStatePublisher()
                
                result.response = "True"

            else:
                self.get_logger().info("Links are NOT in contact. End.")
                result.response = "False"
        

            self.msgToPublish.data = 'data:"end"'
            self.contactPublisher.publish(self.msgToPublish)
        

        else:
            #Detach Models in ignition
            self.dettachModelIgnition()
            #Remove model in URDF
            merge.removeModel(self.filename, self.parentLink, self.childLink )

            #Reset robot state publisher
            self.restartStatePublisher()

            result.response = "True"

        result.response = "Contact Done: " + result.response + " || Error: " + str(self.error)
        goal_handle.succeed()
        

        return result



def main(args=None):
    rclpy.init(args=args)


    attachable_joint_action_server = AttachableJointActionServer()
    
    while rclpy.ok():
        rclpy.spin_once(attachable_joint_action_server)
    

if __name__ == '__main__':
    main()

