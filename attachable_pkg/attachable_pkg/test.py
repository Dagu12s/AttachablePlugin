from dis import pretty_flags
from fileinput import filename
import os
import sys
from xml.dom import minicompat 
import xacro
from lxml import etree
from lxml.builder import E # lxml only !
from lxml.builder import ElementMaker # lxml only !
from xml.etree.ElementTree import ElementTree
from string import digits

import mergeROSModels as merge


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
import mergeROSModels as merge

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterValue



filename = "../models/myfirst"
filename2 = "../models/urdfcreated"

# modelsList = ["body" , "leg"]
# modelNames = [ "0", "0"]
# parentLinks = ["body_0"]
# childLinks = ["leg_0"]

# merge.createURDF(filename, modelsList, modelNames, parentLinks, childLinks)


# model = "leg"
# modelNames =  "leg_1"
# parentLinks = "AttachableLink_1_body_1"
# childLinks = "AttachableLink_1_leg_1"

# merge.addModel(filename, parentLinks, childLinks)

# model = "leg"
# modelNames =  "leg2"
# parentLinks = "AttachableLink_2_body_1"
# childLinks = "AttachableLink_1_leg_2"

# merge.addModel(filename,  parentLinks, childLinks)

# model = "leg"
# modelNames =  "leg3"
# parentLinks = "AttachableLink_3_body_1"
# childLinks = "AttachableLink_1_leg_3"

# merge.addModel(filename,  parentLinks, childLinks)

# model = "leg"
# modelNames =  "leg1"
# parentLinks = "AttachableLink_1_body_1"
# childLinks = "AttachableLink_1_leg_1"
# 
# merge.addModel(filename,  parentLinks, childLinks)
# 

# ================================================================================================================= 
# model = "leg"
# modelNames =  "leg1"
# parentLinks = "AttachableLink_1_body_1"
# childLinks = "AttachableLink_1_leg_1"

# merge.removeModel(filename,  parentLinks, childLinks)
print("hola")

parentLinks = ["AttachableLink_1_body_1","AttachableLink_2_body_1","AttachableLink_3_body_1","AttachableLink_2_body_2","AttachableLink_3_body_2"]
childLinks =  ["AttachableLink_1_leg_1","AttachableLink_1_leg_2","AttachableLink_1_body_2","AttachableLink_1_leg_3","AttachableLink_1_leg_4"]

merge.createURDF2( filename2, parentLinks, childLinks)



# ================================================================================================================= 


# class myclient(Node):
#     def __init__(self):
#         super().__init__('myclient')
#         self.cliente = self.create_client(SetParameters, '/robot_state_publisher/set_parameters')
#         while not self.cliente.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('service not available, waiting again...')
#         self.req = SetParameters.Request()
#         self.parametervalue = ParameterValue()
#         self.parameter = Parameter()
#     def send_request(self):

#         print(1)
#         self.parametervalue.type=4
#         self.parametervalue.bool_value=False
#         self.parametervalue.integer_value=0
#         self.parametervalue.double_value=0.0
#         self.parametervalue.string_value='<?xml version="1.0" ?><robot name="test"><link name="bodyBase_body_1"><visual><origin rpy="0 0 0" xyz="0 0 0"/><geometry><mesh filename="package://rm2_simulation/models/rm2_body/meshes/b_simple.dae"/></geometry></visual><collision><geometry><mesh filename="package://rm2_simulation/models/rm2_body/meshes/body_collision.stl"/></geometry></collision><inertial><mass value="1.8"/><inertia ixx="2.501" ixy="0" ixz="0" iyy="2.501" iyz="0.0" izz="5"/></inertial></link><link name="AttachableLink_1_body_1"/><joint name="AttachableLinkJoint_1_body_1" type="fixed"><parent link="bodyBase_body_1"/><child link="AttachableLink_1_body_1"/><origin rpy="0 0 0" xyz="0.0 0 0.085"/></joint></robot>'
#         #self.parametervalue.string_value='<?xml version="1.0" ?><robot name="test"><link name="bodyBase_body_1"><visual><origin rpy="0 0 0" xyz="0 0 0"/><geometry><mesh filename="package://rm2_simulation/models/rm2_body/meshes/b_simple.dae"/></geometry></visual><collision><origin rpy="0 0 0" xyz="0 0 0"/><geometry><mesh filename="package://rm2_simulation/models/rm2_body/meshes/body_collision.stl"/></geometry></collision><inertial><mass value="1.8"/><inertia ixx="2.501" ixy="0" ixz="0" iyy="2.501" iyz="0.0" izz="5"/></inertial></link><link name="AttachableLink_1_body_1"/><joint name="AttachableLinkJoint_1_body_1" type="fixed"><parent link="bodyBase_body_1"/><child link="AttachableLink_1_body_1"/><origin rpy="0 0 0" xyz="0.0 0 0.085"/></joint><link name="AttachableLink_2_body_1"/><joint name="AttachableLinkJoint_2_body_1" type="fixed"><parent link="bodyBase_body_1"/><child link="AttachableLink_2_body_1"/><origin rpy="-2.0944 0 0" xyz="0.0 0.0736 -0.0425"/></joint><link name="AttachableLink_3_body_1"/><joint name="AttachableLinkJoint_3_body_1" type="fixed"><parent link="bodyBase_body_1"/><child link="AttachableLink_3_body_1"/><origin rpy="2.0944 0 0" xyz="0.0 -0.0736 -0.0425"/></joint></robot>'
#         self.parametervalue.byte_array_value=[]
#         self.parametervalue.bool_array_value=[]
#         self.parametervalue.integer_array_value=[]
#         self.parametervalue.double_array_value=[]
#         self.parametervalue.string_array_value=[]

#         self.parameter.name = 'robot_description'
#         self.parameter.value = self.parametervalue
#         print(2)
#         self.req.parameters = [self.parameter]
#         print(3)
#         self.future = self.cliente.call_async(self.req)
#         print(4)




# def main(args =None):
#     rclpy.init(args=args)

#     my_client = myclient()
#     my_client.send_request()



#     my_client.destroy_node()
#     rclpy.shutdown()
#     print("END")


# if __name__ == '__main__':
#     main()

# model = modelNames.split("_")[0]
# <!--robot name="test"><link name="bodyBase_body_1"><visual><origin rpy="0 0 0" xyz="0 0 0"/><geometry><mesh filename="package://rm2_simulation/models/rm2_body/meshes/b_simple.dae"/></geometry></visual><collision><geometry><mesh filename="package://rm2_simulation/models/rm2_body/meshes/body_collision.stl"/></geometry></collision><inertial><mass value="1.8"/><inertia ixx="2.501" ixy="0" ixz="0" iyy="2.501" iyz="0.0" izz="5"/></inertial></link><link name="AttachableLink_1_body_1"/><joint name="AttachableLinkJoint_1_body_1" type="fixed"><parent link="bodyBase_body_1"/><child link="AttachableLink_1_body_1"/><origin rpy="0 0 0" xyz="0.0 0 0.085"/></joint></robot-->

# '<robot name="test"><link name="bodyBase_body_1"><visual><origin rpy="0 0 0" xyz="0 0 0"/><geometry><mesh filename="package://rm2_simulation/models/rm2_body/meshes/b_simple.dae"/></geometry></visual><collision><geometry><mesh filename="package://rm2_simulation/models/rm2_body/meshes/body_collision.stl"/></geometry></collision><inertial><mass value="1.8"/><inertia ixx="2.501" ixy="0" ixz="0" iyy="2.501" iyz="0.0" izz="5"/></inertial></link><link name="AttachableLink_1_body_1"/><joint name="AttachableLinkJoint_1_body_1" type="fixed"><parent link="bodyBase_body_1"/><child link="AttachableLink_1_body_1"/><origin rpy="0 0 0" xyz="0.0 0 0.085"/></joint></robot>'
# print(model)
#'<robot name="test"><link name="bodyBase_body_1"><visual><origin rpy="0 0 0" xyz="0 0 0"/><geometry><mesh filename="package://rm2_simulation/models/rm2_body/meshes/b_simple.dae"/></geometry></visual><collision><geometry><mesh filename="package://rm2_simulation/models/rm2_body/meshes/body_collision.stl"/></geometry></collision><inertial><mass value="1.8"/><inertia ixx="2.501" ixy="0" ixz="0" iyy="2.501" iyz="0.0" izz="5"/></inertial></link><link name="AttachableLink_1_body_1"/><joint name="AttachableLinkJoint_1_body_1" type="fixed"><parent link="bodyBase_body_1"/><child link="AttachableLink_1_body_1"/><origin rpy="0 0 0" xyz="0.0 0 0.085"/></joint></robot>'
# parent = parentLinks.replace("AttachableLink_","")
# print(parent)

# parentLinks_split = parentLinks.split("_")
# childLinks_split = childLinks.split("_")
# child = "AttachableJoint_" + parentLinks.split("_")[1] + parentLinks.split("_")[2] + parentLinks.split("_")[3] + "_" + childLinks.split("_")[1]+ childLinks.split("_")[2]+ childLinks.split("_")[3]

# print(child)
# urdf_file = "myfirst.xacro"

# xacro_file = os.path.join("models",urdf_file)

# robot_description_config = xacro.process_file(xacro_file)
# robot_desc = robot_description_config.toxml()
  
# print(robot_desc)


# E = ElementMaker(nsmap={"xacro" : "http://wiki.ros.org/xacro"})

# I = ElementMaker(namespace="http://wiki.ros.org/xacro")

# number = sys.argv[1]
# print(number)
# robot = E.robot
# body = I.body
# xacro_file = (
#     robot(
#         name="test",
#         body(sufix="bodyLink")
#         )
#     )

#E = etree.Element("robot",name="test" xmlns:xacro="http://wiki.ros.org/xacro")
# robot_desc = xacro_file.toxml()
# print(robot_desc)

# def NAME(*args): # class is a reserved word in Python
#      return {"name":' '.join(args)}
# def TYPE(*args): # class is a reserved word in Python
#      return {"type":' '.join(args)}


# ROBOT = ElementMaker(nsmap={"xacro" : "http://wiki.ros.org/xacro"})
# xacrons =  ElementMaker( namespace="http://wiki.ros.org/xacro")



# xacro_file = (
#   ROBOT.robot(
#       xacrons("include",filename="import_path.xacro")
#     )
#   )

# xacro_file.append(xacrons( "body", sufix = "body_link"))


# print(xacro_file.tag)


# # tree = ElementTree(xacro_file)

# # with open("myfirst2.xacro","w") as urdf_file:
# #     tree.write(urdf_file,  encoding='unicode')
#     #urdf_file.write(etree.tostring(xacro_file, pretty_print=True))


# # robot_description_config = xacro.process_file(xacro_file)
# # robot_description_config = xacro.process(xacro_file)
# # robot_desc = robot_description_config.toxml()


# #print(etree.tostring(xacro_file, pretty_print=True))

# with open("myfirst2.xacro","wb") as urdf_file:
#     urdf_file.write(etree.tostring(xacro_file, pretty_print=True))
# h

# robot_description_config = xacro.process_file("myfirst2.xacro",pretty_print=True)
# #robot_description_config = xacro.parse(etree.tostring(xacro_file, pretty_print=True,encoding="unicode"))
# robot_desc = robot_description_config.toprettyxml()

# with open("myfirst2.urdf","w") as urdf_file:
#     urdf_file.write(robot_desc)