
# AttachablePlugin
=======
# AttachablePlugin (AttachableJoint)
This is a plugin for Ignitions that generates a joint Dinamically during simulation with a topic where you send a string that contains parent model, parent link, child model and child link.
It can be used to grab things in gazebo and for modular robots.

Download repository
~~~
git clone 
~~~
it should create a directory called "AttachablePlugin"



Include the plugin in the .sdf world.

It does not need to be inside a model. You have to add the topic for creating the link and the topic for destroy the link
you may have to change the filename
~~~
<plugin filename=" ... /libAttachableJoint.so" name="attachable_joint::AttachableJoint">
</plugin>
~~~



Create and Destroy the Link Dinamically

For creating the link you have to send a ignition::msgs::StringMsg with this architecture:
[parentModel][ParentLink][ChildModel][ChildLink][attach]
If you want to detach use [detach]
like this
~~~
ign topic -t /AttachableJoint -m ignition.msgs.StringMsg -p 'data:"[parentModel][ParentLink][ChildModel][ChildLink][attach]"'
~~~

You can send it from ROS2, see https://github.com/ignitionrobotics/ros_ign/tree/melodic/ros_ign_bridge







# Installation
I compiled the binary for ROS 2 Foxy (Ubuntu 20.04) you can use .so or compile it yourself. To compile you need ignition from source.



# Instalation from Source:
Add the plugin to ignition (workspace is where ignition from source is installed):

Download repository
~~~
git clone 
~~~
it should create a directory called "AttachablePlugin" in "home"

You can paste the files directly inside the directory "system_plugin" in "/workspace/src/ign-gazebo/examples/examples/system_plugin"
or you can make link to the file.


~~~
cd ~/workspace/src/ign-gazebo/examples/examples/system_plugin

rm -r AttachableJoint.*
rm CMakeLists.txt
ln -s /AttachablePlugin/AttachableJoint/CMakeLists.txt 
ln -s /AttachablePlugin/AttachableJoint/AttachableJoint.cc 
ln -s /AttachablePlugin/AttachableJoint/AttachableJoint.hh
ln -s /AttachablePlugin/AttachableJoint/AttachableJoint.sdf

~~~


For build the plugin, create a directory named build inside "system_plugin" and run:
~~~
mkdir build
cd build
cmake ..
make
~~~


Add library:
~~~
cd ign-gazebo/examples/plugins/system_plugin
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=`pwd`/build
~~~

