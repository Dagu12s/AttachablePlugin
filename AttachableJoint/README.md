# AttachablePlugin (AttachableJoint)
This is a plugin for Ignitions that generates a joint Dinamically during simulation with a topic where you send a string that contains parent model, parent link, child model and child link.
It can be used to grab things in gazebo and for modular robots.

Add the plugin in
~~~
cd ign-gazebo/examples/plugin/system_plugin
~~~

For build the plugin, create a directory named build and run:
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

#Include the plugin in the .sdf world.
It does not need to be inside a model. You have to add the topic for creating the link and the topic for destroy the link
you may have to change the filename
~~~
<plugin filename="ign-gazebo/examples/plugin/system_plugin/build/libAttachableJoint.so" name="attachable_joint::AttachableJoint">
    <detachtopic>/attachablejoint/detach</detachtopic>
    <attachtopic>/attachablejoint/attach</attachtopic>
</plugin>
~~~

# Create and Destroy the Link Dinamically

For creating the link you have to send a ignition::msgs::StringMsg with this architecture:
[parentModel][ParentLink][ChildModel][ChildLink]
like this
~~~
ign topic -t /box2/attach -m ignition.msgs.StringMsg -p 'data:"[parentModel][ParentLink][ChildModel][ChildLink]"'
~~~
You can send it from ROS2, see https://github.com/ignitionrobotics/ros_ign/tree/melodic/ros_ign_bridge

