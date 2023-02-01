#!/usr/bin/env python3
"""
Created on Mon Dec  2 17:03:34 2019

@author: mahmoud
"""
import math
from rospkg import RosPack
import xml.etree.ElementTree as ET 

global gzb_world
def generate_pose(x1, y1, x2, y2):
    if x2-x1==0:
        print ( "\t  <pose frame=''>{} {}  1.4 0 0 {}</pose>".format( (x2+x1)/2, (y2+y1)/2, 1.57 ), file = gzb_world)#x1, y1, math.atan((y2-y1)/(x2-x1)) )
    else:
        print ("\t  <pose frame=''>{} {}  1.4 0 0 {}</pose>".format( (x2+x1)/2, (y2+y1)/2, math.atan( (y2-y1)/(x2-x1) )  ), file = gzb_world)#x1, y1, math.atan((y2-y1)/(x2-x1)) )

def generate_size(x1, y1, x2, y2):
    l = math.sqrt( pow(y2-y1,2) + pow(x2-x1,2)  )
    print ("\t     <size> {} .2  2.8 </size>".format( l ), file = gzb_world)
    
    
    
def generate_obstacle(x1, y1, x2, y2, idx):
     print ('''
     <model name='grey_wall_{}'>
      <static>1</static>
      <link name='link'>
      '''.format(idx) , file = gzb_world)
     generate_pose(x1, y1, x2, y2)     
     print ('''    
        <collision name='collision'>
          <geometry>
            <box>
            ''' , file = gzb_world)
     generate_size(x1, y1, x2, y2)
     print ('''
      \t   </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <box>
            ''', file = gzb_world)
     generate_size(x1, y1, x2, y2)
     print ('''
      \t   </box>
       </geometry>
          <material>
            <script>
              <uri>model://grey_wall/materials/scripts</uri>
              <uri>model://grey_wall/materials/textures</uri>
              <name>vrc/grey_wall</name>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>
    '''.format( 1,1) , file = gzb_world)#math.sqrt( pow(y2-y1,2) + pow(x2-x1,2)  ) )



def parseXML(xmlfile): 
    tree = ET.parse(xmlfile) 
    root = tree.getroot() 
    idx = 0
    for item in root:
        if item.tag == 'obstacle':
            idx= idx+1
            x1= item.attrib['x1']
            y1= item.attrib['y1']
            x2= item.attrib['x2']
            y2= item.attrib['y2']
            generate_obstacle( float(x1), float(y1), float(x2), float(y2), idx)

    
def generate_gzb_world( pedsim_file_name ): 
    rospack1 = RosPack()
    pkg_path = rospack1.get_path('pedsim_simulator')
    xml_scenario =  pkg_path + "/scenarios/" + pedsim_file_name #bo_airport.xml"
    rospack2 = RosPack()
    pkg_path = rospack2.get_path('pedsim_gazebo_plugin')
    gazebo_world =  pkg_path + "/worlds/" + pedsim_file_name.split('.')[0] + ".world" #bo_airport.xml"
    global gzb_world    
    with open(gazebo_world, 'w') as gzb_world:
        print ("<?xml version=\"1.0\" ?>", file = gzb_world)
        print ('''
    <!-- this file is auto generated using pedsim_gazebo_plugin pkg -->    
    <sdf version="1.5">
      <world name="default">
      
      <!-- Ground Plane -->
      
      <include>
        <uri>model://ground_plane</uri>
      </include>
    
      <include>
        <uri>model://sun</uri>
      </include>
    
        ''', file = gzb_world)
        # use the parse() function to load and parse an XML file
    #    xmlfile =  pkg_path + "/scenarios/obstacle.xml"
        parseXML(xml_scenario)
        print ('''
            <plugin name="ActorPosesPlugin" filename="libActorPosesPlugin.so">
        </plugin>
    
    
      </world>
    </sdf>
    
        ''', file = gzb_world)
    print ("gazbo world has been generated: {}".format( gazebo_world))
      
      
      
def generate_launch_file( pedsim_file_name ): 
    rospack2 = RosPack()
    pkg_path = rospack2.get_path('pedsim_gazebo_plugin')
    launch_file =  pkg_path + "/launch/" + pedsim_file_name.split('.')[0] + ".launch" #bo_airport.xml"
    with open(launch_file, 'w') as launch:
        print ('''
        <launch>

  <arg name="robot_model"                       default="locobot_base"/>
  <arg name="robot_name"                        default="locobot"/>
  <arg name="arm_model"                         default="$(eval 'mobile_' + arg('robot_model').split('_')[1])"/>
  <arg name="show_lidar"                        default="true"/>
  <arg name="show_gripper_bar"                  default="false"/>
  <arg name="show_gripper_fingers"              default="false"/>
  <arg name="external_urdf_loc"                 default=""/>
  <arg name="use_rviz"                          default="false"/>
  <arg name="rviz_frame"                        default="$(arg robot_name)/base_footprint"/>
  <arg name="gui"                               default="true"/>
  <arg name="debug"                             default="false"/>
  <arg name="paused"                            default="false"/>
  <arg name="recording"                         default="false"/>
  <arg name="use_sim_time"                      default="true"/>
  <arg name="use_position_controllers"          default="false"/>
  <arg name="use_trajectory_controllers"        default="false"/>
  <arg name="dof"                               default="5"/>
  <arg name="kbd_teleop" default="false"/>
  <arg name="rqt_teleop" default="true"/>
  <arg name="visualize" default="true"/>
  <arg name="with_robot" default="true"/>
  <arg name="absolute_loco" default="true"/>

  <arg name="simulation_factor" default="1.0"/> <!-- Speed up -->
  <arg name="update_rate" default="25.0"/> <!-- Hz -->

  <env name="GAZEBO_RESOURCE_PATH"              value="$(find interbotix_xslocobot_gazebo)"/>

  <rosparam file="$(find interbotix_xslocobot_gazebo)/config/locobot_gazebo_controllers.yaml" command="load" ns="$(arg robot_name)"/>

  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="gui"                               value="$(arg gui)" />
    <arg name="debug"                             value="$(arg debug)" />
    <arg name="paused"                            value="$(arg paused)"/>
    <arg name="recording"                         value="$(arg recording)"/>
    <arg name="use_sim_time"                      value="$(arg use_sim_time)"/>
    <arg name="world_name" value="$(find pedsim_gazebo_plugin)/worlds/{}.world"/>
  </include>


  <include file="$(find interbotix_xslocobot_descriptions)/launch/xslocobot_description.launch">
    <arg name="robot_model"                       value="$(arg robot_model)"/>
    <arg name="robot_name"                        value="$(arg robot_name)"/>
    <arg name="show_lidar"                        value="$(arg show_lidar)"/>
    <arg name="show_gripper_bar"                  value="$(arg show_gripper_bar)"/>
    <arg name="show_gripper_fingers"              value="$(arg show_gripper_fingers)"/>
    <arg name="external_urdf_loc"                 value="$(arg external_urdf_loc)"/>
    <arg name="use_rviz"                          value="$(arg use_rviz)"/>
    <arg name="rviz_frame"                        value="$(arg rviz_frame)"/>
  </include>


  <!-- Simulator -->
  <include file="$(find pedsim_simulator)/launch/simulator.launch">
    <arg name="kbd_teleop" value="$(arg kbd_teleop)"/>
    <arg name="rqt_teleop" value="$(arg rqt_teleop)"/>
    <arg name="scene_file" value="$(find pedsim_simulator)scenarios/{}.xml"/>
    <arg name="with_robot" value="$(arg with_robot)"/>
    <arg name="simulation_factor" value="$(arg simulation_factor)"/>
    <arg name="update_rate" value="$(arg update_rate)"/>
    <arg name="default_queue_size" value="10"/>
    <arg name="max_robot_speed" value="1.5"/>
    <arg name="robot_mode" value="1"/>
    <arg name="enable_groups" value="true"/>
    <arg name="pose_initial_x" value="0"/>
    <arg name="pose_initial_y" value="0"/>
    <arg name="pose_initial_theta" value="0"/>
    <arg name="absolute_loco"  value="$(arg absolute_loco)"/>"/>
  </include>

  <!-- Visualizer -->
  <include file="$(find pedsim_visualizer)/launch/visualizer.launch"/>

  <!-- Rviz -->
  <node pkg="rviz" type="rviz" name="rviz" args="-d $(find pedsim_simulator)/rviz/social_contexts_activities.rviz" if="$(arg visualize)"/>
  <node
    name="urdf_spawner"
    pkg="gazebo_ros"
    type="spawn_model"
    respawn="false"
    output="screen"
    ns="$(arg robot_name)"
	  args="-urdf -model $(arg robot_name) -param robot_description"/>
	  
  <node pkg="pedsim_gazebo_plugin" type="spawn_pedsim_agents.py" name="spawn_pedsim_agents"  output="screen"/>


</launch>
'''.format(pedsim_file_name.split('.')[0], pedsim_file_name.split('.')[0]), file = launch)
    print ("launch file has been generated: {}".format( launch_file ))
 

      
if __name__ == "__main__": 

    pedsim_file_name = input(">> enter pedsim scenaria name: file_name.xml \n")

    # genrate gazebo wolrd 
    generate_gzb_world( pedsim_file_name )     
    generate_launch_file( pedsim_file_name ) 
    print (">> after you launch the scenario using pedsim_simulator, launch the generated world using: ")
    print (" \" $roslaunch pedsim_gazebo_plugin {}.launch\"  ".format( pedsim_file_name.split('.')[0] ))
    
