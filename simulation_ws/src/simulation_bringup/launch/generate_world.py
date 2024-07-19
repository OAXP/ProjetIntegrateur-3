# python file to randomly generate gazebo world : https://gitlab.com/polytechnique-montr-al/inf3995/20241/equipe-106/INF3995-106/-/issues/13

import random
from ament_index_python.packages import get_package_share_directory
import os

num_cubes = 4
cube_poses = []
for _ in range(num_cubes):
    x = random.choice([random.uniform(-4.0, -1.0), random.uniform(1.0, 4.0)]) 
    y = random.choice([random.uniform(-4.0, -1.0), random.uniform(1.0, 4.0)])
    z = 0     
    yaw = random.uniform(0, 2 * 3.14159)  
    cube_poses.append((x, y, z, 0, 0, yaw))  

cubes = ''
for i, pose in enumerate(cube_poses):
    cubes += f"""
    <model name="random_cube_{i + 1}">
      <pose>{pose[0]} {pose[1]} {pose[2]} {pose[3]} {pose[4]} {pose[5]}</pose> 
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size> 
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Blue</name>
            </script>
          </material>
        </visual>
      </link>
    </model>
    """

# Insert the generated SDF cubes into the main SDF file
sdf_content = f"""
<?xml version="1.0"?>
<sdf version="1.8">
  <world name="demo">
    <!-- Define the physics properties -->
    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>

    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>


    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- Enclosing walls -->
    <model name="enclosing_wall_1">
      <pose>5 0 1.0 0 0 1.57</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>10 0.1 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>10 0.1 2</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Red</name>
            </script>
          </material>
        </visual>
      </link>
    </model>

     <model name="enclosing_wall_2">
      <pose>0 5 1.0 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>10 0.1 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>10 0.1 2</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Red</name>
            </script>
          </material>
        </visual>
      </link>
    </model>

    <model name="enclosing_wall_3">
      <pose>-5 0 1.0 0 0 1.57</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>10 0.1 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>10 0.1 2</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Red</name>
            </script>
          </material>
        </visual>
      </link>
    </model>

    <model name="enclosing_wall_4">
      <pose>0 -5 1.0 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>10 0.1 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>10 0.1 2</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Red</name>
            </script>
          </material>
        </visual>
      </link>
    </model>
    {cubes}
    <model name="diff_drive" ns="limo_0">
      <self_collide>true</self_collide>
      <pose>0 0 0.35 0 0 0</pose>
      <include merge="true">
        <uri>package://limo_description/models/limo_diff_drive</uri>
      </include>
    </model>

    <model name="diff_drive1" ns="limo_1">
      <self_collide>true</self_collide>
      <pose>0 1 0.35 0 0 3.14159</pose>
      <include merge="true">
        <uri>package://limo_description/models/limo_diff_drive1</uri>
      </include>
    </model>
  </world>
</sdf>
"""

pkg_project_gazebo = get_package_share_directory('ros_gazebo_pkg')
path = os.path.join(pkg_project_gazebo, 'worlds', 'diff_drive.sdf')
with open(path, 'w') as f:
    f.write(sdf_content)


