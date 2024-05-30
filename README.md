**Robot Skill Language**

**Skills and skillsets to move Donatello Turtle**
****
Start by changing directory to your workspace!

**1. Install packages**

<ins> **Make sure that you have installed the following packages:** </ins>


* sudo apt install ros-humble-desktop-full

* sudo apt install ros-humble-rqt*

<ins> **Add these commands to the .bashrc file:** </ins>

* source /opt/ros/humble/setup.bash        

* source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash     
      
* source ~/ros2_ws/install/setup.bash       


<ins> **Install the turtlesim package:** </ins>

We use turtlesim package to control the donatello turtle, so needs to install turtlesim package.

* sudo apt update

* sudo apt install ros-galactic-turtlesim
****
<ins> **Run following command to create 3 directories - build, install, and log in your workspace:** </ins>

* colcon build  --symlink-install
****
 
**2. Clone Package RSL_Donatello_Skillsets**

Before running the following command create a folder src in your workspace and cd directory to this folder in Terminator, to copy all necessary files in src folder.

* git clone https://github.com/me2m/skillsets_for_donatello.git
****

**3. Generate Skillset Codes and User Packages for Donatello**

Run the following command from the same directory.

* python3 -m robot_language turtle.rl -g turtle.json

* python3 -m robot_language turtle.rl -g turtle.json -p donatello
****
    

**4. Build and Source**

Before running skillsets we need to build and source the project again. Run the following commands.

* colcon build

* source install/setup.bash
****


**5. Running Skillsets**

<ins> **Initializing, Customizing and running Donatello Turtle:** </ins>

* ros2 run turtlesim turtlesim_node

* ros2 service call /kill turtlesim/srv/Kill "name: turtle1"

* ros2 service call /kill turtlesim/srv/Kill "{name: donatello}"

* ros2 service call /clear std_srvs/srv/Empty "{}"

* ros2 service call /spawn turtlesim/srv/Spawn "{x: 5.0, y: 5.0, name: 'donatello'}"

* ros2 service call /donatello/set_pen turtlesim/srv/SetPen "{r: 75, g: 0, b: 130, width: 5}"

* ros2 run donatello donatello_node
****

<ins> **Initial Position and Status of Donatello turtle:** </ins>

* ros2 topic echo /donatello_node/turtle_skillset/data/pose

* ros2 topic echo /donatello_node/turtle_skillset/status

* ros2 topic pub -1 /donatello_node/turtle_skillset/event_request turtle_skillset_interfaces/msg/EventRequest "{id: '', name: 'authority_to_skill'}"
****

<ins> **Running 'Move Forward' skill** </ins>

* ros2 topic echo /donatello_node/turtle_skillset/skill/move_forward/response

* ros2 topic pub -1 /donatello_node/turtle_skillset/skill/move_forward/request turtle_skillset_interfaces/msg/SkillMoveForwardRequest "{id: '', input: { distance: 2.0, speed: 0.2 }}"
****

<ins> **Running 'Move In Circle' skill** </ins>

* ros2 topic echo /donatello_node/turtle_skillset/skill/move_in_circle/response

* ros2 topic pub -1 /donatello_node/turtle_skillset/skill/move_in_circle/request turtle_skillset_interfaces/msg/SkillMoveInCircleRequest "{id: '', input: { radius: 2.0, speed: 0.2 }}"
****

<ins> **Running 'Rotate Angle' skill** </ins>

* ros2 topic echo /donatello_node/turtle_skillset/skill/rotate_angle/response

* ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=donatello/cmd_vel

* ros2 topic pub -1 /donatello_node/turtle_skillset/skill/rotate_angle/request turtle_skillset_interfaces/msg/SkillRotateAngleRequest "{id: '', input: { angle: 314, speed: 0.5}}"
****
