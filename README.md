**Robot Skill Language**

**Skills and skillsets to move Donatello Turtle**


**1. Install packages**

**Make sure that you have installed the following packages:**


sudo apt install ros-humble-desktop-full

source /opt/ros/humble/setup.bash        

source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash     
      
source ~/ros2_ws/install/setup.bash        
      
colcon build  --symlink-install
****

**2. Generate Skillset Codes and User Packages for Donatello**

python3 -m robot_language turtle.rl -g turtle.json

sudo apt install ros-humble-xacro
****
    

**3. Build and Source**

colcon build

source install/setup.bash
****


**4. Running Skillsets**

**Initializing, Customizing and running Donatello Turtle:**

ros2 run turtlesim turtlesim_node

ros2 service call /kill turtlesim/srv/Kill "name: turtle1"

ros2 service call /kill turtlesim/srv/Kill "{name: donatello}"

ros2 service call /clear std_srvs/srv/Empty "{}"

ros2 service call /spawn turtlesim/srv/Spawn "{x: 5.0, y: 5.0, name: 'donatello'}"

ros2 service call /donatello/set_pen turtlesim/srv/SetPen "{r: 75, g: 0, b: 130, width: 5}"

ros2 run donatello donatello_node
****

**Initial Position and Status of Donatello turtle:**

ros2 topic echo /donatello_node/turtle_skillset/data/pose

ros2 topic echo /donatello_node/turtle_skillset/status

ros2 topic pub -1 /donatello_node/turtle_skillset/event_request turtle_skillset_interfaces/msg/EventRequest "{id: '', name: 'authority_to_skill'}"
****

**Running 'Move Forward' skill**

ros2 topic echo /donatello_node/turtle_skillset/skill/move_forward/response

ros2 topic pub -1 /donatello_node/turtle_skillset/skill/move_forward/request turtle_skillset_interfaces/msg/SkillMoveForwardRequest "{id: '', input: { distance: 2.0, speed: 0.2 }}"
****

**Running 'Move In Circle' skill**

ros2 topic echo /donatello_node/turtle_skillset/skill/move_in_circle/response

ros2 topic pub -1 /donatello_node/turtle_skillset/skill/move_in_circle/request turtle_skillset_interfaces/msg/SkillMoveInCircleRequest "{id: '', input: { radius: 2.0, speed: 0.2 }}"
****
