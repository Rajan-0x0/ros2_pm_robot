Robot Skill Language

Skills and skillsets to move Donatello Turtle

python3 -m robot_language turtle.rl -g turtle.json

python3 -m robot_language turtle.rl -g turtle.json -p donatello

colcon build

source install/setup.bash

ros2 run turtlesim turtlesim_node

ros2 service call /kill turtlesim/srv/Kill "name: turtle1"
ros2 service call /kill turtlesim/srv/Kill "{name: donatello}"
ros2 service call /clear std_srvs/srv/Empty "{}"
ros2 service call /spawn turtlesim/srv/Spawn "{x: 5.0, y: 5.0, name: 'donatello'}"
ros2 service call /donatello/set_pen turtlesim/srv/SetPen "{r: 75, g: 0, b: 130, width: 5}"

ros2 run donatello donatello_node

ros2 topic echo /donatello_node/turtle_skillset/data/pose

ros2 topic echo /donatello_node/turtle_skillset/status

ros2 topic pub -1 /donatello_node/turtle_skillset/event_request turtle_skillset_interfaces/msg/EventRequest "{id: '', name: 'authority_to_skill'}"




ros2 topic echo /donatello_node/turtle_skillset/skill/move_forward/response

ros2 topic pub -1 /donatello_node/turtle_skillset/skill/move_forward/request turtle_skillset_interfaces/msg/SkillMoveForwardRequest "{id: '', input: { distance: 2.0, speed: 0.2 }}"




ros2 topic echo /donatello_node/turtle_skillset/skill/move_in_circle/response

ros2 topic pub -1 /donatello_node/turtle_skillset/skill/move_in_circle/request turtle_skillset_interfaces/msg/SkillMoveInCircleRequest "{id: '', input: { radius: 2.0, speed: 0.2 }}"
