# RT_I-Assignement_I

The ROS nodes here contained allow for controlling an holonomic robot by computing the velocity needed to drive it towards a target position.
the target position is here defined as a simple couple of random values between (-6.0, 6.0).
All nodes and custom services are contained in the package **sol_1_pkg**, along with their documentation.

## Running the code

The nodes presented can be started together by calling the launchfile included in the package

```bash
$roslaunch sol_1_pkg sol_launch.xml
```

Alternatively, the single nodes can be called, individually, with the commands:

```bash
$$rosrun stage_ros stageros $(rospack find assignment1)/world/exercise.world

$rosrun sol_1_pkg robot_controlloer_server.py

$rosrun sol_1_pkg robot_controller_client.py
```

It is noteworthy, that the implemend nodes provide the control of the holonomic robot only but no actual simulation.
For this reason, the simulation of both the environment and the robot has to be : that node should both read
messages sent in the topic _"/cmd_vel"_ and publish its odometry data in the
topic _"/odom"_.

---------

## Dependencies

The nodes in the package perform the computation by reading from the topics _"/odom"_ and publishing in _"/cmd_vel"_.
The latters are supplied by an external simulation node, in the specific case the simulator shared by Professor Carmine
Tommaso Recchiuto at the link https://github.com/CarmineD8/assignment1.git which can be 
launched with has been here used:

```bash
$rosrun stage_ros stageros $(rospack find assignment1)/world/exercise.world
```
Note that that package in turn requires **stage_ros**, which can be obtained with
```bash
$sudo apt-get install ros-<your_ros_version>-stage-ros
```

---------

## Relational Graph

![relations between nodes and topics](Sol_1.png)
- **cobot controller** is subscribed to _"/odom"_ and publishes in _"/cmd_vel"_, while 
	calling Services _"/new_target"
- **new_target** serves for Service _"/new_target"_

## Nodes

Two nodes are present, one Service Server and one publishing the velocity
informations.

#### robot controller

This node is the one responsible for publishing velocity data on the topic _"/cmd_vel"_.
It does so by reading the estimated position of the active robot (messages present, 
again, in _"/odom"_) and, if the current target hasn't been reached, calling the Service
TargetVel, which carries in its _response_ the _x_ and _y_ component of the velocity that
this node will then publish in _"cmd_vel"_. If, otherwise, the target position has been
reached the node will call the Service TargetPos which will yield in its _response_ field
the coordinates of a new point to reach on the map.

#### new_target

Server node serving TargetPos, for the Service _"target_position"_. When a request is issued 
it fills the _response_ field of the Service with a random position, a composed of two random 
_x_ and _y_ coordinates.

---------

## Custom Messages and Services

To retrieve next target position and current velocity value to publish by _holo_movement_ two
custom Services where created, along with the Servers mentioned in the **Nodes** paragraph.

### Services

#### TargetPos

Used to retrieve the destination of the robot, hold in the _request_ segment (while the _response_ is empty).

#### TargetVel

Used to retrieve the current velocity hold, _request_ field contains current estimated position and destination,
while _response_ contains a Twist in which are inserted the _x_ and _y_ component of the velocity.


