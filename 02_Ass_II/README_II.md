#  RT_I-Assignement_II

### Content description 

The ROS package **final_assignment** here presented is to be intended as complementar to the 'gmapping' one,
since all nodes herein are developed in a way that does not require any modification to be made to those ones.
The content of the package is the following:

![package_tree](images/final_tree.png)
- **CMakeLists.txt:** the cmake file of the package;
- **simulation_gmapping.launch:** tha is required for the definition of the robot and of the simulation environment
	it includes the creation of the simulation in **Gazebo** and its visualizarion in **rviz** with pretuned
 	parameters.
	All the required setting for the simulation are stored in the **worlds**, **urdf** and **param** folders
- **final_launcher.launch:** the launch files for the nodes required for the control of the robot including:
	- the move_base node via the **move_base.launch** launcher
	- the bug_o nodes importing the **go_to_point_service_m.py** and **wall_follow_service_m.py** scripts
	- the user interface via the **user_interface.py** script
	- it itializes all the parameters for the robot control.
- **user_interface.launch:** the other launch file, inside it are defined the nodes that interface with the
	user and shall thus be run separately from the one constantly printing on video the position of the
	robot;

---

### Computational graph and communications

![rqt_graph](images/graph_final.png)
**move_base** node is directly the node responsible for the control of the robot.
from _'/cmd_vel'_ (and relative remapped versions plus the multiplexer governing them, more on that later), to the 
remapped 'go_to_point' and 'wall_follower' switch, of course both _'/odom'_ and _'/move_base/goal'_ and both the
topics notifying of target reached or unreachable. 
The node _'/user_interface'_ is invoked once that the target it seareched to define the next action of the robot.

---

### Compiling and running

Both the _'gmapping'_ and _'final_assignment'_ packages are **necessary** to run the package here provided (and thus
required by the CMake file). If all three packages are present on the machine it's sufficient to run 
```bash
# catkin_make
```
in the root directory of the ROS workspace to compile everything.

Two separated launch files are included in the package, both need to be run simultaneously on separate shells
in order to control the robot and observe its position. Moreover, the _simulation_gmapping.launch_ file present inside
_'final_assignment'_ package has to be run in another shell window to run both **rviz** and **Gazebo**, necessary for 
the simulation.
```bash
# roslaunch final_assignment simulation_gmapping.launch

# roslaunch nonholo_control nonholo_control.launch
```
---

### Robot behaviour

The robot behavior can be defined via the user interface selecting in between six possible differet choices:
[1] The robot tries to reach a random target position selected between six different predefined spots.
[2] The robot tries to reach a user defined position selected between six different spots.
[3] The robot points to the closer wall and, once reached, it starts following it for a predefined amount of time.
[4] The robot keeps its position for a specified amount of time.
[5] The robot changes the planning algorithm between the dijkstra's and the 'bug0' one.

Notes:	- The robot is spawn at the location [-4,8]
	- The dijkstra's algorithm is selected during the initialization as the default path planning algorithm
	- The first target is initilaized at the location [-4,7]
	- If the robot is not capable of reaching the new target in two minuts when the 'bug0' algorithm is active
	  it automatically switches to the  dijkstra's('move_base') one. 
	

---

### Design choices

. However, being said packages (_'final_assignment'_ in particular) not designed to be used
in this way, the work that had to be done was quite the challenge and required some non-trivial component to work properly, together with
some "unelegant" (but, I believe, necessary) choice.
The atwo algorithms, 'move_base' and 'bug0', have very different approaches to the problem
- the former basing its behaviour on actions and messages, being able to automatically react to a new goal instance and constantly
	emitting status messages;
- the latter retrieving the target position from a parameter server in specific states of its operation, not being able to react to
	external stimuli;	
This meant that a lot of the operations and services provided by the **nonholo_control** package have to deal with very different
communication mechanisms, trying to hide this differences (abstracting the inner workings) from the the **robot_mainframe** node,
which has been developed in a way that allows for other planning algorithms to be added with very little modification to the code.
The first problem encountered was to avoid crosstalk on the _'/cmd_vel'_ topic, since both algorithms output a driving 
message tehere by default: for that reason, the velocity output topics of those nodes have been remapped, and a multiplexer has been 
used to shift control over the channel to either of the two, depending on the user preferences. A similar problem was faced when
dealing with the "wall follower" behaviour, to be implemented independently from the planning algorithm used at any given time:
since 'bug_m' would generally control the services for switching on and off the "go_to_point" and "wall_follower" behaviours its
requests had to be silenced while the robot was following the walls by user choice (or it would try to drive the robot back to its
previous target). For this end a service resembling a mutex, but actually closer to a conditional router, was designed and, once
again, the services from the algorithms remapped to retrieve which node made each request.
Finally, what probably constituted the most labour intensive part of the design and debugging process was working around the target
retrieval method of the 'bug_m': as mentioned, this originally happened in two very specific cases, either at the booting of the node
(where two values provided by the parameter server are read) or whenever a previous target was reached, when a call to a user interface
service was made before reading the parameters (which, at the time of the response, would be updated with new values inserted by the 
user via the UI). Unfortunatly, this was not viable since this service call has been used to detect the reaching of a target from
'bug_m' perspective, so the current user interface could not communicate directly with this service. But the response to such service
had to wait until a new target was issued from the user, or 'bug' node would read the target coordinates *before* the update.
A conditional variable, coupled with a mutex, has been used to prevent such service callback to return before a new target was issued 
(detected by reading the _'/move_base/goal'_ topic): however, being all services and topics for the node queued by default in the same
Callback Queue, and being it checked by a single Spinner, using a conditional variable in that service callback would mean block the
entire process, making the system hang indefinitely (since no messages from _'/move_base/goal'_ would be read, thus never modifying
the conditional variable). To avoid this issue a separate Callback Queue, together with an additional spinner run in a new thread of the node,
was designed, so that the blocking of that queue would not prevent all other messages and services calls from being served.
This, however, could not work in all scenarios: whenever a target was deemed unreachable and the recovery behaviour adopted, the target would
fall back to its previous (safe) value, which means the robot changed target without having reached the previous one: in other words,
there is no way of communicating such change to the 'bug' node, which would not recognize the new goal as the valid one, leading to
numerous problems (eg. the bug0 algorithm would not stop, evaluating the proximity with respect to the old goal while being driven towards 
the new one): the only solution found for that was to commpletely restart the 'bug' node, taking advantage of the fact that the first target 
position is retrived at node booting. That's been accomplished using the *roslaunch* API from within a python script to launch a new instance 
of the node (which would by default shut down the previous one). It's by no mean an optimal strategy, but it's the only one that could work 
(at least that I knew of) without modifying the 'bug_m.py' script. -->

---

### Limitations and improvements
<!---
Of course, one main issue related to the approach followed is that the entire project is largely unoptimized, many more services and
checks are implemented with respect to what would have been by modifying few lines of code in the 'bug_m' script. On top of that, the 
recovery behaviour is quite rudimental and, due to the way the system operates, it automatically changes the planning algorithm to 
'move_base', no matter what was used before (but it can easily be reverted back to 'bug0' via the UI once the safe goal is reached).
Even more, having to restart a node to make everything work is generally a strategy to avoid but, since it's technically due to a planning
error, it's not completely inappropriate either: for sure not the most elegant approach.
As a side note, more graphical than anything, once a wall following procedure is started the previous goal is still displayed on 
'rviz', which might confuse an unexperienced user.
Moreover, having always both planners work on each target but reading up only one of the two outputs is surely a waste of resources,
which would need to be optimized in a real world scenario.
Lastly, since the check on a target being reached is leaved to the pre-existing algorithms, the parameters therein define the tolerance
of the final position with respect to the given goal, which can lead to some occasional problem, where the distance from the target would
be just slightly larger than the threshold and both 'bug0' and 'move_base' would need few seconds to stop, even if the robot is not moving 
(but in all tests performed still managed to converge, thanks to 'move_base' target detection). A simple workaround would be to enlarge the
tolerance value in 'bug_m.py' script to something closer to 0.5, which wasn't done here since that value is hardcoded in the script itself. -->

