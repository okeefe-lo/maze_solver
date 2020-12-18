# EECE5550 Group Project
Read-me file for group project.
<br/>
In any terminal use the line `export TURTLEBOT3_MODEL=burger` before running any launch file.
## Maze Mapping
Use the command `rosrun maze_solver master_maze.launch` to begin mapping maze.
The robot will move through the entire maze and return to its starting location, when complete, use `rosrun map_server map_saver -f ~/directory/map_name` to save in desired location.
  
## Maze Navigation
Use the command `rosrun maze_solver master_nav.launch` to begin navigation tool.
Send the command `rostopic pub /target geometry_msgs/Point '{x: -1.25, y: 0.13}'` with desired location. The robot will create a path to that location and wait 5 seconds upon arrivel before heading back to its initial position. If the robot does not move, resend the command.

## Maze Rescue
Use the command `rosrun maze_solver two_maze.launch` to begin rescue.
<br/>
<br/>
The command `rosrun maze_solver two_maze_test1.launch` will launch the follower inside the maze, a 2D nav goal can be selected.
<br/>
The command `rosrun maze_solver two_maze_test2.launch` will launch the follower outside the maze, a 2D nav goal can be selected.
