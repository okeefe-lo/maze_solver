# EECE5550 Group Project
Read-me file for group project.

## Maze Mapping
Use the command 'rosrun maze_solver master_maze.launch' to begin mapping maze.
The robot will move through the entire maze and return to its starting location, when complete, use 'rosrun map_server map_saver -f ~/directory/map_name' to save in desired location.
  
## Maze Navigation
Use the command 'rosrun maze_solver master_nav.launch' to begin navigation.
Select 2D Nav Goal and click on a location on the map. The robot will create a path to that location and wait 5 seconds upon arrivel before heading back to its initial position.

## Maze Rescue
Use the command 'rosrun maze_solver two_maze.launch' to begin rescue.
  
