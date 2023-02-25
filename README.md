# Turtle_go
Turtlesim c++ package for drawing

<img  alt="Coding" width="500" src="media/sim.gif">

This project has 3 Packages:
  - ***turtlesim***  I did not implement this Packages but i customized it
  - ***turtle_go***  position control used to send cmd_vel to turtlesim and recive pose of the robot i implement this with oop to avoid working with nasty global variable
  - ***turtle_sub_cmdvel**** just used to subscribe for cmd_vel and print it out


## Hardpoint coordinates
I used [GeoGebra](https://www.geogebra.org/) to determine hardpoints of letters like following

<img  alt="Coding" width="500" src="media/geogebra1.png">

<br>


<!-- #### Then construct lines -->
<!--  <img  alt="Coding" width="500" src="media/geogebra2.png">  -->
<br>



#### Record the slope from vector and length from segment for each line. 

<img  alt="Coding" width="500" src="media/geogebra3.png">

## Terminal output
<img  alt="Coding" width="750" src="media/terminal.png">

## RQT Graph
<img  alt="Coding" width="500" src="media/rqt_graph.png">

## Future work

 - Make anther node with python (Matplotlib) to plot data in runtime 
## Referance
 - https://github.com/ros/ros_tutorials/tree/821a8ae33c6da1a70e21453392f6980380f670f8/turtlesim
 - http://motion.cs.illinois.edu/RoboticSystems/CoordinateTransformations.html
# turtle-draw
