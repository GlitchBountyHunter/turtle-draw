/****************************************************************************
* Title                 :   turtle_go_sub
* Filename              :   turtle_go_sub.h
* Author                :   Hanie
* Origin Date           :   17-2-2023
* Version               :   1.0.0
* Compiler              :   TBD
* Target                :   TBD
*******************************************************************************/
/** @file turtle_go_sub
 *  @brief This is the header file for the definition of the interface for the turtle_go_sub node
 * 
 */

/******************************************************************************
* Includes
*******************************************************************************/
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "stdio.h"
#include <geometry_msgs/Twist.h>

#include <turtlesim/Pose.h>
#include <geometry_msgs/Pose2D.h>
/******************************************************************************
* Function Prototypes
*******************************************************************************/

void red();

void yellow();

void green();

void blue();

void reset();

/****************************** End of File ***********************************/