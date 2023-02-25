/****************************************************************************
* Title                 :   turtle_go_sub.cpp
* Filename              :   turtle_go_sub
* Author                :   Hanie
* Origin Date           :   17-2-2023
* Version               :   1.0.0
* Compiler              :   TBD
* Target                :   TBD
*******************************************************************************/
/** @file turtle_go_draw
 *  @brief The implementation of turtle_go_sub node to subscribe and print it out
 */

/******************************************************************************
* Includes
*******************************************************************************/
#include "turtle_go_sub.h"


/******************************************************************************
* Global variables
*******************************************************************************/
turtlesim::Pose currentPose;
/******************************************************************************
* Function Definitions
*******************************************************************************/

void cmdVel_pos(const turtlesim::Pose::ConstPtr &msg)
{
  currentPose = *msg;
  green();
  printf("\nPose: ");

  blue();
  printf("\n  x: ");
  reset();
  printf("%0.2f", currentPose.x);

  blue();
  printf("\n  y: ");
  reset();
  printf("%0.2f",currentPose.y);

  blue();
  printf("\n  theta: ");
  reset();
  printf("%0.2f",currentPose.theta);

  red();
  printf("\n-----\n");
} 
void cmdVel_Callback(const geometry_msgs::Twist& msg)
{
    // [%-25s] 
  green();
  printf("\nlinear: ");

  blue();
  printf("\n  x: ");
  reset();
  printf("%0.2f",msg.linear.x);

  blue();
  printf("\n  y: ");
  reset();
  printf("%0.2f",msg.linear.y);

  blue();
  printf("\n  z: ");
  reset();
  printf("%0.2f",msg.linear.z);

  green();
  printf("\nangular: ");

  blue();
  printf("\n  x: ");
  reset();
  printf("%0.2f",msg.angular.x);

  blue();
  printf("\n  y: ");
  reset();
  printf("%0.2f",msg.angular.y);

  blue();
  printf("\n  z: ");
  reset();
  printf("%0.2f",msg.angular.z);

  red();
  printf("\n-----\n");

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "turtle_go_sub");

  ros::NodeHandle n;

 // ros::Subscriber sub = n.subscribe("/turtle1/cmd_vel", 20, cmdVel_Callback);

  ros::Subscriber sub2 = n.subscribe("/turtle1/pose", 20, cmdVel_pos);

  ros::spin();

  return 0;
}


void red()
{
    printf("\033[1;31m");
}

void yellow()
{
    printf("\033[1;33m");
}

void green()
{
    printf("\033[0;32m");
}

void blue()
{
    printf("\033[0;34m");
}


void reset()
{
    printf("\033[0m");
}
/****************************** End of File ***********************************/
