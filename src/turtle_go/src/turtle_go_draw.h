/****************************************************************************
 * Title                 :   turtle_go_draw
 * Filename              :   turtle_go_draw.cpp
 * Author                :   Hanie
 * Origin Date           :   17-2-2023
 * Version               :   1.0.0
 * Compiler              :   TBD
 * Target                :   TBD
 *******************************************************************************/
/** @file turtle_go_draw
 *  @brief The interface definition for the {}
 *
 *  This is the header file for the definition of the interface for the turtle_go_draw node
 *
 */
#ifndef turtle_go_draw_h_
#define turtle_go_draw_h_
/******************************************************************************
 * Includes
 *******************************************************************************/
#include <ros/ros.h>
#include "PID.h"

#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <turtlesim/SetPen.h>
#include <math.h>
#include <string>
/******************************************************************************
 * Constant Macros
 *******************************************************************************/
#define PI 3.141592
/******************************************************************************
 * Configuration
 *******************************************************************************/
#define VEL                             0.5
#define SWITCHING_VEL                   2
#define POSE_TOLERANCE                  0.1
#define DISTANCE_TOLERANCE              0.0001
#define ANGLE_TOLERANCE                 0.0001
/* PID linear velocity output constrain */
#define PID_MIN_LINEAR_VEL              -5
#define PID_MAX_LINEAR_VEL               5
/* PID angular velocity output constrain */
#define PID_MIN_ANGULAR_VEL             -2
#define PID_MAX_ANGULAR_VEL              2
/* PID param of Distance  */
#define PID_LINEAR_KP                 10
#define PID_LINEAR_KI                 0
#define PID_LINEAR_KD                 8
/* PID param of Angle  */
#define PID_ANGLE_KP                    10
#define PID_ANGLE_KI                    0
#define PID_ANGLE_KD                    9
/******************************************************************************
 * Typedefs
*******************************************************************************/
typedef enum{
    CCW = 0,
    CW
}Rotation_direction_t;

typedef enum{
    OFF = 0,
    ON
}Pen_state_t;



class TurtleDraw
{
private:
    ros::NodeHandle nh;
    ros::Publisher PubNode_cmdVel;
    ros::Subscriber SubNode_currentPos;  
    ros::ServiceClient srvClient;       
    turtlesim::Pose currentPos;

    PID turtle_pid_angular;
    PID turtle_pid_linear;

    double _prev_error = 0;
    double _distance(double x1,double y1 ,double x2, double y2 );

public:
    ros::ServiceClient setPenCtrl ;
    void Callback_CurrentPos(const turtlesim::Pose::ConstPtr &msg);

    TurtleDraw(ros::NodeHandle &nh);

    void turtle_goStraight(double len, double velocity);
    void turtle_turn_abs(double theta, double velocity, Rotation_direction_t direction );
    void turtle_turn(double theta);
    void set_Theta(double theta, double velocity);
    void set_Pos_and_Theta_circularPath(double radius, double degree, double velocity); // dont use it 
    void set_Pen(Pen_state_t pState , unsigned int r = 0xFF , unsigned int g =0xFF ,unsigned int b =0xFF , unsigned int width =1);
    void turtle_set_Pos(double x, double y, double velocity);
    void turtle_drawLine(double x1, double y1, double x2, double y2, double velocity);
};


/****************************** End of File ***********************************/

#endif