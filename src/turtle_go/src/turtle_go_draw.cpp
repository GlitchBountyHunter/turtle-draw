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
 *  @brief The implementation of turtle_go_draw node publish cmd_vel
 */

/******************************************************************************
 * Includes
 *******************************************************************************/
#include "turtle_go_draw.h"

/******************************************************************************
 * Function Definitions
 *******************************************************************************/

/*******************************************************************************
 * @brief Callback fn called when pose msgs is recived via pose topic
 * 
 * @param msg : ---
 *******************************************************************************/
void TurtleDraw::Callback_CurrentPos(const turtlesim::Pose::ConstPtr &msg)
{
    currentPos = *msg ; 

    if (currentPos.theta < 0.0)
        currentPos.theta += (2 * PI); // polar angle
    
}
/*******************************************************************************
 * @brief Construct a new TurtleDraw::Turtle Draw object
 * 
 * @param nh node handelar
 *******************************************************************************/



TurtleDraw::TurtleDraw(ros::NodeHandle &nh): turtle_pid_linear( PID_MIN_LINEAR_VEL, PID_MAX_LINEAR_VEL, PID_LINEAR_KP, PID_LINEAR_KI, PID_LINEAR_KD)
                                            , turtle_pid_angular( PID_MIN_ANGULAR_VEL, PID_MAX_ANGULAR_VEL, PID_ANGLE_KP, PID_ANGLE_KI, PID_ANGLE_KD)
{
    this->nh = nh;
    setPenCtrl = nh.serviceClient<turtlesim::SetPen>("/turtle1/set_pen");
    PubNode_cmdVel = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);
    SubNode_currentPos = nh.subscribe("/turtle1/pose", 5, &TurtleDraw::Callback_CurrentPos, this);
    _prev_error = 0;
    
    ros::Rate loop_rate(50); // ROS sampling rate
}


/*******************************************************************************
 * @brief turtle_goStraight used to make robot moving forwad with certain distance and linear vel 
 *        control turtle using cmd_vel by PID control 
 * 
 * @param len : double value for distance to be moved
 * @param velocity : double value for the required linear velocity
 *******************************************************************************/
void TurtleDraw::turtle_goStraight(double len, double velocity)
{
    ROS_INFO("Turtlel moving straight with dist=%0.2f m , vel=%0.2f lenUnit/timeUnit", len, velocity);


    geometry_msgs::Twist _cmd_vel;

    _cmd_vel.linear.x = 0;
    _cmd_vel.linear.y = 0;
    _cmd_vel.angular.z = 0;
    double _x_error = 0, _y_error = 0;
    double _diff_distance;


    // Ref : http://motion.cs.illinois.edu/RoboticSystems/CoordinateTransformations.html


    double _req_x = 0;
    double _req_y = 0;

    int ___ = 0 ;

    if (currentPos.theta <= PI/2)
    {
        _req_x = currentPos.x + len * cos(currentPos.theta);
        _req_y = currentPos.y +  len * sin(currentPos.theta);
    }
    else if (currentPos.theta <= (PI - 0.01)) // <180
    {
        _req_x = currentPos.x - len * cos(PI - currentPos.theta);
        _req_y = currentPos.y +  len * sin(PI - currentPos.theta);
        ___ = 0xf ; 
    }
    else if (currentPos.theta <= (3 / 2.0 * PI)) // <=270
    {
        _req_x = currentPos.x - len * cos(PI + currentPos.theta);
        _req_y = currentPos.y -  len * sin(PI +currentPos.theta);
    }
    else    // <= 360
    {
        _req_x = currentPos.x + len * cos(2 * PI - currentPos.theta);
        _req_y = currentPos.y -  len * sin(2 * PI - currentPos.theta);
    }

    _x_error = currentPos.x - _req_x;
    _y_error = currentPos.y - _req_y;

    _diff_distance = _distance(currentPos.x, currentPos.y, _req_x, _req_y);

    _cmd_vel.linear.x = 1;
    while ( fabs(_diff_distance) > DISTANCE_TOLERANCE )
    {   
        ros::spinOnce();
        _diff_distance = _distance(currentPos.x, currentPos.y, _req_x, _req_y);
                                                                                                                                                                           
        //_diff_distance =  (fabs(_prev_error) > fabs(_diff_distance)) ? _diff_distance : _diff_distance * -1   ;

    if(currentPos.theta <= PI )
    {
       if(_distance(currentPos.x, currentPos.y, 0, 0) < _distance(_req_x ,_req_y , 0, 0) )
        {
            _diff_distance *= -1;  
        }
    }
    else
    {
        if(_distance(currentPos.x, currentPos.y, 0, 0) > _distance(_req_x ,_req_y , 0, 0) )
        {
            _diff_distance *= -1;  
        }
    }
        _cmd_vel.linear.x = turtle_pid_linear.compute(DISTANCE_TOLERANCE ,  _diff_distance);

        printf("\nthe error in x value = %0.2f , y value = %0.2f  ||  , diff_distance = %f" ,(currentPos.x - _req_x) ,(currentPos.y - _req_y) ,_diff_distance);

        PubNode_cmdVel.publish(_cmd_vel);


    }

    _cmd_vel.linear.x = 0;
    PubNode_cmdVel.publish(_cmd_vel);
    
}

/*******************************************************************************
 * @brief turtle_turn_abs function used to rotate the robot CCW or CW with certain vel
 *        with abslute angle  turtle frame without pid control 
 * 
 * @param theta : double value to be rotate in robot axis
 * @param velocity  : double value  as the angular velocity about z axis
 * @param direction : options { CCW, CW } the direction of rotation
 *******************************************************************************/
void TurtleDraw::turtle_turn_abs(double theta, double velocity, Rotation_direction_t direction)
{


    geometry_msgs::Twist _cmd_vel;

    _cmd_vel.linear.x = 0;
    _cmd_vel.linear.y = 0;
    _cmd_vel.linear.z = 0;
    _cmd_vel.angular.x = 0;
    _cmd_vel.angular.y = 0;
    _cmd_vel.angular.z = fabs(velocity);

    double _angle_done = 0.0;
    double _prevAngle = currentPos.theta;
    double _rad = fabs(theta) * PI / 180.0;

    if (direction == CW)
    {
        ROS_INFO("Turtule turning CW with phi=%0.2f Deg , v=%0.2f rad/timeUnit", theta, velocity);

        _cmd_vel.angular.z *= -fabs(velocity);

        while(abs(currentPos.theta - _rad) > ANGLE_TOLERANCE)
        {
            ros::spinOnce();
            printf("\nAngle Error = %f || theta = %f", currentPos.theta-_rad,theta);
            
            PubNode_cmdVel.publish(_cmd_vel);
        }
        _cmd_vel.angular.z = 0;
        PubNode_cmdVel.publish(_cmd_vel);
    }

    else if (direction == CCW)
    {

        ROS_INFO("Turtule turning CCW  with phi=%0.2f Deg , v=%0.2f rad/timeUnit", theta, velocity);

        _cmd_vel.angular.z = fabs(velocity);

        while(abs(currentPos.theta - _rad) > ANGLE_TOLERANCE)
        {
            ros::spinOnce();
            printf("\nAngle Error = %f || theta = %f", currentPos.theta-_rad,theta);
            PubNode_cmdVel.publish(_cmd_vel);
        }
        _cmd_vel.angular.z = 0;
        PubNode_cmdVel.publish(_cmd_vel);
    }
}

/*******************************************************************************
 * @brief function used to rotate the robot with abslute angle  turtle frame 
 *         using pid controller
 * 
 * @param theta  double value of abslute angle
 *******************************************************************************/
void TurtleDraw::turtle_turn(double theta)
{
    geometry_msgs::Twist _cmd_vel;

    _cmd_vel.linear.x = 0;
    _cmd_vel.linear.y = 0;
    _cmd_vel.linear.z = 0;
    _cmd_vel.angular.x = 0;
    _cmd_vel.angular.y = 0;

    double _angle_done = 0.0;
    double _prevAngle = currentPos.theta;
    double _rad = fabs(theta) * PI / 180.0;

    while(abs(currentPos.theta - _rad) > ANGLE_TOLERANCE)
    {
        ros::spinOnce();
        printf("\nAngle Error = %f || theta = %f", currentPos.theta-_rad,theta);
        _cmd_vel.angular.z = turtle_pid_angular.compute(ANGLE_TOLERANCE ,  fabs(_rad - currentPos.theta ));    
        PubNode_cmdVel.publish(_cmd_vel);
    }
    _cmd_vel.angular.z = 0;
    PubNode_cmdVel.publish(_cmd_vel);

}

void TurtleDraw::set_Pen(Pen_state_t pState, unsigned int r, unsigned int g, unsigned int b, unsigned int width)
{

    turtlesim::SetPen pen;
    pen.request.r = r;
    pen.request.g = g;
    pen.request.b = b;
    pen.request.off = !pState;
    pen.request.width = width;
    if (!setPenCtrl.call(pen))
    {
        ROS_INFO("setPenCtrl");
    }

    if (pState)
    {
        ROS_INFO("Pen is Enable ");
    }
    else
    {
        ROS_INFO("Pen is Disble ");
    }
}

void TurtleDraw::set_Theta(double theta, double velocity)
{


    ROS_INFO("Turtle turning to theta=%0.2f , with  vel=%0.2f rad/timeUnit", theta, velocity);

    double _currentPos = currentPos.theta;

    geometry_msgs::Twist _cmd_vel;

    double radAngle = theta * PI / 180.0;
    double actualTheta = currentPos.theta;

    while (fabs(radAngle - actualTheta) > ANGLE_TOLERANCE)
    {
        ros::spinOnce();
        actualTheta = currentPos.theta;
        bool clockwise = true;

        if (clockwise)
            _cmd_vel.angular.z = -1 * velocity;
        else
            _cmd_vel.angular.z = velocity;

        PubNode_cmdVel.publish(_cmd_vel);
    }

    _cmd_vel.angular.z = 0;
    PubNode_cmdVel.publish(_cmd_vel);
}


void TurtleDraw::turtle_set_Pos(double x, double y, double velocity)
{
    double local_theta = atan2(-(currentPos.y - y), -(currentPos.x - x));
    if (local_theta < 0.0)
        local_theta += (2 * PI); // polar angle

    double local_distance = _distance(x, y, currentPos.x, currentPos.y);

    //turtle_turn_abs(local_theta*180/PI , velocity ,CCW);
    turtle_turn(local_theta*180/PI );

    turtle_goStraight( local_distance , velocity);
}


/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                cringe function do not activate it
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~      
void TurtleDraw::set_Pos_then_Rotate(double x, double y, double theta, double velocity)
{

    ROS_INFO("Turtle go to  point (x=%0.2f, y=%0.2f) vel=%0.2f lenUnit/timeUnit", x, y, velocity);

    double phi = atan2((y - currentPos.y), (x - currentPos.x));

    geometry_msgs::Twist _cmd_vel;
/*
    while (ANGLE_TOLERANCE < fabs(phi - currentPos.theta))
    {
        ros::spinOnce();
        _cmd_vel.angular.z = velocity;
        PubNode_cmdVel.publish(_cmd_vel);
        printf("\n%f",fabs(phi - currentPos.theta));
    }

    turtle_turn_abs(phi,velocity,CCW);
    _cmd_vel.angular.z = 0;
    PubNode_cmdVel.publish(_cmd_vel);

    while (fabs(x - currentPos.x) > POSE_TOLERANCE)
    {
        ros::spinOnce();
        _cmd_vel.linear.x = velocity;
        PubNode_cmdVel.publish(_cmd_vel);
    }

    _cmd_vel.linear.x = 0;
    PubNode_cmdVel.publish(_cmd_vel);

    this->set_Theta(theta, velocity);
}
*/

/*******************************************************************************
 * @brief function to draw line using two points
 * 
 * @param x0 value x0 of starting point
 * @param y0 value y0 of starting point
 * @param x1 value x1 of ending point
 * @param y1 value y1 of ending poi
 * @param velocity 
 *******************************************************************************/
void TurtleDraw::turtle_drawLine(double x0, double y0, double x1, double y1, double velocity)
{
    double local_theta = PI/2 - atan2((y1 - y0), (x1 - x0));
    double local_distance = _distance(x0, y0, x1, y1);

    turtle_set_Pos(x0,y0,velocity);
    //turtle_turn_abs(local_theta, velocity ,CCW);
    turtle_turn(local_theta);
    turtle_goStraight( local_distance , velocity);

}

/*******************************************************************************
 * @brief function return double value of Euclidean distance

 * 
 * @param x0 value x0 of starting point
 * @param y0 value y0 of starting point
 * @param x1 value x1 of ending point
 * @param y1 value y1 of ending point
 * @return double value of Euclidean distance
 *******************************************************************************/
double TurtleDraw::_distance(double x0,double y0 ,double x1, double y1)
{
    double _x_diff = x1 - x0;
    double _y_diff = y1 - y0;

    return sqrt((_x_diff * _x_diff) + (_y_diff * _y_diff));
}

/****************************** End of File ***********************************/

