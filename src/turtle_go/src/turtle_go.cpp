
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
 *  @brief The implementation of turtle_go MAIN node
 */

/******************************************************************************
 * Includes
 *******************************************************************************/

#include <iostream>
#include <ros/ros.h>

#include "turtle_go_draw.h"
/******************************************************************************
 * Macros
 *******************************************************************************/

/******************************************************************************
 * main Function
 *******************************************************************************/
int main(int argc, char **argv)
{

    ros::init(argc, argv, "Turtle_go");
    ros::NodeHandle nodeHandle;

    TurtleDraw ctrl(nodeHandle);

    ctrl.turtle_set_Pos(5,5,VEL);
    ctrl.turtle_set_Pos(2,2,VEL);
    ctrl.turtle_drawLine(5,5,5,9,VEL);
    ctrl.turtle_set_Pos(1,1,VEL);
    ctrl.turtle_set_Pos(6,6,VEL);
    ctrl.turtle_set_Pos(5,5,VEL);
    ctrl.turtle_turn_abs(90,VEL, CCW);

    //ctrl.set_Pos_then_Rotate(3.0, 10.0, 180.0, VEL);

    //ctrl.turtle_goStraight(6, VEL);
    //ctrl.turtle_turn_abs(90.0, VEL, CCW);
    //ctrl.turtle_goStraight(6, VEL);

    //ctrl.turtle_goStraight(3, 2);

    //ctrl.turtle_goStraight(1.0, VEL);
    //ctrl.turtle_goStraight(1.0, VEL);
    /*
    ctrl.turtle_goStraight(2, VEL);
    ctrl.turtle_turn_abs(0.0, VEL, CCW);
    ctrl.turtle_goStraight(1, VEL);

    ctrl.turtle_turn_abs(120.0, VEL, CCW);
    ctrl.turtle_goStraight(1, VEL);

    ctrl.set_Pen(ON, 0x0, 0x0, 0x0, 5);

    ctrl.turtle_turn_abs(30.0, VEL, CW);
    ctrl.turtle_goStraight(1.0, VEL);

    ctrl.turtle_turn_abs(330.0, VEL, CW);
    ctrl.turtle_goStraight(3.0, VEL);


    ctrl.turtle_turn_abs(200.0, VEL, CW);
    ctrl.turtle_goStraight(1.0, VEL);
    ctrl.set_Pos_then_Rotate(3.0, 10.0, 180.0, VEL);
*/
    //ctrl.turtle_drawLine(5,5,10,10,1);



    /*
    ctrl.turtle_goStraight(1.0, VEL);
    ctrl.turtle_goStraight(1.0, VEL);
    ctrl.turtle_goStraight(1.0, VEL);

    ctrl.turtle_turn_abs(45.0, VEL, CCW);

    ctrl.turtle_goStraight(1.0, VEL);
    ctrl.turtle_goStraight(0.5, VEL);

    ctrl.turtle_turn_abs(90, VEL, CCW);
    ctrl.turtle_goStraight(1.0, VEL);
    ctrl.turtle_goStraight(0.5, VEL);

    ctrl.turtle_turn_abs(90.0, VEL, CCW);
    ctrl.turtle_goStraight(1.0, VEL);
    ctrl.turtle_goStraight(0.5, VEL);

    ctrl.turtle_turn_abs(90.0, VEL, CCW);
    ctrl.turtle_turn_abs(90.0, VEL, CCW);

    ctrl.turtle_goStraight(1.0, VEL);
    ctrl.turtle_goStraight(1.0, VEL);
    ctrl.turtle_goStraight(1.0, VEL);

    ctrl.set_Pen(OFF, 0x00, 0x00, 0x00, 5);

    ctrl.turtle_turn_abs(90.0, VEL, CCW);
    ctrl.turtle_goStraight(3.0, VEL);
    */
    return 0;
}
/*******************************************************************************/
