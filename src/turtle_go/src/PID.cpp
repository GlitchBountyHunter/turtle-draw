/****************************************************************************
* Title                 :   PID
* Filename              :   PID.cpp
* Author                :   Hanie
* Origin Date           :   22-2-2023
* Version               :   1.0.0
* Compiler              :   TBD
* Target                :   TBD
*******************************************************************************/
/** @file PID.cpp
 *  @brief The implementation for PID
 */

/******************************************************************************
* Includes
*******************************************************************************/
#include "PID.h"


/******************************************************************************
* Function Definitions
*******************************************************************************/


/*******************************************************************************
 * @brief Construct a new PID object
 * 
 * @param min_val  double value of min_val to be saturated @
 * @param max_val double value of max_val to be saturated @
 * @param kp   
 * @param ki 
 * @param kd 
 *******************************************************************************/
PID::PID(double min_val, double max_val, double kp, double ki, double kd):
    _min_val(min_val),
    _max_val(max_val),
    _kp(kp),
    _ki(ki),
    _kd(kd)
{
}
/*******************************************************************************
 * @brief compute fn
 * 
 * @param ref  double value of set point
 * @param actual_value double value of measurement (setPoint)
 * @return double 
 *******************************************************************************/
double PID::compute(double ref, double actual_value)
{
    double _error;
    double _pid;

    _error = ref - actual_value;
    _integral += _error;
    _derivative = _error - _prev_error;

    if(ref == 0 && _error == 0)
    {
        _integral = 0; 
    }

    _pid = (_kp * _error) + (_ki * _integral) + (_kd * _derivative);
    _prev_error = _error;

    return _constrain(_pid, _min_val, _max_val);
}

/*******************************************************************************
 * @brief updateConstants fn to update PID param
 * 
 * @param kp 
 * @param ki 
 * @param kd 
 *******************************************************************************/
void PID::updateConstants(double kp, double ki, double kd)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
}
/*******************************************************************************
 * @brief _constrain private function return saturated value 
 * 
 * @param val 
 * @param min 
 * @param max 
 * @return double value
 *******************************************************************************/
double PID::_constrain(double &val, double &min , double &max)
{
    if(val < min) {
        return min;
    }
    else if(max < val) {
        return max;
    }
    else
        return val;
}

/****************************** End of File ***********************************/


