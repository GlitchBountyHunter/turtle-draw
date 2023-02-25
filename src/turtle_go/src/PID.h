/****************************************************************************
* Title                 :   PID
* Filename              :   PID.h
* Author                :   Hanie
* Origin Date           :   22-2-2023
* Version               :   1.0.0
* Compiler              :   TBD
* Target                :   TBD
*******************************************************************************/
/** @file PID.h
 *  @brief The interface definition for the PID
 * 
 */

#ifndef PID_H
#define PID_H
/******************************************************************************
* Includes
*******************************************************************************/


/******************************************************************************
* Typedefs
*******************************************************************************/
class PID
{
        private:
        double _min_val;
        double _max_val;
        double _kp;
        double _ki;
        double _kd;
        double _integral;
        double _derivative;
        double _prev_error;
        double _constrain(double &val, double &min , double &max);

    public:
        PID(double min_val, double max_val, double kp, double ki, double kd);
        double compute(double setpoint, double measured_value);
        void updateConstants(double kp, double ki, double kd);


};

#endif

/****************************** End of File ***********************************/





