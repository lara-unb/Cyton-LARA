#ifndef VIPER_H_INCLUDED
#define VIPER_H_INCLUDED


#include "kinematics.h"
#include <math.h>



#define VIPER_DOF 6

static const struct
{
    int dof;
    double theta[VIPER_DOF];
    double d[VIPER_DOF];
    double a[VIPER_DOF];
    double alpha[VIPER_DOF];

} VIPER_DH =
{
VIPER_DOF, //dof
{0, -M_PI/2, M_PI, 0, M_PI/2, 0}, //theta
{0.335, 0, 0, 0.295, 0, 0.150}, //d
{0.075, 0.27, -0.090, 0, 0, 0},//a
{-M_PI/2, 0, M_PI/2, -M_PI/2, M_PI/2, 0}//alpha
};

DH getViperParam(void)
{
    DH robot;
    robot.dof = VIPER_DH.dof;
    robot.theta = (double *)VIPER_DH.theta;
    robot.d = (double *)VIPER_DH.d;
    robot.a = (double *)VIPER_DH.a;
    robot.alpha = (double *)VIPER_DH.alpha;

    return robot;
}



#endif // VIPER_H_INCLUDED
