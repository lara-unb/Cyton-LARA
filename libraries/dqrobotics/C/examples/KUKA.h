#ifndef KUKA_H_INCLUDED
#define KUKA_H_INCLUDED

#include "kinematics.h"

#define KUKA_DOF 7

static const struct
{
    int dof;
    double theta[KUKA_DOF];
    double d[KUKA_DOF];
    double a[KUKA_DOF];
    double alpha[KUKA_DOF];

} KUKA_DH =
{
KUKA_DOF, //dof
{0, 0, 0, 0, 0, 0, 0}, //theta
{0.310, 0, 0.4, 0, 0.39, 0, 0}, //d
{0, 0, 0, 0, 0, 0, 0},//a
{M_PI/2, -M_PI/2, -M_PI/2, M_PI/2, M_PI/2, -M_PI/2, 0}//alpha
};

DH getKukaParam(void)
{
    DH robot;
    robot.dof = KUKA_DH.dof;
    robot.theta = (double *)KUKA_DH.theta;
    robot.d = (double *)KUKA_DH.d;
    robot.a = (double *)KUKA_DH.a;
    robot.alpha = (double *)KUKA_DH.alpha;

    return robot;
}

#endif // KUKA_H_INCLUDED
