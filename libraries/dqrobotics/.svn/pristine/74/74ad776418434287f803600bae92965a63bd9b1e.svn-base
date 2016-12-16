#ifndef KINEMATICS_H_INCLUDED
#define KINEMATICS_H_INCLUDED

#include <math.h>

#include "dq.h"
#include "gmatrix.h"


typedef struct
{
    int dof;
    double *theta;
    double *d;
    double *a;
    double *alpha;
} DH;

DQ dq_fkm(DH param, double *theta);
void dq_jacobian(DH param, PGMATRIX J, double *theta);


#define DQ_DECLARE_JACOBIAN(jacob ,param)\
GMATRIX_DECLARE(jacob,8,param.dof)

#define DQ_DECLARE_PINV_JACOBIAN(pinv_jacob ,param)\
GMATRIX_DECLARE(pinv_jacob,param.dof,8)

#define DQ_DECLARE_JOINTS(vec ,param)\
GMATRIX_DECLARE(vec,param.dof,1)

#endif // KINEMATICS_H_INCLUDED
