#include <math.h>

#include "dq.h"
#include "kinematics.h"
#include "gmatrix.h"



DQ dh2dq(DH param, double angle, int i);
DQ get_p(DH param, DQ q);

DQ dh2dq(DH param, double angle, int i) {

    double  theta = param.theta[i],
            d = param.d[i],
            a = param.a[i],
            alpha = param.alpha[i],
            d2=d/2,
            a2=a/2;

            DQ dq;

            dq.q[0]=cos((angle)/2)*cos(alpha/2);
            dq.q[1]=cos((angle)/2)*sin(alpha/2);
            dq.q[2]=sin((angle)/2)*sin(alpha/2);
            dq.q[3]=sin((angle)/2)*cos(alpha/2);



            dq.q[4]=-d2*dq.q[3]-a2*dq.q[1];
            dq.q[5]=-d2*dq.q[2]+a2*dq.q[0];
            dq.q[6]=d2*dq.q[1]+a2*dq.q[3];
            dq.q[7]=d2*dq.q[0]-a2*dq.q[2];

            return dq;

 }


DQ get_p(DH param, DQ q)
{
    DQ p;

    p.q[0] = 0;
    p.q[1]=q.q[1]*q.q[3] + q.q[0]*q.q[2];
    p.q[2]=q.q[2]*q.q[3] - q.q[0]* q.q[1];
    p.q[3]=(pow(q.q[3],2)-pow(q.q[2],2)-pow(q.q[1],2)+pow(q.q[0],2))/2;
    p.q[4]=0;
    p.q[5]=q.q[1]*q.q[7]+q.q[5]*q.q[3]+q.q[0]*q.q[6]+q.q[4]*q.q[2];
    p.q[6]=q.q[2]*q.q[7]+q.q[6]*q.q[3]-q.q[0]*q.q[5]-q.q[4]*q.q[1];
    p.q[7]=q.q[3]*q.q[7]-q.q[2]*q.q[6]-q.q[1]*q.q[5]+q.q[0]*q.q[4];

    return p;
}

DQ dq_fkm(DH param, double *theta)
{
    int i = 0;
    DQ q=dq_unit();

    for(i=0;i<param.dof;i++)
    {
        q = dq_mult(q,dh2dq(param,theta[i],i));
    }
    return q;
}


void dq_jacobian(DH param, PGMATRIX J, double *theta)
{
    DQ q_effector = dq_fkm(param,theta);
    DQ q = dq_unit();
    DQ p,aux;

    int i,row;

    PGMATRIX_SETSIZE(J,8,param.dof);

    for(i=0; i <= param.dof-1;i++)
    {
        p = get_p(param, q);
        q = dq_mult(q,dh2dq(param,theta[i],i));
        aux = dq_mult(p,q_effector);

        for (row = 0;row < 8; row++)
        {
            PGMATRIX_DATA(J,row+1,i+1) = aux.q[row];
        }
    }
}





