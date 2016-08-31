#include "dq.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>


DQ dq_set(double *q)
{
    DQ dq;

    memcpy(dq.q,q,sizeof(DQ));
    return dq;
}

DQ dq_plus(DQ dq1, DQ dq2)
{
    DQ dq;
    int i = 0;

    for(i = 0; i < 8; i++)
        dq.q[i]=dq1.q[i]+dq2.q[i];

        return dq;
}

DQ dq_minus(DQ dq1, DQ dq2)
{
    DQ dq;
    int i = 0;

    for(i = 0; i < 8; i++)
        dq.q[i]=dq1.q[i]-dq2.q[i];

    return dq;
}

QUAT quat_mult(QUAT q1, QUAT q2)
{
    QUAT quat;
    double *p = q1.q, *q = q2.q;

    quat.q[0] = p[0]*q[0]-p[1]*q[1]-p[2]*q[2]-p[3]*q[3];
    quat.q[1] = p[1]*q[0]+p[0]*q[1]-p[3]*q[2]+p[2]*q[3];
    quat.q[2] = p[2]*q[0]+p[3]*q[1]+p[0]*q[2]-p[1]*q[3];
    quat.q[3] = p[3]*q[0]-p[2]*q[1]+p[1]*q[2]+p[0]*q[3];

    return quat;
}

QUAT quat_plus(QUAT q1, QUAT q2)
{
    QUAT quat;
    int i = 0;

    for(i = 0; i < 4; i++)
    {
        quat.q[i] = q1.q[i] + q2.q[i];
    }
    return quat;
}

QUAT quat_set(double *q)
{
    QUAT quat;

    memcpy(quat.q,q,sizeof(QUAT));

    return quat;
}

DQ dq_mult(DQ dq1, DQ dq2)
{
    QUAT dq1p, dq1d, dq2p, dq2d, primary, dual;
    DQ dq;

    dq1p = quat_set(dq1.p);
    dq1d = quat_set(dq1.d);
    dq2p = quat_set(dq2.p);
    dq2d = quat_set(dq2.d);

    primary = quat_mult(dq1p,dq2p);
    dual = quat_plus(quat_mult(dq1p,dq2d),quat_mult(dq1d,dq2p));

    memcpy(dq.p,primary.q,sizeof(QUAT));
    memcpy(dq.d,dual.q,sizeof(QUAT));

    return dq;
}

QUAT quat_mult_const(QUAT q, double c)
{
    QUAT quat;
    int i = 0;

    for (i = 0; i < 4; i++)
    {
        quat.q[i] = q.q[i]*c;
    }

    return quat;
}

QUAT quat_conj(QUAT q)
{
    QUAT quat;
    int i = 0;

    quat.q[0] = q.q[0];

    for(i = 1; i < 4; i++)
    {
        quat.q[i] = -q.q[i];
    }
    return quat;
}

DQ dq_conj(DQ dualquat)
{
    DQ q;
    QUAT quat;

    quat = quat_conj(quat_set(dualquat.p));
    memcpy(q.p, quat.q, sizeof(QUAT));

    quat = quat_conj(quat_set(dualquat.d));
    memcpy(q.d, quat.q, sizeof(QUAT));
    return q;
}

DQ dq_unit_tq(QUAT t, QUAT q)
{
    DQ dq;
    QUAT dual;

    dual = quat_mult_const(quat_mult(t,q),0.5);

    memcpy(dq.p,q.q,sizeof(QUAT));
    memcpy(dq.d,dual.q,sizeof(QUAT));

    return dq;
}

DQ dq_unit(void)
{
    DQ dq;

    memset(dq.q,0,sizeof(DQ));
    dq.q[0] = 1.0;

    return dq;
}

DQ_DECOMP decompose(DQ q)
{
    DQ_DECOMP aux;
    QUAT p = quat_set(q.p), d = quat_set(q.d);

    aux.t = quat_mult(d,quat_conj(p));
    aux.t = quat_mult_const(aux.t,2.0);

    aux.theta = 2*acos(p.q[0]);

    if(aux.theta == 0)
    {
        aux.axis[0] = 0;
        aux.axis[1] = 0;
        aux.axis[2] = 1;
    }
    else
    {
        int i = 0;
        QUAT temp = quat_mult_const(p,1.0/sin(aux.theta/2));

        for(i = 0; i < 3; i++) aux.axis[i] = temp.q[i+1];
    }

    return aux;
}

void dq_print(char *name, DQ dq)
{
    int i = 0;
    printf("\n%s:\n",name);

    for(i = 0; i < 4; i++)
    {
        printf("%lf ",dq.p[i]);
    }
    printf("+ E ");
    for(i = 0; i < 4; i++)
    {
        printf("%lf ",dq.d[i]);
    }
}
