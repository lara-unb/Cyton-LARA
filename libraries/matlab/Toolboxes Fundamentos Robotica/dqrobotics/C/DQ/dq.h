#ifndef DQ_H_INCLUDED
#define DQ_H_INCLUDED
//! \TODO implement dq_get
typedef union
{
    struct
    {
        double q[8];
    };
    struct
    {
        double p[4];
        double d[4];
    };

} DQ;

typedef struct
{
    double q[4];
} QUAT;

typedef struct
{
    QUAT t;
    double axis[3];
    double theta;

} DQ_DECOMP;

DQ dq_set(double *q);
DQ dq_plus(DQ dq1, DQ dq2);
DQ dq_minus(DQ dq1, DQ dq2);
QUAT quat_mult(QUAT q1, QUAT q2);
QUAT quat_plus(QUAT q1, QUAT q2);
QUAT quat_set(double *q);
DQ dq_mult(DQ dq1, DQ dq2);
QUAT quat_mult_const(QUAT q, double c);
QUAT quat_conj(QUAT q);
DQ dq_conj(DQ dualquat);
DQ dq_unit_tq(QUAT t, QUAT q);
DQ dq_unit(void);


#define DQ_PRINT(dq) dq_print(#dq,dq)
void dq_print(char *name, DQ dq);


#endif // DQ_H_INCLUDED
