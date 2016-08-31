#ifndef OPERAND_H_INCLUDED
#define OPERAND_H_INCLUDED

#include "kinematics.h"

typedef struct
{
    char type;
    union
    {
        DQ dq;
        GMATRIX m;
        double s;
    };
} OPERAND;

typedef enum {type_dq, type_matrix, type_scalar} TYPE_OP;

#define OP_DECLARE_JACOBIAN(jacob ,param)\
            DQ_DECLARE_JACOBIAN(__op__ ## jacob ,param);\
            OPERAND jacob = matrix_to_op(__op__## jacob)


#define OP_DECLARE_PINV_JACOBIAN(pinv_jacob ,param)\
            DQ_DECLARE_PINV_JACOBIAN(__op__ ## pinv_jacob ,param);\
            OPERAND pinv_jacob = matrix_to_op(__op__ ## pinv_jacob) \


#define OP_DECLARE_JOINTS(vec ,param)\
            DQ_DECLARE_JOINTS(__op__ ## vec ,param);\
            OPERAND vec = matrix_to_op(__op__ ## vec)




#define PRINT_OP(dq) print_op(#dq,dq)
void print_op(char *name, OPERAND op);

OPERAND op_mult(OPERAND op1, OPERAND op2);
OPERAND op_plus(OPERAND op1, OPERAND op2);
OPERAND op_matrix_pseudoinverse(OPERAND mat_inv, OPERAND mat);

void op_jacobian(DH param, OPERAND J, double *theta);
OPERAND op_fkm(DH param, double *theta);



OPERAND scalar_to_op(double scalar);
OPERAND dq_to_op(DQ dq);
OPERAND matrix_to_op(GMATRIX mat);
TYPE_OP type_op(OPERAND t);






#endif // OPERAND_H_INCLUDED
