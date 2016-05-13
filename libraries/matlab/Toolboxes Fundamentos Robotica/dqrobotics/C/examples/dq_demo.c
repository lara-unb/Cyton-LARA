#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "dq.h"
#include "kinematics.h"
#include "operand.h"

#include "KUKA.h"
#include "viper.h"

int main(void)
{
    printf("\n\n****** TEST DQ LIBRARY ******");

    printf("\nDQ c,b = {{{2.0000,-3.0000,1.0000,25.0000,6.0000,-7.0000,-5.0000,0.3000}}};");
    DQ b = {{{2.0000,-3.0000,1.0000,25.0000,6.0000,-7.0000,-5.0000,0.3000}}};

    printf("\nDQ dq1 = {{{1,2,3,4,5,69,7,8}}};");
    DQ dq1 = {{{1,2,3,4,5,69,7,8}}};

    printf("\nQUAT t = {{0, 1, -0.3, 0.7}};");
    printf("\nQUAT q = {{0.9010, 0.1302, 0.0868, 0.1564}};");
    QUAT t = {{0, 1, -0.3, 0.7}};
    QUAT q = {{cos(M_PI/4), sin(M_PI/4)*0.1/(0.1*0.1+0.7*0.7+0.3*0.3), sin(M_PI/4)*0.7/(0.1*0.1+0.7*0.7+0.3*0.3), sin(M_PI/4)*0.3/(0.1*0.1+0.7*0.7+0.3*0.3)}};


    DQ_PRINT(dq_unit_tq(t,q));
    DQ_PRINT(dq_mult(dq1,b));
    DQ_PRINT(dq_plus(dq1,dq1));
    DQ_PRINT(dq_conj(dq1));


    printf("\n\n****** TEST OPERAND LIBRARY ******");

    printf("\nFirst we define a scalar:");
    OPERAND s = scalar_to_op(-3.0);
    PRINT_OP(s);

    PRINT_OP(dq_to_op(dq1));

    printf("\nNow we do s*dq");
    OPERAND dqm = op_mult(s,dq_to_op(dq1));
    PRINT_OP(dqm);

    printf("\nwhich should be the same as dq*s");
    dqm = op_mult(dq_to_op(dq1),s);
    PRINT_OP(dqm);

    printf("\nLet's see about the Jacobian");
    DQ_DECLARE_JACOBIAN(kuka_jacob, getKukaParam());

    GMATRIX_ONES(kuka_jacob);
    PRINT_OP(matrix_to_op(kuka_jacob));

    printf("\nLet's multiply it by a constant from the right side");
    OPERAND mat_times_constant = op_mult(matrix_to_op(kuka_jacob),scalar_to_op(5));
    PRINT_OP(scalar_to_op(5));
    PRINT_OP(mat_times_constant);

    printf("\nLet's multiply it by the inverse of the same constant from the left side");
    mat_times_constant = op_mult(matrix_to_op(kuka_jacob),scalar_to_op(1.0/5.0));
    PRINT_OP(mat_times_constant);

    printf("\nHow about the inverse?");
    OP_DECLARE_PINV_JACOBIAN(kuka_pinv_jacob, getKukaParam());
    GMATRIX_ONES(kuka_pinv_jacob.m);
    PRINT_OP(kuka_pinv_jacob);

    OPERAND result = op_mult(kuka_pinv_jacob,dq_to_op(dq1));
    PRINT_OP(dq_to_op(dq1));
    PRINT_OP(result);

    printf("\n\n****** TEST KINEMATICS LIBRARY ******");

    printf("\nNow we are ready to use the kinematics library");
    double theta[] = {M_PI/4,-M_PI/2,-M_PI/3,M_PI/10,-M_PI/10,M_PI/5,0};
    OPERAND fkm = op_fkm(getKukaParam(),theta);
    PRINT_OP(fkm);

    OP_DECLARE_JACOBIAN(jacobian, getKukaParam());
    OP_DECLARE_PINV_JACOBIAN(pinv, getKukaParam());

    op_jacobian(getKukaParam(),jacobian, theta);
    PRINT_OP(jacobian);

    op_matrix_pseudoinverse(pinv,jacobian);
    PRINT_OP(pinv);


    printf("\nUsing the low level kinematic functions");

    double theta_kuka[] = {M_PI/4,-M_PI/2,-M_PI/3,M_PI/10,-M_PI/10,M_PI/5, 0};
    DQ_DECLARE_JACOBIAN(kuka_gmatrix, getKukaParam());
    dq_jacobian(getKukaParam(), &kuka_gmatrix, theta_kuka);
    DQ kuka_fkm_dq = dq_fkm(getKukaParam(),theta_kuka);
    DQ_PRINT(kuka_fkm_dq);


    GMATRIX_PRINT(kuka_gmatrix);

    getchar();

}
