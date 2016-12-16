#include <stdio.h>
#include <stdlib.h>
#include "operand.h"
#include "gmatrix_linalg.h"


GMATRIX GMATRIX_COL_VECTOR_FILL(GMATRIX mat, GMATRIX_FLOAT *data, int size);
#define GMATRIX_MULTIPLY(Mat1, Mat2) PGMATRIX_MULTIPLY(&Mat1, &Mat2)
void PGMATRIX_MULTIPLY(PGMATRIX mat1,PGMATRIX mat2);


OPERAND scalar_to_op(double scalar)
{
    OPERAND op = {type_scalar,{{{{scalar}}}}};
    return op;
}

OPERAND dq_to_op(DQ dq)
{
    OPERAND op = {type_dq,{dq}};
    return op;
}

OPERAND matrix_to_op(GMATRIX mat)
{
    OPERAND op;

    op.type = type_matrix;
    op.m = mat;
    return op;
}

TYPE_OP type_op(OPERAND t)
{
    return t.type;
}



/*! \fn op_mult
    \brief Given two general operands, multiply them.

    \param first operand
    \param second operand

    Accepted operands: scalars, dual quaternions and matrices.

    Return a scalar if OP1 = OP2 = Scalar
    Return a dual quaternion if one operand is a dual quaternion and the other one is a scalar
    Return a matrix if one operand is a dual quaternion and the other is a matrix
    Return a matrix if one operand is a matrix and the other is a scalar

*/
OPERAND op_mult(OPERAND op1, OPERAND op2)
{

        switch(type_op(op1))
        {
            case type_scalar:
                switch (type_op(op2))
                {
                    case type_scalar: return scalar_to_op(op1.s*op2.s);
                    case type_dq    :
                    {
                        OPERAND op_dq = op2;
                        int i = 0;

                        for(i=0; i < 8; i++) op_dq.dq.q[i] *= op1.s;

                        return op_dq;
                    }
                    case type_matrix:
                    {
                        GMATRIX mat = op2.m;
                        GMATRIX_MULTIPLY_CONST(mat, op1.s);

                        return matrix_to_op(mat);
                    }
                }
            case type_dq:
                switch (type_op(op2))
                {
                    case type_scalar:
                    {
                        OPERAND op_dq = op1;
                        int i = 0;

                        for(i=0; i < 8; i++) op_dq.dq.q[i] *= op2.s;

                        return op_dq;
                    }
                    case type_dq    : return dq_to_op(dq_mult(op1.dq, op2.dq));
                    case type_matrix:
                    {
                        fprintf(stderr,"\ndq * matrix is not handled by the op_mult function!");
                        abort();
                    }
                }
            case type_matrix:
                switch (type_op(op2))
                {
                    case type_scalar:
                    {
                        GMATRIX mat = op1.m;
                        GMATRIX_MULTIPLY_CONST(mat, op2.s);

                        return matrix_to_op(mat);
                    }
                    case type_dq    :
                    {
                        GMATRIX_DECLARE(dqmat,8,1);
                        GMATRIX_COL_VECTOR_FILL(dqmat,op2.dq.q,8);
                        GMATRIX_MULTIPLY(op1.m,dqmat);

                        return op1;
                    }
                    case type_matrix:
                    {
                        fprintf(stderr,"\nmatrix * matrix is not handled by the op_mult function!");
                        abort();
                    }
                }
        }
}

OPERAND op_matrix_pseudoinverse(OPERAND mat_inv, OPERAND mat)
{
	int n;
	GMATRIX_FLOAT tol, norm;

	PGMATRIX pApinv = &mat_inv.m, pA = &mat.m;

	GMATRIX_DECLARE(U, pA->Nr,pA->Nc);
	GMATRIX_DECLARE(S, pA->Nc,pA->Nc);
	GMATRIX_DECLARE(V, pA->Nc,pA->Nc);
	GMATRIX_DECLARE(MatDummy, pA->Nr, pA->Nc);

	PGMATRIX pU = &U, pS = &S, pV = &V, pMatDummy = &MatDummy;



	GMATRIX_ASSERT("PGMATRIX_PSEUDOINVERSE",pA->Nr <= 0);
	GMATRIX_ASSERT("PGMATRIX_PSEUDOINVERSE",pA->Nc <= 0);
	GMATRIX_ASSERT("PGMATRIX_PSEUDOINVERSE",pMatDummy->MaxSize < pA->Nr*pA->Nc);


	// Compute SVD:
	PGMATRIX_SVD(pU,pS,pV,pA,FALSE);
	// Compute norm:
	norm = (GMATRIX_FLOAT)(fabs(PGMATRIX_DATA(pS,1,1)));
	for(n=2;n<=pA->Nc;++n){
		norm = GMATRIXMACRO_MAX(norm,(GMATRIX_FLOAT)(fabs(PGMATRIX_DATA(pS,n,n))));
	}
	// Compute tolerance:
	tol = (GMATRIXMACRO_MAX(pA->Nr,pA->Nc)) * norm * (GMATRIX_FLOAT)(1e-12);

	// Compute inv(S):
	for(n=1;n<=pA->Nc;++n){
		if (fabs(PGMATRIX_DATA(pS,n,n))<=tol){
			PGMATRIX_DATA(pS,n,n) = 0.0;
		}
		else{
			PGMATRIX_DATA(pS,n,n) = (GMATRIX_FLOAT)(1.0) / PGMATRIX_DATA(pS,n,n);
		}
	}
	// Compute pinv:
	PGMATRIX_MULTIPLY_COPY_EXTENDED(pMatDummy,pS,FALSE,pU,TRUE);
	if (pApinv==NULL){
		PGMATRIX_MULTIPLY_COPY_EXTENDED(pA,pV,FALSE,pMatDummy,FALSE);
	}
	else{
		PGMATRIX_MULTIPLY_COPY_EXTENDED(pApinv,pV,FALSE,pMatDummy,FALSE);
	}

	return matrix_to_op(*pApinv);
}


OPERAND op_fkm(DH param, double *theta)
{
    return dq_to_op(dq_fkm(param, theta));
}

void op_jacobian(DH param, OPERAND J, double *theta)
{
    if(type_op(J) != type_matrix)
    {
        fprintf(stderr,"\nThe second argument of the op_jacobian function must be a matrix operand");
        abort();
    }

    dq_jacobian(param, &J.m, theta);
}




void print_op(char *name, OPERAND op)
{
    switch(type_op(op))
    {
        case type_scalar:
            printf("\n%s: %lf",name, op.s);
            break;
        case type_matrix:
            PGMATRIX_PRINT_NAMED(name,&op.m);
            break;
        case type_dq:
            dq_print(name,op.dq);
            break;
    }
}


#define GMATRIX_MULTIPLY(Mat1, Mat2) PGMATRIX_MULTIPLY(&Mat1, &Mat2)
void PGMATRIX_MULTIPLY(PGMATRIX mat1,PGMATRIX mat2)
{
	int i,j;
	int k;
	GMATRIX_FLOAT Acc;
	GMATRIX_DECLARE(temp,mat1->Nr, mat2->Nc);


	GMATRIX_ASSERT("GMATRIX_MULTIPLY",mat1->Nc != mat2->Nr);

	for(i=1;i<=mat1->Nr;++i)
	{
		for(j=1;j<=mat2->Nc;++j)
		{
			Acc = 0.0;
			for(k=1;k<=mat1->Nc;++k)
			{
				Acc += PGMATRIX_DATA(mat1,i,k)*PGMATRIX_DATA(mat2,k,j);
			}
			GMATRIX_DATA(temp,i,j) = Acc;
		}
	}
	PGMATRIX_COPY(mat1,&temp);
}

GMATRIX GMATRIX_COL_VECTOR_FILL(GMATRIX mat, GMATRIX_FLOAT *data, int size)
{
    int i = 0;

    GMATRIX_ASSERT("GMATRIX_COL_VECTOR_FILL",mat.MaxSize < size);
    GMATRIX_SETSIZE(mat,size, 1);
    for(i=0;i < size; i++) mat.Data[i] = data[i];

    return mat;
}

OPERAND op_plus(OPERAND op1, OPERAND op2)
{

        switch(type_op(op1))
        {
            case type_scalar:
                switch (type_op(op2))
                {
                    case type_scalar: return scalar_to_op(op1.s+op2.s);
                    case type_dq    :
                    {
                        fprintf(stderr,"\nScalar + dq is not handled by the OP_plus function!");
                        abort();
                    }
                    case type_matrix:
                    {
                        fprintf(stderr,"\nScalar + matrix is not handled by the OP_plus function!");
                        abort();
                    }
                }
            case type_dq:
                switch (type_op(op2))
                {
                    case type_scalar:
                    {
                        fprintf(stderr,"\nScalar + dq is not handled by the OP_plus function!");
                        abort();
                    }
                    case type_dq    : return dq_to_op(dq_plus(op1.dq, op2.dq));
                    case type_matrix:
                    {
                        fprintf(stderr,"\ndq + matrix is not handled by the OP_plus function!");
                        abort();
                    }
                }
            case type_matrix:
                switch (type_op(op2))
                {
                    case type_scalar:
                    {
                        fprintf(stderr,"\ndq + matrix is not handled by the OP_plus function!");
                        abort();
                    }
                    case type_dq    :
                    {
                        fprintf(stderr,"\ndq + matrix is not handled by the OP_plus function!");
                        abort();
                    }
                    case type_matrix:
                    {
                        fprintf(stderr,"\nmatrix sum is not handled by the OP_plus function! Use the GMATRIX function instead");
                        abort();
                    }
                }
        }
}



