#include "qp_solver.h"

using namespace qpOASES;
void qp_test(){
    
    /* Setup data of first QP. */
	real_t H[2 * 2] = { 1.0, 0.0, 0.0, 0.5 };
	real_t A[1 * 2] = { 1.0, 1.0 };
	real_t g[2] = { 1.5, 1.0 };
	real_t lb[2] = { 0.5, -2.0 };
	real_t ub[2] = { 5.0, 2.0 };
	real_t lbA[1] = { -1.0 };
	real_t ubA[1] = { 2.0 };

    QProblem example(2, 1);

    int_t nWSR = 10;
	example.init(H, g, A, lb, ub, lbA, ubA, nWSR);

	real_t xOpt[2];
	example.getPrimalSolution(xOpt);
    
	printf("\n xOpt = [ %e, %e ]; objVal = %e\n\n", xOpt[0], xOpt[1], example.getObjVal());

    return;
}

int main(){
	qp_test();
	return 0;
}
