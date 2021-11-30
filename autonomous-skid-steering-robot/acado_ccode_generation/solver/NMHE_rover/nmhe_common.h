/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#ifndef NMHE_COMMON_H
#define NMHE_COMMON_H

#include <math.h>
#include <string.h>

#ifndef __MATLAB__
#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */
#endif /* __MATLAB__ */

/** \defgroup NMHE ACADO CGT generated module. */
/** @{ */

/** qpOASES QP solver indicator. */
#define NMHE_QPOASES  0
#define NMHE_QPOASES3 1
/** FORCES QP solver indicator.*/
#define NMHE_FORCES   2
/** qpDUNES QP solver indicator.*/
#define NMHE_QPDUNES  3
/** HPMPC QP solver indicator. */
#define NMHE_HPMPC    4
#define NMHE_GENERIC    5

/** Indicator for determining the QP solver used by the ACADO solver code. */
#define NMHE_QP_SOLVER NMHE_QPOASES

#include "nmhe_qpoases_interface.hpp"


/*
 * Common definitions
 */
/** User defined block based condensing. */
#define NMHE_BLOCK_CONDENSING 0
/** Compute covariance matrix of the last state estimate. */
#define NMHE_COMPUTE_COVARIANCE_MATRIX 0
/** Flag indicating whether constraint values are hard-coded or not. */
#define NMHE_HARDCODED_CONSTRAINT_VALUES 1
/** Indicator for fixed initial state. */
#define NMHE_INITIAL_STATE_FIXED 0
/** Number of control/estimation intervals. */
#define NMHE_N 100
/** Number of online data values. */
#define NMHE_NOD 0
/** Number of path constraints. */
#define NMHE_NPAC 0
/** Number of control variables. */
#define NMHE_NU 2
/** Number of differential variables. */
#define NMHE_NX 8
/** Number of algebraic variables. */
#define NMHE_NXA 0
/** Number of differential derivative variables. */
#define NMHE_NXD 0
/** Number of references/measurements per node on the first N nodes. */
#define NMHE_NY 8
/** Number of references/measurements on the last (N + 1)st node. */
#define NMHE_NYN 6
/** Total number of QP optimization variables. */
#define NMHE_QP_NV 208
/** Number of integration steps per shooting interval. */
#define NMHE_RK_NIS 2
/** Number of Runge-Kutta stages per integration step. */
#define NMHE_RK_NSTAGES 4
/** Providing interface for arrival cost. */
#define NMHE_USE_ARRIVAL_COST 1
/** Indicator for usage of non-hard-coded linear terms in the objective. */
#define NMHE_USE_LINEAR_TERMS 0
/** Indicator for type of fixed weighting matrices. */
#define NMHE_WEIGHTING_MATRICES_TYPE 1


/*
 * Globally used structure definitions
 */

/** The structure containing the user data.
 * 
 *  Via this structure the user "communicates" with the solver code.
 */
typedef struct NMHEvariables_
{
int dummy;
/** Matrix of size: 101 x 8 (row major format)
 * 
 *  Matrix containing 101 differential variable vectors.
 */
real_t x[ 808 ];

/** Matrix of size: 100 x 2 (row major format)
 * 
 *  Matrix containing 100 control variable vectors.
 */
real_t u[ 200 ];

/** Column vector of size: 800
 * 
 *  Matrix containing 100 reference/measurement vectors of size 8 for first 100 nodes.
 */
real_t y[ 800 ];

/** Column vector of size: 6
 * 
 *  Reference/measurement vector for the 101. node.
 */
real_t yN[ 6 ];

/** Matrix of size: 8 x 8 (row major format) */
real_t W[ 64 ];

/** Matrix of size: 6 x 6 (row major format) */
real_t WN[ 36 ];

/** Matrix of size: 8 x 8 (row major format)
 * 
 *  Arrival cost term: inverse of the covariance matrix.
 */
real_t SAC[ 64 ];

/** Column vector of size: 8
 * 
 *  Arrival cost term: a priori state estimate.
 */
real_t xAC[ 8 ];

/** Matrix of size: 8 x 8 (row major format)
 * 
 *  Arrival cost term: Cholesky decomposition, lower triangular,  of the inverse of the state noise covariance matrix.
 */
real_t WL[ 64 ];


} NMHEvariables;

/** Private workspace used by the auto-generated code.
 * 
 *  Data members of this structure are private to the solver.
 *  In other words, the user code should not modify values of this 
 *  structure. 
 */
typedef struct NMHEworkspace_
{
/** Column vector of size: 71 */
real_t rhs_aux[ 71 ];

real_t rk_ttt;

/** Row vector of size: 90 */
real_t rk_xxx[ 90 ];

/** Matrix of size: 4 x 88 (row major format) */
real_t rk_kkk[ 352 ];

/** Row vector of size: 90 */
real_t state[ 90 ];

/** Column vector of size: 800 */
real_t d[ 800 ];

/** Column vector of size: 800 */
real_t Dy[ 800 ];

/** Column vector of size: 6 */
real_t DyN[ 6 ];

/** Matrix of size: 800 x 8 (row major format) */
real_t evGx[ 6400 ];

/** Matrix of size: 800 x 2 (row major format) */
real_t evGu[ 1600 ];

/** Row vector of size: 10 */
real_t objValueIn[ 10 ];

/** Row vector of size: 8 */
real_t objValueOut[ 8 ];

/** Matrix of size: 800 x 8 (row major format) */
real_t Q1[ 6400 ];

/** Matrix of size: 800 x 8 (row major format) */
real_t Q2[ 6400 ];

/** Matrix of size: 200 x 2 (row major format) */
real_t R1[ 400 ];

/** Matrix of size: 200 x 8 (row major format) */
real_t R2[ 1600 ];

/** Matrix of size: 8 x 8 (row major format) */
real_t QN1[ 64 ];

/** Matrix of size: 8 x 6 (row major format) */
real_t QN2[ 48 ];

/** Column vector of size: 8 */
real_t DxAC[ 8 ];

/** Matrix of size: 24 x 18 (row major format) */
real_t acA[ 432 ];

/** Column vector of size: 24 */
real_t acb[ 24 ];

/** Matrix of size: 8 x 8 (row major format) */
real_t acP[ 64 ];

/** Row vector of size: 25 */
real_t rk_actemp[ 25 ];

/** Matrix of size: 8 x 8 (row major format) */
real_t acVL[ 64 ];

/** Matrix of size: 8 x 8 (row major format) */
real_t acHx[ 64 ];

/** Matrix of size: 8 x 2 (row major format) */
real_t acHu[ 16 ];

/** Matrix of size: 8 x 8 (row major format) */
real_t acXx[ 64 ];

/** Matrix of size: 8 x 2 (row major format) */
real_t acXu[ 16 ];

/** Column vector of size: 8 */
real_t acXTilde[ 8 ];

/** Column vector of size: 8 */
real_t acHTilde[ 8 ];

/** Matrix of size: 8 x 8 (row major format) */
real_t T[ 64 ];

/** Matrix of size: 40400 x 2 (row major format) */
real_t E[ 80800 ];

/** Matrix of size: 40400 x 2 (row major format) */
real_t QE[ 80800 ];

/** Matrix of size: 800 x 8 (row major format) */
real_t QGx[ 6400 ];

/** Column vector of size: 800 */
real_t Qd[ 800 ];

/** Column vector of size: 808 */
real_t QDy[ 808 ];

/** Matrix of size: 200 x 8 (row major format) */
real_t H10[ 1600 ];

/** Matrix of size: 208 x 208 (row major format) */
real_t H[ 43264 ];

/** Matrix of size: 200 x 208 (row major format) */
real_t A[ 41600 ];

/** Column vector of size: 208 */
real_t g[ 208 ];

/** Column vector of size: 208 */
real_t lb[ 208 ];

/** Column vector of size: 208 */
real_t ub[ 208 ];

/** Column vector of size: 200 */
real_t lbA[ 200 ];

/** Column vector of size: 200 */
real_t ubA[ 200 ];

/** Column vector of size: 208 */
real_t x[ 208 ];

/** Column vector of size: 408 */
real_t y[ 408 ];


} NMHEworkspace;

/* 
 * Forward function declarations. 
 */


/** Performs the integration and sensitivity propagation for one shooting interval.
 *
 *  \param rk_eta Working array to pass the input values and return the results.
 *  \param resetIntegrator The internal memory of the integrator can be reset.
 *
 *  \return Status code of the integrator.
 */
int nmhe_integrate( real_t* const rk_eta, int resetIntegrator );

/** Export of an ACADO symbolic function.
 *
 *  \param in Input to the exported function.
 *  \param out Output of the exported function.
 */
void nmhe_rhs_forw(const real_t* in, real_t* out);

/** Preparation step of the RTI scheme.
 *
 *  \return Status of the integration module. =0: OK, otherwise the error code.
 */
int nmhe_preparationStep(  );

/** Feedback/estimation step of the RTI scheme.
 *
 *  \return Status code of the qpOASES QP solver.
 */
int nmhe_feedbackStep(  );

/** Solver initialization. Must be called once before any other function call.
 *
 *  \return =0: OK, otherwise an error code of a QP solver.
 */
int nmhe_initializeSolver(  );

/** Initialize shooting nodes by a forward simulation starting from the first node.
 */
void nmhe_initializeNodesByForwardSimulation(  );

/** Shift differential variables vector by one interval.
 *
 *  \param strategy Shifting strategy: 1. Initialize node 101 with xEnd. 2. Initialize node 101 by forward simulation.
 *  \param xEnd Value for the x vector on the last node. If =0 the old value is used.
 *  \param uEnd Value for the u vector on the second to last node. If =0 the old value is used.
 */
void nmhe_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd );

/** Shift controls vector by one interval.
 *
 *  \param uEnd Value for the u vector on the second to last node. If =0 the old value is used.
 */
void nmhe_shiftControls( real_t* const uEnd );

/** Get the KKT tolerance of the current iterate.
 *
 *  \return The KKT tolerance value.
 */
real_t nmhe_getKKT(  );

/** Calculate the objective value.
 *
 *  \return Value of the objective function.
 */
real_t nmhe_getObjective(  );

/** Use this function to update the arrival cost.
 *
 *  \param reset Reset S_{AC}. Set it to 1 to initialize arrival cost calculation, and later should set it to 0.
 */
int nmhe_updateArrivalCost( int reset );


/* 
 * Extern declarations. 
 */

extern NMHEworkspace nmheWorkspace;
extern NMHEvariables nmheVariables;

/** @} */

#ifndef __MATLAB__
#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */
#endif /* __MATLAB__ */

#endif /* NMHE_COMMON_H */