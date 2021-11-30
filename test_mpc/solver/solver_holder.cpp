#include "acado_common.h"
#include "acado_auxiliary_functions.h"
#include "solver_holder.h"

#include <stdio.h>
#include <math.h>
#include "ros/ros.h"
#include <nav_msgs/Path.h>

/* Some convenient definitions. */
#define NX          ACADO_NX  /* Number of differential state variables.  */
#define NXA         ACADO_NXA /* Number of algebraic variables. */
#define NU          ACADO_NU  /* Number of control inputs. */
#define NOD         ACADO_NOD  /* Number of online data values. */

#define NY          ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN         ACADO_NYN /* Number of measurements/references on node N. */

#define N           ACADO_N   /* Number of intervals in the horizon. */

//#define NUM_STEPS   5        /* Number of real-time iterations. */
//#define VERBOSE     0         /* Show iterations: 1, silent: 0.  */

/* Global variables used by the solver. */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;


//MPC_solver::MPC_solver() : num_steps(0) { }
MPC_solver::MPC_solver(int n): num_steps(n) {
	int i;
	/* Initialize the solver. */
	acado_initializeSolver();

	/* Initialize the states and controls. */
	for (i = 0; i < NX * (N + 1); ++i)  acadoVariables.x[ i ] = 0.0;
	for (i = 0; i < NU * N; ++i)  acadoVariables.u[ i ] = 0.0;

	/* Initialize the measurements/reference. */
	for (i = 0; i < NY * N; ++i)  acadoVariables.y[ i ] = 0.0;
};

int MPC_solver::reinitialize(){
	int i;
	/* Initialize the solver. */
	acado_initializeSolver();

	/* Initialize the states and controls. */
	for (i = 0; i < NX * (N + 1); ++i)  acadoVariables.x[ i ] = 0.0;
	for (i = 0; i < NU * N; ++i)  acadoVariables.u[ i ] = 0.0;

	/* Initialize the measurements/reference. */
	for (i = 0; i < NY * N; ++i)  acadoVariables.y[ i ] = 0.0;
	return 0;
};

double * MPC_solver::solve_mpc(double input_arr[7], double online_arr[600], double target_arr[7]) {

	/* currentState_targetValue parsing:
	0 - 6: current theta
	7 - 13: goal theta

	output j_dot[7]
	*/
	
	//printf("\ncurrent: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", x0[0], x0[1], x0[2], x0[3], x0[4], x0[5], x0[6] );
	//printf("\n___goal: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", yN[0], yN[1], yN[2], yN[3], yN[4], yN[5], yN[6] );
	/* Some temporary variables. */
	int    i, j, iter;
	acado_timer t;
	
	// in model we have 7 theta and 7 theta_dot, we assign fixed reference with zero velocity
        
        // tracking goal
	for (j = 0; j < N; j++) {
		//for (i = 0; i < 7; i++)  acadoVariables.y[j*NY + i ] = yN[ i ];
                for (i = 0; i < 7; i++)  acadoVariables.y[ j*NY + i ] = target_arr[ i ];
//ROS_INFO("y_%.3i_%.3i = %.3g", j,i, target_arr[ i ]);
	
	}
	
        // terminal goal
	for (i = 0; i < NYN; ++i)  acadoVariables.yN[ i ] = target_arr[ i ];
//ROS_INFO("yN_%.3i = %.3g", i, target_arr[ i ]);}

	/* Initialize online data. */
	/*for (j = 0; j < N; ++j) {
		for (i = 0; i < NOD; ++i)  acadoVariables.od[ j*NOD+i ] = online_arr[i];
	}*/

	for (j = 0; j < N*NOD; ++j) {
 		acadoVariables.od[ j ] = online_arr[ j ];
//ROS_INFO("od_%.3i = %.3g", j, online_arr[ j ]);
        }

        //printf("values %.3f %.3f %.3f %.3f", human_spheres[44], human_spheres[45], human_spheres[46], human_spheres[47]);

	/* MPC: initialize the current state feedback. */
#if ACADO_INITIAL_STATE_FIXED
	for (i = 0; i < 7; ++i) acadoVariables.x0[ i ] = input_arr[ i ];
//ROS_INFO("x0_%.3i = %.3g", i, input_arr[ i ]);}
#endif

	for (i = 0; i < NU * N; ++i)  acadoVariables.u[ i ] = 0.0;

	acado_tic( &t );
	for(iter = 0; iter < num_steps; ++iter)
	    {
		/* Prepare for the RTI step. */
		acado_preparationStep();
		/* Compute the feedback step. */
		acado_feedbackStep( );
	    }
	real_t te = acado_toc( &t );
	real_t KKT_val = acado_getKKT();

	ROS_INFO("Time: %.3g ms; KKT = %.3e", 1e3 * te, KKT_val);

    static double output[1000];//[N*30]

	//static double joint_commands[2];
//ROS_INFO("u_result_0 = %.3g", acadoVariables.u[ 0 ]);
//ROS_INFO("u_result_1 = %.3g", acadoVariables.u[ 1 ]);
        for (i = 0; i < NU; ++i) {
         output[i] =
            //(target_arr[5 + i] + 0.01 * acadoVariables.u[i]) /
			acadoVariables.x[NX + 4 + i]/0.098; //wheel radius
	}

    //static double pred_traj_[N*3];
    for (int i = 0; i < N; i++)
    {
        output[i*3+0+2] = acadoVariables.x[i * NX + 0]; //x
			// ROS_INFO("x %i", i);

        output[i*3+1+2] =acadoVariables.x[i * NX + 1]; //y
			// ROS_INFO("y %i", i);

		output[i*3+2+2] = acadoVariables.x[i * NX + 2]; //theta
			// ROS_INFO("t %i", i);

    }
    	// ROS_INFO("pred finish");

	if (KKT_val > 0.01) {
		acado_initializeSolver();

		/* Initialize the states and controls. */
		for (i = 0; i < NX * (N + 1); ++i)  acadoVariables.x[ i ] = 0.0;
		for (i = 0; i < NU * N; ++i)  acadoVariables.u[ i ] = 0.0;

		/* Initialize the measurements/reference. */
		for (i = 0; i < NY * N; ++i)  acadoVariables.y[ i ] = 0.0;
	} else {
		acado_shiftStates(2, 0, 0);
		acado_shiftControls( 0 );
	}
	// ROS_INFO("pred finish");
	return output;
	//return acadoVariables.x;
}
