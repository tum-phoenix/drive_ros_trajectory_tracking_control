// Andromeda controller, 2016-11-09, Mikhail Pak, MIT license.
#ifndef ANDROMEDA_CONTROLLER
#define ANDROMEDA_CONTROLLER

#include <stddef.h> // for size_t
#include <math.h> // for fmax, fmaxf, fmin, fminf, sqrt, sqrtf, atan2 and atan2f, fabs and fabsf


// ================================================================================================
// Type definitions
// ================================================================================================
// Define a type for real numbers.
//
// I'd like to be flexible; we should start with double and see how it goes;
// then investigate whether float is faster.
typedef double Real;
#define FUN_MIN fmin // double
#define FUN_MAX fmax // double
#define FUN_ABS fabs // double
#define FUN_SQRT sqrt // double
#define FUN_ATAN2 atan2 // double
// typedef float Real;
// #define FUN_MIN fminf // float
// #define FUN_MAX fmaxf // float
// #define FUN_ABS fabsf // float
// #define FUN_SQRT sqrtf // float
// #define FUN_ATAN2 atan2f // float
// ================================================================================================


// ================================================================================================
// Constants
// ================================================================================================
// Define horizon length.
#ifndef HORIZON_LEN
#define HORIZON_LEN 10
#endif
// Define number of nodes in the polygonal chain.
#ifndef CHAIN_NUM_NODES
#define CHAIN_NUM_NODES 15
#endif
// Define the number of states.
// This is also the dimension of matrices Q and P.
#define NUM_STATES 4
// Define the number of inputs.
// This is also the dimension of matrix R.
#define NUM_INPUTS 2
// Define the time step in seconds.
#ifndef TIME_STEP
#define TIME_STEP 0.100
#endif
// Define the wheel base in metres.
#ifndef WHEEL_BASE
#define WHEEL_BASE 0.208
#endif
// Define the numerical fudge factor to avoid division by zero.
// Should be approximately equal to the square root of the machine epsilon.
#define EPS_FUDGE 1.0e-7
// Define the relative tolerance for almost straight path segments.
#ifndef CURVATURE_M_TOL
#define CURVATURE_M_TOL 1.0e-3
#endif
// Define the virtual radius of curvature for straight path segments in metres.
#ifndef CURVATURE_HUGE_RADIUS
#define CURVATURE_HUGE_RADIUS 400.0
#endif
// ================================================================================================


// ================================================================================================
// Function declarations
// ================================================================================================
extern void state_transition(const Real *restrict x_curr,
                             const Real *restrict u_curr,
                             const Real *restrict v_curr,
                             Real *restrict x_next);


void obj_fun(const Real *restrict x_0,
             const Real *restrict v,
             const Real *restrict y_ref,
             const Real *restrict phi_ref,
             const Real *restrict u_1,
             const Real *restrict u_2,
             const Real *restrict q_diag,
             const Real *restrict r_diag,
             const Real *restrict p_diag,
             Real *restrict val);


void gradient(const Real *restrict x_0,
              const Real *restrict v,
              const Real *restrict y_ref,
              const Real *restrict phi_ref,
              const Real *restrict u_1,
              const Real *restrict u_2,
              const Real *restrict q_diag,
              const Real *restrict r_diag,
              const Real *restrict p_diag,
              Real *restrict grad_u_1,
              Real *restrict grad_u_2);


void optimize(const Real *restrict x_0,
              const Real *restrict v,
              const Real *restrict y_ref,
              const Real *restrict phi_ref,
              const Real *restrict u_1_init,
              const Real *restrict u_2_init,
              const Real *restrict q_diag,
              const Real *restrict r_diag,
              const Real *restrict p_diag,
              const size_t max_num_iter,
              const Real alpha,
              const Real beta_1,
              const Real beta_2,
              const Real u_1_lb,
              const Real u_1_ub,
              const Real u_2_lb,
              const Real u_2_ub,
              Real *restrict u_1_star,
              Real *restrict u_2_star);


void compute_radius_of_curvature(const Real *restrict nodes_x,
                                 const Real *restrict nodes_y,
                                 const Real sq_link_length,
                                 Real *restrict r_o_c);


void generate_velocity_and_references(const Real *restrict nodes_x,
                                      const Real *restrict nodes_y,
                                      const Real link_length,
                                      const Real *restrict min_speed,
                                      const Real *restrict max_speed,
                                      const Real max_lateral_acc,
                                      Real *restrict v,
                                      Real *restrict y_ref,
                                      Real *restrict phi_ref);


void call_andromeda(const Real *restrict x_0,
                    const Real *restrict q_diag,
                    const Real *restrict r_diag,
                    const Real *restrict p_diag,
                    const Real *restrict nodes_x,
                    const Real *restrict nodes_y,
                    const Real link_length,
                    const Real *restrict min_speed,
                    const Real *restrict max_speed,
                    const Real max_lateral_acc,
                    const size_t max_num_iter,
                    const Real alpha,
                    const Real beta_1,
                    const Real beta_2,
                    const Real u_1_lb,
                    const Real u_1_ub,
                    const Real u_2_lb,
                    const Real u_2_ub,
                    Real *restrict v_star,
                    Real *restrict u_1_star,
                    Real *restrict u_2_star);
// ================================================================================================


#endif /* ANDROMEDA_CONTROLLER */
