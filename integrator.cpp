/**
 * @file integrator.cpp
 * @brief integration methods.
 *
 * @author Andrew Wesson (awesson)
 */

#include "integrator.h"

/**
 * Uses the basic Euler integration method, x' = x + dx/dt * dt.
 * @param sys The system to integrate
 * @param dt The time step to integrate over
 */
void EulerIntegrator::integrate_pos( IntegrableSystem& sys, double dt ) const
{
    int size = sys.size_pos();

    if (size == 0)
        return;
    state.resize( size );
    deriv_state.resize( size );

    // get the current state
    sys.get_state_pos( &state[0] );

    // compute the current derivative
    sys.eval_deriv_pos( &deriv_state[0] );

    // update the state
    for(int i = 0; i < size; ++i){
        state[i] += deriv_state[i]*dt;
    }

    // set the updated state
    sys.set_state_pos( &state[0] );
}

/**
 * Uses the basic Euler integration method, x' = x + dx/dt * dt.
 * @param sys The system to integrate
 * @param dt The time step to integrate over
 */
void EulerIntegrator::integrate_vel( IntegrableSystem& sys, double dt ) const
{
    int size = sys.size_vel();

    if (size == 0)
        return;
    state.resize( size );
    deriv_state.resize( size );

    // get the current state
    sys.get_state_vel( &state[0] );

    // compute the current derivative
    sys.eval_deriv_vel( &deriv_state[0] );

    // update the state
    for(int i = 0; i < size; ++i){
        state[i] += deriv_state[i]*dt;
    }

    // set the updated state
    sys.set_state_vel( &state[0] );
}

/**
 * Uses the basic Euler integration method, x' = x + dx/dt * dt.
 * @param sys The system to integrate
 * @param dt The time step to integrate over
 */
void EulerRBIntegrator::integrate_pos( IntegrableSystem& sys, double dt, int i ) const
{
    int size = sys.size_pos();
	int body_size = size / sys.num_bodies();

    if (size == 0)
        return;
    state.resize( size );
    deriv_state.resize( size );

    // get the current state
    sys.get_state_pos( &state[0] + i*body_size, i );

    // compute the current derivative
    sys.eval_deriv_pos( &deriv_state[0] + i*body_size, i );

    // update the state
    for(int ii = 0; ii < size; ++ii){
        state[ii] += deriv_state[ii]*dt;
    }

    // set the updated state
    sys.set_state_pos( &state[0] + i*body_size, i );
}

/**
 * Uses the basic Euler integration method, x' = x + dx/dt * dt.
 * @param sys The system to integrate
 * @param dt The time step to integrate over
 */
void EulerRBIntegrator::integrate_vel( IntegrableSystem& sys, double dt, int i ) const
{
    int size = sys.size_vel();
	int body_size = size / sys.num_bodies();

    if (size == 0)
        return;
    state.resize( size );
    deriv_state.resize( size );

    // get the current state
    sys.get_state_vel( &state[0] + i*body_size, i );

    // compute the current derivative
    sys.eval_deriv_vel( &deriv_state[0] + i*body_size, i );

    // update the state
    for(int ii = 0; ii < size; ++ii){
        state[ii] += deriv_state[ii]*dt;
    }

    // set the updated state
    sys.set_state_vel( &state[0] + i*body_size, i );
}
