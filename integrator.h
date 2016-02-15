/**
 * @file integrator.hpp
 * @brief Interfaces for integrators and systems.
 *
 * @author Kristin Siu (kasiu)
 * @author Eric Butler (edbutler)
 */

#pragma once

#include <vector>

/**
 * Interface for an ODE system that can be solved with an integrator.
 * Similar to the interface discussed in the class notes, see those
 * and the writeup for further details.
 *
 * And integrable system is represented as a state vector of reals and the
 * current time. It provides functionality to get/set this state and compute
 * the derivative at the current time.
 *
 * The flock will be a particle system that implements this interface., where
 * the state vector is the positions and velocities of all the particles.
 */
class IntegrableSystem
{
public:

    virtual ~IntegrableSystem() { }

    /**
     * @return The number of elements in the state array.
     *   This should be constant.
     */
    virtual unsigned int size_pos() const = 0;

    /**
     * Queries the current state.
     * @param arr[out] An array of at least this->size() in length. Where the
     *   state vector will be stored.
     * @param time[out] Where the current time will be stored.
     */
    virtual void get_state_pos( double* arr) const = 0;

    /**
     * Sets the current state, overriding the given state.
     * @param arr[in] An array of at least this->size() in length. The state
     *   vector to be set.
     * @param time[in] The time to be set.
     */
    virtual void set_state_pos( const double* arr) = 0;

    /**
     * Compute the derivative of the current state at the current time,
     * storing the result in the given vector.
     * @param deriv_result[out] An array of at least this->size() in length.
     *   The computed derivative of the state vector is stored here.
     */
    virtual void eval_deriv_pos( double* deriv_result ) = 0;

    /**
     * @return The number of elements in the state array.
     *   This should be constant.
     */
    virtual unsigned int size_vel() const = 0;

    /**
     * Queries the current state.
     * @param arr[out] An array of at least this->size() in length. Where the
     *   state vector will be stored.
     * @param time[out] Where the current time will be stored.
     */
    virtual void get_state_vel( double* arr) const = 0;

    /**
     * Sets the current state, overriding the given state.
     * @param arr[in] An array of at least this->size() in length. The state
     *   vector to be set.
     * @param time[in] The time to be set.
     */
    virtual void set_state_vel( const double* arr) = 0;

    /**
     * Compute the derivative of the current state at the current time,
     * storing the result in the given vector.
     * @param deriv_result[out] An array of at least this->size() in length.
     *   The computed derivative of the state vector is stored here.
     */
    virtual void eval_deriv_vel( double* deriv_result ) = 0;


	/****************************
	* for RB contact resolution *
	*****************************/
	
	/**
     * Queries the current state.
     * @param arr[out] An array of at least this->size() in length. Where the
     *   state vector will be stored.
     * @param time[out] Where the current time will be stored.
     */
    virtual void get_state_pos( double *arr, int i) const = 0;

    /**
     * Sets the current state, overriding the given state.
     * @param arr[in] An array of at least this->size() in length. The state
     *   vector to be set.
     * @param time[in] The time to be set.
     */
    virtual void set_state_pos( const double *arr, int i) = 0;

    /**
     * Compute the derivative of the current state at the current time,
     * storing the result in the given vector.
     * @param deriv_result[out] An array of at least this->size() in length.
     *   The computed derivative of the state vector is stored here.
     */
    virtual void eval_deriv_pos( double *deriv_result, int i) = 0;

    /**
     * Queries the current state.
     * @param arr[out] An array of at least this->size() in length. Where the
     *   state vector will be stored.
     * @param time[out] Where the current time will be stored.
     */
    virtual void get_state_vel( double *arr, int i) const = 0;

    /**
     * Sets the current state, overriding the given state.
     * @param arr[in] An array of at least this->size() in length. The state
     *   vector to be set.
     * @param time[in] The time to be set.
     */
    virtual void set_state_vel( const double *arr, int i) = 0;

    /**
     * Compute the derivative of the current state at the current time,
     * storing the result in the given vector.
     * @param deriv_result[out] An array of at least this->size() in length.
     *   The computed derivative of the state vector is stored here.
     */
    virtual void eval_deriv_vel( double *deriv_result, int i ) = 0;

	/**
     * @return The number of actual objects in the system.
     *   This should be constant.
     */
	virtual unsigned int num_bodies() const = 0;
};

/**
 * Interface for integrators that can step the simulation of a system.
 */
class Integrator
{
public:
    Integrator() { }
    virtual ~Integrator() { }

    /**
     * Step the simulation of the given system by the given timestep.
     * @param sys The system to integrate. It should be integrated starting
     *   from the system's current step.
     * @param dt The length of the time step to integrate.
     */
    virtual void integrate_pos( IntegrableSystem& sys, double dt ) const = 0;
    virtual void integrate_vel( IntegrableSystem& sys, double dt ) const = 0;

    // used for storing state vectors locally
    // without allocating memory every time.
    typedef std::vector<double> StateList;
};

/**
 * Interface for integrators that can step the simulation of a system one body at a time.
 */
class RBIntegrator
{
public:
    RBIntegrator() { }
    virtual ~RBIntegrator() { }

    /**
     * Step the simulation of the given system by the given timestep.
     * @param sys The system to integrate. It should be integrated starting
     *   from the system's current step.
     * @param dt The length of the time step to integrate.
     */
    virtual void integrate_pos( IntegrableSystem& sys, double dt, int i ) const = 0;
    virtual void integrate_vel( IntegrableSystem& sys, double dt, int i ) const = 0;

    // used for storing state vectors locally
    // without allocating memory every time.
    typedef std::vector<double> StateList;
};

/**
 * Uses the basic Euler integration method, x' = x + dx/dt * dt.
 */
class EulerIntegrator : public Integrator
{
public:
    EulerIntegrator() { }
	virtual ~EulerIntegrator() { state.clear(); deriv_state.clear();}
    virtual void integrate_pos( IntegrableSystem& sys, double dt ) const;
    virtual void integrate_vel( IntegrableSystem& sys, double dt ) const;
private:
    mutable StateList state;
    mutable StateList deriv_state;
};

/**
 * Uses the basic Euler integration method, x' = x + dx/dt * dt.
 * Integrates one body at a time.
 */
class EulerRBIntegrator : public RBIntegrator
{
public:
    EulerRBIntegrator() { }
	virtual ~EulerRBIntegrator() { state.clear(); deriv_state.clear();}
    virtual void integrate_pos( IntegrableSystem& sys, double dt, int i ) const;
    virtual void integrate_vel( IntegrableSystem& sys, double dt, int i ) const;
private:
    mutable StateList state;
    mutable StateList deriv_state;
};
