#pragma once

#include <gfx/vec2.h>
#include <vector>
#include <stack>
#include <stdlib.h>
#include "Body.h"
#include "integrator.h"

#define Ks 100.0f
#define Kd 100.0f
#define POS_STATE_SIZE 7
#define VEL_STATE_SIZE 6
#define g 9.8

class System : public IntegrableSystem
{
public:
	System(std::vector<Body*> &bVector);
	~System(void);

	void zero_forces();
	void add_gravity();
	bool collsion_detect(double* prev_pos, double* prev_vel);
	bool contact_detect(int iter, bool is_shock_prop);
	virtual void eval_deriv_pos(double xdot[]);
	virtual void eval_deriv_vel(double xdot[]);
	virtual void get_state_pos(double x[]) const;
	virtual void get_state_vel(double x[]) const;
	virtual void get_state_pos(double x[], int i) const;
	virtual void get_state_vel(double x[], int i) const;
	virtual void get_state_pos(double x[], Body *b) const;
	virtual void get_state_vel(double x[], Body *b) const;
	virtual void get_bodies(std::vector<Body*> &);
	virtual void set_state_pos(const double x[]);
	virtual void set_state_vel(const double x[]);
	virtual void set_state_pos(const double x[], int i);
	virtual void set_state_vel(const double x[], int i);
	virtual void set_state_pos(const double x[], Body *b);
	virtual void set_state_vel(const double x[], Body *b);
	virtual void eval_deriv_pos( double xdot[], int i);
	virtual void eval_deriv_vel( double xdot[], int i);
	void topological_tarjan();
	void saveOutputData(std::vector<BodyInfo> &);
	virtual unsigned int num_bodies() const;
	virtual unsigned int size_pos() const;
	virtual unsigned int size_vel() const;

	std::vector<Body*> bVector;
	int size;

private:
	bool resolve_collisions(Body *b1, Body *b2, Vec3 r1, Vec3 r2, Vec3 normal, int iter, bool is_contact);
	void strongconnect(Body* b, int &index);
};
