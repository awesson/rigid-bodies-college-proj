// LocalRigidBodies.cpp : Defines the entry point for the console application.
//
// Andrew Wesson
//

#include "Body.h"
#include "imageio.h"
#include "System.h"
#include "integrator.h"
#include "Box.h"
#include "csapp.h"

#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <GLUT/glut.h>
#include <time.h>

/* macros */
#define MAX_COLLISIONS 5
#define MAX_CONTACTS 5
#define MAX_SHOCK_PROP 1
#define rot_ang PI/5.0

/* global variables */

static int frame_time = 16;
static float dt;
static float prev_fps_taken_time;
static int dsim;
static int dump_frames;
static int frame_number;

static std::vector<Body*> bVector;

static int win_id;
static int win_x, win_y;
static int mouse_down[3];
static int mouse_release[3];
static int mouse_shiftclick[3];
static int omx, omy, mx, my;
static int hmx, hmy;
// flag for whether the mouse has already been registered as being down
static bool clicked;

static RBIntegrator* integrator;
static System* sys = NULL;

// camera data
static Vec3 camera(0.0, 10.0, -10.0);
static Vec3 target(0.0, 0.0, 0.0);
static GLfloat light_position[4] = { -100.0, 1000, -100.0, 1.0 };

// networking data
static int start_time, reset_time;

// globals for tarjan's algorithm
extern std::vector<Body*> top_sorted;
extern std::stack<Body*> S;
extern int SCC_num;

static double *prev_pos, *prev_vel, *y_vel;

/*********************************************************************
* free/clear/allocate simulation data
**********************************************************************/

static void free_data ( void )
{
	delete sys;
	bVector.clear();
	delete integrator;
	delete[] prev_pos;
	delete[] prev_vel;
	delete[] y_vel;
}

/*********************************************************************
* initialization functions to set up different scenes
**********************************************************************/
static void init_slide()
{
	const double dist = 1.;
	const Vec3 center(0.0, -10.0, 0.0);
	const Vec3 x_offset(dist, 0.0, 0.0);
	const Vec3 y_offset(0.0, dist, 0.0);
	const Vec3 z_offset(0.0, 0.0, dist);

	// floor
	bVector.push_back(new Body(center, Quaternion(Vec3(0.0, 0.0, 1.0), rot_ang), new Box(Color3(1.0, 1.0, .5)), Vec3(20, 20, 20), 1.0, .7, 0.0));

	bVector.push_back(new Body(center + (10*(sin(rot_ang) + cos(rot_ang)) + .5*(cos(rot_ang) - sin(rot_ang)))*y_offset + (10*(cos(rot_ang) - sin(rot_ang)) - .5*(sin(rot_ang) + cos(rot_ang)))*x_offset, Quaternion(Vec3(0.0, 0.0, 1.0), rot_ang), new Box(Color3(.1, .7, .1)), Vec3(1, 1, 1), 1.0, 1.0, 1.0));
}

static void init_combo()
{
	const double dist = 1.;
	const Vec3 center(5.0, 10.0, 0.0);
	const Vec3 x_offset(dist, 0.0, 0.0);
	const Vec3 y_offset(0.0, dist + 100*EPSILON, 0.0);
	const Vec3 z_offset(0.0, 0.0, dist);
	const double second_wave_offset = 9;
	
	light_position[1] = 2000.0;

	// floor
	bVector.push_back(new Body(center - 110*y_offset, Quaternion::Identity, new Box(Color3(1.0, 1.0, .5)), Vec3(200, 200, 200), .4, 0.5, 0.0));
	bVector.push_back(new Body(center - (3 + 5.0*sqrt(2) - 14.75/sqrt(2))*y_offset + (3 - 4.75/sqrt(2))*x_offset,  Quaternion(Vec3(0.0, 0.0, 1.0), PI/4.0), new Box(Color3(.7, 0.0, .0)), Vec3(10, .5, 10), .4, .5, 0.0));
	bVector.push_back(new Body(center - (3 + 5.0*sqrt(2) - 14.75/sqrt(2))*y_offset - (10 + 3.25/sqrt(2))*x_offset,  Quaternion(Vec3(0.0, 0.0, 1.0), -PI/4.0), new Box(Color3(0.0, 0.2, .7)), Vec3(10, .5, 10), .4, .5, 0.0));
	
	bVector.push_back(new Body(center - (-second_wave_offset + (3 + 5.0*sqrt(2) - 14.75/sqrt(2)))*y_offset + (3 - 4.75/sqrt(2))*x_offset,  Quaternion(Vec3(0.0, 0.0, 1.0), PI/4.0), new Box(Color3(.7, 0.0, .0)), Vec3(10, .5, 10), .4, .5, 0.0));
	bVector.push_back(new Body(center - (-second_wave_offset + (3 + 5.0*sqrt(2) - 14.75/sqrt(2)))*y_offset - (10 + 3.25/sqrt(2))*x_offset,  Quaternion(Vec3(0.0, 0.0, 1.0), -PI/4.0), new Box(Color3(0.0, 0.2, .7)), Vec3(10, .5, 10), .4, .5, 0.0));
	// right
	bVector.push_back(new Body(center + (5*(sqrt(2) - 1) + 2)*y_offset - (.5*sqrt(2) - 3)*x_offset + 2*z_offset, Quaternion(Vec3(0.0, 0.0, 1.0), PI/4.0), new Box(Color3(.1, .7, .1)), Vec3(1, 1, 1), .7, .5, 1.0));
	bVector.push_back(new Body(center + (5*(sqrt(2) - 1) + .7)*y_offset - (.5*sqrt(2) - 1.7)*x_offset + 1.5*z_offset, Quaternion(Vec3(0.0, 0.0, 1.0), PI/4.0), new Box(Color3(.1, .7, .1)), Vec3(1, 1, 1), .7, .5, 1.0));
	bVector.push_back(new Body(center + (5*(sqrt(2) - 1) + 1.7)*y_offset - (.5*sqrt(2) - 2.7)*x_offset - 2*z_offset, Quaternion(Vec3(0.0, 0.0, 1.0), PI/4.0), new Box(Color3(.1, .7, .1)), Vec3(1.7, 1, 1), .7, .5, 1.0));
	bVector.push_back(new Body(center + (5*(sqrt(2) - 1) + .5)*y_offset - (.5*sqrt(2) - 1.5)*x_offset - 1.5*z_offset, Quaternion(Vec3(0.0, 0.0, 1.0), PI/4.0), new Box(Color3(.1, .7, .1)), Vec3(1, 1, 1), .7, .5, 1.0));
	bVector.push_back(new Body(center + (5*(sqrt(2) - 1) + 2)*y_offset - (.5*sqrt(2) - 3)*x_offset, Quaternion(Vec3(0.0, 0.0, 1.0), PI/4.0), new Box(Color3(.1, .7, .1)), Vec3(1, 1, 1), .7, .5, 1.0));
	bVector.push_back(new Body(center + (5*(sqrt(2) - 1) + 1)*y_offset - (.5*sqrt(2) - 2)*x_offset, Quaternion(Vec3(0.0, 0.0, 1.0), PI/4.0), new Box(Color3(.1, .7, .1)), Vec3(1, 1, 1.5), .7, .5, 1.0));
	bVector.push_back(new Body(center + (5*(sqrt(2) - 1) + 2 + 3.5)*y_offset - (.5*sqrt(2) - 3)*x_offset + 2*z_offset, Quaternion(Vec3(0.0, 0.0, 1.0), PI/4.0), new Box(Color3(.1, .7, .1)), Vec3(1, 1, 1), .7, .5, 1.0));
	bVector.push_back(new Body(center + (5*(sqrt(2) - 1) + 1.7 + 3.5)*y_offset - (.5*sqrt(2) - 2.7)*x_offset - 2*z_offset, Quaternion(Vec3(0.0, 0.0, 1.0), PI/4.0), new Box(Color3(.1, .7, .1)), Vec3(1.7, 1, 1), .7, .5, 1.0));
	bVector.push_back(new Body(center + (5*(sqrt(2) - 1) + 2 + 3.5)*y_offset - (.5*sqrt(2) - 3)*x_offset, Quaternion(Vec3(0.0, 0.0, 1.0), PI/4.0), new Box(Color3(.1, .7, .1)), Vec3(1, 1, 1), .7, .5, 1.0));
	
	bVector.push_back(new Body(center + (second_wave_offset + (5*(sqrt(2) - 1) + 2))*y_offset - (.5*sqrt(2) - 3)*x_offset + 2*z_offset, Quaternion(Vec3(0.0, 0.0, 1.0), PI/4.0), new Box(Color3(.1, .7, .1)), Vec3(1, 1, 1), .7, .5, 1.0));
	bVector.push_back(new Body(center + (second_wave_offset + (5*(sqrt(2) - 1) + .7))*y_offset - (.5*sqrt(2) - 1.7)*x_offset + 1.5*z_offset, Quaternion(Vec3(0.0, 0.0, 1.0), PI/4.0), new Box(Color3(.1, .7, .1)), Vec3(1, 1, 1), .7, .5, 1.0));
	bVector.push_back(new Body(center + (second_wave_offset + (5*(sqrt(2) - 1) + 1.7))*y_offset - (.5*sqrt(2) - 2.7)*x_offset - 2*z_offset, Quaternion(Vec3(0.0, 0.0, 1.0), PI/4.0), new Box(Color3(.1, .7, .1)), Vec3(1.7, 1, 1), .7, .5, 1.0));
	bVector.push_back(new Body(center + (second_wave_offset + (5*(sqrt(2) - 1) + .5))*y_offset - (.5*sqrt(2) - 1.5)*x_offset - 1.5*z_offset, Quaternion(Vec3(0.0, 0.0, 1.0), PI/4.0), new Box(Color3(.1, .7, .1)), Vec3(1, 1, 1), .7, .5, 1.0));
	bVector.push_back(new Body(center + (second_wave_offset + (5*(sqrt(2) - 1) + 2))*y_offset - (.5*sqrt(2) - 3)*x_offset, Quaternion(Vec3(0.0, 0.0, 1.0), PI/4.0), new Box(Color3(.1, .7, .1)), Vec3(1, 1, 1), .7, .5, 1.0));
	bVector.push_back(new Body(center + (second_wave_offset + (5*(sqrt(2) - 1) + 1))*y_offset - (.5*sqrt(2) - 2)*x_offset, Quaternion(Vec3(0.0, 0.0, 1.0), PI/4.0), new Box(Color3(.1, .7, .1)), Vec3(1, 1, 1.5), .7, .5, 1.0));
	bVector.push_back(new Body(center + (second_wave_offset + (5*(sqrt(2) - 1) + 2 + 3.5))*y_offset - (.5*sqrt(2) - 3)*x_offset + 2*z_offset, Quaternion(Vec3(0.0, 0.0, 1.0), PI/4.0), new Box(Color3(.1, .7, .1)), Vec3(1, 1, 1), .7, .5, 1.0));
	bVector.push_back(new Body(center + (second_wave_offset + (5*(sqrt(2) - 1) + 1.7 + 3.5))*y_offset - (.5*sqrt(2) - 2.7)*x_offset - 2*z_offset, Quaternion(Vec3(0.0, 0.0, 1.0), PI/4.0), new Box(Color3(.1, .7, .1)), Vec3(1.7, 1, 1), .7, .5, 1.0));
	bVector.push_back(new Body(center + (second_wave_offset + (5*(sqrt(2) - 1) + 2 + 3.5))*y_offset - (.5*sqrt(2) - 3)*x_offset, Quaternion(Vec3(0.0, 0.0, 1.0), PI/4.0), new Box(Color3(.1, .7, .1)), Vec3(1, 1, 1), .7, .5, 1.0));
	//left
	bVector.push_back(new Body(center + (5*(sqrt(2) - 1) + 2)*y_offset - (3.5*sqrt(2) + 10)*x_offset + 2*z_offset, Quaternion(Vec3(0.0, 0.0, 1.0), PI/4.0), new Box(Color3(1, .7, .1)), Vec3(1, 1, 1), .7, .5, 1.0));
	bVector.push_back(new Body(center + (5*(sqrt(2) - 1) + 1.5)*y_offset - (3.5*sqrt(2) + 9.5)*x_offset - 1.5*z_offset, Quaternion(Vec3(0.0, 0.0, 1.0), PI/4.0), new Box(Color3(1, .7, .1)), Vec3(1, 1, 1), .7, .5, 1.0));
	bVector.push_back(new Body(center + (5*(sqrt(2) - 1) + .8)*y_offset - (3.5*sqrt(2) - 4.7 + 13.5)*x_offset + 2*z_offset, Quaternion(Vec3(0.0, 0.0, 1.0), PI/4.0), new Box(Color3(1, .7, .1)), Vec3(1, 1.7, 1), .7, .5, 1.0));
	bVector.push_back(new Body(center + (5*(sqrt(2) - 1) + .5)*y_offset - (3.5*sqrt(2) - 4.5 + 13)*x_offset - 1.5*z_offset, Quaternion(Vec3(0.0, 0.0, 1.0), PI/4.0), new Box(Color3(1, .7, .1)), Vec3(1, 1, 1), .7, .5, 1.0));
	bVector.push_back(new Body(center + (5*(sqrt(2) - 1) + 2)*y_offset - (3.5*sqrt(2) - 3 + 13)*x_offset, Quaternion(Vec3(0.0, 0.0, 1.0), PI/4.0), new Box(Color3(1, .7, .1)), Vec3(1, 1, 1), .7, .5, 1.0));
	bVector.push_back(new Body(center + (5*(sqrt(2) - 1) + 1)*y_offset - (3.5*sqrt(2) - 5 + 14)*x_offset, Quaternion(Vec3(0.0, 0.0, 1.0), PI/4.0), new Box(Color3(1, .7, .1)), Vec3(1, 1, 1.5), .7, .5, 1.0));
	bVector.push_back(new Body(center + (5*(sqrt(2) - 1) + 1.5 + 3.5)*y_offset - (3.5*sqrt(2) + 9.5)*x_offset - 1.5*z_offset, Quaternion(Vec3(0.0, 0.0, 1.0), PI/4.0), new Box(Color3(1, .7, .1)), Vec3(1, 1, 1), .7, .5, 1.0));
	bVector.push_back(new Body(center + (5*(sqrt(2) - 1) + .8 + 3.5)*y_offset - (3.5*sqrt(2) - 4.7 + 13.5)*x_offset + 2*z_offset, Quaternion(Vec3(0.0, 0.0, 1.0), PI/4.0), new Box(Color3(1, .7, .1)), Vec3(1, 1.7, 1), .7, .5, 1.0));
	bVector.push_back(new Body(center + (5*(sqrt(2) - 1) + 1 + 3.5)*y_offset - (3.5*sqrt(2) - 5 + 14)*x_offset, Quaternion(Vec3(0.0, 0.0, 1.0), PI/4.0), new Box(Color3(1, .7, .1)), Vec3(1, 1, 1.5), .7, .5, 1.0));
	
	bVector.push_back(new Body(center + (second_wave_offset + (5*(sqrt(2) - 1) + 2))*y_offset - (3.5*sqrt(2) + 10)*x_offset + 2*z_offset, Quaternion(Vec3(0.0, 0.0, 1.0), PI/4.0), new Box(Color3(1, .7, .1)), Vec3(1, 1, 1), .7, .5, 1.0));
	bVector.push_back(new Body(center + (second_wave_offset + (5*(sqrt(2) - 1) + 1.5))*y_offset - (3.5*sqrt(2) + 9.5)*x_offset - 1.5*z_offset, Quaternion(Vec3(0.0, 0.0, 1.0), PI/4.0), new Box(Color3(1, .7, .1)), Vec3(1, 1, 1), .7, .5, 1.0));
	bVector.push_back(new Body(center + (second_wave_offset + (5*(sqrt(2) - 1) + .8))*y_offset - (3.5*sqrt(2) - 4.7 + 13.5)*x_offset + 2*z_offset, Quaternion(Vec3(0.0, 0.0, 1.0), PI/4.0), new Box(Color3(1, .7, .1)), Vec3(1, 1.7, 1), .7, .5, 1.0));
	bVector.push_back(new Body(center + (second_wave_offset + (5*(sqrt(2) - 1) + .5))*y_offset - (3.5*sqrt(2) - 4.5 + 13)*x_offset - 1.5*z_offset, Quaternion(Vec3(0.0, 0.0, 1.0), PI/4.0), new Box(Color3(1, .7, .1)), Vec3(1, 1, 1), .7, .5, 1.0));
	bVector.push_back(new Body(center + (second_wave_offset + (5*(sqrt(2) - 1) + 2))*y_offset - (3.5*sqrt(2) - 3 + 13)*x_offset, Quaternion(Vec3(0.0, 0.0, 1.0), PI/4.0), new Box(Color3(1, .7, .1)), Vec3(1, 1, 1), .7, .5, 1.0));
	bVector.push_back(new Body(center + (second_wave_offset + (5*(sqrt(2) - 1) + 1))*y_offset - (3.5*sqrt(2) - 5 + 14)*x_offset, Quaternion(Vec3(0.0, 0.0, 1.0), PI/4.0), new Box(Color3(1, .7, .1)), Vec3(1, 1, 1.5), .7, .5, 1.0));
	bVector.push_back(new Body(center + (second_wave_offset + (5*(sqrt(2) - 1) + 1.5 + 3.5))*y_offset - (3.5*sqrt(2) + 9.5)*x_offset - 1.5*z_offset, Quaternion(Vec3(0.0, 0.0, 1.0), PI/4.0), new Box(Color3(1, .7, .1)), Vec3(1, 1, 1), .7, .5, 1.0));
	bVector.push_back(new Body(center + (second_wave_offset + (5*(sqrt(2) - 1) + .8 + 3.5))*y_offset - (3.5*sqrt(2) - 4.7 + 13.5)*x_offset + 2*z_offset, Quaternion(Vec3(0.0, 0.0, 1.0), PI/4.0), new Box(Color3(1, .7, .1)), Vec3(1, 1.7, 1), .7, .5, 1.0));
	bVector.push_back(new Body(center + (second_wave_offset + (5*(sqrt(2) - 1) + 1 + 3.5))*y_offset - (3.5*sqrt(2) - 5 + 14)*x_offset, Quaternion(Vec3(0.0, 0.0, 1.0), PI/4.0), new Box(Color3(1, .7, .1)), Vec3(1, 1, 1.5), .7, .5, 1.0));
}

static void init_single_box()
{
	const double dist = 1.;
	const Vec3 center(0.0, 0.0, 0.0);
	const Vec3 x_offset(dist, 0.0, 0.0);
	const Vec3 y_offset(0.0, dist, 0.0);
	const Vec3 z_offset(0.0, 0.0, dist);

	// floor
	bVector.push_back(new Body(center - 0.5*y_offset, Quaternion::Identity, new Box(Color3(1.0, 1.0, .5)), Vec3(100, 1, 100), 0.5, 0.5, 0));

	bVector.push_back(new Body(center + 5*y_offset, Quaternion(Vec3(0.0, 0.0, 1.0), PI/4.0), new Box(Color3(.1, .7, .1)), Vec3(1, 1, 1), 1.0, 0.5, 1));
}

static void init_small_pile()
{
	const double dist = 1.;
	const Vec3 center(0.0, 0.0, 0.0);
	const Vec3 x_offset(dist, 0.0, 0.0);
	const Vec3 y_offset(0.0, dist, 0.0);
	const Vec3 z_offset(0.0, 0.0, dist);

	// floor
	bVector.push_back(new Body(center - 50*y_offset, Quaternion::Identity, new Box(Color3(1.0, 1.0, .5)), Vec3(100, 100, 100), .6, 0.5, 0));

	bVector.push_back(new Body(center + 3*y_offset - 4*x_offset + 0.5*z_offset, Quaternion::Identity, new Box(Color3(.1, .7, .1)), Vec3(2, 1, 1), 1.0, 0.5, 0.5));
	bVector.push_back(new Body(center + 5.5*y_offset - 2.2*x_offset + z_offset, Quaternion::Identity, new Box(Color3(.1, .7, .1)), Vec3(2, 1, 1), 1.0, 0.5, 0.5));
	bVector.push_back(new Body(center + 3*y_offset - x_offset + 0.5*z_offset, Quaternion(Vec3(0.0, 0.0, 1.0), PI/8.0), new Box(Color3(.1, .7, .1)), Vec3(1, 1, 1), 1.0, 0.5, 1));
	bVector.push_back(new Body(center + 1.7*y_offset - 1.5*x_offset, Quaternion(Vec3(0.0, 0.0, 1.0), PI/4.0), new Box(Color3(.1, .7, .1)), Vec3(1, 1, 1), 1.0, 0.5, 1.f));
	bVector.push_back(new Body(center + 2*y_offset - 5*x_offset + 2.5*z_offset, Quaternion::Identity, new Box(Color3(.1, .7, .1)), Vec3(2, 1, 1), 1.0, 0.5, 0.5));
	bVector.push_back(new Body(center + 6.5*y_offset - 3.2*x_offset - z_offset, Quaternion::Identity, new Box(Color3(.1, .7, .1)), Vec3(2, 1, 1), 1.0, 0.5, 0.5));
	bVector.push_back(new Body(center + 3*y_offset - 2*x_offset + 1.5*z_offset, Quaternion(Vec3(0.0, 0.0, 1.0), PI/8.0), new Box(Color3(.1, .7, .1)), Vec3(1, 1, 1), 1.0, 0.5, 1));
	bVector.push_back(new Body(center + 4.7*y_offset - 3.5*x_offset, Quaternion(Vec3(0.0, 0.0, 1.0), PI/4.0), new Box(Color3(.1, .7, .1)), Vec3(1, 1, 1), 1.0, 0.5, 1.f));
}

static void init_high_pile()
{
	const double dist = 1.;
	Vec3 center(0.0, 0.0, 0.0);
	const Vec3 x_offset(dist, 0.0, 0.0);
	const Vec3 y_offset(0.0, dist, 0.0);
	const Vec3 z_offset(0.0, 0.0, dist);
	
	const int iter=3;
	const double restitution = .6;
	const double z_sep = 5.5;
	const double x_sep = 7.5;
	double x_adj = -3;

	// floor
	bVector.push_back(new Body(center - .5*y_offset, Quaternion::Identity, new Box(Color3(1.0, 1.0, .5)), Vec3(1000, 1, 1000), restitution, 0.5, 0));
	
	// walls
	bVector.push_back(new Body(center + 10.1*y_offset + (.5 + x_sep*((double)iter/2.0))*x_offset + x_adj*x_offset, Quaternion::Identity, new Box(Color3(1.0, .5, .5)), Vec3(1, 20, z_sep*iter), restitution, 0.5, 0));
	bVector.push_back(new Body(center + 10.1*y_offset - (.5 + x_sep*((double)iter/2.0))*x_offset + x_adj*x_offset, Quaternion::Identity, new Box(Color3(1.0, .5, .5)), Vec3(1, 20, z_sep*iter), restitution, 0.5, 0));
	bVector.push_back(new Body(center + 10.1*y_offset + (.5 + z_sep*((double)iter/2.0))*z_offset + x_adj*x_offset, Quaternion::Identity, new Box(Color3(1.0, .5, .5)), Vec3(x_sep*iter, 20, 1), restitution, 0.5, 0));
	bVector.push_back(new Body(center + 10.1*y_offset - (.5 + z_sep*((double)iter/2.0))*z_offset + x_adj*x_offset, Quaternion::Identity, new Box(Color3(1.0, .5, .5)), Vec3(x_sep*iter, 20, 1), restitution, 0.5, 0));

	// 217 total objects
	for(int i = 0; i < iter; i++){
		for(int k = 0; k < iter; k++){
			for(int z = 0; z < iter; z++){
				bVector.push_back(new Body(center + (3+18*iter + (i-2)*18)*y_offset - (4 + (k-1)*x_sep)*x_offset + (0.5 + (z-1)*z_sep)*z_offset, Quaternion::Identity, new Box(Color3(.1, .7, .1)), Vec3(2, 1, 1), restitution, 0.5, 0.5));
				bVector.push_back(new Body(center + (5+18*iter + (i-2)*18)*y_offset - (1.2 + (k-1)*x_sep)*x_offset + (z-1)*z_sep*z_offset, Quaternion::Identity, new Box(Color3(.1, .7, .1)), Vec3(2, 1, 1), 1.0, 0.5, 0.5));
				bVector.push_back(new Body(center + (3+18*iter + (i-2)*18)*y_offset - (k-1)*x_sep*x_offset + (0.5 + (z-1)*z_sep)*z_offset, Quaternion(Vec3(0.0, 0.0, 1.0), PI/8.0), new Box(Color3(.1, .7, .1)), Vec3(1, 1, 1), restitution, 0.5, 1));
				bVector.push_back(new Body(center + (1.7+18*iter + (i-2)*18)*y_offset - (1.5 + (k-1)*x_sep)*x_offset + (z-1)*z_sep*z_offset, Quaternion(Vec3(0.0, 0.0, 1.0), PI/4.0), new Box(Color3(.1, .7, .1)), Vec3(1, 1, 1), restitution, 0.5, 1.f));
				bVector.push_back(new Body(center + (2+18*iter + (i-2)*18)*y_offset - (5 + (k-1)*x_sep)*x_offset + (2.5 + (z-1)*z_sep)*z_offset, Quaternion::Identity, new Box(Color3(.1, .7, .1)), Vec3(2, 1, 1), restitution, 0.5, 0.5));
				bVector.push_back(new Body(center + (6.5+18*iter + (i-2)*18)*y_offset - (3.2 + (k-1)*x_sep)*x_offset + (z-1)*z_sep*z_offset, Quaternion::Identity, new Box(Color3(.1, .7, .1)), Vec3(2, 1, 1), restitution, 0.5, 0.5));
				bVector.push_back(new Body(center + (3+18*iter + (i-2)*18)*y_offset - (2 + (k-1)*x_sep)*x_offset + (1.5 + (z-1)*z_sep)*z_offset, Quaternion(Vec3(0.0, 0.0, 1.0), PI/8.0), new Box(Color3(.1, .7, .1)), Vec3(1, 1, 1), restitution, 0.5, 1));
				bVector.push_back(new Body(center + (4.7+18*iter + (i-2)*18)*y_offset - (3.5 + (k-1)*x_sep)*x_offset + (z-1)*z_sep*z_offset, Quaternion(Vec3(0.0, 0.0, 1.0), PI/4.0), new Box(Color3(.1, .7, .1)), Vec3(1, 1, 1), restitution, 0.5, 1));
			}
		}
	}
	
	center += Vec3(0.0, iter*18, 0.0);
	Quaternion rot = Quaternion::Identity;//Quaternion(Vec3(0.0, 1.0, 0.0), PI/4.0);
	for(int i = 0; i < iter; i++){
		for(int k = 0; k < iter; k++){
			for(int z = 0; z < iter; z++){
				bVector.push_back(new Body(rot*(center + (3+18*iter + (i-2)*18)*y_offset - (4 + (k-1)*x_sep)*x_offset + (0.5 + (z-1)*z_sep)*z_offset), Quaternion::Identity, new Box(Color3(.1, .7, .1)), Vec3(2, 1, 1), restitution, 0.5, 0.5));
				bVector.push_back(new Body(rot*(center + (5+18*iter + (i-2)*18)*y_offset - (1.2 + (k-1)*x_sep)*x_offset + (z-1)*z_sep*z_offset), Quaternion::Identity, new Box(Color3(.1, .7, .1)), Vec3(2, 1, 1), restitution, 0.5, 0.5));
				bVector.push_back(new Body(rot*(center + (3+18*iter + (i-2)*18)*y_offset - (k-1)*x_sep*x_offset + (0.5 + (z-1)*z_sep)*z_offset), Quaternion(Vec3(0.0, 0.0, 1.0), PI/8.0), new Box(Color3(.1, .7, .1)), Vec3(1, 1, 1), restitution, 0.5, 1));
				bVector.push_back(new Body(rot*(center + (1.7+18*iter + (i-2)*18)*y_offset - (1.5 + (k-1)*x_sep)*x_offset + (z-1)*z_sep*z_offset), Quaternion(Vec3(0.0, 0.0, 1.0), PI/4.0), new Box(Color3(.1, .7, .1)), Vec3(1, 1, 1), restitution, 0.5, 1.f));
				bVector.push_back(new Body(rot*(center + (2+18*iter + (i-2)*18)*y_offset - (5 + (k-1)*x_sep)*x_offset + (2.5 + (z-1)*z_sep)*z_offset), Quaternion::Identity, new Box(Color3(.1, .7, .1)), Vec3(2, 1, 1), restitution, 0.5, 0.5));
				bVector.push_back(new Body(rot*(center + (6.5+18*iter + (i-2)*18)*y_offset - (3.2 + (k-1)*x_sep)*x_offset + (z-1)*z_sep*z_offset), Quaternion::Identity, new Box(Color3(.1, .7, .1)), Vec3(2, 1, 1), restitution, 0.5, 0.5));
				bVector.push_back(new Body(rot*(center + (3+18*iter + (i-2)*18)*y_offset - (2 + (k-1)*x_sep)*x_offset + (1.5 + (z-1)*z_sep)*z_offset), Quaternion(Vec3(0.0, 0.0, 1.0), PI/8.0), new Box(Color3(.1, .7, .1)), Vec3(1, 1, 1), restitution, 0.5, 1));
				bVector.push_back(new Body(rot*(center + (4.7+18*iter + (i-2)*18)*y_offset - (3.5 + (k-1)*x_sep)*x_offset + (z-1)*z_sep*z_offset), Quaternion(Vec3(0.0, 0.0, 1.0), PI/4.0), new Box(Color3(.1, .7, .1)), Vec3(1, 1, 1), restitution, 0.5, 1));
			}
		}
	}
}

static void init_big_pile()
{
	const double dist = 1.;
	const Vec3 center(0.0, 0.0, 0.0);
	const Vec3 x_offset(dist, 0.0, 0.0);
	const Vec3 y_offset(0.0, dist, 0.0);
	const Vec3 z_offset(0.0, 0.0,dist);

	// floor
	bVector.push_back(new Body(center - 50*y_offset, Quaternion::Identity, new Box(Color3(1.0, 1.0, .5)), Vec3(100, 100, 100), .3, 0.5, 0));

	bVector.push_back(new Body(center + 5*y_offset + 2.5*x_offset + z_offset, Quaternion(Vec3(0.0, 0.0, 1.0), PI/6.0), new Box(Color3(.1, .8, .7)), Vec3(1, 1, 1), .7, 0.5, 1));
	bVector.push_back(new Body(center + 4.5*y_offset + 2*x_offset - z_offset, Quaternion::Identity, new Box(Color3(.7, .0, .4)), Vec3(1, 1, 1), .7, 0.5, 1));
	bVector.push_back(new Body(center + 4.5*y_offset + 3.3*x_offset - .5*z_offset, Quaternion::Identity, new Box(Color3(1., .4, .1)), Vec3(1, 1, 1), .7, 0.5, 1));
	bVector.push_back(new Body(center + 8*y_offset + 2.5*x_offset + z_offset, Quaternion(Vec3(0.0, 0.0, 1.0), PI/6.0), new Box(Color3(.0, .4, .2)), Vec3(1, 1, 1), .7, 0.5, 1));
	bVector.push_back(new Body(center + 7*y_offset + 2*x_offset - z_offset, Quaternion(Vec3(0.0, 1.0, 1.0), PI/6.0), new Box(Color3(.0, .1, .7)), Vec3(1, 1, 1), .7, 0.5, 1));
	bVector.push_back(new Body(center + 7.5*y_offset + 3.3*x_offset - .5*z_offset, Quaternion::Identity, new Box(Color3(.3, .3, .3)), Vec3(1, 1, 1), .7, 0.5, 1));
	bVector.push_back(new Body(center + 3.5*y_offset + 1*x_offset, Quaternion::Identity, new Box(Color3(.1, .7, .1)), Vec3(2, 1, 3), .7, 0.5, 1./6.));
	bVector.push_back(new Body(center + 1.5*y_offset + 2*x_offset, Quaternion::Identity, new Box(Color3(.1, .7, .1)), Vec3(2, 2, 2), .7, 0.5, .125));
	bVector.push_back(new Body(center + 6*y_offset + 3*x_offset, Quaternion(Vec3(0.0, 0.0, 1.0), PI/2.5), new Box(Color3(.1, .7, .1)), Vec3(1, 2, 2), .7, 0.5, .25));
}

static void init_stack()
{
	const double dist = 1.;
	const Vec3 center(0.0, 0.0, 0.0);
	const Vec3 x_offset(dist, 0.0, 0.0);
	const Vec3 y_offset(0.0, dist, 0.0);
	const Vec3 z_offset(0.0, 0.0,dist);

	// floor
	bVector.push_back(new Body(center - 100*y_offset, Quaternion::Identity, new Box(Color3(1.0, 1.0, .5)), Vec3(200, 200, 200), .3, 0.5, 0));

	bVector.push_back(new Body(center + 9.5*y_offset + 2.5*x_offset + 2.5*z_offset, Quaternion::Identity, new Box(Color3(.1, .8, .7)), Vec3(1, 1, 1), .4, 0.5, 1));
	bVector.push_back(new Body(center + 10.7*y_offset + 2*x_offset + 1.0*z_offset, Quaternion::Identity, new Box(Color3(.7, .0, .4)), Vec3(1, 1, 1), .4, 0.5, 1));
	bVector.push_back(new Body(center + 9.5*y_offset + 2.3*x_offset + 1.0*z_offset, Quaternion::Identity, new Box(Color3(1., .4, .1)), Vec3(1, 1, 1), .4, 0.5, 1));
	bVector.push_back(new Body(center + 9.5*y_offset + 1.2*x_offset + 1.0*z_offset, Quaternion::Identity, new Box(Color3(.6, .4, .4)), Vec3(1, 1, 1), .4, 0.5, 1));
	bVector.push_back(new Body(center + 9.5*y_offset + 2.5*x_offset - z_offset, Quaternion::Identity, new Box(Color3(.0, .4, .2)), Vec3(1.5, 1.5, 1.5), .7, 0.5, 1.0/3.375));
	bVector.push_back(new Body(center + 50*y_offset + 2*x_offset - 4.5*z_offset, Quaternion::Identity, new Box(Color3(.3, .3, .3)), Vec3(2, 2, 2), .7, 0.5, .125));
	bVector.push_back(new Body(center + 8.5*y_offset + 2*x_offset - 1*z_offset, Quaternion::Identity, new Box(Color3(.1, .7, .1)), Vec3(4, .3, 10), .4, 0.5, 1./6.));
	bVector.push_back(new Body(center + 4.1*y_offset + 2*x_offset, Quaternion::Identity, new Box(Color3(.1, .7, .1)), Vec3(2, 8, 2), .4, 0.5, 1.0/32.0));
}

static void init_tall_stack()
{
	const double dist = 1.;
	const Vec3 center(0.0, 0.0, 0.0);
	const Vec3 x_offset(dist, 0.0, 0.0);
	const Vec3 y_offset(0.0, dist, 0.0);
	const Vec3 z_offset(0.0, 0.0,dist);
	
	double box_height = 1.0;

	// floor
	bVector.push_back(new Body(center - .5*y_offset, Quaternion::Identity, new Box(Color3(1.0, 1.0, .5)), Vec3(200, 1, 200), .3, 0.5, 0));

	for(int i = 0; i < 1; i++)
	{
		bVector.push_back(new Body(center + ((1.5 + 100000*EPSILON)*box_height + (box_height + 100000*EPSILON)*i)*y_offset + (i % 1)*.05*x_offset,  Quaternion(Vec3(0.0, 0.0, 1.0), PI/2.5), new Box(Color3((i % 5)/15.0 + 0.67, (i % 4)/12.0 + 0.67, (i % 2)/6.0 + 0.67)), Vec3(1, 1, 1), .4, 0.5, 1));
	}
}

static void init_system( int i )
{
	clicked = false;
	switch(i)
	{
		case 0:
			init_single_box();
			break;
		case 1:
			init_slide();
			break;
		case 2:
			init_small_pile();
			break;
		case 3:
			init_high_pile();
			break;
		case 4:
			init_big_pile();
			break;
		case 5:
			init_stack();
			break;
		case 6:
			init_combo();
			break;
		case 7:
			init_tall_stack();
			break;
		default:
			init_small_pile();
			break;
	}

	sys = new System(bVector);
	
	prev_pos = new double[sys->size_pos()];
	prev_vel = new double[sys->size_vel()];
	y_vel = new double[VEL_STATE_SIZE];
	memset(y_vel, 0, sizeof(double)*VEL_STATE_SIZE);
}

/*********************************************************************
* OpenGL specific drawing routines
**********************************************************************/

static void pre_display ( void )
{
	glViewport ( 0, 0, win_x, win_y );
	glClearColor ( 0.0f, 0.0f, 0.0f, 1.0f );
	glClear ( GL_COLOR_BUFFER_BIT  | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
	glMatrixMode ( GL_PROJECTION );
	glLoadIdentity ();
	gluPerspective(180.0 / 4.0, float(win_x)/float(win_y), 0.01, 1000.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(camera[0], camera[1], camera[2], target[0], target[1], target[2], 0, 1, 0);
}

static void post_display ( void )
{
	// Write frames if necessary.
	if (dump_frames) {
		const int FRAME_INTERVAL = 3;
		if ((frame_number % FRAME_INTERVAL) == 0) {
			const unsigned int w = glutGet(GLUT_WINDOW_WIDTH);
			const unsigned int h = glutGet(GLUT_WINDOW_HEIGHT);
			unsigned char * buffer = (unsigned char *) malloc(
				w * h * 4 * sizeof(unsigned char));
			if (!buffer)
				exit(-1);
			glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, buffer);
			char filename[13];
			sprintf(filename, "img%.5i.png", frame_number / FRAME_INTERVAL);
			printf("Dumped %s.\n", filename);
			saveImageRGBA(filename, buffer, w, h);
		}
		frame_number++;
	}

	glutSwapBuffers ();
}

static void remap_GUI()
{
	int ii, size = sys->num_bodies();
	for(ii=0; ii<size; ii++)
	{
		sys->bVector[ii]->reset();
	}
}

/*********************************************************************
* GLUT callback routines
**********************************************************************/

static void key_func ( unsigned char key, int x, int y )
{
	switch ( key )
	{
		case ' ':
		remap_GUI();
		break;

		case 'Q':
		case 'q':
		case 27:
		free_data ();
		exit ( 0 );
		break;
	}
}

static void mouse_func ( int button, int state, int x, int y )
{
	omx = mx = x;
	omy = my = y;

	if(!mouse_down[0]){hmx=x; hmy=y;}
	if(mouse_down[button]) mouse_release[button] = state == GLUT_UP;
	if(mouse_down[button]) mouse_shiftclick[button] = glutGetModifiers()==GLUT_ACTIVE_SHIFT;
	mouse_down[button] = state == GLUT_DOWN;
}

static void motion_func ( int x, int y )
{
	mx = x;
	my = y;

	//rotate view:
	if (mouse_down[0]) {
		Vec3 vec = camera - target;
		float len = norm(vec);
		float theta_yaw = atan2(vec[2], vec[0]);
		float theta_pitch = atan2(vec[1], sqrt(vec[0] * vec[0] + vec[2] * vec[2]));
		theta_yaw += (mx-omx) / double(win_x) / len * 40;
		theta_pitch += (my-omy) / double(win_y) / len * 40;
		if (theta_pitch > 0.4 * M_PI) theta_pitch = 0.4 * M_PI;
		if (theta_pitch <-0.4 * M_PI) theta_pitch =-0.4 * M_PI;

		camera = Vec3(cos(theta_yaw)*cos(theta_pitch),sin(theta_pitch),sin(theta_yaw)*cos(theta_pitch)) * len + target;
	}

	//pan view:
	if (mouse_down[1]) {
		Vec3 to = camera - target;
		unitize(to);
		Vec3 right = cross(to, Vec3(0,1,0));
		unitize(right);
		Vec3 up = -cross(to, right);
		float len = norm(camera - target);
		camera += right * (mx-omx) / double(win_x) * len;
		camera += up * (my-omy) / double(win_y) * len;
		target += right * (mx-omx) / double(win_x) * len;
		target += up * (my-omy) / double(win_y) * len;
	}

	//zoom view:
	if (mouse_down[2]) {
		Vec3 vec = camera - target;
		float len = norm(vec);
		len *= pow(2, (my-omy) / double(win_y) * 10);
		if (len < 1) len = 1;
		if (len > 1000) len = 1000;
		camera = ((camera - target) / norm(camera - target)) * len + target;
	}

	omx = mx;
	omy = my;
}

static void reshape_func ( int width, int height )
{
	glutSetWindow ( win_id );
	glutReshapeWindow ( width, height );

	win_x = width;
	win_y = height;
}

/***********************************************************************
* Build a contact graph in the system based on the current state.
************************************************************************/
void create_contact_graph()
{
	// clear contact graph
	for(int i = 0; i < sys->num_bodies(); ++i)
		sys->bVector[i]->in_contact_list.clear();

	Vec3 p, p1, p2, normal;
	// create contact graph
	for(int i = 0; i < sys->num_bodies(); ++i){
		// static objects should never be considered as resting on anything
		if(sys->bVector[i]->inv_mass != 0){
			// evolve each object along the y-axis while keeping the others stationary and test for intersection
			sys->get_state_pos(prev_pos + i*POS_STATE_SIZE, i);
			sys->get_state_vel(prev_vel + i*VEL_STATE_SIZE, i);

			// grab the y component of the velocity and keep only that.
			y_vel[1] = prev_vel[i*VEL_STATE_SIZE + 1];
			if(y_vel[1] > 0)
			{
				// Make sure that the object moves down or else there might
				// be an object above this one that will then count as being below it.
				y_vel[1] = -y_vel[1];
			}
			else
			{
				// Increase the current velocity by a factor of 3 to account for
				// any increase in velocity due to future contact resolutions.
				y_vel[1] *= 3;
			}
			sys->set_state_vel(y_vel, i);
			integrator->integrate_pos(*sys, dt, i);

			for(int k = 0; k < sys->num_bodies(); ++k){
				// add the contact to the bodies list if there is one
#if USE_XENOCOLLIDE
				if(k != i && Body::intersection_test(sys->bVector[k], sys->bVector[i], p1, p2, normal))
#else
				if(k != i && sys->bVector[k]->intersection_test(sys->bVector[i], p, normal))
#endif
				{
					sys->bVector[i]->in_contact_list.push_back(sys->bVector[k]);
				}
			}
			
			// Reset this body
			sys->set_state_pos(prev_pos + i*POS_STATE_SIZE, i);
			sys->set_state_vel(prev_vel + i*VEL_STATE_SIZE, i);
		}
	}

	// sort bodies based on the new contact graph
	sys->topological_tarjan();
}

#define PERFORMANCE 1
static void idle_func ( int value )
{
	// cap the fps
	glutTimerFunc (frame_time, idle_func, 0 );
	
	// calculate fps and dt and reset system if necessary
	int cur_time = glutGet(GLUT_ELAPSED_TIME);
	if(cur_time - prev_fps_taken_time > 3000)
	{
		printf("fps: %g\n", 1000.0*frame_number/(double) (cur_time - prev_fps_taken_time));
		prev_fps_taken_time = cur_time;

		if(reset_time > 0){
			if(cur_time - start_time > reset_time){
				start_time = cur_time;
				remap_GUI();
			}
		}

		frame_number = 0;
	}
	
	// randomly shuffle the body array to eliminate bias
	for(int ii = 0; ii < 15; ii++)
	{
		int jj = rand() % sys->num_bodies();
		int kk = rand() % sys->num_bodies();
		if(sys->bVector[jj]->inv_mass > 0 && sys->bVector[kk]->inv_mass > 0)
		{
			Body* temp = sys->bVector[jj];
			sys->bVector[jj] = sys->bVector[kk];
			sys->bVector[kk] = temp;
		}
	}
	// update the local copy
	sys->get_bodies(bVector);
	

	/***********************/
	/* collision detection */
	/***********************/

	// get x and v
	for(int i = 0; i < sys->num_bodies(); ++i){
		sys->get_state_pos(prev_pos + i*POS_STATE_SIZE, i);
		sys->get_state_vel(prev_vel + i*VEL_STATE_SIZE, i);
	}

	// set system to x' and v'
	sys->zero_forces();
	sys->add_gravity();
	for(int i = 0; i < sys->num_bodies(); ++i){
		integrator->integrate_vel(*sys, dt, i);
		integrator->integrate_pos(*sys, dt, i);
	}

	// find and resolve collisions
	int count;
	for(count = 0; count < MAX_COLLISIONS; count++){
		if(sys->collsion_detect(integrator, dt, prev_pos, prev_vel))
		{
			// set the system back to x and v where v has collision info
			for(int i = 0; i < sys->num_bodies(); ++i){
				sys->set_state_pos(prev_pos + i*POS_STATE_SIZE, i);
				sys->set_state_vel(prev_vel + i*VEL_STATE_SIZE, i);
			}
			// get new x' and v'
			sys->zero_forces();
			sys->add_gravity();
			for(int i = 0; i < sys->num_bodies(); ++i){
				integrator->integrate_vel(*sys, dt, i);
				integrator->integrate_pos(*sys, dt, i);
			}
		}
		else
		{
			break;
		}
	}
	
#if PERFORMANCE
	printf("collision iterations: %d\n", count);
#endif

	// set the system back to x and v where v has final collision info
	for(int i = 0; i < sys->num_bodies(); ++i){
		sys->set_state_pos(prev_pos + i*POS_STATE_SIZE, i);
		sys->set_state_vel(prev_vel + i*VEL_STATE_SIZE, i);
	}

	// update forces
	sys->zero_forces();
	sys->add_gravity();

	/*********************/
	/* contact detection */
	/*********************/

	// integrate velocity
	for(int i = 0; i < sys->num_bodies(); ++i){
		integrator->integrate_vel(*sys, dt, i);
	}

	create_contact_graph();
	
	// Save off current x
	for(int i = 0; i < sys->num_bodies(); ++i){
		sys->get_state_pos(prev_pos + i*POS_STATE_SIZE, i);
	}
	
	// Set state to x', v'
	for(int i = 0; i < sys->num_bodies(); ++i){
		integrator->integrate_pos(*sys, dt, i);
	}

	// resolve the contacts in the contact graph
	for(count = 0; count < MAX_CONTACTS + MAX_SHOCK_PROP; count++)
	{
		if(sys->contact_detect(integrator, dt, prev_pos, count, count >= MAX_CONTACTS))
		{
			// Set state back to x, v' now that it has the new v'.
			for(int i = 0; i < sys->num_bodies(); ++i){
				sys->set_state_pos(prev_pos + i*POS_STATE_SIZE, i);
			}

			// Set state to the new x', v' before testing for contacts again
			for(int i = 0; i < sys->num_bodies(); ++i){
				integrator->integrate_pos(*sys, dt, i);
			}
		}
		else
		{
			break;
		}
	}
	
#if PERFORMANCE
	printf("contact iterations: %d\n", count);
	printf("--------------------------------\n");
#endif

	frame_number++;

	glutSetWindow ( win_id );
	glutPostRedisplay ();
}

static void display_func ( void )
{	
	pre_display();
	
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);

	// draw bodies
	for(int ii = 0; ii < ((System *) sys)->num_bodies(); ++ii)
	{
		sys->bVector[ii]->draw();
	}
	
	post_display();
 	glutSwapBuffers();
}


/*
----------------------------------------------------------------------
open_glut_window --- open a glut compatible window and set callbacks
----------------------------------------------------------------------
*/

static void open_glut_window ( void )
{
    glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGB | 
         				 GLUT_DEPTH | GLUT_STENCIL );

    glutInitWindowPosition ( 0, 0 );
    glutInitWindowSize ( win_x, win_y );
    win_id = glutCreateWindow ( "Rigid Bodies!" );

    glutKeyboardFunc ( key_func );
    glutMouseFunc ( mouse_func );
    glutMotionFunc ( motion_func );
    glutReshapeFunc ( reshape_func );
    glutTimerFunc(frame_time, idle_func, 0);
	//glutIdleFunc(idle_func);
    glutDisplayFunc ( display_func );

    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_POLYGON_SMOOTH);

    // enable depth testing and lighting
    glEnable(GL_NORMALIZE);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
	glEnable(GL_CULL_FACE);
  	glEnable(GL_DEPTH_TEST);
}

/**********************************************************************
 * main --- main routine
 **********************************************************************/
int main ( int argc, char ** argv )
{
	glutInit ( &argc, argv );

	integrator = new EulerRBIntegrator();

	dt = 0.016f;
	dsim = 0;
	dump_frames = 0;
	frame_number = 0;

	if(argc > 1)
	{
		init_system(atoi(argv[1]));
	}
	else
	{
		init_system(-1);
	}

	win_x = 1440;
	win_y = 900;
	open_glut_window ();

	start_time = glutGet(GLUT_ELAPSED_TIME);
	prev_fps_taken_time = start_time;

	srand(time(NULL));

	glutMainLoop ();

	exit ( 0 );
}

