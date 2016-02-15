// backend.cpp : Defines the entry point for the console application.
//

#include "Body.h"
#include "imageio.h"
#include "System.h"
#include "integrator.h"
#include "Box.h"
#include "csapp.h"
#include "fps.h"

#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <GLUT/glut.h>
#include <time.h>

/* macros */
#define MAX_COLLISIONS 5
#define MAX_CONTACTS 10
#define MAX_SHOCK_PROP 100
#define rot_ang PI/6.0

/* global variables */

static float dt;
static int dsim;
static int dump_frames;
static int frame_number;

// static Rigid Body list *bList;
static std::vector<Body*> bVector;
static std::vector<BodyInfo> bodyInfoList;

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
Vec3 camera(0.0, 10.0, -20.0);
Vec3 target(0.0, 0.0, 0.0);

// networking data
int port, prev_time, start_time, reset_time;

/*
----------------------------------------------------------------------
free/clear/allocate simulation data
----------------------------------------------------------------------
*/

static void free_data ( void )
{
    bodyInfoList.clear();
    delete sys;
    bVector.clear();
    delete integrator;
}

static void clear_data ( void )
{
    int ii; 
    for(ii=0; ii < sys->num_bodies(); ii++){
        bVector[ii]->reset();
    }
}

/*************************************************
 * initialization functions to set up different scenes
 *************************************************/
static void init_slide()
{
    const double dist = 1.;
    const Vec3 center(0.0, -10.0, 0.0);
    const Vec3 x_offset(dist, 0.0, 0.0);
    const Vec3 y_offset(0.0, dist, 0.0);
    const Vec3 z_offset(0.0, 0.0, dist);

    // floor
    bVector.push_back(new Body(center, Quaternion(Vec3(0.0, 0.0, 1.0), rot_ang), new Box(Color3(1.0, 1.0, .5)), Vec3(20, 20, 20), 1.0, .7, 0.0));
    
    bVector.push_back(new Body(center + (10*(sin(rot_ang) + cos(rot_ang)) + .5*(cos(rot_ang) - sin(rot_ang)) + 10000000*EPSILON)*y_offset + (10*(cos(rot_ang) - sin(rot_ang)) - .5*(sin(rot_ang) + cos(rot_ang)) + 10000000*EPSILON)*x_offset, Quaternion(Vec3(0.0, 0.0, 1.0), rot_ang), new Box(Color3(.1, .7, .1)), Vec3(1, 1, 1), 1.0, 1.0, 1.0));
}

static void init_combo()
{
    const double dist = 1.;
    const Vec3 center(5.0, 0.0, 0.0);
    const Vec3 x_offset(dist, 0.0, 0.0);
    const Vec3 y_offset(0.0, dist + 100*EPSILON, 0.0);
    const Vec3 z_offset(0.0, 0.0, dist);

    // floor
    bVector.push_back(new Body(center - (3 + 5.0*sqrt(2) - 14.75/sqrt(2))*y_offset + (3 - 4.75/sqrt(2))*x_offset,  Quaternion(Vec3(0.0, 0.0, 1.0), PI/4.0), new Box(Color3(.7, 0.0, .0)), Vec3(10, .5, 10), .4, .5, 0.0));
    bVector.push_back(new Body(center - (3 + 5.0*sqrt(2) - 14.75/sqrt(2))*y_offset - (10 + 3.25/sqrt(2))*x_offset,  Quaternion(Vec3(0.0, 0.0, 1.0), -PI/4.0), new Box(Color3(0.0, 0.2, .7)), Vec3(10, .5, 10), .4, .5, 0.0));
    bVector.push_back(new Body(center - 60*y_offset, Quaternion::Identity, new Box(Color3(1.0, 1.0, .5)), Vec3(100, 100, 100), .4, 0.5, 0.0));
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
}

static void init_single_box()
{
    const double dist = 1.;
    const Vec3 center(0.0, 0.0, 15.0);
    const Vec3 x_offset(dist, 0.0, 0.0);
    const Vec3 y_offset(0.0, dist, 0.0);
    const Vec3 z_offset(0.0, 0.0, dist);

    // floor
    bVector.push_back(new Body(center - 5*x_offset, Quaternion::Identity, new Box(Color3(1.0, 1.0, .5)), Vec3(10, 10, 10), 0.5, 0.5, 0));

    bVector.push_back(new Body(center + 4*x_offset, Quaternion::Identity, new Box(Color3(.1, .7, .1)), Vec3(1, 1, 1), 1.0, 0.5, 1));
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
    const Vec3 center(0.0, 0.0, 0.0);
    const Vec3 x_offset(dist, 0.0, 0.0);
    const Vec3 y_offset(0.0, dist, 0.0);
    const Vec3 z_offset(0.0, 0.0, dist);

    // floor
    bVector.push_back(new Body(center - 500*y_offset, Quaternion::Identity, new Box(Color3(1.0, 1.0, .5)), Vec3(1000, 1000, 1000), .6, 0.5, 0));
	
	int iter=3; // 217 total objects
    for(int i = 0; i < iter; i++){
        for(int k = 0; k < iter; k++){
            for(int z = 0; z < iter; z++){
                bVector.push_back(new Body(center + (3+18*iter + (i-2)*18)*y_offset - (4 + (k-2)*7.5)*x_offset + (0.5 + (z-2)*15)*z_offset, Quaternion::Identity, new Box(Color3(.1, .7, .1)), Vec3(2, 1, 1), 1.0, 0.5, 0.5));
                bVector.push_back(new Body(center + (5+18*iter + (i-2)*18)*y_offset - (1.2 + (k-2)*7.5)*x_offset + (z-2)*15*z_offset, Quaternion::Identity, new Box(Color3(.1, .7, .1)), Vec3(2, 1, 1), 1.0, 0.5, 0.5));
                bVector.push_back(new Body(center + (3+18*iter + (i-2)*18)*y_offset - (k-2)*7.5*x_offset + (0.5 + (z-2)*15)*z_offset, Quaternion(Vec3(0.0, 0.0, 1.0), PI/8.0), new Box(Color3(.1, .7, .1)), Vec3(1, 1, 1), 1.0, 0.5, 1));
                bVector.push_back(new Body(center + (1.7+18*iter + (i-2)*18)*y_offset - (1.5 + (k-2)*7.5)*x_offset + (z-2)*15*z_offset, Quaternion(Vec3(0.0, 0.0, 1.0), PI/4.0), new Box(Color3(.1, .7, .1)), Vec3(1, 1, 1), 1.0, 0.5, 1.f));
                bVector.push_back(new Body(center + (2+18*iter + (i-2)*18)*y_offset - (5 + (k-2)*7.5)*x_offset + (2.5 + (z-2)*15)*z_offset, Quaternion::Identity, new Box(Color3(.1, .7, .1)), Vec3(2, 1, 1), 1.0, 0.5, 0.5));
                bVector.push_back(new Body(center + (6.5+18*iter + (i-2)*18)*y_offset - (3.2 + (k-2)*7.5)*x_offset + (z-2)*15*z_offset, Quaternion::Identity, new Box(Color3(.1, .7, .1)), Vec3(2, 1, 1), 1.0, 0.5, 0.5));
                bVector.push_back(new Body(center + (3+18*iter + (i-2)*18)*y_offset - (2 + (k-2)*7.5)*x_offset + (1.5 + (z-2)*15)*z_offset, Quaternion(Vec3(0.0, 0.0, 1.0), PI/8.0), new Box(Color3(.1, .7, .1)), Vec3(1, 1, 1), 1.0, 0.5, 1));
                bVector.push_back(new Body(center + (4.7+18*iter + (i-2)*18)*y_offset - (3.5 + (k-2)*7.5)*x_offset + (z-2)*15*z_offset, Quaternion(Vec3(0.0, 0.0, 1.0), PI/4.0), new Box(Color3(.1, .7, .1)), Vec3(1, 1, 1), 1.0, 0.5, 1));
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
    bVector.push_back(new Body(center - 50*y_offset, Quaternion::Identity, new Box(Color3(1.0, 1.0, .5)), Vec3(100, 100, 100), .3, 0.5, 0));

    bVector.push_back(new Body(center + 9.5*y_offset + 2.5*x_offset + 2.5*z_offset, Quaternion::Identity, new Box(Color3(.1, .8, .7)), Vec3(1, 1, 1), .4, 0.5, 1));
    bVector.push_back(new Body(center + 10.7*y_offset + 2*x_offset + 1.0*z_offset, Quaternion::Identity, new Box(Color3(.7, .0, .4)), Vec3(1, 1, 1), .4, 0.5, 1));
    bVector.push_back(new Body(center + 9.5*y_offset + 2.3*x_offset + 1.0*z_offset, Quaternion::Identity, new Box(Color3(1., .4, .1)), Vec3(1, 1, 1), .4, 0.5, 1));
	bVector.push_back(new Body(center + 9.5*y_offset + 1.2*x_offset + 1.0*z_offset, Quaternion::Identity, new Box(Color3(.6, .4, .4)), Vec3(1, 1, 1), .4, 0.5, 1));
    bVector.push_back(new Body(center + 9.5*y_offset + 2.5*x_offset - z_offset, Quaternion::Identity, new Box(Color3(.0, .4, .2)), Vec3(1.5, 1.5, 1.5), .7, 0.5, 1.0/3.375));
    bVector.push_back(new Body(center + 50*y_offset + 2*x_offset - 4.5*z_offset, Quaternion::Identity, new Box(Color3(.3, .3, .3)), Vec3(2, 2, 2), .7, 0.5, .125));
    bVector.push_back(new Body(center + 8.5*y_offset + 2*x_offset - 1*z_offset, Quaternion::Identity, new Box(Color3(.1, .7, .1)), Vec3(4, .3, 10), .4, 0.5, 1./6.));
    bVector.push_back(new Body(center + 4.1*y_offset + 2*x_offset, Quaternion::Identity, new Box(Color3(.1, .7, .1)), Vec3(2, 8, 2), .4, 0.5, 1.0/32.0));
}

static void init_system( void )
{
    clicked = false;
    //init_slide();
    //init_combo();
    //init_high_pile();
    //init_small_pile();
    //init_big_pile();
    init_stack();
    sys = new System(bVector);

    //allocate memory for the body list sent to clients
    for(int i = 0; i < sys->num_bodies(); ++i){
        BodyInfo *b = new BodyInfo();
        bodyInfoList.push_back(*b);
    }
}

/*
----------------------------------------------------------------------
OpenGL specific drawing routines
----------------------------------------------------------------------
*/

static void pre_display ( void )
{
    glViewport ( 0, 0, win_x, win_y );
    glClearColor ( 0.0f, 0.0f, 0.0f, 1.0f );
    glClear ( GL_COLOR_BUFFER_BIT  | GL_DEPTH_BUFFER_BIT);
    glMatrixMode ( GL_PROJECTION );
    glLoadIdentity ();
    gluPerspective(180.0 / 4.0, 1, 0.01, 10000.0);
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

static void draw_bodies ( void )
{
    for(int i = 0; i < sys->num_bodies(); ++i)
        bVector[i]->draw();
}

static void remap_GUI()
{
    int ii, size = sys->num_bodies();
    for(ii=0; ii<size; ii++)
    {
      bVector[ii]->reset();
    }
}

/*
----------------------------------------------------------------------
GLUT callback routines
----------------------------------------------------------------------
*/

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

/**
 * Build a contact graph in the system based on the current state.
 **/
void create_contact_graph(double *prev_pos, double *prev_vel, bool is_initial){
    // clear contact graph
    for(int i = 0; i < sys->num_bodies(); ++i){
        bVector[i]->in_contact_list.clear();
    }

    Vec3 p, normal;
    ContactInfo c;
    // create contact graph
    for(int i = 0; i < sys->num_bodies(); ++i){
        // hack to make sure static objects are never considered resting on anything
        if(bVector[i]->inv_mass != 0){
            // evolve each object keeping the others stationary and test for intersection
            sys->get_state_pos(prev_pos + i*POS_STATE_SIZE, i);
            sys->get_state_vel(prev_vel + i*VEL_STATE_SIZE, i);

			if(is_initial) // if this is the first time then use gravity
				integrator->integrate_vel(*sys, dt, i);
            integrator->integrate_pos(*sys, dt, i);

            for(int k = 0; k < sys->num_bodies(); ++k){
                // add the contact to the bodies list if there is one
                if(k != i && bVector[k]->intersection_test(bVector[i], p, normal)){
                    c.b = bVector[k];
                    c.p = p;
                    c.normal = normal;
                    bVector[i]->in_contact_list.push_back(c);
                }
            }
            sys->set_state_pos(prev_pos + i*POS_STATE_SIZE, i);
            sys->set_state_vel(prev_vel + i*VEL_STATE_SIZE, i);
        }
    }

    // sort bodies based on the new contact graph
    sys->topological_tarjan();
    // update the local copy
    sys->get_bodies(bVector);
}

static void idle_func ( int value )
{ 
	// cap the fps
	glutTimerFunc (frame_time, idle_func, 0 );
	
    double prev_pos[sys->size_pos()];
    double prev_vel[sys->size_vel()];

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
	int count = 0;
	while(sys->collsion_detect(prev_pos, prev_vel) && count < MAX_COLLISIONS){
		count++;
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

	// create the initial contact graph
	create_contact_graph(prev_pos, prev_vel, true);

    // integrate velocity
    for(int i = 0; i < sys->num_bodies(); ++i){
        integrator->integrate_vel(*sys, dt, i);
    }
	
	// resolve the contacts in the contact graph
    for(count = 0; sys->contact_detect(count, false) && count < MAX_CONTACTS; count++){
		// update the contact graph using the new velocities
		create_contact_graph(prev_pos, prev_vel, false);
    }

	// update the contact graph using the new velocities
	create_contact_graph(prev_pos, prev_vel, false);

	if(count == MAX_CONTACTS){
		// shock propagation
		for(;sys->contact_detect(count, true) && count < MAX_SHOCK_PROP; count++){
			// update the contact graph using the new velocities
			create_contact_graph(prev_pos, prev_vel, false);
		}
	}

    // update position
    for(int i = 0; i < sys->num_bodies(); ++i){
        integrator->integrate_pos( *sys, dt, i );
    }

    // calculate fps and reset system is necessary
    if(frame_number == 100){
        int cur_time = glutGet(GLUT_ELAPSED_TIME);
        printf("fps: %g\n", 1000*100.0/(double) (cur_time - prev_time));
        prev_time = cur_time;

        if(reset_time > 0){
            if(cur_time - start_time > reset_time){
                start_time = prev_time;
                remap_GUI();
            }
        }

        frame_number = 0;
    }

	// update the data we are sending to clients
    sys->saveOutputData(bodyInfoList);

    frame_number++;

    glutSetWindow ( win_id );
    glutPostRedisplay ();
}

// don't display anything on the backend
static void display_func ( void ){ }


/*
----------------------------------------------------------------------
open_glut_window --- open a glut compatible window and set callbacks
----------------------------------------------------------------------
*/

static void open_glut_window ( void )
{
    glutInitDisplayMode ( GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH );

    glutInitWindowPosition ( 0, 0 );
    glutInitWindowSize ( win_x, win_y );
    win_id = glutCreateWindow ( "Rigid Bodies!" );

    glutKeyboardFunc ( key_func );
    glutMouseFunc ( mouse_func );
    glutMotionFunc ( motion_func );
    glutReshapeFunc ( reshape_func );
    glutTimerFunc (frame_time, idle_func, 0 );
    glutDisplayFunc ( display_func );
}

/* each thread created for a frontend client runs this function. It continuously sends
   the information to the client about the body positions */
void *frontend_conn(void * ptr){
    int clientfd = *((int *) ptr);

    if(pthread_detach(pthread_self()) != 0){
      // pthread detach failed
      return NULL;
    }

    free(ptr);

    int num_bodies = sys->num_bodies();
    // send the number of bodies to the client
    if(rio_writen(clientfd, &num_bodies, sizeof(int)) < 0){
        printf("error writing to client, closing connection...\n");
        Close(clientfd);
        pthread_exit(NULL);
    }

    while(1){
      if(rio_writen(clientfd, &bodyInfoList[0], sizeof(BodyInfo)*num_bodies) < 0){
            printf("error writing to client, closing connection...\n");
            Close(clientfd);
            pthread_exit(NULL);
        }
    }
}

/*********************************************
sends body information to the frontend clients
***********************************************/
void *sender_thread(void * ptr){
    int listenfd, *clientfdp;
    unsigned int clientlen;
    struct sockaddr_in clientaddr;
    pthread_t tid;

    if((listenfd = open_listenfd(port)) < 0){
        // error opening port socket so don't start main loop
        printf("error opening port %d\n", port);
        exit(0);
    }

    while (1) {
        clientlen = sizeof(clientaddr);
        if((clientfdp = (int *) malloc(sizeof(int)))){ // only continue if malloc successful
            // and only continue if accept is successful
            if((*clientfdp = accept(listenfd, (SA *)&clientaddr, &clientlen)) >= 0){
                printf("opened connection to new frontend client\n");
                pthread_create(&tid, NULL, frontend_conn, clientfdp);
            }
        }
    }
}

/*
----------------------------------------------------------------------
main --- main routine
----------------------------------------------------------------------
*/

int main ( int argc, char ** argv )
{
    glutInit ( &argc, argv );

    integrator = new EulerRBIntegrator();

    if (argc < 2) {
        fprintf(stderr, "usage: %s <port> [loop time]\n", argv[0]);
        exit(0);
    }
    port = atoi(argv[1]);

    if(argc == 3)
        reset_time = 1000*atoi(argv[2]);
    else
        reset_time = -1;

    dt = 0.005f;
    dsim = 0;
    dump_frames = 0;
    frame_number = 0;
    
    init_system();
    
    win_x = 900;
    win_y = 900;
    open_glut_window ();

    if(signal(SIGPIPE, SIG_IGN) == SIG_ERR){
    	// signal failed to replace response of SIGPIPE so don't start main loop
        exit(1);
    }

    start_time = glutGet(GLUT_ELAPSED_TIME);
    prev_time = start_time;

    // create thread to listen for incoming connections
    pthread_t tid;
    pthread_create(&tid, NULL, sender_thread, NULL);

    glutMainLoop ();

    exit ( 0 );
}

