// frontend.cpp : Defines the entry point for the console application.
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

/* macros */
#define MAX_COLLISIONS 5
#define MAX_CONTACTS 100
#define rot_ang PI/6.0
#define MAX_LEN 100

/* global variables */
static int dump_frames;
static int frame_number;

// static Rigid Body list bList;
static std::vector<BodyInfo> bVector;

static int win_id;
static int win_x, win_y;
static int mouse_down[3];
static int mouse_release[3];
static int mouse_shiftclick[3];
static int omx, omy, mx, my;
static int hmx, hmy;

// global cube model used for rendering
Mesh* mesh;

// camera data
Vec3 camera(0.0, 10.0, -20.0);
Vec3 target(0.0, 0.0, 0.0);

// network data
char hostname[MAX_LEN];
int port, prev_time, start_time, num_bodies;

/*
----------------------------------------------------------------------
free/clear/allocate simulation data
----------------------------------------------------------------------
*/

static void free_data ( void )
{
  bVector.clear();
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
    gluPerspective(180.0 / 4.0, (float) win_x/(float) win_y, 0.01, 10000.0);
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
            // glRasterPos2i(0, 0);
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
    if(frame_number == 100){
        int cur_time = glutGet(GLUT_ELAPSED_TIME);
        printf("fps: %g\n", 1000*100.0/(double) (cur_time - prev_time));
        prev_time = cur_time;
        frame_number = 0;
    }

    frame_number++;

    for(int i = 0; i < num_bodies; ++i){
        glMatrixMode(GL_MODELVIEW);

        // model transformations
        glPushMatrix();
        glTranslated(bVector[i].Pos[0], bVector[i].Pos[1], bVector[i].Pos[2]);
        Vec3 axis;
        double angle;
        bVector[i].Orientation.to_axis_angle(&axis, &angle);
        glRotated(angle*180/PI, axis[0], axis[1], axis[2]);
        glScaled(bVector[i].size[0], bVector[i].size[1], bVector[i].size[2]);

        // set color
        float arr[4];
        bVector[i].color.to_array(arr);
        glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE,   arr );

        mesh->render();
        glPopMatrix();
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
    case 'q':
    case 'Q':
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
        theta_yaw += (mx-omx) / double(win_x) / len * 100;
        theta_pitch += (my-omy) / double(win_y) / len * 100;
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

static void idle_func ( int value )
{
	// limit fps
    glutTimerFunc(frame_time, idle_func, 0);

    glutSetWindow ( win_id );
    glutPostRedisplay ();
}

static void display_func ( void )
{
    pre_display ();
    draw_bodies();
    post_display ();
}


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

    glClearColor ( 0.0f, 0.0f, 0.0f, 1.0f );
    glClear ( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glutSwapBuffers ();
    glClear ( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glutSwapBuffers ();

    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_POLYGON_SMOOTH);

        // enable depth testing and lighting
    glEnable(GL_NORMALIZE);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);
    GLfloat light_position[] = { 1.0, 4.5, 10.0, 0.0 };
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);

    pre_display ();

    glutKeyboardFunc ( key_func );
    glutMouseFunc ( mouse_func );
    glutMotionFunc ( motion_func );
    glutReshapeFunc ( reshape_func );
    glutTimerFunc(frame_time, idle_func, 0);
    glutDisplayFunc ( display_func );
}

/*************************************************
listens to the backend and updates the bodies list
**************************************************/
void *reader_thread(void * ptr){
    int serverfd;
    rio_t rio_backend;

    if(pthread_detach(pthread_self()) != 0){
      // pthread detach failed
      return NULL;
    }
    
    /* open connection to backend */
    if ((serverfd = open_clientfd(hostname, port)) < 0) {
        printf("error opening connection with backend\n");
        exit(0);
    }
    
    rio_readinitb(&rio_backend, serverfd);
    
    // read once to get the number of bodies
    if(rio_readnb(&rio_backend, &num_bodies, sizeof(int)) < 0){
        printf("error reading data from backend\n");
        close(serverfd);
        exit(0);
    }
    
    bVector.resize(num_bodies);
    
    while(1){
        if(rio_readnb(&rio_backend, (&(bVector[0])), sizeof(BodyInfo)*num_bodies) < 0){
            printf("error reading data from backend\n");
            close(serverfd);
            exit(0);
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

    if ( argc < 3 ) {
        printf("usage: %s [hostname] [port]\n", argv[0]);
        exit(0);
    } else{
        strncpy(hostname, argv[1], MAX_LEN);
        port = atoi(argv[2]);
    }

    printf ( "\n\nHow to use this application:\n\n" );
    printf ( "\t Toggle construction/simulation display with the spacebar key\n" );
    printf ( "\t Dump frames by pressing the 'd' key\n" );
    printf ( "\t Quit by pressing the 'q' key\n" );
    
    mesh = new BoxMesh();
    
    win_x = 1480;
    win_y = 920;
    open_glut_window ();

    start_time = glutGet(GLUT_ELAPSED_TIME);
    prev_time = start_time;

    pthread_t tid;
    pthread_create(&tid, NULL, reader_thread, NULL);

    glutMainLoop ();

    exit ( 0 );
}

