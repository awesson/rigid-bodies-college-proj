#include "Body.h"
#include <GLUT/glut.h>

#define EPSILON 1e-6

Body::Body(const Vec3 & i_ConstructPos, const Quaternion & i_ConstructOrien,
           Model* i_model, Vec3 i_size, const double i_restitution,
           const double i_coef_friction, const double i_inv_mass) :
           ConstructPos(i_ConstructPos), ConstructOrien(i_ConstructOrien),
           Position(i_ConstructPos), Orientation(i_ConstructOrien),
           Velocity(Vec3(0.0, 0.0, 0.0)), Momentum(Vec3(0.0, 0.0, 0.0)),
           Omega(Vec3(0.0, 0.0, 0.0)), AngularMomentum(Vec3(0.0, 0.0, 0.0)),
           forces(Vec3(0.0, 0.0, 0.0)), torques(Vec3(0.0, 0.0, 0.0)),
           model(i_model), size(i_size), radius(norm(size)), inv_mass(i_inv_mass),
           construct_inv_mass(i_inv_mass), restitution(i_restitution),
           coef_friction(i_coef_friction), index(-1), lowlink(-1), in_stack(false)
{
    // calculate derived quantities
    Orientation.to_matrix(&R);
    transpose(&R_t, R);
    model->get_Iinv(Iinv_body, size, inv_mass);
    Iinv = R*Iinv_body*R_t;
}

Body::~Body(void)
{
    delete model;
}

void Body::reset()
{
    Position = ConstructPos;
    Orientation = ConstructOrien;
    Velocity = Vec3(0.0, 0.0, 0.0);
    Momentum = Vec3(0.0, 0.0, 0.0);
    Omega = Vec3(0.0, 0.0, 0.0);
    AngularMomentum = Vec3(0.0, 0.0, 0.0);
    forces = Vec3(0.0, 0.0, 0.0);
    torques = Vec3(0.0, 0.0, 0.0);
    Orientation.to_matrix(&R);
    transpose(&R_t, R);
    model->get_Iinv(Iinv_body, size, inv_mass);
    Iinv = R*Iinv_body*R_t;
}

void Body::draw()
{
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glTranslated(Position[0], Position[1], Position[2]);
    Vec3 axis;
    double angle;
    Orientation.to_axis_angle(&axis, &angle);
    glRotated(angle*180/PI, axis[0], axis[1], axis[2]);
    glScaled(size[0], size[1], size[2]);
    model->render();
    glPopMatrix();
}

/**
 * p is position of collision in world space
 * and normal is in world coordinates.
 * Calls the models respective intersection test
 * to easily handle special cases such as box-box or box-sphere
 **/
// bool Body::intersection_test(Body *body_o, Vec3 &p, Vec3 &normal)
// {
// 	model->intersection_test(body_o->model, normal);
// }

/**
* p is position of collision in world space
* and normal is in world coordinates and points away from this body.
* Will take the deepest non-separating point as the interference point.
**/
bool Body::intersection_test(Body *body_o, Vec3 &p, Vec3 &normal)
{
    // check bounding sphere intersection
    if(norm(Position - body_o->Position) > radius + body_o->radius)
        return false;

    int num_pen_verts = 0;
    Vec3 inter_point(0,0,0);
    normal = Vec3(0,0,0);
    for(int i = 0; i < body_o->model->num_vertices(); ++i){
        Vec3 pos, w_pos, n;
        w_pos = body_o->get_vertex_world_position(i);
        pos = get_vertex_in_body_space(w_pos);
        if(model->intersection_test(pos, n)){
            Vec3 r1 = w_pos - Position;
            Vec3 r2 = w_pos - body_o->Position;
            Vec3 u_rel = body_o->get_vel(r2) - get_vel(r1);
            // rotate normal to world space
            n = Orientation*n;
            unitize(n);
            if(u_rel*n < 0.0){ // point is non-separating
                inter_point += w_pos;
                num_pen_verts++;
                //normal += n;
            }
        }
    }
    p = inter_point / num_pen_verts;
    //normal /= num_pen_verts;
    // find the closest normal to the average point
    Vec3 local_p = get_vertex_in_body_space(p);
    double abs_x = fabs(local_p[0]);
    double abs_y = fabs(local_p[1]);
    double abs_z = fabs(local_p[2]);
    if(abs_x < abs_y){
        if(abs_y < abs_z){ // closest to z-face
            normal = Vec3(0,0,local_p[2]/abs_z);
        } else{            // closest to y-face
            normal = Vec3(0,local_p[1]/abs_y,0);
        }
    } else{
        if(abs_x < abs_z){ // closest to z-face
            normal = Vec3(0,0,local_p[2]/abs_z);
        } else{            // closest to x-face
            normal = Vec3(local_p[0]/abs_x,0,0);
        }
    }
    normal = Orientation*normal;
    unitize(normal);
    return num_pen_verts != 0;
}

void Body::getInfo(BodyInfo &b){
    b.Pos = Position;
    b.Orientation = Orientation;
    b.size = size;
    b.color = model->material->diffuse;
}

/**
* calculates K as defined in Rigid Bodies Paper
**/
Matrix3 Body::get_K(Vec3 world_rel_pos)
{
    Matrix3 r_star, r_star_t;
    r_star = star(world_rel_pos);
    transpose(&r_star_t, r_star);
    return Matrix3::Identity*inv_mass + r_star_t * Iinv * r_star;
}

/**
* returns the world velocity of v + w x r
**/
Vec3 Body::get_vel(Vec3 world_rel_pos)
{
    return Velocity + cross(Omega, world_rel_pos);
}

/**
* transform ith vertex position into world coordinates
**/
Vec3 Body::get_vertex_world_position(int i)
{
    Vec3 p = model->mesh->get_vertex(i).position;
    // scale p
    for(int k = 0; k < 3; ++k)
        p[k] *= size[k];
    // rotate p
    p = Orientation*p;
    // translate p
    p += Position;
    return p;
}

/**
* transform ith vertex normal into world coordinates
**/
Vec3 Body::get_vertex_world_normal(int i)
{
    Vec3 n = model->mesh->get_vertex(i).normal;
    // scale
    for(int k = 0; k < 3; ++k)
        n[k] /= (double) size[k];
    unitize(n);
    // rotate n
    return Orientation*n;
}

/**
* transform world position into body coordinates
**/
Vec3 Body::get_vertex_in_body_space(Vec3 world_pos)
{
    // translate pos
    world_pos -= Position;
    // rotate p
    world_pos = conjugate(Orientation)*world_pos;
    // scale p
    for(int k = 0; k < 3; ++k)
        world_pos[k] /= (double) size[k];
    //printf("local pos: %f %f %f\n", world_pos[0], world_pos[1], world_pos[2]);
    return world_pos;
}

/**
 * computes a matrix equivalent of v cross
 **/
Matrix3 Body::star(Vec3 v)
{
    return Matrix3( 0.0, -v[2],  v[1],
                    v[2],   0.0, -v[0],
                    -v[1],  v[0],  0.0);
}
