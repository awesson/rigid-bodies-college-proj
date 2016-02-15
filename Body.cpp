#include "Body.h"
#include <GLUT/glut.h>

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

#if USE_XENOCOLLIDE
/**
 * p1 and p2 are the positions of collision in world space on each body
 * and normal is normal of the collision also in world space.
 * Uses the Xeno collide algorithm to detect collisions.
 **/
bool Body::intersection_test(Body *body1, Body* body2, Vec3 &p1, Vec3 & p2, Vec3 &normal)
{
	Vec3 v0 = body2->Position - body1->Position; // Center of Minkowski difference
	double dist_between_centers = norm(v0);
	
	// check bounding sphere intersection
	if(dist_between_centers > body1->radius + body2->radius)
	{
		return false;
	}
	
	// Centers overlap
	if(IsZero(dist_between_centers))
	{
		// Pick an arbitrary direction
		v0 = Vec3(0, EPSILON, 0);
	}
	
	// Get the inverse rotations of both bodies so we can transform the world normals into body space
	Quaternion inv_orientation1, inv_orientation2;
	inv_orientation1 = conjugate(body1->Orientation);
	inv_orientation2 = conjugate(body2->Orientation);
	
	// Get the closest support point on the convex hull of the Minkowski difference
	normal = -v0;
	Vec3 v11 = body1->model->GetSupportPoint(inv_orientation1*(-normal));
	body1->TransformBodyToWorld(v11);
	Vec3 v12 = body2->model->GetSupportPoint(inv_orientation2*normal);
	body2->TransformBodyToWorld(v12);
	Vec3 v1 = v12 - v11;
	
    if (v1*normal <= 0.0)
    { // v0 is on the surface of convex hull of the minkowski difference
	  // and the origin in on the outside, in the direction of the normal.
        return false;
    }
	
	normal = cross(v1, v0);
	if(IsZero(norm(normal)))
	{ // v0, v1 and the origin are in a line and since v1 is away from the center,
	  // this means the origin is closer to the center than a support point is
	  // along a direct line, so the origin has to be inside the minkowski difference.
		normal = v1 - v0;
		unitize(normal);
		p1 = v11;
		p2 = v12;
		// printf("found intersection at p1:(%g, %g, %g), p2:(%g, %g, %g), normal:(%g, %g, %g)\n", p1[0], p1[1], p1[2],
		// 																					  p2[0], p2[1], p2[2],
		// 																					normal[0], normal[1], normal[2]);
		return true;
	}
	
	Vec3 v21 = body1->model->GetSupportPoint(inv_orientation1*(-normal));
	body1->TransformBodyToWorld(v21);
	Vec3 v22 = body2->model->GetSupportPoint(inv_orientation2*normal);
	body2->TransformBodyToWorld(v22);
	Vec3 v2 = v22 - v21;
	
	if(v2*normal <= 0.0)
	{ // v0 is on the surface of convex hull of the minkowski difference
	  // and the origin in on the outside, in the direction of the normal.
		return false;
	}
	
	normal = cross(v1- v0, v2 - v0);
	if(normal*v0 > -EPSILON)
	{ // Normal is pointing away from the origin. Need to swap the support
		// points so that the normal will point toward the origin.
		Vec3 temp = v1;
		v1 = v2;
		v2 = temp;
		temp = v11;
		v11 = v21;
		v21 = temp;
		temp = v12;
		v12 = v22;
		v22 = temp;
		normal = -normal;
	}
	
	// Find a portal
	while(true)
	{
		Vec3 v31 = body1->model->GetSupportPoint(inv_orientation1*(-normal));
		body1->TransformBodyToWorld(v31);
		Vec3 v32 = body2->model->GetSupportPoint(inv_orientation2*normal);
		body2->TransformBodyToWorld(v32);
		Vec3 v3 = v32 - v31;
		
		if(v3*normal <= 0.0)
		{ // v0 is on the surface of convex hull of the minkowski difference
		  // and the origin in on the outside, in the direction of the normal.
			return false;
		}
		
		// origin is outside (v1, v0, v3), eliminate v2 and loop
		if (cross(v1, v3)*v0 < 0.0)
		{
			v2 = v3;
			v21 = v31;
			v22 = v32;
			normal = cross((v1 - v0), (v3 - v0));
			continue;
		}

		// origin is outside (v3, v0, v2), eliminate v1 and loop
		if (cross(v3, v2)*v0 < 0.0)
		{
			v1 = v3;
			v11 = v31;
			v12 = v32;
			normal = cross((v3 - v0), (v2 - v0));
			continue;
		}

		// refine the portal.
		while (true)
		{
			normal = cross(v2 - v1, v3 - v1);
			unitize(normal);
			float dot = normal*v1;

			Vec3 v41 = body1->model->GetSupportPoint(inv_orientation1*(-normal));
			body1->TransformBodyToWorld(v41);
			Vec3 v42 = body2->model->GetSupportPoint(inv_orientation2*normal);
			body2->TransformBodyToWorld(v42);
			Vec3 v4 = v42 - v41;

			double delta = (v4 - v3)*normal;
			double separation = -(v4*normal);

			// modified to deal with nans, need to verify that my logic is still correct.
			if (!(delta > 1e-4) || !(separation < 0.0))
			{
				if (dot > -EPSILON)
				{
					double b0 = cross(v1, v2)*v3;
					double b1 = cross(v3, v2)*v0;
					double b2 = cross(v0, v1)*v3;
					double b3 = cross(v2, v1)*v0;

					double sum = b0 + b1 + b2 + b3;

					if (sum <= 0.0)
					{
						b0 = 0.0;
						b1 = cross(v2, v3)*normal;
						b2 = cross(v3, v1)*normal;
						b3 = cross(v1, v2)*normal;

						sum = b1 + b2 + b3;
					}

					double inv = (1.0 / sum);

					Vec3 wa = (b0*body1->Position + b1*v11 + b2*v21 + b3*v31) * inv;
					Vec3 wb = (b0*body2->Position + b1*v12 + b2*v22 + b3*v32) * inv;

					p1 = ((v41 - wa)*normal * normal) + wa;
					p2 = ((v42 - wb)*normal * normal) + wb;
					
					// printf("FOund intersection at p1:(%g, %g, %g), p2:(%g, %g, %g), normal:(%g, %g, %g)\n", p1[0], p1[1], p1[2],
					// 																					  p2[0], p2[1], p2[2],
					// 																					normal[0], normal[1], normal[2]);
					// printf("with b1 at postition (%g, %g, %g) and b2 at postition (%g, %g, %g)\n", body1->Position[0], body1->Position[1], body1->Position[2],
					// 	body2->Position[0], body2->Position[1], body2->Position[2]);
					unitize(normal);
					return true;
				}

				return false;
			}

			double d1 = cross(v4, v1)*v0;
			if (d1 < 0.0f)
			{
				double d2 = cross(v4, v2)*v0;
				if (d2 < 0.0f)
				{
					v1 = v4;
					v11 = v41;
					v12 = v42;
				}
				else
				{
					v3 = v4;
					v31 = v41;
					v32 = v42;
				}
			}
			else
			{
				double d3 = cross(v4, v3)*v0;
				if (d3 < 0.0f)
				{
					v2 = v4;
					v21 = v41;
					v22 = v42;
				}
				else
				{
					v1 = v4;
					v11 = v41;
					v12 = v42;
				}
			}
		}
	}
}
#else // USE_XENOCOLLIDE

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
		pos = w_pos;
        get_vertex_in_body_space(pos);
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
	Vec3 local_p = p;
    get_vertex_in_body_space(local_p);
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
#endif

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
Vec3 Body::get_vertex_world_position(int i) const
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

void Body::TransformBodyToWorld(Vec3 &body_pos) const
{
    // scale body_pos
    for(int k = 0; k < 3; ++k)
        body_pos[k] *= size[k];
    // rotate body_pos
    body_pos = Orientation*body_pos;
    // translate p
    body_pos += Position;
}

/**
* transform ith vertex normal into world coordinates
**/
Vec3 Body::get_vertex_world_normal(int i) const
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
void Body::get_vertex_in_body_space(Vec3 &world_pos) const
{
    // translate pos
    world_pos -= Position;
    // rotate pos
    world_pos = conjugate(Orientation)*world_pos;
    // scale pos
    for(int k = 0; k < 3; ++k)
        world_pos[k] /= (double) size[k];
    //printf("local pos: %f %f %f\n", world_pos[0], world_pos[1], world_pos[2]);
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
