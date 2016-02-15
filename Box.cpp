
/**
 * @file model.cpp
 * @brief Model class
 *
 * @author Eric Butler (edbutler)
 * @author Zeyang Li (zeyangl)
 */

#include "Box.h"
#include <iostream>
#include <cstring>
#include <string>
#include <fstream>
#include <sstream>

Box::Box(Color3 color)
{
    mesh = new BoxMesh();
    material = new Material();
    material->ambient = Color3(1.0, 1.0, 1.0);
    material->diffuse = color;
    material->specular = Color3::White;
}

Box::~Box() { delete mesh; delete material; }

void Box::render() const
{
    if ( !mesh )
        return;
    if ( material )
        material->set_gl_state();
    glutSolidCube(1.0);
    if ( material )
        material->reset_gl_state();
}

void Box::get_Iinv(Matrix3& Iinv, Vec3 size, double inv_mass)
{
    Vec3 c1, c2, c3;
    c1 = Vec3(12.0*inv_mass / (size[1]*size[1] + size[2]*size[2]), 0.0, 0.0);
    c2 = Vec3(0.0, 12.0*inv_mass / (size[0]*size[0] + size[2]*size[2]), 0.0);
    c3 = Vec3(0.0, 0.0, 12.0*inv_mass / (size[0]*size[0] + size[1]*size[1]));
    Iinv = Matrix3(c1, c2, c3);
}

int Box::num_vertices() const
{
    return mesh->num_vertices();
}

inline float sign(float f)
{
	return (f > 0.0) ? 1.f : -1.f;
}

#if USE_XENOCOLLIDE
Vec3 Box::GetSupportPoint(const Vec3& local_normal) const
{
	if(IsZero(local_normal[0]))
	{
		if(IsZero(local_normal[1]))
		{
			// Front or back face center
			return Vec3(0.0, 0.0, local_normal[2] < 0 ? -0.5 : 0.5);
		}
		else if(IsZero(local_normal[2]))
		{
			// Top or bottom face center
			return Vec3(0.0, local_normal[1] < 0 ? -0.5 : 0.5, 0.0);
		}
		
		// Edge center
		return Vec3(0.0,
					(local_normal[1] < 0.0 ? -0.5 : 0.5),
					(local_normal[2] < 0.0 ? -0.5 : 0.5));
	}
	else if(IsZero(local_normal[1]))
	{
		if(IsZero(local_normal[2]))
		{
			// Left or right face center
			return Vec3(local_normal[0] < 0 ? -0.5 : 0.5, 0.0, 0.0);
		}
		
		// Edge center
		return Vec3((local_normal[0] < 0.0 ? -0.5 : 0.5),
					0.0,
					(local_normal[2] < 0.0 ? -0.5 : 0.5));
	}
	else if(IsZero(local_normal[2]))
	{
		// Edge center
		return Vec3((local_normal[0] < 0.0 ? -0.5 : 0.5),
					(local_normal[1] < 0.0 ? -0.5 : 0.5),
					0.0);
	}
	
	// Corner
	return Vec3((local_normal[0] < 0.0 ? -0.5 : 0.5),
				(local_normal[1] < 0.0 ? -0.5 : 0.5),
				(local_normal[2] < 0.0 ? -0.5 : 0.5));
}
#else // USE_XENOCOLLIDE

bool Box::intersection_test(Vec3 p, Vec3 &normal) const{	
	if(p[0] < .5 && p[0] > -.5){
		if(p[1] < .5 && p[1] > -.5){
			if(p[2] < .5 && p[2] > -.5){
				// find the closest normal
				double abs_x, abs_y, abs_z;
				abs_x = fabs(p[0]);
				abs_y = fabs(p[1]);
				abs_z = fabs(p[2]);
				if(abs_x < abs_y){
					if(abs_y < abs_z){ // closest to z-face
						normal = Vec3(0,0,sign(p[2]));
					} else{            // closest to y-face
						normal = Vec3(0,sign(p[1]),0);
					}
				} else{
					if(abs_x < abs_z){ // closest to z-face
						normal = Vec3(0,0,sign(p[2]));
					} else{            // closest to x-face
						normal = Vec3(sign(p[0]),0,0);
					}
				}
				return true;
			}
		}
	}

	return false;
}
#endif // USE_XENOCOLLIDE

