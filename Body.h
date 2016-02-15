#pragma once

#include <gfx/vec2.h>
#include "quaternion.h"
#include "matrix.h"
#include "Model.h"

struct BodyInfo{
	Vec3 Pos;
	Quaternion Orientation;
	Vec3 size;
	Color3 color;
};

// contact information relative to another body
// used for the contact graph
struct ContactInfo;

class Body
{
public:

    Body(const Vec3 & i_ConstructPos, const Quaternion & i_ConstructOrien, Model* i_model,
        Vec3 i_size, const double restitution, const double coef_friction, const double i_inv_mass);
    ~Body(void);

    void reset();
    void draw();
    bool intersection_test(Body* body_o, Vec3& p, Vec3 &normal);
    Vec3 get_vertex_world_position(int i);
    Vec3 get_vertex_world_normal(int i);
    Vec3 get_vertex_in_body_space(Vec3 world_pos);
    void getInfo(BodyInfo &);
    Matrix3 get_K(Vec3 pos);
    Vec3 get_vel(Vec3 pos);
    Matrix3 star(Vec3 v);

    const Vec3 ConstructPos;
    const Quaternion ConstructOrien;
    Vec3 Position;
    Matrix3 R;
    Matrix3 R_t;
    Quaternion Orientation;
    Vec3 Velocity;
    Vec3 Momentum;
    Vec3 Omega;
    Vec3 AngularMomentum;
    Vec3 forces;
    Vec3 torques;
    Model* model;
    Matrix3 Iinv_body;
    Matrix3 Iinv;
	//Matrix3 construct_Iinv;
    Vec3 size;
    const double radius; // bounding sphere radius
    double inv_mass;
	const double construct_inv_mass;
    const double restitution;
    const double coef_friction;

    // the contact graph. Holds the bodies which this one rests on top of
    std::vector<ContactInfo> in_contact_list;
    int index;
    int lowlink;
    bool in_stack;
	int SCC_num;
};

// contact information relative to another body
// used for the contact graph
struct ContactInfo{
	Body *b; // body in contact with
	Vec3 p; // position of contact
	Vec3 normal; // normal of contact
};
