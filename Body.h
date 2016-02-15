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

class Body
{
public:

    Body(const Vec3 & i_ConstructPos, const Quaternion & i_ConstructOrien, Model* i_model,
        Vec3 i_size, const double restitution, const double coef_friction, const double i_inv_mass);
    ~Body(void);

    void reset();
    void draw();
#if USE_XENOCOLLIDE
    static bool intersection_test(Body* body1, Body* body2, Vec3& p1, Vec3& p2, Vec3 &normal);
#else
	bool intersection_test(Body *body_o, Vec3 &p, Vec3 &normal);
#endif
    Vec3 get_vertex_world_position(int i) const;
	void TransformBodyToWorld(Vec3 &body_pos) const;
    Vec3 get_vertex_world_normal(int i) const;
    void get_vertex_in_body_space(Vec3 &world_pos) const;
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
    std::vector<Body*> in_contact_list;
    int index;
    int lowlink;
    bool in_stack;
    int SCC_num;
};
