#include "Body.h"

class BoxBody : public Body
{
	virtual bool intersection_test(Body* body_o, Vec3& p, Vec3 &normal);
};