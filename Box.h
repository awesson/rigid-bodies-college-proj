#include "Model.h"
#include "BoxMesh.h"
#include <OpenGL/gl.h>

class Box : public Model{
public:
    Box(Color3);
    virtual ~Box();

    virtual void render() const;
    virtual void get_Iinv(Matrix3& Iinv, Vec3 size, double inv_mass);
    virtual int num_vertices() const;
#if USE_XENOCOLLIDE
    virtual Vec3 GetSupportPoint(const Vec3& normal) const;
#else
	virtual bool intersection_test(Vec3 p, Vec3 &normal) const;
#endif
};
