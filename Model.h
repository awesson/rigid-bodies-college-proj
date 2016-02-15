/**
 * @file model.hpp
 * @brief Model class
 *
 */

#pragma once

#include "Mesh.h"
#include "Material.h"
#include "gfx/vec4.h"
#include "matrix.h"

#define USE_XENOCOLLIDE 1

/**
 * A Trianglular mesh with an inertia tenser and signed distance function.
 */
class Model
{
public:

    Model(){}
    virtual ~Model(){}

    virtual void render() const = 0;
    virtual void get_Iinv( Matrix3& Iinv, Vec3 size, double inv_mass) = 0;
    virtual int num_vertices() const = 0;
#if USE_XENOCOLLIDE
    virtual Vec3 GetSupportPoint(const Vec3& normal) const = 0;
#else
	virtual bool intersection_test(Vec3 p, Vec3 &normal) const = 0;
#endif

    Mesh* mesh;
    Material* material;
};

