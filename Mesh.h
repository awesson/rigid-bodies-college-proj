/**
 * @file mesh.hpp
 * @brief Mesh class and OBJ loader.
 *
 * @author Eric Butler (edbutler)
 * @author Zeyang Li (zeyangl)
 */

#pragma once

#include "gfx/vec2.h"
#include "gfx/vec3.h"
#include <OpenGL/gl.h>
#include <GLUT/glut.h>
#include <vector>
#include <cassert>

struct MeshVertex
{
    Vec3 position;
    Vec3 normal;
    Vec2 tex_coord;
};

struct MeshTriangle
{
    // index into the vertex list of the 3 vertices
    unsigned int vertices[3];
    // index into the edge list of the 3 edges
    unsigned int edges[3];
};

struct MeshEdge
{
    // index into the vertex list of the 2 vertices
    unsigned int vertices[2];
    // added vertex for subdivision
    unsigned int new_v;
};

/**
 * A mesh of triangles.
 */
class Mesh
{
public:

    Mesh() {}
    virtual ~Mesh() {}

    /**
     * Loads the model into a list of triangles and vertices.
     * @return True on success.
     */

    /// Get a pointer to the triangles.
    virtual const MeshTriangle* get_triangles() const = 0;
    /// get the ith triangle
    virtual const MeshTriangle get_triangle(int i) const = 0;
    /// The number of elements in the triangle array.
    virtual size_t num_triangles() const = 0;
    /// Get a pointer to the edges.
    virtual const MeshEdge* get_edges() const = 0;
    /// get the ith edge
    virtual const MeshEdge get_edge(int i) const = 0;
    /// The number of elements in the edge array.
    virtual size_t num_edges() const = 0;
    /// Get a pointer to the vertices.
    virtual const MeshVertex* get_vertices() const = 0;
    /// get the ith vertex
    virtual const MeshVertex get_vertex(int i) const = 0;
    /// The number of elements in the vertex array.
    virtual size_t num_vertices() const = 0;

    /// Returns true if the loaded model contained normal data.
    virtual bool are_normals_valid() const = 0;
    /// Returns true if the loaded model contained texture coordinate data.
    virtual bool are_tex_coords_valid() const = 0;

    // scene loader stores the filename of the mesh here
    std::string filename;

    /// Creates opengl data for rendering and computes normals if needed
    virtual bool create_gl_data() = 0;
    /// Renders the mesh using opengl.
    virtual void render() const = 0;

protected:

    typedef std::vector< MeshTriangle > MeshTriangleList;
    typedef std::vector< MeshEdge > MeshEdgeList;
    typedef std::vector< MeshVertex > MeshVertexList;

    // The list of all triangles in this model.
    MeshTriangleList triangles;

    // The list of all edges in this model.
    MeshEdgeList edges;

    // The list of all vertices in this model.
    MeshVertexList vertices;

    bool has_tcoords;
    bool has_normals;

    typedef std::vector< float > FloatList;
    typedef std::vector< unsigned int > IndexList;

    // the vertex data used for GL rendering
    FloatList vertex_data;
    // the index data used for GL rendering
    IndexList index_data;

    // prevent copy/assignment
    Mesh( const Mesh& );
    Mesh& operator=( const Mesh& );

};

