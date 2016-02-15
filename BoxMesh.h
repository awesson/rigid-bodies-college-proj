/**
 * @file BoxMesh.h
 * @brief Mesh class for a cube.
 *
 * @author Andrew Wesson (awesson)
 */

#pragma once

#include "Mesh.h"

class BoxMesh : public Mesh
{
public:
    BoxMesh();
    virtual ~BoxMesh();

    /**
     * Loads the model into a list of triangles and vertices.
     * @return True on success.
     */
    //bool load();

    /// Get a pointer to the triangles.
    virtual const MeshTriangle* get_triangles() const;
    /// get the ith triangle
    virtual const MeshTriangle get_triangle(int i) const;
    /// The number of elements in the triangle array.
    virtual size_t num_triangles() const;
    /// Get a pointer to the edges.
    virtual const MeshEdge* get_edges() const;
    /// get the ith edge
    virtual const MeshEdge get_edge(int i) const;
    /// The number of elements in the edge array.
    virtual size_t num_edges() const;
    /// Get a pointer to the vertices.
    virtual const MeshVertex* get_vertices() const;
    /// get the ith vertex
    virtual const MeshVertex get_vertex(int i) const;
    /// The number of elements in the vertex array.
    virtual size_t num_vertices() const;

    /// Returns true if the loaded model contained normal data.
    virtual bool are_normals_valid() const;
    /// Returns true if the loaded model contained texture coordinate data.
    virtual bool are_tex_coords_valid() const;

    /// create 4 new triangles for each current one
    void subdivide();

    /// Creates opengl data for rendering and computes normals if needed
    virtual bool create_gl_data();
    /// Renders the mesh using opengl.
    virtual void render() const;

private:
    bool find_edge(MeshEdge e, MeshEdgeList edges, int* index);
};
