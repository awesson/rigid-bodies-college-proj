/**
 * @file BoxMesh.cpp
 * @brief Mesh class for a cube.
 *
 * @author Andrew Wesson (awesson)
 * @author Eric Butler (edbutler)
 * @author Zeyang Li (zeyangl)
 */

#include "BoxMesh.h"
#include <iostream>
#include <cstring>
#include <string>
#include <fstream>
#include <sstream>
#include <map>

// number of floats per vertex
#define VERTEX_SIZE_WT 8
#define VERTEX_SIZE_WOT 6

struct TriIndex
{
    int vertex;
    int normal;
    int tcoord;

    bool operator<( const TriIndex& rhs ) const {
        if ( vertex == rhs.vertex ) {
            if ( normal == rhs.normal ) {
                return tcoord < rhs.tcoord;
            } else {
                return normal < rhs.normal;
            }
        } else {
            return vertex < rhs.vertex;
        }
    }
};

struct Face
{
    TriIndex v[3];
};

enum ObjFormat
{
    VERTEX_ONLY = 1 << 0,
    VERTEX_UV = 1 << 1,
    VERTEX_NORMAL = 1 << 2,
    VERTEX_UV_NORMAL = 1 << 3
};

BoxMesh::BoxMesh()
{
    has_tcoords = false;
    has_normals = false;

    // add vertices
    MeshVertex v;
    for(int i = 0; i < 2; ++i)
        for(int j = 0; j < 2; ++j)
            for(int k = 0; k < 2; ++k){
                v.position = Vec3(i-0.5,j-0.5,k-0.5);
                vertices.push_back(v);
            }

    unsigned int edge_list[18][2] = {{1, 2}, // 1
                                {2, 3}, // 2
                                {3, 4}, //3
                                {2, 4}, //4
                                {1, 3}, //5
                                {5, 6}, //6
                                {6, 7}, //7
                                {7, 8}, //8
                                {6, 8}, //9
                                {5, 7}, //10
                                {2, 5}, //11
                                {2, 6}, //12
                                {4, 6},//13
                                {4, 8},//14
                                {4, 7},//15
                                {3, 7},//16
                                {3, 5},//17
                                {1, 5}};//18

    // add edges
    MeshEdge e[18];
    for(int k = 0; k < 18; ++k)
        for(int i = 0; i < 2; ++i)
            e[k].vertices[i] = edge_list[k][i] - 1;

    for(int i = 0; i < 18; ++i)
        edges.push_back(e[i]);

    unsigned int face_list[12][3] = {{1, 2, 3},
                                     {4, 3, 2},
                                     {7, 6, 5},
                                     {6, 7, 8},
                                     {2, 6, 4},
                                     {4, 6, 8},
                                     {1, 3, 5},
                                     {7, 5, 3},
                                     {3, 4, 7},
                                     {4, 8, 7},
                                     {5, 2, 1},
                                     {2, 5, 6}};

    unsigned int face_edge_list[12][3] = {{1, 2, 5},
                                          {2, 3, 4},
                                          {6, 7, 10},
                                          {7, 8, 9},
                                          {4, 12, 13},
                                          {9, 13, 14},
                                          {5, 17, 18},
                                          {10, 16, 17},
                                          {3, 15, 16},
                                          {8, 14, 15},
                                          {1, 11, 18},
                                          {6, 11, 12}};


    // add faces
    MeshTriangle faces[12];
    for(int k = 0; k < 12; ++k){
        for(int i = 0; i < 3; ++i){
            faces[k].vertices[i] = face_list[k][i] - 1;
            faces[k].edges[i] = face_edge_list[k][i] - 1;
        }
    }

    for(int i = 0; i < 12; ++i)
        triangles.push_back(faces[i]);

    // subdivide for the intersection tests
    for(int i = 0; i < 2; ++i)
    	subdivide();

    create_gl_data();
}

BoxMesh::~BoxMesh()
{ 
	vertices.clear();
	triangles.clear();
	edges.clear();
	vertex_data.clear();
	index_data.clear();
}

const MeshTriangle* BoxMesh::get_triangles() const
{
    return triangles.empty() ? NULL : &triangles[0];
}

const MeshTriangle BoxMesh::get_triangle(int i) const
{
    return triangles[i];
}

size_t BoxMesh::num_triangles() const
{
    return triangles.size();
}

const MeshEdge* BoxMesh::get_edges() const
{
    return edges.empty() ? NULL : &edges[0];
}

const MeshEdge BoxMesh::get_edge(int i) const
{
    return edges[i];
}

size_t BoxMesh::num_edges() const
{
    return edges.size();
}

const MeshVertex* BoxMesh::get_vertices() const
{
    return vertices.empty() ? NULL : &vertices[0];
}

const MeshVertex BoxMesh::get_vertex(int i) const
{
    return vertices[i];
}

size_t BoxMesh::num_vertices() const
{
    return vertices.size();
}

bool BoxMesh::are_normals_valid() const
{
    return has_normals;
}

bool BoxMesh::are_tex_coords_valid() const
{
    return has_tcoords;
}

/**
 * Splits each triangle into 4 new triangles, without affecting the geometry.
 **/
void BoxMesh::subdivide(){
    MeshVertexList new_vertices = vertices;
    MeshEdgeList new_edges;
    MeshTriangleList new_triangles;

    new_vertices.resize(vertices.size() + edges.size());
    new_triangles.resize(4 * triangles.size());

    // add new vertices
    for(int i = 0; i < edges.size(); ++i){
        new_vertices[vertices.size() + i].position = (vertices[edges[i].vertices[0]].position + vertices[edges[i].vertices[1]].position)/2.0;
        new_vertices[vertices.size() + i].normal = vertices[edges[i].vertices[0]].normal + vertices[edges[i].vertices[1]].normal;
        unitize(new_vertices[vertices.size() + i].normal);
        edges[i].new_v = vertices.size() + i;
    }

    // go through current faces and add the new 4 faces associated with it and the new edges
    for(int i = 0; i < triangles.size(); ++i){
        unsigned int interior_edge_indices[3];

        // add the faces containing one old vertex
        for(int v = 0; v < 3; ++v){
            int index = 0;
            unsigned int new_vs[2][2];

            // find new vertices for this face
            for(int k = 0; k < 3; ++k){
                if(edges[triangles[i].edges[k]].vertices[0] == triangles[i].vertices[v]){
                    for(int l = 0; l < 3; ++l)
                        if(edges[triangles[i].edges[k]].vertices[1] == triangles[i].vertices[l])
                            new_vs[index][1] = l;
                    new_vs[index][0] = edges[triangles[i].edges[k]].new_v;
                    index++;
                } else if(edges[triangles[i].edges[k]].vertices[1] == triangles[i].vertices[v]){
                    for(int l = 0; l < 3; ++l)
                        if(edges[triangles[i].edges[k]].vertices[0] == triangles[i].vertices[l])
                            new_vs[index][1] = l;
                    new_vs[index][0] = edges[triangles[i].edges[k]].new_v;
                    index++;
                }
            }

            // add vertices
            new_triangles[4*i + v].vertices[v] = triangles[i].vertices[v];
            new_triangles[4*i + v].vertices[new_vs[0][1]] = new_vs[0][0];
            new_triangles[4*i + v].vertices[new_vs[1][1]] = new_vs[1][0];

            // add edges
            MeshEdge e;

            // add edges with triangles[i].vertices[v] in it
            for(int k = 0; k < 2; k++){
                e.vertices[0] = triangles[i].vertices[v];
                e.vertices[1] = new_vs[k][0];
                int index;
                if(find_edge(e, new_edges, &index))
                    new_triangles[4*i + v].edges[k] = index;
                else{
                    new_triangles[4*i + v].edges[k] = new_edges.size();
                    new_edges.push_back(e);
                }
            }

            // add edge between two new vertices
            e.vertices[0] = new_vs[0][0];
            e.vertices[1] = new_vs[1][0];
            new_triangles[4*i + v].edges[2] = new_edges.size();
            new_edges.push_back(e);
            interior_edge_indices[v] = new_triangles[4*i + v].edges[2];
        }

        // add the face with the three new vertices
        for(int k = 0; k < 3; ++k){
            new_triangles[4*i + 3].edges[k] = interior_edge_indices[k];
            for(int e = 0; e < 3; ++e)
                if((edges[triangles[i].edges[e]].vertices[0] != triangles[i].vertices[k])
                    && (edges[triangles[i].edges[e]].vertices[1] != triangles[i].vertices[k])){
                    new_triangles[4*i + 3].vertices[k] = edges[triangles[i].edges[e]].new_v;
                }
        }
    }

    vertices = new_vertices;
    edges = new_edges;
    triangles = new_triangles;
}

bool BoxMesh::find_edge(MeshEdge e, MeshEdgeList edges, int* index){
    for(int i = 0; i < edges.size(); ++i){
        if((e.vertices[0] == edges[i].vertices[0] || e.vertices[0] == edges[i].vertices[1])
            && (e.vertices[1] == edges[i].vertices[0] || e.vertices[1] == edges[i].vertices[1])){
            *index = i;
            return true;
        }
    }
    return false;
}

bool BoxMesh::create_gl_data()
{
    // if no vertices, nothing to do
    if ( vertices.empty() || triangles.empty() ) {
        return false;
    }

    // compute normals if needed
    if ( !has_normals ) {
        // first zero out
        for ( int i = 0; i < vertices.size(); ++i ) {
            vertices[i].normal = Vec3(0.0, 0.0, 0.0);
        }

        // then sum in all triangle normals
        for ( int i = 0; i < triangles.size(); ++i ) {
            Vec3 pos[3];
            for ( size_t j = 0; j < 3; ++j ) {
                pos[j] = vertices[triangles[i].vertices[j]].position;
            }
            Vec3 normal = cross( pos[1] - pos[0], pos[2] - pos[0] );
            unitize( normal );
            for ( int j = 0; j < 3; ++j ) {
                vertices[triangles[i].vertices[j]].normal += normal;
            }
        }

        // then normalize
        for ( int i = 0; i < vertices.size(); ++i ) {
            unitize( vertices[i].normal );
        }

        has_normals = true;
    }

    // build vertex data
    if(has_tcoords)
        vertex_data.resize( vertices.size() * VERTEX_SIZE_WT );
    else
        vertex_data.resize( vertices.size() * VERTEX_SIZE_WOT );
    float* vertex = &vertex_data[0];
    for ( int i = 0; i < vertices.size(); ++i ) {
        if(has_tcoords){
            for(int k = 0; k < 2; ++k)
                vertex[k] = vertices[i].tex_coord[k];
            vertex += 2;
        }
        for(int k = 0; k < 3; ++k){
            vertex[k] = vertices[i].normal[k];
        }
        vertex += 3;
        for(int k = 0; k < 3; ++k)
            vertex[k] = vertices[i].position[k];
        vertex += 3;
    }

    // build index data
    index_data.resize( triangles.size() * 3 );
    unsigned int* index = &index_data[0];

    for ( int i = 0; i < triangles.size(); ++i ) {
        index[0] = triangles[i].vertices[0];
        index[1] = triangles[i].vertices[1];
        index[2] = triangles[i].vertices[2];
        index += 3;
    }
    return true;
}

void BoxMesh::render() const
{
    assert( index_data.size() > 0 );
    if(has_tcoords)
        glInterleavedArrays( GL_T2F_N3F_V3F, VERTEX_SIZE_WT * sizeof vertex_data[0], &vertex_data[0] );
    else
        glInterleavedArrays( GL_N3F_V3F, VERTEX_SIZE_WOT * sizeof vertex_data[0], &vertex_data[0] );

     glDrawElements( GL_TRIANGLES, index_data.size(), GL_UNSIGNED_INT, &index_data[0] );
 }

