#ifndef MESH_H
#define MESH_H

#include "Types.h"
#include "Vertex.h"
#include "Edge.h"
#include "Face.h"
#include "HalfEdge.h"

class Mesh {
public:
    // default constructor
    Mesh();
    
    // copy constructor
    Mesh(const Mesh& mesh);
        
    // read mesh from file
    bool read(const string& fileName);
    
    // write mesh to file
    bool write(const string& fileName) const;
            
    // member variables
    vector<HalfEdge> halfEdges;
    vector<Vertex> vertices;
    vector<Edge> edges;
    vector<Face> faces;
    vector<HalfEdgeIter> boundaries;

private:
    // center mesh about origin and rescale to unit radius
    void normalize();
};

#endif
