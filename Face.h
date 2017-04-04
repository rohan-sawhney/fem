#ifndef FACE_H
#define FACE_H

#include "Types.h"

class Face {
public:
    // one of the halfedges associated with this face
    HalfEdgeIter he;
    
    // id between 0 and |F|-1
    int index;
    
    // quasi conformal error
    Vector3d qcError;
    
    // checks if this face lies on boundary
    bool isBoundary() const;
    
    // returns normal to face
    Vector3d normal() const;
    
    // returns face area
    double area() const;
    
    // returns xy positions and indices of face vertices
    void vertexData(vector<Vector2d>& x, vector<int>& index, int offset) const;
};

#endif
