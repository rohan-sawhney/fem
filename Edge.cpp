#include "Edge.h"
#include "HalfEdge.h"
#include "Vertex.h"

double Edge::length() const
{
    const Vector3d& a = he->vertex->position;
    const Vector3d& b = he->flip->vertex->position;
    
    return (b-a).norm();
}

bool Edge::isBoundary() const
{
    return he->onBoundary || he->flip->onBoundary;
}
