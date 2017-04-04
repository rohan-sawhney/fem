#include "Face.h"
#include "HalfEdge.h"
#include "Vertex.h"
#include "Edge.h"

bool Face::isBoundary() const
{
    return he->onBoundary;
}

Vector3d Face::normal() const
{
    const Vector3d& a = he->vertex->position;
    const Vector3d& b = he->next->vertex->position;
    const Vector3d& c = he->next->next->vertex->position;
    
    return (b-a).cross(c-a);
}

double Face::area() const
{
    if (isBoundary()) {
        return 0;
    }
    
    return 0.5 * normal().norm();
}

void Face::vertexData(vector<Vector2d>& x, vector<int>& index, int offset) const
{
    int i = 0;
    HalfEdgeIter h = he;
    do {
        const Vector3d& a = h->vertex->position;
        x[i] = Vector2d(a.x(), a.y());
        index[i] = h->vertex->index;
        
        if (offset > 0) {
            const Vector3d& b = h->next->vertex->position;
            Vector3d c = (a + b)/2.0;
            x[i+3] = Vector2d(c.x(), c.y());
            index[i+3] = offset + h->edge->index;
        }
        
        i++;
        
        h = h->next;
    } while (h != he);
}
