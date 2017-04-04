#include "HalfEdge.h"
#include "Vertex.h"
#include "Edge.h"

double HalfEdge::cotan() const
{
    if (onBoundary) return 0.0;
    
    const Vector3d& p0 = vertex->position;
    const Vector3d& p1 = next->vertex->position;
    const Vector3d& p2 = next->next->vertex->position;
    
    Vector3d v1 = p2 - p1;
    Vector3d v2 = p2 - p0;
    
    return v1.dot(v2) / v1.cross(v2).norm();
}

void HalfEdge::vertexData(vector<Vector2d>& x, vector<int>& index, int offset) const
{
    const Vector3d& a = vertex->position;
    x[0] = Vector2d(a.x(), a.y());
    index[0] = vertex->index;
    
    const Vector3d& b = next->vertex->position;
    x[1] = Vector2d(b.x(), b.y());
    index[1] = next->vertex->index;
    
    if (offset > 0) {
        Vector3d c = (a + b)/2.0;
        x[2] = Vector2d(c.x(), c.y());
        index[2] = offset + edge->index;
    }
}
