#ifndef TYPES_H
#define TYPES_H

#include <stdlib.h>
#include <string>
#include <vector>
#include <unordered_map>
#include <iostream>
#include "math.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SparseCore>

using namespace std;
using namespace placeholders;
using namespace Eigen;

class Vertex;
class Edge;
class Face;
class HalfEdge;
class Mesh;

typedef vector<HalfEdge>::iterator HalfEdgeIter;
typedef vector<HalfEdge>::const_iterator HalfEdgeCIter;
typedef vector<Vertex>::iterator VertexIter;
typedef vector<Vertex>::const_iterator VertexCIter;
typedef vector<Edge>::iterator EdgeIter;
typedef vector<Edge>::const_iterator EdgeCIter;
typedef vector<Face>::iterator FaceIter;
typedef vector<Face>::const_iterator FaceCIter;

#endif
