#include "Mesh.h"
#include "MeshIO.h"

Mesh::Mesh()
{
    
}

Mesh::Mesh(const Mesh& mesh)
{
    *this = mesh;
}

bool Mesh::read(const string& fileName)
{
    ifstream in(fileName.c_str());

    if (!in.is_open()) {
        cerr << "Error: Could not open file for reading" << endl;
        return false;
    }
    
    bool readSuccessful = false;
    if ((readSuccessful = MeshIO::read(in, *this))) {
        normalize();
    }
    
    in.close();
    return readSuccessful;
}

bool Mesh::write(const string& fileName) const
{
    ofstream out(fileName.c_str());
    
    if (!out.is_open()) {
        cerr << "Error: Could not open file for writing" << endl;
        return false;
    }
    
    MeshIO::write(out, *this);
    
    out.close();
    return false;
}

void Mesh::normalize()
{
    // compute center of mass
    Vector3d cm = Vector3d::Zero();
    for (VertexCIter v = vertices.begin(); v != vertices.end(); v++) {
        cm += v->position;
    }
    cm /= (double)vertices.size();
    
    // translate to origin and determine radius
    double rMax = 0;
    for (VertexIter v = vertices.begin(); v != vertices.end(); v++) {
        v->position -= cm;
        rMax = max(rMax, v->position.norm());
    }
    
    // rescale to unit sphere
    for (VertexIter v = vertices.begin(); v != vertices.end(); v++) {
        v->position /= rMax;
    }
}
