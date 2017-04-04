#ifndef RENDER_DATA_H
#define RENDER_DATA_H

#include "Shader.h"
#include "Mesh.h"

struct GLVertex {
    Vector3f position;
    Vector3f normal;
    Vector3f barycenter;
    Vector3f color;
    Vector2f uv;
};

struct GLPickVertex {
    Vector3f position;
    Vector3f color;
};

class GLMesh {
public:
    // constructor
    GLMesh(Mesh& mesh0);
    
    // setup
    void setup(const vector<Vector3f>& colors);
    
    // update
    void update(const vector<Vector3f>& colors);
    
    // draw
    void draw(Shader& shader) const;
    
    // draw
    void drawPick(Shader& shader) const;
    
    // reset
    void reset();
    
    // member variables
    vector<GLVertex> vertices;
    vector<GLPickVertex> pickVertices;
    Mesh& mesh;

private:
    // fills buffers
    void fillBuffers(const vector<Vector3f>& colors);
    
    // fills pick buffers
    void fillPickBuffers();
    
    // member variables
    GLuint vao, pickVao;
    GLuint vbo, pickVbo;
};

#endif
