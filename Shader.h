#ifndef SHADER_H
#define SHADER_H

#ifdef __APPLE_CC__
#include <OpenGL/gl3.h>
#include <OpenGL/gl3ext.h>
#define __gl_h_
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include <stdlib.h>
#include <string>
#include <iostream>

using namespace std;

class Shader {
public:
    // constructor
    Shader();
    
    // set up
    void setup(const string& dir0,
               const string& vertexFile,
               const string& geometryFile,
               const string& fragmentFile);
    
    // use
    void use();
    
    // reset
    void reset();
    
    // member variable
    GLuint program;
    
private:
    // reads shader code
    bool readShaderCode(const string& file, string& code);
    
    // compiles shader
    GLuint compileShader(const string& file, GLenum type);
    
    // member variables
    string dir;
    GLuint vertex;
    GLuint geometry;
    GLuint fragment;
};

#endif
