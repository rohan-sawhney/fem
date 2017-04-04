#include "Shader.h"
#include <fstream>
#include <sstream>

Shader::Shader()
{
    
}

bool Shader::readShaderCode(const string& file, string& code)
{
    ifstream in;
    string path;
    if (!file.empty()) path = dir + "/" + file;
    
    in.open(path.c_str());
    if (in.is_open()) {
        stringstream stream;
        stream << in.rdbuf();
        
        code = stream.str();
        in.close();
        
    } else {
        return false;
    }
    
    return true;
}

GLuint Shader::compileShader(const string& file, GLenum type)
{
    GLint success;
    GLchar infoLog[512];
    string code;
    GLuint shader = 0;
    
    if (readShaderCode(file, code)) {
        const GLchar *shaderCode = code.c_str();
        shader = glCreateShader(type);
        glShaderSource(shader, 1, &shaderCode, NULL);
        glCompileShader(shader);
        glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
        if (!success) {
            glGetShaderInfoLog(shader, 512, NULL, infoLog);
            cout << "Error: " << infoLog << endl;
            shader = 0;
        }
    }
    
    return shader;
}

void Shader::setup(const string& dir0,
                   const string& vertexFile,
                   const string& geometryFile,
                   const string& fragmentFile)
{
    // compile shaders
    dir = dir0;
    vertex = compileShader(vertexFile, GL_VERTEX_SHADER);
    geometry = compileShader(geometryFile, GL_GEOMETRY_SHADER);
    fragment = compileShader(fragmentFile, GL_FRAGMENT_SHADER);

    // create program and attach shaders
    program = glCreateProgram();
    if (vertex) glAttachShader(program, vertex);
    if (geometry) glAttachShader(program, geometry);
    if (fragment) glAttachShader(program, fragment);
    
    GLint success;
    GLchar infoLog[512];
    
    glLinkProgram(program);
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success) {
        glGetProgramInfoLog(program, 512, NULL, infoLog);
        cout << "Error: " << infoLog << endl;
    }
}

void Shader::use()
{
    glUseProgram(program);
}

void Shader::reset()
{
    glDetachShader(program, vertex);
    glDeleteShader(vertex);
    
    glDetachShader(program, geometry);
    glDeleteShader(geometry);
    
    glDetachShader(program, fragment);
    glDeleteShader(fragment);
    
    glDeleteProgram(program);
}
