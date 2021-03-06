#ifndef CAMERA_H
#define CAMERA_H

#include "Types.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#define DEG_TO_RAD 0.0174532

enum CameraMovement {
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT,
    UP,
    DOWN
};

class Camera {
public:
    // constructor
    Camera(const Vector3f& pos0 = Vector3f(0.0, 0.0, 2.0),
           const Vector3f& up0 = Vector3f(0.0, 1.0, 0.0),
           const float& fov0 = 60.0, const float& near0 = 0.1, const float& far0 = 100);
    
    // process keyboard
    void processKeyboard(CameraMovement key, const float& dt);
    
    // process mouse
    void processMouse(const float& dx, const float& dy);
    
    // process scroll
    void processScroll(const float& scroll);
    
    // returns projection matrix
    glm::mat4 projectionMatrix(float x, float y);
    
    // returns view matrix
    glm::mat4 viewMatrix();
    
    // member variables
    Vector3f pos;
    Vector3f dir;
    Vector3f up;
    float fov;
    float aspect;
    float near;
    float far;

private:
    // update
    void updateDirection();
    
    // member variables
    Vector3f right;
    
    float yaw;
    float pitch;
    float speed;
};

#endif

