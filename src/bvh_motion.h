#include <GL/glew.h>
#include <GL/gl.h>
#include <glm/glm.hpp>
#include <GL/glut.h>
#include <vector>
#include <string>
#include "joint.h"
#pragma once

/*
    FRAME and POSE
*/
class BVHMotion{

    public: std::vector<Joint> joints;
            GLfloat frameTime = 0.0f;
            int numFrames = 0;
            std::vector<std::vector<GLfloat>> motionData;
    
    BVHMotion() {}

    
    
    void setNumFrames(int nf);

    void setFrameTime(GLfloat fTime);

};
