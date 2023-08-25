#include <GL/glew.h>
#include <iostream>
#include <GL/gl.h>
#include <glm/glm.hpp>
#include <GL/glut.h>
#include <vector>
#include <string>
#pragma once



enum TransformationType {
    CAN_ROTATE_TRANSLATE,
    CAN_ROTATE,
    CAN_TRANSLATE,
    NO_CHANNEL
    };
    


class Joint{

    public:

    glm::vec3 position;
    bool isthereChild;
    int depth = 0;
    int numChannels = 0;
    std::string jointName = "";
    TransformationType t;
    std::vector<std::string> realOrder;
    std::vector<std::string> transformationOrder;
    bool canTranslateRotate = false;
    bool canRotate = false;
    bool canTranslate = false;

    Joint(std::string jName) {
        jointName = jName;
    } 

    Joint(GLfloat x, GLfloat y, GLfloat z) {
        position.x = x;
        position.y = y;
        position.z = z;
    }

    int channelIndex(std::string str);
    void setRealOrder();
    int howManyChannels();

};
