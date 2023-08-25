#include <GL/glew.h>
#include <iostream>
#include <GL/gl.h>
#include <glm/glm.hpp>
#include <GL/glut.h>
#include <vector>
#include <string>
#include "bvh_motion.h"


std::vector<std::string> split(std::string str, char delimiter);

std::string trim(std::string str);

std::string trimTwoWays(std::string str);

std::vector<std::string> readBVHFiles();

BVHMotion parse(std::string filename);
