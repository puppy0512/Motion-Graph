#include <iostream>
#include <GL/glew.h>
#include <GL/gl.h> 
#include <glm/glm.hpp>
#include <GL/glut.h>
#include <cstring>
#include <string.h>
#include <math.h>
#include <glm/gtx/vector_angle.hpp>
#include <deque>
#include <algorithm>
#include "imgui.h"
#include "imgui_impl_glut.h"
#include "imgui_impl_opengl3.h"
#include "motion_graph.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include <GL/freeglut.h>
#include <GLFW/glfw3.h>

// Comes with Translation, Rotation, and other transformations
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "parse_bvh.h"

int width, height;
GLUquadric* qobj;
GLuint textureID;
const float RGB = 255.0f;
const glm::vec3 upVector(0.0f, 1.0f, 0.0f);
MotionGraph motionGraph;




BVHMotion bvhMotion;
std::vector<std::string> bvhList;



int iFrame = 0;
using namespace std;
static float proportionTime = 1.0f;
static float size_figure = 1.0f;
static float xMove = 0.0f;
static float yMove = 0.0f;
static float zMove = 0.0f;


static float xRot = 0.0f;
static float yRot = 0.0f;
static float zRot = 0.0f;

std::string bvhFileName;
const char** constBvhFileName;
int selectedItem = 0;


static bool show_demo_window = true;
static bool show_another_window = false;

const float eye[3] = 
{
    0.0f,
    200.5f,
    500.3f
};

const float ori[3] = 
{
    0.0f,
    0.0f,
    -20.0f
};

const float up[3] = 
{
    0.0f,
    1.0f,
    0.0f
};


void loadGlobalCoord() {
    glLoadIdentity();
    gluLookAt(eye[0], eye[1], eye[2], ori[0], ori[1], ori[2], 0, 1, 0);
}


void resize(int w, int h) {
    width = w;
    height = h;
    glViewport(0, 0, w, h);

    // World Coordinate
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    // Near Clipping plane and Far Clipping plane
    gluPerspective(45.0f, (GLfloat) w / (GLfloat) h, 0.1f, 10000.0f);
    //ImGui_ImplGLUT_ReshapeFunc(w, h);


}


/*
    Given a next point's position, drawCylinder draws a cylinder starting from a previous position

*/
void drawCylinder(glm::vec3 nextPoint) {


    glm::vec3 objPos(0.0f, 0.0f, 0.0f);
    glm::vec3 targetPos(nextPoint.x, nextPoint.y, nextPoint.z);

    glm::vec3 offsetVector = targetPos - objPos;


    GLfloat distance = glm::length(offsetVector);
    glm::vec3 axis = glm::cross(glm::vec3(0.0f, 0.0f, 1.0f), offsetVector);

    GLfloat angle = 180.0f / M_PI * acos(glm::dot(glm::vec3(0,0, 1), offsetVector) / distance);
    
    glPushMatrix();
        glColor3f(86.0f / RGB, 198.0f / RGB, 169.0f / RGB);

    //if(nextPoint.x == 0.0f || nextPoint.y == 0.0f || nextPoint.z == 0.0f) {
        glRotatef(angle, axis.x ,axis.y, axis.z);
        gluCylinder(qobj, 1.7f, 1.7f, distance, 30, 50);
    //}
    glPopMatrix();
            glColor3f(RGB, RGB, RGB);

}


void drawHead(GLfloat f) {

    glPushMatrix();
    glutSolidCube(f);
    glPopMatrix();

}

void transformJoint(deque<GLfloat> channelMovement, glm::vec3 offsetPosition, 
int numChannel, vector<std::string> realOrder) {

size_t findIndex;

    if(realOrder.size() == 6) {

        glTranslatef(offsetPosition.x + channelMovement[0], offsetPosition.y + channelMovement[1], offsetPosition.z + channelMovement[2]);
        //TRANSFORMATION

        // MAKE SURE THAT I ROTATE IN WHAT BVH TOLD ME TO DO

        for(int i = 3; i < 6; i++) {
            if(realOrder[i].compare("Xrot") == 0) {
                glRotatef(channelMovement[i],1, 0, 0);
            } else if (realOrder[i].compare("Yrot") == 0) {
                glRotatef(channelMovement[i],0, 1, 0);
            } else {
                glRotatef(channelMovement[i],0, 0, 1);
            }
        }
    } else if(realOrder.size() == 3) {
        glTranslatef(offsetPosition.x, offsetPosition.y, offsetPosition.z);


        for(int i = 0; i < 3; i++) {
            if(realOrder[i].compare("Xrot") == 0) {
                glRotatef(channelMovement[i],1, 0, 0);
            } else if (realOrder[i].compare("Yrot") == 0) {
                glRotatef(channelMovement[i],0, 1, 0);
            } else {
                glRotatef(channelMovement[i],0, 0, 1);
            }
        }
    } else {
        
    }
}


void update(int v) {
    iFrame++;
    glutPostRedisplay();
    glutTimerFunc(bvhMotion.frameTime/ proportionTime * 1000 , update, 0);
}

void loadTexture() {
    int width, height, channels;
    unsigned char* image = stbi_load("../src/checkerboard.jpg", &width, &height, &channels, 3);

    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_2D, textureID);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, image);
    stbi_image_free(image);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

}

void animateBVH() {
    int ithData = 0;

    // cout << "IFRAME: " << endl;
    // cout << iFrame << endl;
    // cout << "NUM FRAMES: " << endl;
    // cout << bvhMotion.numFrames << endl;

    if(iFrame >= bvhMotion.numFrames) {
        iFrame = 0;
        if (motionGraph.firstTime == 1){
            bvhMotion = motionGraph.traverseOneTime(0);

        } else {
        //if(iFrame >= bvhMotion.numFrames) {
            bvhMotion = motionGraph.traverseOneTime(0);

            // cout << "Current motion's number of frames" << endl;
            // cout << bvhMotion.numFrames << endl;
            
            // cout << "Next Node's Transition ID:" << endl;
            // cout << motionGraph.nextNode.transitionID << endl;

            // cout << "Next Node's frame number: " << endl;
            // cout << motionGraph.nextNode.frame << endl;

            
        }        
    }



    glm::vec3 rootPosition = bvhMotion.joints[0].position;
    glm::vec3 originPosition(0.0f, 0.0f, 0.0f);

//>>>>>>>>>>>>>>>>>
    vector<GLfloat> oneFrameMotion = bvhMotion.motionData[iFrame];


    deque<GLfloat> channelMovement;
    unsigned int j;
    unsigned int k;


            for(j = 0; j < bvhMotion.joints.size(); j++) {
                int numChannel = bvhMotion.joints[j].howManyChannels();
                



                for (k = 0; k < numChannel; k++) {
                    channelMovement.push_back(oneFrameMotion[ithData+k]);
                    
                }



                ithData = ithData + numChannel;
                int tempChannel = numChannel;


                // WHEN IT IS A ROOT
                if(j == 0) {                   
                    channelMovement[0] += xMove;
                    channelMovement[1] += yMove;
                    channelMovement[2] += zMove;


                    for(int i = 3; i < 6; i++) {
                        if(bvhMotion.joints[j].realOrder[i].compare("Xrot") == 0) {
                            channelMovement[i] += xRot;
                        } else if (bvhMotion.joints[j].realOrder[i].compare("Yrot") == 0) {
                            channelMovement[i] += yRot;
                        } else {
                            channelMovement[i] += zRot;
                        }
                    }
                    //channelMovement[3] += xRot;
                    //channelMovement[4] += yRot;
                    //channelMovement[5] += zRot;
                    transformJoint(channelMovement, bvhMotion.joints[j].position, numChannel, bvhMotion.joints[j].realOrder);
                                   channelMovement.clear();

                    glPushMatrix();
                    drawCylinder(bvhMotion.joints[j+1].position);

                } else if (j == bvhMotion.joints.size() - 1) {
                    
                    transformJoint(channelMovement, rootPosition, numChannel, bvhMotion.joints[j].realOrder);
                                   channelMovement.clear();


                    // IF it reaches the end of the BVH file
                    int depthDifference = bvhMotion.joints[j].depth + 1;
                    for(k = 0; k <= (depthDifference); k++) {
                        glPopMatrix();
                    }
                }
                // IF OTHER JOINTS HAVE A CHILD
                else if(bvhMotion.joints[j].isthereChild) {
                    transformJoint(channelMovement, bvhMotion.joints[j].position, numChannel, bvhMotion.joints[j].realOrder);
                                   channelMovement.clear();
                    
                    glPushMatrix();
                    drawCylinder(bvhMotion.joints[j+1].position);
                    

                // IF OTHER JOINTS DO NOT HAVE A CHILD
                } else  {
                    
                    //cout << "joint: " << j << endl;
                    transformJoint(channelMovement, bvhMotion.joints[j].position, numChannel, bvhMotion.joints[j].realOrder);
                                   channelMovement.clear();

                    if(bvhMotion.joints[j].jointName.substr(0,4).compare("Head") == 0 || bvhMotion.joints[j].jointName.substr(0,4).compare("head") == 0) {
                        //cout << channelMovement[0] << " "  << channelMovement[1] << " " << channelMovement[2] << endl;
                        drawHead(3.0f);
                        
                    }
                    //glPopMatrix();
                    
                    
                    // DIFFERENCE IN DEPTH OF JOINTS IN A HIERACHY-MODEL
                    int depthDifference = bvhMotion.joints[j].depth - (bvhMotion.joints[j+1].depth - 1);
                    
                    for(k = 0; k < (depthDifference); k++) {
                        glPopMatrix();
                    }
                    //cout << "Pop how many times: " << (k+1) << endl;



                    glPushMatrix();
                    drawCylinder(bvhMotion.joints[j+1].position);
                }

                channelMovement.clear();
            }

}




void display() {
    GLfloat ambientColor[] = {1.0f, 1.0f, 1.0f, 1.0f};
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambientColor);

    GLfloat lightColor0[] = {1.0, 1.0f, 1.0f, 1.0f};
    GLfloat lightPos0[] = {0.0f, 3.0f, 0.0f, 1.0f};

    
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightColor0);
    glLightfv(GL_LIGHT0, GL_POSITION, lightPos0);
      GLfloat spotExp = 20.0;
  glLightf(GL_LIGHT0, GL_SPOT_EXPONENT, spotExp);
    GLfloat SPOTDIRECTION[] = { 0.0f, 2.0f, 0.0f };
glLightfv(GL_LIGHT0, GL_SPOT_DIRECTION, SPOTDIRECTION);


    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    // ImGui_ImplOpenGL3_NewFrame();
    // ImGui_ImplGLUT_NewFrame();
    // ImGui::NewFrame();


    glMatrixMode(GL_MODELVIEW);







    loadGlobalCoord();

        // Load and set up the texture

    glClear(GL_COLOR_BUFFER_BIT);
    glClearColor(RGB, RGB, RGB, 1.0f);
    glPushMatrix();
    loadTexture();
    // Enable texture mapping
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, textureID);

    glBegin(GL_QUADS);
    glTexCoord2f(0.0, 0.0);
    glVertex3f(-300, 0, -300);
    glTexCoord2f(4.0, 0.0);
    glVertex3f(300, 0, -300);
    glTexCoord2f(4.0, 4.0);
    glVertex3f(300, 0, 300);
    glTexCoord2f(0.0, 4.0);
    glVertex3f(-300, 0, 300);
    glEnd();
    glPopMatrix();


    //Disable texture mapping
    glDisable(GL_TEXTURE_3D);

    unsigned int i;
    unsigned int j;
    unsigned int k;

    std::vector<Joint>::iterator itv = bvhMotion.joints.begin();


    for (i = 0; i < bvhMotion.joints.size(); i++) {
        //cout << bvhMotion.joints[i].jointName << endl;
        //cout << bvhMotion.joints[i].position.x << " " << bvhMotion.joints[i].position.y << " " << bvhMotion.joints[i].position.z << " " << endl;
    }
    

    //bvhMotion = motionGraph.traverseOneTime(2);


    // ANIMATE A BVH
    animateBVH();


// IMGUI SET UP


            // ImGui::Begin("BVH Model Viewer");

            // ImGui::Text("BVH File");
            // // Create a dropdown combo with dynamic items
            // if (ImGui::BeginCombo("BVH file", constBvhFileName[selectedItem])) {
            //     for(int i = 0; i < bvhList.size(); i++) {
            //         bool isSelected = (selectedItem == i);
            //         if(ImGui::Selectable(constBvhFileName[i], isSelected)) {
            //             selectedItem = i;

            //             bvhMotion = parse(constBvhFileName[i]);
            //             bvhFileName = constBvhFileName[i];

            //             //cout << selectedItem << endl;
            //         }
            //         if (isSelected) {
            //             ImGui::SetItemDefaultFocus();
            //     }
            //     }
            //     ImGui::EndCombo();
            // }


            // ImGui::Text("TRANSLATION");
            // ImGui::SliderFloat("scale", &size_figure, 0.5f, 4.0f);
            // ImGui::SliderFloat("dx", &xMove, -300.0f, 300.0f);
            // ImGui::SliderFloat("dy", &yMove, -300.0f, 300.0f);
            // ImGui::SliderFloat("dz", &zMove, -300.0f, 300.0f);

            // ImGui::Text("ROTATION");
            // ImGui::SliderFloat("rotX", &xRot, -90.0f, 90.0f);
            // ImGui::SliderFloat("rotY", &yRot, -90.0f, 90.0f);
            // ImGui::SliderFloat("rotZ", &zRot, -90.0f, 90.0f);
        



            // ImGui::End();

            // ImGui::Render();
            // ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());





    glutSwapBuffers();
    glFlush();
}



void rendering() {
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_COLOR_MATERIAL);


    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);

    glClearColor(116.0f / RGB, 209.0f / RGB, 234.0f / RGB, 1.0f);
    //glClearColor(0.0f / RGB, 0.0f / RGB, 234.0f / RGB, 1.0f);

    qobj = gluNewQuadric();
    gluQuadricNormals(qobj, GLU_SMOOTH);
}



int main(int argc, char** argv)
{
    bvhList = readBVHFiles();
    constBvhFileName = new const char*[bvhList.size()];






    for(size_t i = 0; i < bvhList.size(); i++) {
        constBvhFileName[i] = bvhList[i].c_str();
    }

    constBvhFileName[bvhList.size()] = nullptr;

 





    for(int i = 0; i < 2; i++) {
        cout << i << endl;
        motionGraph.initialize(0, i, i, parse(bvhList[i]));
    }

    //motionGraph.printGraph();
    motionGraph.createNewTransition();
        //cout << "ewewe" << endl;
    //motionGraph.printGraph();

    // size_t arrayLength = 0;
    // while(constBvhFileName[arrayLength] != nullptr) {
    //     ++arrayLength;
    // }

    // std::cout << arrayLength << endl;

    // bvhMotion = parse(constBvhFileName[selectedItem]);
    // bvhFileName = constBvhFileName[selectedItem];

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);


    glutInitWindowSize(1000, 1000);
    glutInitWindowPosition(50, 0);
    glutCreateWindow("BVH PLAYER");



    rendering();
    loadGlobalCoord();
    
    
    
    /*
        GET FUNCTIONS INSTALLED
    */
    // ImGui_ImplGLUT_InstallFuncs();
    glutReshapeFunc(resize);
    glutDisplayFunc(display);
    glutTimerFunc(bvhMotion.frameTime * 1000.0f, update, 0);


    // IMGUI_CHECKVERSION();
    // ImGui::CreateContext();
    // ImGuiIO& io = ImGui::GetIO(); (void)io;



    // ImGui::StyleColorsDark();
    // ImGui_ImplGLUT_Init();
    // ImGui_ImplOpenGL3_Init();



    glutMainLoop();

    // ImGui_ImplOpenGL3_Shutdown();
    // ImGui_ImplGLUT_Shutdown();
    // ImGui::DestroyContext();

    return 0;

}
