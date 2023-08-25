#include "motion_graph.h"
#include <glm/glm.hpp>
#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/rotate_vector.hpp>
/**
 * Norm Squared Difference
 * in Orientations of joints
 * in Root's position
 * in angular velocity
 * 
*/

void MotionGraph::addEdge (int startFrame, int endFrame, int startTransitionID, int endTransitionID, BVHMotion transition) {
     {
        bool startExist = false;
        bool endExist = false;

        Edge newTransition;
        Node startN;
        Node endN;

        startN.frame = startFrame;
        startN.transitionID = startTransitionID;

        endN.frame = endFrame;
        endN.transitionID = endTransitionID;

        newTransition.b = transition;


       for (int i = 0; i < this->nodes.size(); i++) {

            // If same frame in same transition occurs? = node already exists just add an outgoing edge to the node
            if(nodes[i].frame == startN.frame && nodes[i].transitionID == startN.transitionID) {

                for(int j =  0; j < this->nodes[i].outgoingEdges.size(); j++) {
                    //An edge points to the frame and transition Id of end node
                    if(nodes[i].outgoingEdges[j].frame == endN.frame && nodes[i].outgoingEdges[j].transitionID == endN.transitionID) {
                       return;
                    }
                }

                    newTransition.frame = endN.frame;
                    newTransition.transitionID = endN.transitionID;
                    this->nodes[i].outgoingEdges.push_back(newTransition);
                    startExist = true;
                    break;
            }
       }

       for (int i = 0; i < this->nodes.size(); i++) {
            //for (Edge tempE : nodes[i].outgoingEdges) {
                if(nodes[i].frame == endN.frame && nodes[i].transitionID == endN.transitionID) {
                    endExist = true;
                    break;
                }
            //}
       }


        /**
         * src represents the ID of bvh motion 
         * To access bvhList[i].motionData[k][j] will be a list
         * jth joint's channel-movement in kth frame in ith motion   
        */

       Edge tempEdge2;


       // Break down one transition into two

        if(!startExist) {
            for (int i = 0 ; i < this->nodes.size(); i++) {
                //cout << j << endl;
                            //                 cout << "J: " << endl;
                            // cout << j << endl;
                    // Iterate through every node
                    Node n = this->nodes[i];
                    //     cout << "HOW MANY" << endl;
                    //     cout << n.outgoingEdges.size() << endl;
                    // for(int a = 0; a < n.outgoingEdges.size(); a++) {
                    //     cout << "TRANSITION & FRAME " << endl;
                    //     cout << n.outgoingEdges[a].transitionID << endl;
                    //     cout << n.outgoingEdges[a].frame << endl;
                    // }
                    //if n.
                    //cout << nodes.size() << endl;
                        for( int j = 0; j < n.outgoingEdges.size(); j++) {
                            // cout << "K: " << endl;
                            // cout << k << endl;
                            //cout << n.outgoingEdges.size() << endl;

                            // Iterate through every outgoing edge from that node
                            Edge e = n.outgoingEdges[j];

                            // If the node's transition ID is same as a start node of new transition?
                            if(n.transitionID == startN.transitionID) {

                            // cout << "TRANSITION ID: " << endl;
                            // cout << n.transitionID << endl;
                            //int newEndFrame = e.frame;
                                // n < startN < e.frame = endFrame?
                                if(n.frame < startN.frame && startN.frame < e.frame) {
                                    //int newEndFrame = e.frame;



                                    cout << "CUT IT YO!" << endl;
                                    cout << n.frame << endl;
                                    cout << startN.frame << endl;
                                    cout << e.frame << endl;







                                Edge tempEdge;

                                // new edge = new transition has a start node of new transition and transition
                                // N ----------> startN
                                tempEdge.frame = startN.frame;
                                tempEdge.transitionID = startN.transitionID;
                                
                                BVHMotion tempB;

                                
                                // n --------- startN --------- end
                                int numFrameNewTransition = startN.frame - n.frame;


                                tempB.frameTime = e.b.frameTime;
                                tempB.joints = e.b.joints;

                                BVHMotion tempB2;

                                 cout << "START" << endl;
                                // Motion Data 
                                // BREAK DOWN the motions n-------------startN

                                cout << "THE CURRENT EDGE HAS THIS MUCH: " << endl;
                                cout << e.b.motionData.size() << endl; // 1023 < 1053

                                for (unsigned int k = n.frame; k < startN.frame; k++) {
                                    tempB.motionData.push_back(e.b.motionData[k-n.frame]);
                                }

                                tempB.numFrames = tempB.motionData.size();
                                 cout << "SIZE1: " << tempB.motionData.size() << endl;

                                for (unsigned int k = startN.frame; k < e.frame;k++) {
                                    //cout << i << endl; // 1023
                                    //cout << e.b.motionData.size() << endl;
                                    tempB2.motionData.push_back(e.b.motionData[k - n.frame]);
                                }
                                
                                 cout << "SIZE2: " << tempB2.motionData.size() << endl;
                                tempEdge.b = tempB;


                                this->nodes[i].outgoingEdges[j].b = tempB;
                                this->nodes[i].outgoingEdges[j].transitionID = startN.transitionID;
                                this->nodes[i].outgoingEdges[j].frame = startN.frame;

                                //startN.incomingEdges.push_back(tempEdge);


                                ///////////////////////////////////////////////////////////////

                                Edge tempEdge2;

                                tempEdge2.frame =  e.frame;
                                tempEdge2.transitionID = n.transitionID;


                                tempB2.numFrames = tempB2.motionData.size();
                                tempB2.frameTime = e.b.frameTime;
                                tempB2.joints = e.b.joints;



                                tempEdge2.b = tempB2;

                                /// NEW TRANSITION
                                // Edge newTransitionEdge;

                                // newTransitionEdge = newTransition;

                                
                                startN.outgoingEdges.push_back(tempEdge2);
                                // startN.outgoingEdges.push_back(newTransitionEdge);


                                this->nodes.push_back(startN);




                                goto end_loops;
                                //n.outgoingEdges.
                                }
                            }
                    }



            }

        }

        end_loops:
        if(!endExist) {

            for (int i = 0 ; i < this->nodes.size(); i++) {
                    Node n = this->nodes[i];
                    //if n.
                    //cout << nodes.size() << endl;
                        for( int j = 0; j < n.outgoingEdges.size(); j++) {

                            //cout << n.outgoingEdges.size() << endl;
                            Edge e = this->nodes[i].outgoingEdges[j];

                            if(n.transitionID == endN.transitionID) {

                                if(n.frame < endN.frame && endN.frame < e.frame) {

                                    cout << "CUT IT YO!" << endl;
                                    cout << n.frame << endl;
                                    cout << endN.frame << endl;
                                    cout << e.frame << endl;

                                    Edge tempEdge;


                                    // new edge = new transition has a start node of new transition and transition
                                    tempEdge.frame = endN.frame;
                                    tempEdge.transitionID = n.transitionID;
                                    cout << "END" << endl;

                                    // cout << "THE CURRENT EDGE HAS THIS MUCH: " << endl;
                                    // cout << e.b.motionData.size() << endl; // 1023 < 1053


                                    BVHMotion tempB;

                                    
                                    // n --------- endN --------- end
                                    int numFrameNewTransition = endN.frame - n.frame;

                                    //tempB.numFrames = numFrameNewTransition;
                                    tempB.frameTime = e.b.frameTime;
                                    tempB.joints = e.b.joints;


                                    // Motion Data 
                                    // BREAK DOWN the motions n-------------startN
                                    for (unsigned int k = n.frame; k < endN.frame; k++) {
                                        tempB.motionData.push_back(e.b.motionData[k - n.frame]);
                                    }
                                    tempB.numFrames = tempB.motionData.size();
                                    cout << "SIZE1: " << tempB.motionData.size() << endl;
                                    // cout << numFrameNewTransition << endl;
                                    
                                    //tempEdge.b = tempB;


                                    this->nodes[i].outgoingEdges[j].b = tempB;
                                    this->nodes[i].outgoingEdges[j].transitionID = endN.transitionID;
                                    this->nodes[i].outgoingEdges[j].frame = endN.frame;
                                    //startN.incomingEdges.push_back(tempEdge);


                                    ///////////////////////////////////////////////////////////////
                                        Edge tempEdge2;

                                        tempEdge2.frame = n.frame + tempB.motionData.size();
                                        tempEdge2.transitionID = endN.transitionID;

                                        BVHMotion tempB2;

                                        int numFrameNewEnd = e.frame - endN.frame;


                                        tempB2.frameTime = e.b.frameTime;
                                        tempB2.joints = e.b.joints;



                                    // Motion Data 
                                    for (unsigned int k = endN.frame; k < e.frame; k++) {
                                        tempB2.motionData.push_back(e.b.motionData[k - n.frame]);
                                    }
                                        tempB2.numFrames = tempB2.motionData.size();
                                    cout << "SIZE2: " << tempB2.motionData.size() << endl;

                                    tempEdge2.b = tempB2;
                                    tempEdge2.frame = e.frame;
                                
                                    
                                    //cout << "ROROROROROO" << endl;
                                    //cout << tempEdge2.frame << endl;
                                    endN.outgoingEdges.push_back(tempEdge2);
                                    //cout << endN.frame << endl;
                                    this->nodes.push_back(endN);
                                    goto end_loops2;




                        }
                    }
                } 
            }
        }
        end_loops2:
        return;
    }
}

GLfloat MotionGraph::squarednormDifferences(glm::vec3 v1, glm::vec3 v2) {

    glm::vec3 result = v1 - v2;
    GLfloat squaredNorm = glm::dot(result, result);


    return squaredNorm;

}



/**
 * Measures position difference of a root of two frames 
*/
GLfloat MotionGraph::differenceInRoots(vector<vector<GLfloat>> frames1, vector<vector<GLfloat>> frames2, vector<Joint> joints, int pIndex, int tIndex, int rclIndex) {


    vector<GLfloat> centerFrame1 = frames1[frames1.size() / 2];
    glm::vec3 pelvisOffset = joints[pIndex].position;
    glm::vec3 thoraxOffset = joints[tIndex].position;
    glm::vec3 rclavicleOffset = joints[rclIndex].position;

    glm::vec3 translatePelvis;
    glm::vec3 rotatePelvis;
    glm::vec3 rotateThorax;
    glm::vec3 rotateRclavicle;

    //translatePelvis.x = centerFrame1[0];
    //translatePelvis.y = centerFrame1[1];
    //translatePelvis.z = centerFrame1[2];


    // 0, 1, 2 ? ... x, y, z ?
    for(int i = 0; i < 3; i++) {
        translatePelvis[i] = centerFrame1[i];
        rotatePelvis[i] = centerFrame1[4 + (pIndex * 3) + i];
        rotateThorax[i] = centerFrame1[4 + (tIndex * 3) + i];
        rotateRclavicle[i] = centerFrame1[4 + (rclIndex * 3) + i];
    }

    glm::vec3 rotationXAxis(1.0f, 0.0f, 0.0f);
    glm::vec3 rotationYAxis(0.0f, 1.0f, 0.0f);
    glm::vec3 rotationZAxis(0.0f, 0.0f, 1.0f);

    // Z X Y
    GLfloat angleX = rotatePelvis[1];
    GLfloat angleY = rotatePelvis[2];
    GLfloat angleZ = rotatePelvis[0];
    glm::mat4 rotateX = glm::rotate(glm::mat4(1.0f), glm::radians(angleX), rotationXAxis);
    glm::mat4 rotateY = glm::rotate(glm::mat4(1.0f), glm::radians(angleY), rotationYAxis);
    glm::mat4 rotateZ = glm::rotate(glm::mat4(1.0f), glm::radians(angleZ), rotationZAxis);
    glm::mat4 translationMatrix = glm::translate(glm::mat4(1.0f), pelvisOffset);

    glm::mat4 rotationMatrix = rotateX * rotateY * rotateZ;

    glm::mat4 transformPelvis = translationMatrix * rotationMatrix;
    
    
    glm::vec4 posPelvis = translationMatrix * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);


    angleX = rotateThorax[1];
    angleY = rotateThorax[2];
    angleZ = rotateThorax[0];
    rotateX = glm::rotate(glm::mat4(1.0f), glm::radians(angleX), rotationXAxis);
    rotateY = glm::rotate(glm::mat4(1.0f), glm::radians(angleY), rotationYAxis);
    rotateZ = glm::rotate(glm::mat4(1.0f), glm::radians(angleZ), rotationZAxis);
    translationMatrix = glm::translate(glm::mat4(1.0f), thoraxOffset);
    
    rotationMatrix = rotateX * rotateY * rotateZ;

    glm::mat4 transformThorax = translationMatrix * rotationMatrix;
    
    glm::vec4 posThorax = transformPelvis * translationMatrix * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);

    glm::vec3 posThoraxThree = posThorax;

    angleX = rotateRclavicle[1];
    angleY = rotateRclavicle[2];
    angleZ = rotateRclavicle[0];
    rotateX = glm::rotate(glm::mat4(1.0f), glm::radians(angleX), rotationXAxis);
    rotateY = glm::rotate(glm::mat4(1.0f), glm::radians(angleY), rotationYAxis);
    rotateZ = glm::rotate(glm::mat4(1.0f), glm::radians(angleZ), rotationZAxis);
    translationMatrix = glm::translate(glm::mat4(1.0f), rclavicleOffset);
    
    rotationMatrix = rotateX * rotateY * rotateZ;

    //glm::mat4 transformRcl = translationMatrix * rotationMatrix;
    
    glm::vec4 posRcl = transformThorax * translationMatrix * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);


    glm::vec3 shoulderVec = posRcl - posThorax;
    
    glm::vec3 thorax = posThorax - posPelvis;

    glm::vec3 forwardVec = glm::normalize(glm::cross(shoulderVec, thorax));
    //cout << forwardVec.x << endl;
    glm::vec3 rightVector = glm::normalize(glm::cross(forwardVec, glm::vec3(0.0f, 1.0f, 0.0f)));
    glm::vec3 newUp = glm::normalize(glm::cross(forwardVec, rightVector));

    glm::mat4 newTransform = glm::lookAt(posThoraxThree, forwardVec, newUp);

    float sumOffsetInFrame1 = 0.0f;

    for(int i = 0; i < frames1.size(); i++) {
        
        
        glm::vec3 smallVec(frames1[i][0], frames1[i][1], frames1[i][2]);
        glm::vec3 result = newTransform * glm::vec4(smallVec, 1.0f);
        sumOffsetInFrame1 += glm::length(result);
    
    }



    //Calculate

    vector<GLfloat> centerFrame2 = frames2[frames2.size() / 2];

    for(int i = 0; i < 3; i++) {
        translatePelvis[i] = centerFrame2[i];
        rotatePelvis[i] = centerFrame2[4 + (pIndex * 3) + i];
        rotateThorax[i] = centerFrame2[4 + (tIndex * 3) + i];
        rotateRclavicle[i] = centerFrame2[4 + (rclIndex * 3) + i];
    }

    // Z X Y
    angleX = rotatePelvis[1];
    angleY = rotatePelvis[2];
    angleZ = rotatePelvis[0];
    rotateX = glm::rotate(glm::mat4(1.0f), glm::radians(angleX), rotationXAxis);
    rotateY = glm::rotate(glm::mat4(1.0f), glm::radians(angleY), rotationYAxis);
    rotateZ = glm::rotate(glm::mat4(1.0f), glm::radians(angleZ), rotationZAxis);
    translationMatrix = glm::translate(glm::mat4(1.0f), pelvisOffset);

    rotationMatrix = rotateX * rotateY * rotateZ;

    transformPelvis = translationMatrix * rotationMatrix;
    
    
    posPelvis = translationMatrix * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);


    angleX = rotateThorax[1];
    angleY = rotateThorax[2];
    angleZ = rotateThorax[0];
    rotateX = glm::rotate(glm::mat4(1.0f), glm::radians(angleX), rotationXAxis);
    rotateY = glm::rotate(glm::mat4(1.0f), glm::radians(angleY), rotationYAxis);
    rotateZ = glm::rotate(glm::mat4(1.0f), glm::radians(angleZ), rotationZAxis);
    translationMatrix = glm::translate(glm::mat4(1.0f), thoraxOffset);
    
    rotationMatrix = rotateX * rotateY * rotateZ;

    transformThorax = translationMatrix * rotationMatrix;
    
    posThorax = transformPelvis * translationMatrix * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);

    posThoraxThree = posThorax;

    angleX = rotateRclavicle[1];
    angleY = rotateRclavicle[2];
    angleZ = rotateRclavicle[0];
    rotateX = glm::rotate(glm::mat4(1.0f), glm::radians(angleX), rotationXAxis);
    rotateY = glm::rotate(glm::mat4(1.0f), glm::radians(angleY), rotationYAxis);
    rotateZ = glm::rotate(glm::mat4(1.0f), glm::radians(angleZ), rotationZAxis);
    translationMatrix = glm::translate(glm::mat4(1.0f), rclavicleOffset);
    
    rotationMatrix = rotateX * rotateY * rotateZ;

    //glm::mat4 transformRcl = translationMatrix * rotationMatrix;
    
    posRcl = transformThorax * translationMatrix * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);


    shoulderVec = posRcl - posThorax;
    
    thorax = posThorax - posPelvis;

    forwardVec = glm::normalize(glm::cross(shoulderVec, thorax));
    //cout << forwardVec.x << endl;
    rightVector = glm::normalize(glm::cross(forwardVec, glm::vec3(0.0f, 1.0f, 0.0f)));
    newUp = glm::normalize(glm::cross(forwardVec, rightVector));

    newTransform = glm::lookAt(posThoraxThree, forwardVec, newUp);

    float sumOffsetInFrame2 = 0.0f;

    for(int i = 0; i < frames2.size(); i++) {
        
        
        glm::vec3 smallVec(frames2[i][0], frames2[i][1], frames2[i][2]);
        glm::vec3 result = newTransform * glm::vec4(smallVec, 1.0f);
        sumOffsetInFrame2 += glm::length(result);
    
    }

    //return squarednormDifferences(root1, root2);
    cout << pow(sumOffsetInFrame1 - sumOffsetInFrame2,2)/ 10000 << endl;
    return pow(sumOffsetInFrame1 - sumOffsetInFrame2, 2)/ 10000;
}



/**
 * Measures difference in a 
 * ort1 has x, y, z angle respectfully
 * QUESTION!!!!!!!!!!!!
*/
GLfloat MotionGraph::differenceInOrienation(glm::vec3 ort1, glm::vec3 ort2) {

    float angleX_radians = glm::radians(ort1.y);
    float angleY_radians = glm::radians(ort1.z);
    float angleZ_radians = glm::radians(ort1.x);

// cout << "ORIETNATION 1: " << endl;
//     cout << ort1.x << endl;
//         cout << ort1.y << endl;
//             cout << ort1.z << endl;

    float angleX_radians2 = glm::radians(ort2.y);
    float angleY_radians2 = glm::radians(ort2.z);
    float angleZ_radians2 = glm::radians(ort2.x);


// cout << "ORIENTATION 2: " << endl;
//     cout << ort2.x << endl;
//         cout << ort2.y << endl;
//             cout << ort2.z << endl;

    glm::vec3 xUnit(1.0f, 0.0f, 0.0f);
    glm::vec3 yUnit(0.0f, 1.0f, 0.0f);
    glm::vec3 zUnit(0.0f, 0.0f, 1.0f);


    glm::quat quatX = glm::normalize(angleAxis(angleX_radians, xUnit));
    glm::quat quatY = glm::normalize(angleAxis(angleY_radians, yUnit));
    glm::quat quatZ = glm::normalize(angleAxis(angleZ_radians, zUnit));

    glm::quat quatX2 = glm::normalize(angleAxis(angleX_radians2, xUnit));
    glm::quat quatY2 = glm::normalize(angleAxis(angleY_radians2, yUnit));
    glm::quat quatZ2 = glm::normalize(angleAxis(angleZ_radians2, zUnit));

    glm::quat quat1 = glm::normalize(quatZ * quatY * quatX);
    glm::quat quat2 = glm::normalize(quatZ2 * quatY2 * quatX2);

    //cout << "RESULT?: " << endl;
    //cout << std::log(std::abs(glm::dot(glm::inverse(quat1), quat2)))  << endl;

    /**
     * log keeps returning negative numbers
     * ORIENTATIONS ARE SAMMEEEE? IS IT POSSIBLE
     * 
     * SOLUTION just negate one of them (negating doesn't change its orientation)
     * 
    */

    GLfloat dotQuatProduct = glm::dot(glm::inverse(quat1), quat2);

    if(dotQuatProduct < 0) {
        dotQuatProduct = glm::dot(glm::inverse(-quat1), quat2);

    } else {
        return std::log(dotQuatProduct);
    }

    return std::log(dotQuatProduct);
    
}



GLfloat MotionGraph::computeAngularVelocity(vector<GLfloat> frame1, vector<GLfloat> frame2, float frameTime1, float frameTime2) {
    float sum = 0.0f;
    glm::vec3 ort1;
    glm::vec3 ort2;
    for(int i = 6; i < frame1.size(); (i = i + 3)) {
        ort1.x = frame1[i];
        ort1.y = frame1[i+1];
        ort1.z = frame1[i+2];


        ort2.x = frame2[i];
        ort2.y = frame2[i+1];
        ort2.z = frame2[i+2];


        sum += differenceInOrienation(ort1, ort2);
    }
    return sum;
}



/**
 * Sum the difference in each joint's orientation
*/
GLfloat MotionGraph::sumDifferenceInOrientation(vector<GLfloat> frame1, vector<GLfloat> frame2) {
    float sum = 0.0f;
    glm::vec3 ort1;
    glm::vec3 ort2;
    unsigned int i = 6;
    for(i = 6; i < frame1.size(); (i = i + 3)) {
        ort1.x = frame1[i];
        ort1.y = frame1[i+1];
        ort1.z = frame1[i+2];





        ort2.x = frame2[i];
        ort2.y = frame2[i+1];
        ort2.z = frame2[i+2];





        sum += differenceInOrienation(ort1, ort2);

    }
    //cout << "SUM: " << endl;
    //cout << sum << endl;
    return sum;
}

