
#include <cmath>
#include <vector>
#include <list>
#include <iostream>
#include <cmath>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/quaternion.hpp>
#include <random>
#include <unordered_map>
#include "parse_bvh.h"
#include "bvh_motion.h"
#include "joint.h"

using namespace std;

/**
 * HAVE TO GENERATE A NEW TRANSITION WITH A NEW MOTION DATA 
 * 1. GENERATE A SET OF NODES WITH INITIAL CLIPS
 * 2. COMPARE EACH FRAME BY FRAME USING A WINDOW
 * 2. MOTION DATA = FILL IT UP WITH LERP and SLERP
 * 4. PRUNE A GRAPH SO THAT IT ONLY HAS A STRONG CONNECTED COMPONENT
 * 
*/

/**
 * User-specified threshold value to prune unnecessary nodes
*/
const float THRESHOLD_VALUE = 3.0f;



/*
    Transitions
*/
struct Edge {

    /*
        Which frame of the motion should I get?
        src = start of the frame of which motion
        dest =
    */

   // IF it is incoming (from where?)
   // If it is outgoing (where it is going to)
   int frame;
   int transitionID;


    /**
     * BVH parsed motion
    */
    BVHMotion b;

};

struct Node {
    int frame;
    int transitionID;
    //vector<Edge> incomingEdges;
    vector<Edge> outgoingEdges;


};


/*
    Choice Point
    Node (motion-id = 0, ...., n ) initial motions
    This from node to node 0 to 1 (one motion)
                           2 to 3
                           4 to 5
                           6 to 7
                           7 to 8 (2N disconnected nodes)
*/



/*
    Motion Graph Structure
    with Adjacency List that has a source to destination
    


*/
class MotionGraph {
public:


    void initialize(int startFrame, int startTransitionID, int endTransitionID, BVHMotion transition) {
        bool startExist = false;
        bool endExist = false;

        Edge e;
        Node startN;
        Node endN;

        startN.frame = startFrame;
        startN.transitionID = startTransitionID;

        endN.frame = startFrame + transition.numFrames;
        endN.transitionID = endTransitionID;

        e.b = transition;
        /**
         * src represents the ID of bvh motion 
         * To access bvhList[i].motionData[k][j] will be a list
         * jth joint's channel-movement in kth frame in ith motion   
        */


       // Break down one transition into two



       for (int i = 0; i < nodes.size(); i++) {

            // If same frame in same transition occurs? = node already exists just add an outgoing edge to the node
            if(nodes[i].frame == startN.frame && nodes[i].transitionID == startTransitionID) {
                    e.frame = endN.frame;
                    e.transitionID = endN.transitionID;
                    nodes[i].outgoingEdges.push_back(e);
                    startExist = true;
            }
            
        
            // if(nodes[i].frame == endN.frame && nodes[i].transitionID == endTransitionID) {
            //         e.frame = startN.frame;
            //         e.transitionID = startN.transitionID;
            //         nodes[i].incomingEdges.push_back(e);
            //         endExist = true;
                
            // }
       }

       if(!startExist) {
                            e.frame = endN.frame;
                    e.transitionID = endN.transitionID;
            startN.outgoingEdges.push_back(e);
            nodes.push_back(startN);
       } 

    //    if(!endExist) {
    //                         e.frame = startN.frame;
    //                 e.transitionID = startN.transitionID;
    //         startN.incomingEdges.push_back(e);
    //         nodes.push_back(endN);
    //    }
    }

    void addEdge (int startFrame, int endFrame, int startTransitionID, int endTransitionID, BVHMotion transition);


/**
 * Calculates a squared norm of difference between two vectors
*/
GLfloat squarednormDifferences(glm::vec3 v1, glm::vec3 v2);


/**
 * Measures position difference of a root of two frames 
*/
GLfloat differenceInRoots(vector<vector<GLfloat>> frames1, vector<vector<GLfloat>> frames2, vector<Joint> joints, int pIndex, int tIndex, int rclIndex);


/**
 * Measures difference in a 
 * ort1 has x, y, z angle respectfully
*/
GLfloat differenceInOrienation(glm::vec3 ort1, glm::vec3 ort);

GLfloat computeAngularVelocity(vector<GLfloat> frame1, vector<GLfloat> frame2, float frameTime1, float frameTime2);

/**
 * Sum the difference in each joint's orientation
*/
GLfloat sumDifferenceInOrientation(vector<GLfloat> frame1, vector<GLfloat> frame2);

    glm::quat changeToQuaternion(glm::vec3 ort) {
        glm::quat xQuat = glm::angleAxis(glm::radians(ort.x), glm::vec3(1.0f, 0.0f, 0.0f));
        glm::quat yQuat = glm::angleAxis(glm::radians(ort.y), glm::vec3(0.0f, 1.0f, 0.0f));
        glm::quat zQuat = glm::angleAxis(glm::radians(ort.z), glm::vec3(0.0f, 0.0f, 1.0f));

        glm::quat quaternion = xQuat * yQuat * zQuat;
        
        return quaternion;
    }

    glm::vec3 quaternionToEuler(const glm::quat& quaternion) {
        glm::vec3 euler;

        euler.x = glm::pitch(quaternion);
        euler.y = glm::yaw(quaternion);
        euler.z = glm::roll(quaternion);

        euler = glm::degrees(euler);

        return euler;
    }

    glm::vec3 slerpEquation(glm::vec3 ort1, glm::vec3 ort2, float p) {
        glm::quat quat1 = changeToQuaternion(ort1);
        glm::quat quat2 = changeToQuaternion(ort2);
        glm::quat result = glm::slerp(quat1, quat2, blendingWeights(p));

        glm::vec3 resultEuler = quaternionToEuler(result);
        return resultEuler;
    }

    glm::vec3 lerpEquation(glm::vec3 root1, glm::vec3 root2, int iFrame ) {
        glm::vec3 newPosition;
        newPosition.x = blendingWeights(iFrame) * root1.x + (1 - blendingWeights(iFrame)) * root2.x;
        newPosition.y = blendingWeights(iFrame) * root1.y + (1 - blendingWeights(iFrame)) * root2.y;
        newPosition.z = blendingWeights(iFrame) * root1.z + (1 - blendingWeights(iFrame)) * root2.z;
        return newPosition;
    }
    
    GLfloat blendingWeights(int p1) {
        // Blending equation
        return (2 * pow((p1 + 1.0f)/ WINDOW_SIZE, 3)) - (3 * pow((p1 + 1.0f)/ WINDOW_SIZE, 2)) + 1;
    }

/**
 * Motion data[i] -> ith frame
 * Each BVH -> motion
 * BVHModel[i].motionData[i]
 * 
 * 
*/
    void createNewTransition() {
        int counter = 0;
        vector<Node>* tempNodes = new std::vector<Node>();
        vector<Node>* tempNodes2 = new std::vector<Node>();
        int a, c = 0;
        
        for (int i = 0; i < nodes.size(); i++) {
            tempNodes->push_back(nodes[i]);
            tempNodes2->push_back(nodes[i]);
            cout << i << endl;
            //cout << tempNodes[i].transitionID << endl;
        }
        /*
            Compare K frames
        */




        int startTransition = 0;
        int endTransition = 0;


        int pelvisIndex, thoraxIndex, rclavicleIndex;



        //cout << nodes.size() << endl;
        for (int a = 0; a < tempNodes->size(); a++) {
    
            for(int b = 0; b < tempNodes->at(a).outgoingEdges.size(); b++) {
                Edge motion = tempNodes->at(a).outgoingEdges[b];


                            for(int m = 0; m < motion.b.joints.size(); m++) {
                                Joint j = motion.b.joints[m];
                                if(j.jointName.substr(0,6).compare("pelvis") == 0) {
                                    pelvisIndex = m; 
                                } 
                                if(j.jointName.substr(0,6).compare("thorax") == 0) {
                                    thoraxIndex = m;
                                                                        
                                }
                                if(j.jointName.substr(0,9).compare("rclavicle") == 0) {

                                    rclavicleIndex = m;
                                }
                            }

                for (int c = 0; c < tempNodes2->size(); c++) {

                    for(int l = 0; l < tempNodes2->at(c).outgoingEdges.size(); l++) {
                        Edge motion2 = tempNodes2->at(c).outgoingEdges[l];
                        startTransition = a;
                        endTransition = c;

                            vector<vector<GLfloat>> windowFrames1;
                            vector<vector<GLfloat>> windowFrames2;


                            GLfloat avgDiffOrientation = 0.0f;
                            GLfloat avgPosDiffInRoots = 0.0f;
                            GLfloat avgAngularVelocity = 0.0f;
                            float distance;

                            int sizeToLoop = 0;

                            // if(motion.b.motionData.size() > motion2.b.motionData.size()) {
                            //     sizeToLoop = motion2.b.motionData.size();
                            // } else {
                            //     sizeToLoop = motion.b.motionData.size();
                            // }

                            /**
                             * Compare two frames
                             * 
                            */
                            for (unsigned int i = 0; i < motion.b.motionData.size() - WINDOW_SIZE; i++) {
                                //cout << i << endl;
                                for (unsigned int j = WINDOW_SIZE; j < motion2.b.motionData.size(); j++) {

                                    windowFrames1.clear();
                                    windowFrames2.clear();

                                    /*
                                        ADD LIST to 
                                        CHECK if
                                        SLERP and LERP
                                    
                                    */

                                    //Sum it all up
                                    for(unsigned int k = 0; k < WINDOW_SIZE; k++) {
                                        windowFrames1.push_back(motion.b.motionData[i+k]);
                                        windowFrames2.push_back(motion2.b.motionData[(j- WINDOW_SIZE) + k]);
                                        avgDiffOrientation += sumDifferenceInOrientation(motion.b.motionData[i+k], motion2.b.motionData[(j - WINDOW_SIZE) + k]);
                                    }
                                    //cout << pow(avgDiffOrientation,2)/10000000 << endl;
                                    distance = pow(avgDiffOrientation,2)/10000000;
                                    
                                        /*
                                        
                                            TRAPPED IN A LOOP
                                        */
                                        //cout << "QUATERNION DIFFERENCE: " << sumDifferenceInOrientation(windowFrame1, windowFrame2) << endl;
                                        // cout << "POSITION DIFFERENCE IN ROOT" << differenceInRoots(windowFrame1, windowFrame2) << endl;
                                        // cout << "ANGULAR VELOCITY: " << computeAngularVelocity(windowFrame1, windowFrame2, motion.b.frameTime, motion2.b.frameTime) << endl;



                                    // avgDiffOrientation += pow(sumDifferenceInOrientation(windowFrame1, windowFrame2),2);
                                    // cout << 

                                    //avgPosDiffInRoots += pow(differenceInRoots(windowFrames1, windowFrames2, motion.b.joints, pelvisIndex, thoraxIndex, rclavicleIndex),2);
                                    
                                    //distance = differenceInRoots(windowFrames1, windowFrames2,  motion.b.joints, pelvisIndex, thoraxIndex, rclavicleIndex);
                                    //distance = 0;
                                    
                                    // distance = avgPosDiffInRoots;
                                    //     //avgAngularVelocity += pow(computeAngularVelocity(windowFrame1, windowFrame2, motion.b.frameTime, motion2.b.frameTime),2);
                                    //     //this->addEdge();
                                    

                                    //                                    //cout << "WEQEWQWE" << endl;
                                    // //avgDiffOrientation /= WINDOW_SIZE;
                                    // //avgPosDiffInRoots /= WINDOW_SIZE;
                                    // //avgAngularVelocity /= WINDOW_SIZE;

                                    // //cout << avgDiffOrientation << endl;
                                    // //cout << avgPosDiffInRoots << endl;
                                    // //cout << avgAngularVelocity << endl;

                                
                                    // //GLfloat distance = avgDiffOrientation; //+ avgPosDiffInRoots + avgAngularVelocity;
                                    
                                    // //cout << distance << endl;


                                    if (0 < distance < this->THRESHOLD_VALUE) {  
                                        cout << distance << endl;
                                        //vector<BVHMotion> newTransitions;
                                        BVHMotion newTransition;
                                        vector<GLfloat> windowf1;
                                        vector<GLfloat> windowf2;

                                        
                                        //cout << distance << endl;
                                        //counter++;
                                        newTransition.frameTime = motion.b.frameTime;
                                        newTransition.numFrames = WINDOW_SIZE;


                                        /*
                                         * PUT MOTION DATA 
                                        */
                                        vector<vector<GLfloat>> newMotionData;
                                        // glm::vec3 mainWindowRoot1(windowFrames1[0][0], windowFrames1[0][1], windowFrames1[0][2]);

                                        // glm::vec3 mainWindowRoot2(windowFrames2[1][0], windowFrames2[1][1], windowFrames2[1][2]);


                                        for (int k = 0; k < WINDOW_SIZE; k++) {

                                            


                                            windowf1 = windowFrames1[k];
                                            windowf2 = windowFrames2[k];

                                            glm::vec3 rootPos1(windowf1[0], windowf1[1], windowf1[2]);
                                            glm::vec3 rootPos2(windowf2[0], windowf2[1], windowf2[2]);
                                            glm::vec3 newRootPos = lerpEquation(rootPos1, rootPos2, k);
    
                                            // glm::vec3 resultPos1(mainWindowRoot1 - rootPos1);
                                            // glm::vec3 resultPos2(mainWindowRoot2 - rootPos2);

                                            // glm::vec3 lerpResult = lerpEquation(resultPos1, resultPos2, k);
                                            // ADD three joint information first
                                            vector<GLfloat> newFrame;
                                            newFrame.clear();
                                            newFrame.push_back(newRootPos.x);
                                            newFrame.push_back(newRootPos.y);
                                            newFrame.push_back(newRootPos.z);

                                            // newFrame.push_back(windowf2[0]);
                                            // newFrame.push_back(windowf2[1]);
                                            // newFrame.push_back(windowf2[2]);

                                             for (int q = 4; q < windowf1.size(); q = q + 3) {

                                            
                                                glm::vec3 angle1(windowf1[q+2], windowf1[q], windowf1[q+1]);
                                                // newFrame.push_back(windowf2[q+2]);
                                                // newFrame.push_back(windowf2[q]);
                                                // newFrame.push_back(windowf2[q+1]);
                                                glm::vec3 angle2(windowf2[q+2], windowf2[q], windowf2[q+1]);
                                                glm::vec3 interpolatedAngle = lerpEquation(angle1, angle2, k);


                                                //  //glm::vec3 interpolatedAngle = slerpEquation(angle1, angle2, k);
                                                newFrame.push_back(interpolatedAngle.x);
                                                newFrame.push_back(interpolatedAngle.y);
                                                newFrame.push_back(interpolatedAngle.z);
                                             }


                                            //for (int )
                                            // for (int q = 4; q < windowf1.size(); q = q + 3) {
                                            //     glm::vec3 angle1(windowf1[q+1], windowf1[q+2], windowf1[q]);
                                            //     glm::vec3 angle2(windowf2[q+1], windowf2[q+2], windowf2[q]);
                                            //     glm::vec3 interpolatedAngle = slerpEquation(angle1, angle2, k);
                                            //     newFrame.push_back(interpolatedAngle.z);
                                            //     newFrame.push_back(interpolatedAngle.x);
                                            //     newFrame.push_back(interpolatedAngle.y);
                                            // }



                                            // NEW FRAME
                                            newMotionData.push_back(newFrame);
                                        }
                                        newTransition.motionData = newMotionData;
                                        newTransition.joints = motion.b.joints;
                                        newTransition.numFrames = WINDOW_SIZE;
                                        newTransition.frameTime = motion.b.frameTime;
                                        //newTransitions.push_back(newTransition);

                                        //cout << "YAYAYAYAY" << endl;
                                        cout << startTransition << endl;
                                        cout << endTransition << endl;
                                        addEdge(i, j-WINDOW_SIZE, startTransition, endTransition, newTransition);
                                        
                                        //cout <<  "counter" << endl;
                                        //cout << counter << endl;
                                    }

                                }


                       
                        }
                    }

                }
            }
        }
        // for (int i = 0; i < newTransitions.size(); i++) {
        //     addEdge(0, 0, newTransitions[i]);
        // }

        //cout << counter << endl;
        //delete tempNodes;
        //delete tempNodes2;
    }


    // Takes an initial node to start with
    BVHMotion traverseOneTime(int startNode) {


        Edge e;

                Edge resultEdge;
        std::uniform_int_distribution<int> nodeDistribution(0, this->nodes.size() - 1);

        std::random_device rd;
        std::mt19937 gen(rd());


        if(this->firstTime) {


            std::uniform_int_distribution<int> distribution(0, this->nodes[startNode].outgoingEdges.size() - 1);
            e = this->nodes[startNode].outgoingEdges[distribution(gen)];
            cout << "START" << endl;



        for (int i = 0; i < this->nodes.size(); i++) {
            if (this->nodes[i].frame == e.frame && this->nodes[i].transitionID == e.transitionID) {

                this->nextNode = this->nodes[i];
                if(this->nextNode.outgoingEdges.size() == 0) {

                do{
                    this->nextNode = this->nodes[nodeDistribution(gen)];
                }while(this->nextNode.outgoingEdges.size() == 0);

                }
                this->firstTime = 0;

                    currentTransformation.x = e.b.motionData[e.b.numFrames - 1][0];
                    currentTransformation.y = e.b.motionData[e.b.numFrames - 1][1];
                    currentTransformation.z = e.b.motionData[e.b.numFrames - 1][2];

                    for(int j = 4; j < e.b.motionData[0].size(); j++) {
                        currentJointOrientation.push_back(e.b.motionData[e.b.motionData.size() - 1][j]);
                    }
                return e.b;
            }
        }
        } else {
            cout << "TRAVERSE" << endl;



        //if(this->nextNode.outgoingEdges.size() > 1) {
        //    std::uniform_int_distribution<int> distribution(0, this->nextNode.outgoingEdges.size() - 1);
        //    e = this->nextNode.outgoingEdges[distribution(gen)];
        //} else {
            if(this->nextNode.outgoingEdges.size() == 0) {

                cout << "DEADLOCK" << endl;
                do{
                    this->nextNode = this->nodes[nodeDistribution(gen)];
                }while(this->nextNode.outgoingEdges.size() == 0);

                std::uniform_int_distribution<int> edgeDistribution(0, this->nextNode.outgoingEdges.size() - 1);
                e = this->nextNode.outgoingEdges[edgeDistribution(gen)];
                // cout << "NEXT NODE INFORMATION: " << endl;
                // cout << "Translation ID: " << endl;
                // cout << this->nextNode.transitionID << endl;
                // cout << "Frame No. :" << endl;
                // cout << this->nextNode.frame << endl;
                
            } else {
                cout << "GO" << endl;
                cout << "CURRENT POSITION" << endl;
                cout << this->nextNode.transitionID << endl;
                cout << this->nextNode.frame << endl;
                cout << endl;
                std::uniform_int_distribution<int> edgeDistribution(0, this->nextNode.outgoingEdges.size() - 1);
                cout << "THE LIST TO CONSIDER" << endl;
                for(Edge e : this->nextNode.outgoingEdges) {
                    // cout << "TRANSITION ID: " << endl;
                    // cout << e.transitionID << endl;
                    // cout << "FRAME?: " << endl;
                    // cout << e.frame << endl;
                    // cout << endl;
                }
                int num = edgeDistribution(gen);
                cout << "RANDOM NUMBER: " << endl;
                cout << num << endl;
                e = this->nextNode.outgoingEdges[edgeDistribution(gen)];



                for (int i = 0; i < this->nodes.size(); i++) {
                    if (this->nodes[i].frame == e.frame && this->nodes[i].transitionID == e.transitionID) {
                        nextNode = this->nodes[i];
                        resultEdge.frame = e.frame;
                        resultEdge.transitionID = e.transitionID;
                        resultEdge.b = e.b;


                        
                        GLfloat originX = resultEdge.b.motionData[0][0];
                        GLfloat originY = resultEdge.b.motionData[0][1];
                        GLfloat originZ = resultEdge.b.motionData[0][2];

                        for(int j = 0; j < resultEdge.b.motionData.size(); j++) {
                            resultEdge.b.motionData[j][0] = (resultEdge.b.motionData[j][0] - originX) + currentTransformation.x;
                            resultEdge.b.motionData[j][1] = (resultEdge.b.motionData[j][1] - originY) + currentTransformation.y;
                            resultEdge.b.motionData[j][2] = (resultEdge.b.motionData[j][2] - originZ) + currentTransformation.z;
                        }

                        for(int j = 0; j < resultEdge.b.motionData.size(); j++) {
                            vector<GLfloat> initialJointOrt = resultEdge.b.motionData[0];
                            for (int k = 4; k < resultEdge.b.motionData[j].size(); k++) {
                                resultEdge.b.motionData[j][k] = (resultEdge.b.motionData[j][k] - initialJointOrt[k]) + currentJointOrientation[k-4];
                            }
                        }

                        for(int j = 4; j < resultEdge.b.motionData[0].size(); j++) {
                            currentJointOrientation[j-4] += (resultEdge.b.motionData[resultEdge.b.motionData.size() - 1][j] - resultEdge.b.motionData[0][j]);
                        }
                        currentTransformation.x += (resultEdge.b.motionData[resultEdge.b.numFrames - 1][0] - resultEdge.b.motionData[0][0]);
                        currentTransformation.y += (resultEdge.b.motionData[resultEdge.b.numFrames - 1][1] - resultEdge.b.motionData[0][1]);
                        currentTransformation.z += (resultEdge.b.motionData[resultEdge.b.numFrames - 1][2] - resultEdge.b.motionData[0][2]);

                        return resultEdge.b;
                    }
                }
                cout << "CAN't FIND" << endl;
            }
        //}
        }
        return resultEdge.b;
    }


    void printGraph() const {

        for (Node n : nodes) {
            cout << endl;
            cout <<"--------------------------" << endl;
            cout << "node ith frame: " << n.frame << endl;
            cout << "node transition ID: " << n.transitionID << endl;
            cout << n.outgoingEdges.size() << endl;
            for (Edge e : n.outgoingEdges) {
                cout << "-----EDGE-----" << endl;
                cout << "to which Frame: " << e.frame << endl;
                cout << "to which Transition: " << e.transitionID << endl;
                cout << "number of frames in motionData: " << e.b.motionData.size() << endl;
            }
        }
        // for (const auto& entry : adjacencyList) {
        //     std::cout << "Node " << entry.first << " is connected to: Node ";
        //     for (Edge neighbor : entry.second) {
        //         std::cout << neighbor.dest << " ";
        //         std::cout << neighbor.b.numFrames << " ";
        //     }
        //     std::cout << std::endl;
        // }
    }
        int firstTime = 1;
            Node nextNode;
private:
    // To represent (start from that edge to what edge)
    // adjacencyList[source] => transition to another node
    std::vector<Node> nodes;
    int WINDOW_SIZE = 25;
    float THRESHOLD_VALUE = 150;
    glm::vec3 currentTransformation = glm::vec3(0,0,0);
    vector<GLfloat> currentJointOrientation;
};
