/*
    The source code to parse a BVH file

*/
#include <fstream>
#include <iostream>
#include <dirent.h>
#include <stdio.h>
#include <string>
#include <cstring>
#include <string.h>
#include <algorithm>
#include <vector>
#include <sstream>
#include "parse_bvh.h"

using namespace std;



vector<string> split(string input, char delimiter) {
    vector<string> answer;
    stringstream ss(input);
    string temp;

    while(getline(ss, temp, delimiter)) {
        answer.push_back(temp);
    }
    return answer;

}

std::string trim(std::string str) {
    size_t first_non_space_pos = str.find_first_not_of(' ');
    
    // Extract the substring starting from the first non-space character
    return str.substr(first_non_space_pos);
}

std::string trimTwoWays(std::string str) {
     // Find the position of the first non-space character
    size_t first_non_space_pos = str.find_first_not_of(' ');

    // Find the position of the last non-space character
    size_t last_non_space_pos = str.find_last_not_of(' ');

    // Extract the substring starting from the first non-space character up to the last non-space character
    return str.substr(first_non_space_pos, last_non_space_pos + 1 - first_non_space_pos);


}

vector<string> readBVHFiles() {
    // GOING THROUGH all the directories

    vector<string> listBVH;

    DIR *dir;
    struct dirent *ent;
    if ((dir = opendir ("../bvhs")) != NULL) {
    /* print all the files and directories within directory */
        while ((ent = readdir (dir)) != NULL) {
            if(strcmp(ent->d_name, ".") != 0 && strcmp(ent->d_name, "..") != 0) {
                listBVH.push_back(ent->d_name);                
            }
            //printf ("%s\n", ent->d_name);
    }
    closedir (dir);
    } else {
        ///* could not open directory */
        perror ("");
        //return NULL;
    }
    return listBVH;
}


BVHMotion parse(std::string filename) {
    vector<Joint> v;

    std::ifstream file("../bvhs/" + filename);
    //std::ifstream file("../bvhs/joy2-1.bvh");
    std::string str;
    string hierarchy = "HIERARCHY";
    string MOTION = "MOTION";
    string JOINT = "JOINT";
    string END = "End";
    string CHANNEL = "CHANNEL";
    string child_indicator = "{";
    string parent_indicator = "}";
    string end_site_indicator = "End";
    BVHMotion bvhMotion;




    // Trying to get rid of leading spaces
    size_t first_non_space_pos;
    size_t m_index = -1;
    size_t child_index, h_index, parent_index, joint_index, channel_index, end_index;

    int depth_count = 0;


    while (std::getline(file, str)) {
        //cout << str << endl;
        if(m_index == -1) {
            m_index = str.find(MOTION);
        }
        // The
        if (m_index == -1) {
            // Where HIERARCHY STARTS
            h_index = str.find(hierarchy);
            child_index = str.find(child_indicator);
            parent_index = str.find(parent_indicator);
            //cout << str << endl;


            if(child_index != -1) {
                std::getline(file,str);


                str = trim(str);
                vector<string> result = split(str, ' ');
                Joint j(std::stof(result[1]), std::stof(result[2]), std::stof(result[3]));
                
                // CHANNEL
                std::getline(file,str);

                channel_index = str.find(CHANNEL);

                // IF THERE IS NO CHANNELS ENTRY?
                if(channel_index != -1) {
                    str = trim(str);
                    vector<string> channel_result = split(str, ' ');


                    if(stoi(channel_result[1]) == 6 ) {
                        j.t = CAN_ROTATE_TRANSLATE;

                        for(int i = 0; i < 6; i++) {
                            j.transformationOrder.push_back(channel_result[2+i]);
                        }

                    } else {

                        for(int i = 0; i < 3; i++) {
                            j.transformationOrder.push_back(channel_result[2+i]);
                        }

                        if(channel_result[2].find("rotation")) {
                            j.t = CAN_ROTATE;
                        } else {
                            j.t = CAN_TRANSLATE;
                        }
                    }
                    //j.setRealOrder();
                    // JOINT
                    std::getline(file,str);
                }
                

                // JOINT
                str = trim(str);
                joint_index = str.find(JOINT);

                // Another bvh file has a keyword, called End Site
                end_index = str.find(END);

                if(end_index != -1) {
                    std::getline(file,str);
                    std::getline(file,str);
                    std::getline(file,str);
                    std::getline(file,str);

                    joint_index = str.find(JOINT);
                }
                
                // JOINT? -> there is a child
                if(joint_index != -1) {
                    vector<string> jName = split(str, ' ');
                    j.depth = depth_count;
                    j.isthereChild = true;
                }
                // NO JOINT -> there is no child
                else {
                    j.depth = depth_count;
                    j.isthereChild = false;
                    
                    while(std::getline(file, str)) {
                        str = trim(str);
                        depth_count--;
                        joint_index = str.find(JOINT);
                        //m_index = str.find("MOTION");
                        if(joint_index != -1) {
                            break;
                        }
                        //depth_count--;
                    }
                }

                depth_count++;
                v.push_back(j);

                // 1 x
                // 2 y
                // 3 z 

            }
            m_index = str.find("MOTION");
        }
    }

    file.clear();
    file.seekg(0);


    while (std::getline(file, str)) {

        m_index = str.find(MOTION);
        
        if(m_index != -1){
                // DEALING WITH FORWARD KINEMATIC PROBLEMS

                std::getline(file, str);
                str = trim(str);
                vector<string> numFrameResult = split(str, ':');
                
                
                
                std::getline(file, str);
                str = trim(str);
                vector<string> frameTimeResult = split(str, ':');


                bvhMotion.setNumFrames(std::stoi(trim(numFrameResult[1])));

                bvhMotion.setFrameTime(std::stof(trim(frameTimeResult[1])));

                
                while(std::getline(file, str)) {
                    str = trimTwoWays(str);
                    vector<string> motionDataResult = split(str, ' ');
                    motionDataResult.pop_back();
                    vector<GLfloat> motionDataFloatResult;
                                                
                    for(unsigned int i = 0; i < motionDataResult.size(); i++) {
                        motionDataFloatResult.push_back(std::stof(trimTwoWays(motionDataResult[i])) );
                    }
                    bvhMotion.motionData.push_back(motionDataFloatResult);
                }
                break;
        }
    }
         


    file.clear();
    file.seekg(0);
    unsigned int i = 0;

    while (std::getline(file, str)) {
        //str = trim(str);
        if(str.find("JOINT") != -1 || str.find("ROOT") != -1) {
            str = trim(str);
            vector<string> jResult = split(str, ' ');
            //cout << str << endl;
            //cout << jResult[1] << endl;
            v[i].jointName = jResult[1];
            i++;
        }
    }

    file.close();

    bvhMotion.joints = v;
    for(i = 0; i < bvhMotion.joints.size(); i++) {
        bvhMotion.joints[i].setRealOrder();
    }

    return bvhMotion;
}
