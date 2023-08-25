#include "joint.h"
    int Joint::howManyChannels() {
        if(this->t == 0) {
            return 6;
        } else if(this->t == 1 || this->t == 2){
            return 3;
        } else {
            return 0;
        }
    }

    void Joint::setRealOrder() {

        realOrder.clear();
        if(transformationOrder.size() == 6) {
            
            for(int i = 0; i < transformationOrder.size(); i++) {
                if(transformationOrder[i].find("Xpos") != -1) {
                    this->realOrder.push_back("Xpos");
                } 
                if(transformationOrder[i].find("Ypos") != -1) {
                    this->realOrder.push_back("Ypos");
                } 
                if(transformationOrder[i].find("Zpos") != -1) {
                    this->realOrder.push_back("Zpos");
                } 
                if(transformationOrder[i].find("Xrot") != -1) {
                    this->realOrder.push_back("Xrot");
                } 
                if(transformationOrder[i].find("Yrot") != -1) {
                    this->realOrder.push_back("Yrot");
                } 
                if(transformationOrder[i].find("Zrot") != -1) {
                    this->realOrder.push_back("Zrot");
                } 
        }
        } else {
            for(int i = 0; i < transformationOrder.size(); i++) {
                if(transformationOrder[i].find("Xrot") != -1) {
                    this->realOrder.push_back("Xrot");
                } 
                if(transformationOrder[i].find("Yrot") != -1) {
                    this->realOrder.push_back("Yrot");
                } 
                if(transformationOrder[i].find("Zrot") != -1) {
                    this->realOrder.push_back("Zrot");
                } 
            }
        }
    }

    // Return the ith index of rotations
    int Joint::channelIndex(std::string str) {
        size_t transformation_index = -1;
        for (unsigned int i = 0; i < transformationOrder.size(); i++) {
            if(transformationOrder[i].find(str) != -1) {
                return i;
            }
        }
        /*
            If can't find anything, just return -1
        */
        return -1;
    }