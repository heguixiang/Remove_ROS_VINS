//
//  VocabularyBinary.cpp
//  VINS_ios
//
//  Created by Yang Liu on 3/13/17.
//  Copyright © 2017 栗大人. All rights reserved.
//

#include "VocabularyBinary.hpp"
#include <opencv2/core/core.hpp>
#include <iostream>
using namespace std;

VINSLoop::Vocabulary::Vocabulary()
: nNodes(0), nodes(nullptr), nWords(0), words(nullptr) {
}

VINSLoop::Vocabulary::~Vocabulary() {
    if (nodes != nullptr) {
        delete [] nodes;
        nodes = nullptr;
    }
    
    if (words != nullptr) {
        delete [] words;
        words = nullptr;
    }
}
    
void VINSLoop::Vocabulary::serialize(ofstream& stream) {
    stream.write((const char *)this, staticDataSize());
    stream.write((const char *)nodes, sizeof(Node) * nNodes);
    stream.write((const char *)words, sizeof(Word) * nWords);
}
    
void VINSLoop::Vocabulary::deserialize(ifstream& stream) {
    stream.read((char *)this, staticDataSize());
    
    nodes = new Node[nNodes];
//	std::cout << "+++++++++sizeof(Node):" << sizeof(Node)  << "num of nNodes:" << nNodes << "staticDataSize:" << staticDataSize() << std::endl;
    stream.read((char *)nodes, sizeof(Node) * nNodes);
//	std::cout << "nodes[0].descriptor:"  << hex << nodes[0].descriptor[0]  << "sizeof(nodes[0].descriptor)" << sizeof(nodes[0].descriptor)<< std::endl;    
    words = new Word[nWords];
    stream.read((char *)words, sizeof(Word) * nWords);
}
