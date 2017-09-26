//
//  VocabularyBinary.hpp
//  VINS_ios
//
//  Created by Yang Liu on 3/13/17.
//  Copyright © 2017 栗大人. All rights reserved.
//

#ifndef VocabularyBinary_hpp
#define VocabularyBinary_hpp

#include <cstdint>
#include <fstream>
#include <string>

namespace VINSLoop {
    
struct Node {
    int32_t nodeId;
    int32_t parentId;
    double weight;
#if (__SIZEOF_PTRDIFF_T__ == 8)
  uint64_t descriptor[4];
#elif (__SIZEOF_PTRDIFF_T__ == 4)
  uint32_t descriptor[8];
#endif
	//uint8_t descriptor[32]; //solomon modified for debug 32 bit system compatibility issue(boost::dynamic_bitset size()!= rhs.size() issue)
};

struct Word {
    int32_t nodeId;
    int32_t wordId;
};

struct Vocabulary {
    int32_t k;
    int32_t L;
    int32_t scoringType;
    int32_t weightingType;
    
    int32_t nNodes;
    int32_t nWords;
    
    Node* nodes;
    Word* words;
    
    Vocabulary();
    ~Vocabulary();
    
    void serialize(std::ofstream& stream);
    void deserialize(std::ifstream& stream);
    
    inline static size_t staticDataSize() {
        return sizeof(Vocabulary) - sizeof(Node*) - sizeof(Word*);
    }
};

}

#endif /* VocabularyBinary_hpp */
