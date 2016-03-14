//
//  main.cpp
//  valueTimeClass
//
//  Created by Student on 2/17/16.
//  Copyright © 2016 Student. All rights reserved.
//

#include <iostream>
#include <vector>

using namespace std;

class timeSeparatedValue {
public:
    timeSeparatedValue (int size = 0): totalSize(size), allValueDatas(vector<double> (size)) {};
    size_t size(){
        return sizeAtEachTime.size();
    }
    size_t totalDataSize() {
        return allValueDatas.size();
    }
    size_t sizeAtTime(int time){
        return sizeAtEachTime[time];
    }
    vector<double>& getDataAtTime(int time){
        int prevSize;
        for (int i = 0; i < time; i++) {
            prevSize += sizeAtTime(i);
        }
        
    }
    //, vector<double> data = {}, vector<size_t> sizeAtTime = {}): totalSize(size), allValueDatas(data) {};
    
private:
    int totalSize;
    vector<double> allValueDatas;
    vector<size_t> sizeAtEachTime;
    
};

int main(int argc, const char * argv[]) {
    // insert code here...
    std::cout << "Hello, World!\n";
    return 0;
}
