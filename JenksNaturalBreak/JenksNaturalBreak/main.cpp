#include "JenksNaturalBreak.hpp"
#include <iostream>
#include <vector>

int main(int c, char** argv) 
{ 
    
    const int n = 1000;
    const int k = 10;
    
    std::cout << "Generating random numbers..." << std::endl;
    
    std::vector<double> values;
    values.reserve(n);
    for (int i=0; i!=n; ++i)
    {
        double v = 100+i/1000;
        values.push_back(v); //populate a distribuiton slightly more interesting than uniform, with a lower density at higher values.
    }
    
    JenksNaturalBreak(values, k);
} // main