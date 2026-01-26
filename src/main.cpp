#include <iostream>
#include "math/vectors.hpp"
#include "math/matrices.hpp"

int main() {
    vec3 right(1.0f, 0.0, 3.0f);
    std::cout << right[0] << "\n";
    std::cout << right[1] << "\n";
    std::cout << right[2] << "\n";

    mat4 m4 = {1.0f, 0.0f, 0.0f, 0.0f,
               0.0f, 1.0f, 0.0f, 0.0f,
               0.0f, 0.0f, 1.0f, 5.0f,
               0.0f, 0.0f, 0.0f, 1.0f 
    };


    mat2 m = {0.0f, 1.0f, 
              2.0f, 3.0f};
    mat2 m2 = {4.0f, 5.0f, 
               6.0f, 7.0f};

    mat3 m3 = mat3();

    Inverse(m);
    Inverse(m3);
    Inverse(m4);
    
    m * m2;
    //std::cout<< "element at index 11: " <<m4[2][3] << "\n";
    //std::cout<< "element at index 11: " << m4._34 << "\n";
    //std::cout<< "element at index 11: " <<m4.asArray[11] << "\n";
    std::cout<< "element at index 0: " << (m * m2).asArray[0] << "\n";
    std::cout<< "element at index 1: " << (m * m2).asArray[1] << "\n";
    std::cout<< "element at index 2: " << (m * m2).asArray[2] << "\n";
    std::cout<< "element at index 3: " << (m * m2).asArray[3] << "\n";
    return 0;
}