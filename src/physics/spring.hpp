#ifndef SPRING_HPP
#define SPRING_HPP

#include "particle.hpp"

class Spring
{
public:
    inline Spring(float _k, float _b, float length)
        : k(_k), b(_b), restLength(length) {}

    Particle* GetP1();
    Particle* GetP2();
    void SetParticles(Particle* _p1, Particle* _p2);
    void SetConstants(float _k, float _b);
    void ApplyForce(float deltaTime);
    
protected:
    Particle* p1;
    Particle* p2;

    // higher the k, higher stifness

    float k; // -n ... 0
    float b; //0 ... 1
    float restLength;
};

#endif