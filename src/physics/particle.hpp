#ifndef PARTICLE_HPP
#define PARTICLE_HPP

#include "rigidbody.hpp"

class Particle : public Rigidbody
{
public:
    Particle();
    void Update(float deltaTime);
    void Render();
    void ApplyForces();
    void SolveConstraints(const std::vector<OBB>& constraints);

    void SetPosition(const vec3& pos);
    vec3 GetPosition();
    void SetBounce(float b);
    float GetBounce();

    void AddImpulse(const vec3& impulse);
    float InvMass();
    void SetMass(float m);
    vec3 GetVelocity();
    void SetFriction(float f);

private:
    vec3 position;
    vec3 oldPosition;
    vec3 forces;
    float bounce;
    vec3 gravity;
    float friction;
    vec3 velocity;
    float mass;
};

#endif