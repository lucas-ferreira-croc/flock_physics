#include "spring.hpp"

namespace Physics
{
    Particle *Spring::GetP1()
    {
        return p1;
    }

    Particle *Spring::GetP2()
    {
        return p2;
    }

    void Spring::SetParticles(Particle *_p1, Particle *_p2)
    {
        p1 = _p1;
        p2 = _p2;
    }

    void Spring::SetConstants(float _k, float _b)
    {
        k = _k;
        b = _b;
    }

    void Spring::ApplyForce(float deltaTime)
    {
        vec3 relativePos = p2->GetPosition() - p1->GetPosition();
        vec3 relativeVel = p2->GetVelocity() - p1->GetVelocity();

        float x = Magnitude(relativePos) - restLength;
        float v = Magnitude(relativeVel);

        // hooke's law
        float F = (-k * x) + (-b * v);

        vec3 impulse = Normalized(relativePos) * F;
        p1->AddImpulse(impulse * p1->InvMass());
        p2->AddImpulse(impulse * -1.0f * p2->InvMass());
    }

}