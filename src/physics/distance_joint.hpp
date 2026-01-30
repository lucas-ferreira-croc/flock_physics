#ifndef DISTANCE_JOINT
#define DISTANCE_JOINT

#include "particle.hpp"

namespace Physics
{
    class DistanceJoint : public Rigidbody
    {
    public:
        void Initialize(Particle *_p1, Particle *_p2, float len);
        void SolveConstraints(const std::vector<OBB> &constraints);
        void Render();

    protected:
        Particle *p1;
        Particle *p2;
        float length;
    };
}

#endif