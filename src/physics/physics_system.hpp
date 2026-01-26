#ifndef PHYSICS_SYSTEM_HPP
#define PHYSICS_SYSTEM_HPP

#include "rigidbody.hpp"

class PhysicsSystem
{
public:

    PhysicsSystem();

    void Update(float deltaTime);
    void Render();

    void AddRigidBody(Rigidbody* body);
    void AddConstraint(const OBB& obb);

    void ClearRigidBodies();
    void ClearConstraints();

protected:
    std::vector<Rigidbody*> bodies;
    std::vector<OBB> constraints;

    std::vector<Rigidbody*> colliders1;
    std::vector<Rigidbody*> colliders2;
    std::vector<CollisionManifold> results;
    
    float LinearProjectionPercent;
    float PenetrationSlack;
    int ImpulseIteration;
};

#endif