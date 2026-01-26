#ifndef RIGIDBODY_VOLUME_HPP
#define RIGIDBODY_VOLUME_HPP

#include "rigidbody.hpp"
#define GRAVITY_CONST vec3(0.0f, -9.82f, 0.0f)

class RigidbodyVolume : public Rigidbody
{
public:
    inline RigidbodyVolume() : 
        restitution(0.5f), mass(1.0f), friction(0.6f) 
    {
        type = RIGIDBODY_TYPE_BASE;    
    }

    inline RigidbodyVolume(int bodyType) : 
        restitution(0.5f), mass(1.0f), friction(0.6f) 
    {
        type = bodyType;    
    }

    ~RigidbodyVolume() {}
    
    void Render();
    void Update(float deltaTime);
    void ApplyForces();
    
    void SynchCollisionVolumes();
    float InvMass();
    void AddLinearImpulse(const vec3& impulse);
    
    mat4 InvTensor();
    virtual void AddRotationImpulse(const vec3& point, const vec3& impulse);

    /// linear
    vec3 position;
    vec3 velocity;
    vec3 forces;
    float mass;
    float restitution;
    float friction;
    OBB box;
    Sphere sphere;

    /// angular
    vec3 orientation;
    vec3 angularVelocity;
    vec3 torques;
};

CollisionManifold FindCollisionFeatures(RigidbodyVolume& rigidbodyA, RigidbodyVolume& rigidbodyB);
void ApplyImpulse(RigidbodyVolume& rigidbodyA, RigidbodyVolume& rigidbodyB, const CollisionManifold& M, int c);

#endif