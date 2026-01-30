// #include "../repo/Code/FixedFunctionPrimitives.h"
#include "rigidbody_volume.hpp"

#define CMP(x, y)                       \
    (fabs((x) - (y)) <= FLT_EPSILON *   \
                            fmaxf(1.0f, \
                                  fmaxf(fabsf(x), fabsf(y))))

namespace Physics
{
    void RigidbodyVolume::Render()
    {
        SynchCollisionVolumes();
        if (type == RIGIDBODY_TYPE_SPHERE)
        {
            //::Render(sphere);
        }
        if (type == RIGIDBODY_TYPE_BOX)
        {
            //::Render(box);
        }
    }

    void RigidbodyVolume::Update(float deltaTime)
    {
        const float damping = 0.98f;

        vec3 acceleration = forces * InvMass();
        velocity = velocity + acceleration * deltaTime;
        velocity = velocity * damping;

        if (fabsf(velocity.x) < 0.001f)
        {
            velocity.x = 0.0f;
        }
        if (fabsf(velocity.y) < 0.001f)
        {
            velocity.y = 0.0f;
        }
        if (fabsf(velocity.z) < 0.001f)
        {
            velocity.z = 0.0f;
        }

        if (type == RIGIDBODY_TYPE_BOX)
        {
            vec3 angularAcceleration = MultiplyVector(torques, InvTensor());
            angularVelocity = angularVelocity + angularAcceleration * deltaTime;
            ;
            angularVelocity = angularVelocity * damping;
        }

        position = position + velocity * deltaTime;

        if (type == RIGIDBODY_TYPE_BOX)
        {
            orientation = orientation + angularVelocity * deltaTime;
        }

        SynchCollisionVolumes();
    }

    void RigidbodyVolume::ApplyForces()
    {
        forces = GRAVITY_CONST * mass;
    }

    void RigidbodyVolume::SynchCollisionVolumes()
    {
        sphere.position = position;
        box.position = position;

        box.orientation = Rotation3x3(
            RAD2DEG(orientation.x),
            RAD2DEG(orientation.y),
            RAD2DEG(orientation.z));
    }

    float RigidbodyVolume::InvMass()
    {
        return mass == 0.0f ? 0.0f : 1.0f / mass;
    }

    void RigidbodyVolume::AddLinearImpulse(const vec3 &impulse)
    {
        velocity = velocity + impulse;
    }

    CollisionManifold FindCollisionFeatures(RigidbodyVolume &rigidbodyA, RigidbodyVolume &rigidbodyB)
    {
        CollisionManifold result;
        ResetCollisionManifold(&result);
        if (rigidbodyA.type == RIGIDBODY_TYPE_SPHERE)
        {
            if (rigidbodyB.type == RIGIDBODY_TYPE_SPHERE)
            {
                result = FindCollisionFeatures(rigidbodyA.sphere, rigidbodyB.sphere);
            }
            else if (rigidbodyB.type == RIGIDBODY_TYPE_BOX)
            {
                result = FindCollisionFeatures(rigidbodyB.box, rigidbodyA.sphere);
                result.normal = result.normal * -1.0f;
            }
        }
        if (rigidbodyA.type == RIGIDBODY_TYPE_BOX)
        {
            if (rigidbodyB.type == RIGIDBODY_TYPE_BOX)
            {
                result = FindCollisionFeatures(rigidbodyA.box, rigidbodyB.box);
            }
            else if (rigidbodyB.type == RIGIDBODY_TYPE_SPHERE)
            {
                result = FindCollisionFeatures(rigidbodyA.box, rigidbodyB.sphere);
            }
        }
        return result;
    }

    mat4 RigidbodyVolume::InvTensor()
    {
        if (mass == 0)
        {
            return mat4(
                0, 0, 0, 0,
                0, 0, 0, 0,
                0, 0, 0, 0,
                0, 0, 0, 0);
        }

        float ix = 0.0f;
        float iy = 0.0f;
        float iz = 0.0f;
        float iw = 0.0f;

        if (mass != 0.0f && type == RIGIDBODY_TYPE_SPHERE)
        {
            float r2 = sphere.radius * sphere.radius;
            float fraction = (2.0f / 5.0f);
            ix = r2 * mass * fraction;
            iy = r2 * mass * fraction;
            iz = r2 * mass * fraction;
            iw = 1.0f;
        }
        else if (mass != 0.0f && type == RIGIDBODY_TYPE_SPHERE)
        {
            vec3 size = box.size * 2.0f;
            float fraction = (1.0f / 12.0f);
            float x2 = size.x * size.x;
            float y2 = size.y * size.y;
            float z2 = size.z * size.z;
            ix = (y2 + z2) * mass * fraction;
            iy = (x2 + z2) * mass * fraction;
            iz = (x2 + y2) * mass * fraction;
            iw = 1.0f;
        }

        return Inverse(mat4(
            ix, 0, 0, 0,
            0, iy, 0, 0,
            0, 0, iz, 0,
            0, 0, 0, iw));
    }

    void RigidbodyVolume::AddRotationImpulse(const vec3 &point, const vec3 &impulse)
    {
        vec3 centerOfMass = position;
        vec3 torque = Cross(point - centerOfMass, impulse);

        vec3 angularAcceleration = MultiplyVector(torque, InvTensor());
        angularVelocity = angularVelocity + angularAcceleration;
    }

    void ApplyImpulse(RigidbodyVolume &rigidbodyA, RigidbodyVolume &rigidbodyB, const CollisionManifold &M, int c)
    {
        // Linear velocity
        float invMass1 = rigidbodyA.InvMass();
        float invMass2 = rigidbodyB.InvMass();
        float invMassSum = invMass1 + invMass2;
        if (invMassSum == 0.0f)
        {
            return;
        }

        vec3 relative1 = M.contacts[c] - rigidbodyA.position;
        vec3 relative2 = M.contacts[c] - rigidbodyB.position;

        mat4 inverseTensor1 = rigidbodyA.InvTensor();
        mat4 inverseTensor2 = rigidbodyB.InvTensor();

        // Relative velocity
        vec3 relativeVelocity = rigidbodyB.velocity - rigidbodyA.velocity;
        vec3 relativeNormal = M.normal;
        Normalize(relativeNormal);

        // if moving away no collision
        if (Dot(relativeVelocity, relativeNormal) > 0.0f)
        {
            return;
        }

        float e = fminf(rigidbodyA.restitution, rigidbodyB.restitution);
        float numerator = (-(1.0f + e) * Dot(relativeVelocity, relativeNormal));
        float d1 = invMassSum;
        vec3 d2 = Cross(MultiplyVector(Cross(relative1, relativeNormal), inverseTensor1), relative1);
        vec3 d3 = Cross(MultiplyVector(Cross(relative2, relativeNormal), inverseTensor2), relative2);
        float denominator = d1 + Dot(relativeNormal, d2 + d3);

        float j = (denominator == 0.0f) ? 0.0f : numerator / denominator;
        if (M.contacts.size() > 0.0f && j != 0.0f)
        {
            j /= (float)M.contacts.size();
        }

        vec3 impulse = relativeNormal * j;
        rigidbodyA.velocity = rigidbodyA.velocity - impulse * invMass1;
        rigidbodyB.velocity = rigidbodyB.velocity + impulse * invMass2;

        rigidbodyA.angularVelocity = rigidbodyA.angularVelocity - MultiplyVector(Cross(relative1, impulse), inverseTensor1);
        rigidbodyB.angularVelocity = rigidbodyB.angularVelocity + MultiplyVector(Cross(relative2, impulse), inverseTensor2);

        // friction section
        vec3 tangent = relativeVelocity - (relativeNormal * Dot(relativeVelocity, relativeNormal));

        if (CMP(MagnitudeSqr(tangent), 0.0f))
        {
            return;
        }
        Normalize(tangent);

        numerator = -Dot(relativeVelocity, tangent);
        d1 = invMassSum;
        d2 = Cross(MultiplyVector(Cross(relative1, tangent), inverseTensor1), relative1);
        d3 = Cross(MultiplyVector(Cross(relative2, tangent), inverseTensor2), relative2);
        denominator = d1 + Dot(tangent, d2 + d3);

        if (denominator == 0.0f)
        {
            return;
        }

        float jt = (denominator == 0.0) ? 0.0f : numerator / denominator;
        if (M.contacts.size() > 0.0f && jt != 0.0f)
        {
            jt /= (float)M.contacts.size();
        }

        if (CMP(jt, 0.0f))
        {
            return;
        }

        float friction = sqrtf(rigidbodyA.friction * rigidbodyB.friction);
        if (jt > j * friction)
        {
            jt = j * friction;
        }
        else if (jt < -j * friction)
        {
            jt = -j * friction;
        }

        // tangent = direction, jt = magnitude
        vec3 tangentImpulse = tangent * jt;

        rigidbodyA.velocity = rigidbodyA.velocity - tangentImpulse * invMass1;
        rigidbodyB.velocity = rigidbodyB.velocity + tangentImpulse * invMass2;

        rigidbodyA.angularVelocity = rigidbodyA.angularVelocity - MultiplyVector(Cross(relative1, tangentImpulse), inverseTensor1);
        rigidbodyB.angularVelocity = rigidbodyB.angularVelocity + MultiplyVector(Cross(relative2, tangentImpulse), inverseTensor2);
    }

}
