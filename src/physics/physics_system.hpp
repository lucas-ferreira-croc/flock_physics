#ifndef PHYSICS_SYSTEM_HPP
#define PHYSICS_SYSTEM_HPP

#include "rigidbody.hpp"
#include "spring.hpp"
#include "cloth.hpp"

namespace Physics
{
    class PhysicsSystem
    {
    public:
        PhysicsSystem();

        void Update(float deltaTime);
        void Render();

        void AddRigidbody(Rigidbody *body);
        void AddConstraint(const OBB &obb);

        void ClearRigidbodys();
        void ClearConstraints();

        void AddSpring(const Spring &spring);
        void ClearSprings();

        void AddCloth(Cloth *cloth);
        void ClearCloths();

        float LinearProjectionPercent;
        float PenetrationSlack;
        int ImpulseIteration;
    protected:
        std::vector<Rigidbody *> bodies;
        std::vector<OBB> constraints;
        std::vector<Spring> springs;
        std::vector<Cloth *> cloths;

        std::vector<Rigidbody *> colliders1;
        std::vector<Rigidbody *> colliders2;
        std::vector<CollisionManifold> results;

   

        bool DebugRender;
        bool DoLinearProjection;
        bool RenderRandomColors;
    };
}

#endif