#ifndef CLOTH_HPP
#define CLOTH_HPP

#include "spring.hpp"
#include <vector>

namespace Physics
{
    class Cloth
    {
    public:
        void Initialize(int gridSize, float distance, const vec3 &position);
        void SetStructuralSprings(float k, float b);
        void SetShearSprings(float k, float b);
        void SetBendSprings(float k, float b);
        void SetParticleMass(float mass);

        void ApplyForces();
        void Update(float deltaTime);
        void ApplySpringForces(float deltaTime);
        void SolveConstraints(const std::vector<OBB> &constraints);
        void Render();

    protected:
        std::vector<Particle> verts;
        std::vector<Spring> structural;
        std::vector<Spring> shear;
        std::vector<Spring> bend;
        float clothSize;
    };
}

#endif