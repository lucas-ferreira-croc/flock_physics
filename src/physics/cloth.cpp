#include "cloth.hpp"

namespace Physics
{
    void Cloth::Initialize(int gridSize, float distance, const vec3 &position)
    {
        float k = -1.0f;
        float b = 0.0f;
        clothSize = gridSize;

        verts.clear();
        structural.clear();
        shear.clear();
        bend.clear();

        verts.resize(gridSize * gridSize);
        float halfSize = (float)(gridSize - 1) * 0.5f;

        if (gridSize < 3)
        {
            gridSize = 3;
        }

        for (int x = 0; x < gridSize; x++)
        {
            for (int z = 0; z < gridSize; z++)
            {
                int i = z * gridSize + x;

                float x_pos = ((float)x + position.x - halfSize) * distance;
                float z_pos = ((float)z + position.z - halfSize) * distance;

                verts[i].SetPosition(vec3(x_pos, position.y, z_pos));
                verts[i].SetMass(1.0f);
                verts[i].SetBounce(0.0f);
                verts[i].SetFriction(0.9f);
            }
        }

        for (int x = 0; x < gridSize; ++x)
        {
            for (int z = 0; z < gridSize - 1; ++z)
            {
                int i = z * gridSize + x;
                int j = (z + 1) * gridSize + x;

                vec3 iPos = verts[i].GetPosition();
                vec3 jPos = verts[j].GetPosition();
                float restLength = Magnitude(iPos - jPos);

                Spring spring(k, b, restLength);
                spring.SetParticles(&verts[i], &verts[j]);
                structural.push_back(spring);
            }
        }

        for (int x = 0; x < gridSize - 1; ++x)
        {
            for (int z = 0; z < gridSize; ++z)
            {
                int i = z * gridSize + x;
                int j = z * gridSize + (x + 1);

                vec3 iPos = verts[i].GetPosition();
                vec3 jPos = verts[j].GetPosition();
                float restLength = Magnitude(iPos - jPos);

                Spring spring(k, b, restLength);
                spring.SetParticles(&verts[i], &verts[j]);
                structural.push_back(spring);
            }
        }

        for (int x = 0; x < gridSize - 1; ++x)
        {
            for (int z = 0; z < gridSize - 1; ++z)
            {
                int i = z * gridSize + x;
                int j = (z + 1) * gridSize + (x + 1);

                vec3 iPos = verts[i].GetPosition();
                vec3 jPos = verts[j].GetPosition();
                float restLength = Magnitude(iPos - jPos);

                Spring spring(k, b, restLength);
                spring.SetParticles(&verts[i], &verts[j]);
                shear.push_back(spring);
            }
        }

        for (int x = 1; x < gridSize - 1; ++x)
        {
            for (int z = 0; z < gridSize - 1; ++z)
            {
                int i = z * gridSize + x;
                int j = (z + 1) * gridSize + (x - 1);

                vec3 iPos = verts[i].GetPosition();
                vec3 jPos = verts[j].GetPosition();
                float restLength = Magnitude(iPos - jPos);

                Spring spring(k, b, restLength);
                spring.SetParticles(&verts[i], &verts[j]);
                shear.push_back(spring);
            }
        }

        for (int x = 0; x < gridSize; ++x)
        {
            for (int z = 0; z < gridSize - 2; ++z)
            {
                int i = z * gridSize + x;
                int j = (z + 2) * gridSize + x;

                vec3 iPos = verts[i].GetPosition();
                vec3 jPos = verts[j].GetPosition();
                float restLength = Magnitude(iPos - jPos);

                Spring spring(k, b, restLength);
                spring.SetParticles(&verts[i], &verts[j]);
                bend.push_back(spring);
            }
        }

        for (int x = 0; x < gridSize - 2; ++x)
        {
            for (int z = 0; z < gridSize; ++z)
            {
                int i = z * gridSize + x;
                int j = z * gridSize + (x + 2);

                vec3 iPos = verts[i].GetPosition();
                vec3 jPos = verts[j].GetPosition();
                float restLength = Magnitude(iPos - jPos);

                Spring spring(k, b, restLength);
                spring.SetParticles(&verts[i], &verts[j]);
                bend.push_back(spring);
            }
        }
    }

    void Cloth::SetStructuralSprings(float k, float b)
    {
        for (int i = 0, size = structural.size(); i < size; i++)
        {
            structural[i].SetConstants(k, b);
        }
    }

    void Cloth::SetShearSprings(float k, float b)
    {
        for (int i = 0, size = shear.size(); i < size; i++)
        {
            shear[i].SetConstants(k, b);
        }
    }

    void Cloth::SetBendSprings(float k, float b)
    {
        for (int i = 0, size = bend.size(); i < size; i++)
        {
            bend[i].SetConstants(k, b);
        }
    }

    void Cloth::SetParticleMass(float mass)
    {
        for (int i = 0, size = verts.size(); i < size; i++)
        {
            verts[i].SetMass(mass);
        }
    }

    void Cloth::ApplyForces()
    {
        for (int i = 0, size = verts.size(); i < size; i++)
        {
            verts[i].ApplyForces();
        }
    }

    void Cloth::Update(float deltaTime)
    {
        for (int i = 0, size = verts.size(); i < size; i++)
        {
            verts[i].Update(deltaTime);
        }
    }

    void Cloth::ApplySpringForces(float deltaTime)
    {
        for (int i = 0, size = structural.size(); i < size; i++)
        {
            structural[i].ApplyForce(deltaTime);
        }

        for (int i = 0, size = shear.size(); i < size; i++)
        {
            shear[i].ApplyForce(deltaTime);
        }

        for (int i = 0, size = bend.size(); i < size; i++)
        {
            bend[i].ApplyForce(deltaTime);
        }
    }

    void Cloth::SolveConstraints(const std::vector<OBB> &constraints)
    {
        for (int i = 0, size = verts.size(); i < size; i++)
        {
            verts[i].SolveConstraints(constraints);
        }
    }

    void Cloth::Render()
    {
    }
}
