#include "particle.hpp"
#include "../math/geometry/3d_geometry.hpp"
//#include "../repo/Code/FixedFunctionPrimitives.h"

Particle::Particle()
{
    type = RIGIDBODY_TYPE_PARTICLE;
    friction = 0.95f;
    gravity = vec3(0.0f, -9.82f, 0.0f);
    mass = 1.0f;
    bounce = 0.7f;
}

void Particle::Update(float deltaTime)
{
    oldPosition = position;
    
    vec3 acceleration = forces * InvMass();

    velocity = velocity * friction + acceleration * deltaTime;
    position = position + velocity * deltaTime;
}

void Particle::Render()
{
    Sphere visual(position, 1.0f);
    //::Render(visual);
}

void Particle::ApplyForces()
{
    forces = gravity * mass;
}

void Particle::SolveConstraints(const std::vector<OBB>& constraints)
{
	int size = constraints.size();
	for (int i = 0; i < size; i++)
	{
		Line traveled(oldPosition, position);
		if (Linetest(constraints[i], traveled)) {
		
			vec3 direction = Normalized(velocity);
			Ray ray(oldPosition, direction);
			RaycastResult result;

			if (Raycast(constraints[i], ray, &result))
			{
				position = result.point + result.normal * 0.003f;

				vec3 vn = result.normal * Dot(result.normal, velocity);
				vec3 vt = velocity - vn;

				oldPosition = position;
				velocity = vt - vn * bounce;
				break;
			}
		}
	}
}

void Particle::SetPosition(const vec3& pos)
{
    position = oldPosition = pos;
}

vec3 Particle::GetPosition()
{
    return position;
}

void Particle::SetBounce(float b)
{
    bounce = b;
}

float Particle::GetBounce()
{
    return bounce;
}

void Particle::AddImpulse(const vec3& impulse)
{
    velocity = velocity + impulse;
}

float Particle::InvMass()
{
    if(mass == 0.0f)
    {
        return 0.0f;
    }

    return 1.0f / mass;
    
}

void Particle::SetMass(float m)
{
    if(m < 0)
    {
        m = 0;
    }
    mass = m;
}

vec3 Particle::GetVelocity()
{
    return velocity;
}

void Particle::SetFriction(float f)
{
    if(f < 0)
    {
        f = 0;
    }

    friction = f;
}
    