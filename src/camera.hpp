#ifndef CAMERA_HPP
#define CAMERA_HPP

#include "math/matrices.hpp"
#include "math/geometry/3d_geometry.hpp"

class Camera
{
public:
    Camera();
    inline virtual ~Camera() {}

    mat4 GetWorldMatrix();
    mat4 GetViewMatrix();
    mat4 GetProjectionMatrix();

    void SetProjection(const mat4& projection);
    void SetWorld(const mat4& view);

    float GetAspect();
    bool IsOrthographic();
    bool IsPerspective();
    bool IsOrthoNormal();
    
    void OrthoNormalize();
    void Resize(int width, int height);
    void Perspective(float fov, float aspect, float zNear, float zFar);
    void Orthographic(float width, float height, float zNear, float zFar);

    Frustum GetFrustum();
protected:
    float m_nFov;
    float m_nAaspect;
    float m_nNear;
    float m_nFar;
    float m_nWidth;
    float m_nHeight;

    mat4 m_matWorld;
    mat4 m_matProj;
    int m_nProjectionMode; // 0 = perspective, 1 = ortho, 2 = user
};

class OrbitCamera : public Camera
{
public:
    OrbitCamera();
    inline virtual ~OrbitCamera() {}

    void Rotate(const vec2& deltaRot, float deltaTime);
    void Zoom(float deltaZoom, float deltaTime);
    void Pan(const vec2 deltaPan, float deltaTime);

    void Update(float deltaTime);

    void SetTarget(const vec3& newTarget);
    void SetZoom(float zoom);
    void SetRotation(const vec2& rotation);
protected:
    vec3 target;
    vec2 panSpeed;

    float zoomDistance;
    vec2 zoomDistanceLimit; // x = min, y = max;
    float zoomSpeed;

    vec2 rotationSpeed;
    vec2 yRotationLimit; // x = min, y = max;
    vec2 currentRotation;
    float ClampAngle(float angle, float min, float max);
};

Camera CreatePerspective(float fieldOfView, float aspectRatio, float nearPlane, float farPlane);
Camera CreateOrtographic(float width, float height, float nearPlane, float farPlane);

#endif