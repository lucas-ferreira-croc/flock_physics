#ifndef _3D_GEOMETRY_HPP
#define _3D_GEOMETRY_HPP

#include <vector>

#include "../vectors.hpp"
#include "../matrices.hpp"

#define AABBSphere(aabb, sphere) \
    SphereAABB(sphere, aabb);
#define OBBSphere(obb, sphere) \
    SphereOBB(sphere, obb);
#define PlaneSphere(plane, sphere) \
    SpherePlane(sphere, plane);
#define OBBAABB(obb, aabb) \
    AABBOBB(aabb, obb);
#define PlaneAABB(plane, aabb) \
    AABBPlane(aabb, plane);
#define PlaneOBB(plane, obb) \
    OBBPlane(obb, plane);
#define SphereTriangle(sphere, triangle) \
    TriangleSphere(triangle, sphere);
#define AABBTriangle(aabb, triangle) \
    TriangleAABB(triangle, aabb)
#define OBBTriangle(obb, triangle) \
    TriangleOBB(triangle, obb);
#define PlaneTriangle(plane, triangle) \
    TrianglePlane(plane, triangle);
#define AABBMesh(aabb, mesh) \
    MeshAABB(mesh, aabb);
#define SphereMesh(sphere, mesh) \
    MeshSphere(mesh, sphere);
#define OBBMesh(obb, mesh) \
    MeshOBB(mesh, obb);
#define PlaneMesh(plane, mesh) \
    MeshPlane(mesh, plane);
#define TriangleMesh(triangle, mesh) \
    MeshTriangle(mesh, triangle);
#define SphereModel(sphere, model) \
    ModelSphere(model, sphere);
#define AABBModel(aabb, model) \
    ModelAABB(model, aabb);
#define OBBModel(obb, model) \
    ModelAABB(model, obb);
#define PlaneModel(plane, model) \
    ModelPlane(model, plane);
#define TriangleModel(triangle, model) \
    ModelTriangle(model, triangle);

typedef vec3 Point;

typedef struct Line
{
    Point start;
    Point end;

    inline Line(){}
    inline Line(const Point& s, const Point& e) : start(s), end(e) {}
} Line;

float Length(const Line& line);
float LengthSq(const Line& line);

typedef struct Ray
{
    Point origin;
    vec3 direction;

    inline Ray() : direction(0.0f, 0.0f, 1.0f) {}
    inline Ray(const Point& o, const vec3& d) :
        origin(o), direction(d) 
    {
        NormalizeDirection();
    }

    void NormalizeDirection()
    {
        Normalize(direction);
    }
    
} Ray;

Ray FromPoints(const Point& from, const Point& to);

typedef struct Sphere
{
    Point position;
    float radius;

    inline Sphere() : radius(1.0f){}
    inline Sphere(const Point& p, float r) :
        position(p), radius(r){}
} Sphere;

typedef struct AABB
{
    Point position;
    // half
    vec3 size;

    inline AABB() : size(1, 1, 1) {}
    inline AABB(const Point& p, const vec3& s) :
        position(p), size(s) {}
} AABB;

vec3 GetMin(const AABB& aabb);
vec3 GetMax(const AABB& aabb);
AABB FromMinMax(const vec3& min, const vec3& max);

typedef struct OBB
{
    Point position;
    vec3 size;
    mat3 orientation;
    
    inline OBB() : size(1, 1, 1) {}
    inline OBB(const Point& p, const vec3& s) :
        position(p), size(s) {}
    inline OBB(const Point& p, const vec3& s, const mat3& o)
        : position(p), size(s), orientation(o) {}
    
} OBB;

typedef struct Plane
{
    vec3 normal;
    float distance;

    inline Plane() : normal(1, 0, 0){}
    inline Plane(const vec3& n, float d) :
        normal(n), distance(d){}

}Plane;

float PlaneEquation(const Point& point, const Plane& plane);

typedef struct Frustum
{
    union
    {
        struct 
        {
            Plane top;
            Plane bottom;
            Plane left;
            Plane right;
            Plane _near;
            Plane _far;
        };
        Plane planes[6];
    };
    inline Frustum() {}
} Frustum;

Point Intersection(Plane p1, Plane p2, Plane p3);
void GetCorners(const Frustum& frustum, vec3* outCorners);

bool Intersects(const Frustum& frustum, const Point& point);
bool Intersects(const Frustum& frustum, const Sphere& sphere);
bool Intersects(const Frustum& frustum, const AABB& aabb);
bool Intersects(const Frustum& frustum, const OBB& obb);

float Classify(const AABB& aabb, const Plane& plane);
float Classify(const OBB& obb, const Plane& plane);

typedef struct Triangle
{
    union
    {
        Point a;
        Point b;
        Point c;
    };

    Point points[3];
    float values[9];

    inline Triangle() {}
    inline Triangle(const Point& p1, const Point& p2, const Point& p3) :
        a(p1), b(p2), c(p3) {}
} Triangle;

typedef struct BVHNode
{   
    AABB bounds;
    BVHNode* children;
    int numTriangles;
    int* triangles;

    BVHNode() : children(0), numTriangles(0), triangles(0) {}
} BVHNode;

typedef struct Mesh 
{
    int numTriangles;
    union 
    {
        Triangle* triangles; // size =  numTriangles
        Point* vertices;     // size = numTriangles * 3;
        float* values;       // size = numTriangles * 3 * 3;
    };

    BVHNode* accelerator;
    Mesh() : numTriangles(0), values(0), accelerator(0) {}
    
} Mesh;


void AccelerateMesh(Mesh& mesh);
void SplitBVHNode(BVHNode* node, const Mesh& model, int depth);
void FreeBVH(BVHNode* node);

class Model
{
public:
    vec3 position;
    vec3 rotation;
    Model* parent;

    inline Model() : parent(0), content(0) {}
    inline Mesh* GetMesh() const 
    {
        return content;
    }

    inline AABB GetBounds() const
    {
        return bounds;
    }

    void SetContent(Mesh* mesh);
protected:
    Mesh* content;
    AABB bounds;
};

mat4 GetWorldMatrix(const Model& model);
OBB GetOBB(const Model& model);

typedef struct Interval 
{
    float min;
    float max;
} Interval;

Interval GetInterval(const AABB& rect, const vec3& axis);
Interval GetInterval(const OBB& rect, const vec3& axis);
Interval GetInterval(const Triangle& triangle, const vec3& axis);

bool OverlapOnAxis(const AABB& aabb, const OBB& obb, const vec3& axis);
bool OverlapOnAxis(const OBB& obb1, const OBB& obb2, const vec3& axis);
bool OverlapOnAxis(const AABB& aabb, const Triangle& triangle, const vec3& axis);
bool OverlapOnAxis(const OBB& obb, const Triangle& triangle, const vec3& axis);
bool OverlapOnAxis(const Triangle& triangle1, const Triangle& triangle2, const vec3& axis);

typedef struct RaycastResult
{
    vec3 point;
    vec3 normal;
    float t;
    bool hit;
} RaycastResult;  

void ResetRaycastResult(RaycastResult* outResult);

bool PointInSphere(const Point& point, const Sphere& sphere);
Point ClosestPoint(const Sphere& sphere, const Point& point);

bool PointInAABB(const Point& point, const AABB& aabb);
Point ClosestPoint(const AABB& aabb, const Point& point);

bool PointInOBB(const Point& point, const OBB& obb);
Point ClosestPoint(const OBB& obb, const Point& point);

bool PointOnPlane(const Point& point, const Plane& plane);
Point ClosestPoint(const Plane& plane, const Point& point);

bool PointInLine(const Point& point, const Line& line);
Point ClosestPoint(const Line& line, const Point& point);

bool PoinOnRay(const Point& point, const Ray& ray);
Point ClosestPoint(const Ray& ray, const Point& point);

bool PointInTriangle(const Point& point, const Triangle& triangle);
Point ClosestPoint(const Triangle& triangle, const Point& point);
Plane FromTriangle(const Triangle& triangle);

bool SphereSphere(const Sphere& s1, const Sphere& s2);
bool SphereAABB(const Sphere& sphere, const AABB& aabb);
bool SphereOBB(const Sphere& sphere, const OBB& obb);
bool SpherePlane(const Sphere& sphere, const Plane& plane);

bool AABBAABB(const AABB& aabb1, const AABB& aabb2);
bool AABBOBB(const AABB& aabb, const OBB& obb);
bool AABBPlane(const AABB& aabb, const Plane& plane);
bool OBBOBB(const OBB& obb1, const OBB& obb2);
bool OBBPlane(const OBB& obb, const Plane& plane);

bool PlanePlane(const Plane& plane1, const Plane& plane2);

bool TriangleSphere(const Triangle& triangle, const Sphere& sphere);
bool TriangleAABB(const Triangle& triangle, const AABB& aabb);
bool TriangleOBB(const Triangle& triangle, const OBB& obb);
bool TrianglePlane(const Triangle& triangle, const Plane& plane);
bool TriangleTriangle(const Triangle& triangle1, const Triangle& triangle2);
bool TriangleTriangleRobust(const Triangle& triangle1, const Triangle& triangle2);
vec3 SatCrossEdge(const vec3& a, const vec3& b, const vec3& c, const vec3& d);
vec3 Barycentric(const Point& point, const Triangle triangle);


bool MeshAABB(const Mesh& mesh, const AABB& aabb);
bool MeshSphere(const Mesh& mesh, const Sphere& sphere);
bool MeshOBB(const Mesh& mesh, const OBB& obb);
bool MeshPlane(const Mesh& mesh, const Plane& plane);
bool MeshTriangle(const Mesh& mesh, const Triangle& triangle);

float ModelRay(const Model& model, const Ray& ray);
bool ModelSphere(const Model& model, const Sphere& sphere);
bool ModelAABB(const Model& model, const AABB& aabb);
bool ModelOBB(const Model& model, const OBB& obb);
bool ModelPlane(const Model& model, const Plane& plane);
bool ModelTriangle(const Model& model, const Triangle& triangle);
bool Linetest(const Model& model, const Line& line);

bool Raycast(const Sphere& sphere, const Ray& ray, RaycastResult* outResult);
bool Raycast(const AABB& aabb, const Ray& ray, RaycastResult* outResult);
bool Raycast(const OBB& obb, const Ray& ray, RaycastResult* outResult);
bool Raycast(const Plane& plane, const Ray& ray, RaycastResult* outResult);
bool Raycast(const Triangle& triangle, const Ray& ray, RaycastResult* outResult);
float MeshRay(const Mesh& mesh, const Ray& ray);

bool Linetest(const Sphere& sphere, const Line& line);
bool Linetest(const AABB& aabb, const Line& line);
bool Linetest(const OBB& obb, const Line& line);
bool Linetest(const Plane& plane, const Line& line);
bool Linetest(const Triangle& triangle, const Line& line);
bool Linetest(const Mesh& mesh, const Line& line);

vec3 Unproject(const vec3& viewportPoint, const vec2& viewportOrigin, const vec2& viewportSize, const mat4& view, const mat4& projection);
Ray GetPickRay(const vec2& viewportPoint, const vec2& viewportOrigin, const vec2& viewportSize, const mat4& view, const mat4& projection);

typedef struct CollisionManifold
{
    bool colliding;
    vec3 normal;
    float depth;
    std::vector<vec3> contacts;
} CollisionManifold;

void ResetCollisionManifold(CollisionManifold* result);

CollisionManifold FindCollisions(const Sphere& sphereA, const Sphere& sphereB);
CollisionManifold FindCollisions(const OBB& obb, const Sphere& sphere);

std::vector<Point> GetVertices(const OBB& obb);
std::vector<Line> GetEdges(const OBB& obb);
std::vector<Plane> GetPlanes(const OBB& obb);
bool ClipToPlane(const Plane& plane, const Line& line, Point* outPoint);
std::vector<Point> ClipEdgesToOBB(std::vector<Line>& edges, const OBB& obb);
float PenetrationDepth(const OBB& obb1, const OBB& obb2, const vec3& axis, bool* outShouldFlip);
CollisionManifold FindCollisionFeatures(const OBB& obbA, const OBB& obbB);

#endif