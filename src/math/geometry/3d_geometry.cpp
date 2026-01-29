#include "3d_geometry.hpp"
#include <cmath>
#include <cfloat>
#include <list>

#define CMP(x, y) \
	(fabsf(x - y) <= FLT_EPSILON * fmaxf(1.0f, fmaxf(fabsf(x), fabsf(y))))

float Length(const Line& line)
{
	return Magnitude(line.start - line.end);
}

float LengthSq(const Line& line)
{
	return MagnitudeSqr(line.start - line.end);
}

Ray FromPoints(const Point& from, const Point& to)
{
	return Ray(from, Normalized(to - from));
}


vec3 GetMin(const AABB& aabb)
{
	vec3 p1 = aabb.position + aabb.size;
	vec3 p2 = aabb.position - aabb.size;

	return vec3(fminf(p1.x, p2.x),
		fminf(p1.y, p2.y),
		fminf(p1.z, p2.z));
}

vec3 GetMax(const AABB& aabb)
{
	vec3 p1 = aabb.position + aabb.size;
	vec3 p2 = aabb.position - aabb.size;

	return vec3(fmaxf(p1.x, p2.x),
		fmaxf(p1.y, p2.y),
		fmaxf(p1.z, p2.z));
}
AABB FromMinMax(const vec3& min, const vec3& max)
{
	return AABB((min + max) * 0.5f, (max - min) * 0.5f);
}

float PlaneEquation(const Point& point, const Plane& plane)
{
	return Dot(point, plane.normal) - plane.distance;
}

bool PointInSphere(const Point& point, const Sphere& sphere)
{
	return MagnitudeSqr(point - sphere.position) < (sphere.radius * sphere.radius);
}

bool PointOnPlane(const Point& point, const Plane& plane) {
	// This should probably use an epsilon!
	//return Dot(point, plane.normal) - plane.distance == 0.0f;
	float dot = Dot(point, plane.normal);
	return CMP(dot - plane.distance, 0.0f);
}

bool PointInAABB(const Point& point, const AABB& aabb) {
	Point min = GetMin(aabb);
	Point max = GetMax(aabb);

	if (point.x < min.x || point.y < min.y || point.z < min.z)
	{
		return false;
	}

	if (point.x > max.x || point.y > max.y || point.z > max.z)
	{
		return false;
	}

	return true;
}

bool PointInOBB(const Point& point, const OBB& obb) {
	vec3 dir = point - obb.position;

	for (int i = 0; i < 3; ++i) {
		const float* orientation = &obb.orientation.asArray[i * 3];
		vec3 axis(orientation[0], orientation[1], orientation[2]);

		float distance = Dot(dir, axis);

		if (distance > obb.size.asArray[i]) {
			return false;
		}
		if (distance < -obb.size.asArray[i]) {
			return false;
		}
	}

	return true;
}

Point ClosestPoint(const Sphere& sphere, const Point& point) {
	vec3 sphereToPoint = point - sphere.position;
	Normalize(sphereToPoint);
	sphereToPoint = sphereToPoint * sphere.radius;

	return sphereToPoint + sphere.position;
}

Point ClosestPoint(const AABB& aabb, const Point& point) {
	Point result = point;
	Point min = GetMin(aabb);
	Point max = GetMax(aabb);

	result.x = (result.x < min.x) ? min.x : result.x;
	result.y = (result.y < min.y) ? min.y : result.y;
	result.z = (result.z < min.z) ? min.z : result.z;

	result.x = (result.x > max.x) ? max.x : result.x;
	result.y = (result.y > max.y) ? max.y : result.y;
	result.z = (result.z > max.z) ? max.z : result.z;

	return point;
}

Point ClosestPoint(const OBB& obb, const Point& point) {
	Point result = obb.position;
	vec3 direction = point - obb.position;

	for (int i = 0; i < 3; i++)
	{
		const float* orientation = &obb.orientation.asArray[i * 3];

		vec3 axis(orientation[0], orientation[1], orientation[2]);

		float distance = Dot(direction, axis);

		if (distance > obb.size.asArray[i])
		{
			distance = obb.size.asArray[i];
		}

		if (distance < obb.size.asArray[i])
		{
			distance = -obb.size.asArray[i];
		}

		result = result + (axis * distance);
	}

	return result;
}

Point ClosestPoint(const Plane& plane, const Point& point) {
	float dot = Dot(plane.normal, point);
	float distance = dot - plane.distance;
	return point - plane.normal * distance;
}

bool PointOnLine(const Point& point, const Line& line) {
	Point closest = ClosestPoint(line, point);
	float distanceSqr = MagnitudeSqr(closest - point);
	return CMP(distanceSqr, 0.0f);
}

Point ClosestPoint(const Line& line, const Point& point) {
	vec3 lineVec = line.end - line.start;
	float t = Dot(point - line.start, lineVec) / Dot(lineVec, lineVec);

	t = fmaxf(t, 0.0f);
	t = fminf(t, 1.0f);

	return line.start + lineVec * t;
}

bool PointOnRay(const Point& point, const Ray& ray) {
	if (point == ray.origin)
	{
		return true;
	}
	vec3 norm = point - ray.origin;
	Normalize(norm);

	float diff = Dot(norm, ray.direction);

	return diff == 1.0f;
}

Point ClosestPoint(const Ray& ray, const Point& point) {
	float t = Dot(point - ray.origin, ray.direction);
	t = fmaxf(t, 0.0f);

	return Point(ray.origin + ray.direction * t);
}

bool SphereSphere(const Sphere& s1, const Sphere& s2) {
	float radiiSum = s1.radius + s2.radius;
	float distanceSqr = MagnitudeSqr(s1.position - s2.position);

	return distanceSqr < radiiSum * radiiSum;
}

bool SphereAABB(const Sphere& sphere, const AABB& aabb)
{
	Point closestPoint = ClosestPoint(aabb, sphere.position);
	float distSqr = MagnitudeSqr(sphere.position - closestPoint);
	float radiusSqr = sphere.radius * sphere.radius;
	return distSqr < radiusSqr;
}

bool SphereOBB(const Sphere& sphere, const OBB& obb)
{
	Point closestPoint = ClosestPoint(obb, sphere.position);
	float distSqr = MagnitudeSqr(sphere.position - closestPoint);
	float radiusSqr = sphere.radius * sphere.radius;
	return distSqr < radiusSqr;
}

bool SpherePlane(const Sphere& sphere, const Plane& plane)
{
	Point closestPoint = ClosestPoint(plane, sphere.position);
	float distSqr = MagnitudeSqr(sphere.position - closestPoint);
	float radiusSqr = sphere.radius * sphere.radius;
	return distSqr < radiusSqr;
}


bool AABBAABB(const AABB& aabb1, const AABB& aabb2)
{
	Point aMin = GetMin(aabb1);
	Point aMax = GetMax(aabb1);

	Point bMin = GetMin(aabb2);
	Point bMax = GetMax(aabb2);

	return (aMin.x <= bMax.x && aMax.x >= bMin.x) &&
		(aMin.y <= bMax.y && aMax.y >= bMin.y) &&
		(aMin.z <= bMax.z && aMax.z >= bMin.z);
}


bool AABBOBB(const AABB& aabb, const OBB& obb)
{
	const float* orientation = obb.orientation.asArray;

	vec3 test[15]
	{
		vec3(1, 0, 0), // aabb axis 1
		vec3(0, 1, 0), // aabb axis 2
		vec3(0, 0, 1), // aabb axis 3
		vec3(orientation[0], orientation[1], orientation[2]), // obb axis 1
		vec3(orientation[3], orientation[4], orientation[5]), // obb axis 2
		vec3(orientation[6], orientation[7], orientation[8]) // obb axis 3
	};

	for (int i = 0; i < 3; i++)
	{
		test[6 + i * 3 + 0] = Cross(test[i], test[2]);
		test[6 + i * 3 + 1] = Cross(test[i], test[5]);
		test[6 + i * 3 + 2] = Cross(test[i], test[8]);
	}

	for (int i = 0; i < 15; i++)
	{
		if (!OverlapOnAxis(aabb, obb, test[i]))
		{
			return false;
		}
	}

	return true;
}

bool AABBPlane(const AABB& aabb, const Plane& plane)
{
	float planeLength = aabb.size.x * fabsf(plane.normal.x) +
		aabb.size.y * fabsf(plane.normal.y) +
		aabb.size.z * fabsf(plane.normal.z);

	float dot = Dot(plane.normal, aabb.position);
	float distance = dot - plane.distance;

	return fabs(distance) <= planeLength;
}

bool OverlapOnAxis(const AABB& aabb, const OBB& obb, const vec3& axis)
{
	Interval a = GetInterval(aabb, axis);
	Interval b = GetInterval(obb, axis);

	return ((b.min <= a.max) && (a.min <= b.max));
}

bool OverlapOnAxis(const OBB& obb1, const OBB& obb2, const vec3& axis)
{
	Interval a = GetInterval(obb1, axis);
	Interval b = GetInterval(obb2, axis);

	return ((b.min <= a.max) && (a.min <= b.max));
}

bool OverlapOnAxis(const AABB& aabb, const Triangle& triangle, const vec3& axis)
{
	Interval a = GetInterval(aabb, axis);
	Interval b = GetInterval(triangle, axis);

	return ((b.min <= a.max) && (a.min <= b.max));
}


bool OverlapOnAxis(const OBB& obb, const Triangle& triangle, const vec3& axis)
{
	Interval a = GetInterval(obb, axis);
	Interval b = GetInterval(triangle, axis);

	return ((b.min <= a.max) && (a.min <= b.max));
}

bool OverlapOnAxis(const Triangle& triangle1, const Triangle& triangle2, const vec3& axis)
{
	Interval a = GetInterval(triangle1, axis);
	Interval b = GetInterval(triangle2, axis);

	return ((b.min <= a.max) && (a.min <= b.max));
}


Interval GetInterval(const Triangle& triangle, const vec3& axis)
{
	Interval result;
	result.min = Dot(axis, triangle.points[0]);
	result.max = result.min;

	for (int i = 1; i < 3; i++)
	{
		float value = Dot(axis, triangle.points[i]);
		result.min = fminf(result.min, value);
		result.max = fmaxf(result.max, value);
	}

	return result;
}

Interval GetInterval(const OBB& rect, const vec3& axis)
{
	vec3 vertex[8];

	vec3 CENTER = rect.position;
	vec3 EXTENSION = rect.size;
	const float* orientation = rect.orientation.asArray;
	vec3 A[] =
	{
		vec3(orientation[0], orientation[1], orientation[2]),
		vec3(orientation[3], orientation[4], orientation[5]),
		vec3(orientation[6], orientation[7], orientation[8]),
	};

	vertex[0] = CENTER + A[0] * EXTENSION[0] + A[1] * EXTENSION[1] + A[2] * EXTENSION[2];
	vertex[1] = CENTER - A[0] * EXTENSION[0] + A[1] * EXTENSION[1] + A[2] * EXTENSION[2];
	vertex[2] = CENTER + A[0] * EXTENSION[0] - A[1] * EXTENSION[1] + A[2] * EXTENSION[2];
	vertex[3] = CENTER + A[0] * EXTENSION[0] + A[1] * EXTENSION[1] - A[2] * EXTENSION[2];
	vertex[4] = CENTER - A[0] * EXTENSION[0] - A[1] * EXTENSION[1] - A[2] * EXTENSION[2];
	vertex[5] = CENTER + A[0] * EXTENSION[0] - A[1] * EXTENSION[1] - A[2] * EXTENSION[2];
	vertex[6] = CENTER - A[0] * EXTENSION[0] + A[1] * EXTENSION[1] - A[2] * EXTENSION[2];
	vertex[7] = CENTER - A[0] * EXTENSION[0] - A[1] * EXTENSION[1] + A[2] * EXTENSION[2];

	Interval result;
	result.min = result.max = Dot(axis, vertex[0]);
	for (int i = 1; i < 8; i++)
	{
		float projection = Dot(axis, vertex[i]);
		result.min = (projection < result.min) ? projection : result.min;
		result.max = (projection > result.max) ? projection : result.max;
	}

	return result;
}

Interval GetInterval(const AABB& rect, const vec3& axis)
{
	vec3 i = GetMin(rect);
	vec3 a = GetMax(rect);

	vec3 vertex[8] =
	{
		vec3(i.x, a.y, a.z),
		vec3(i.x, a.y, i.z),
		vec3(i.x, i.y, a.z),
		vec3(i.x, i.y, i.z),
		vec3(a.x, a.y, a.z),
		vec3(a.x, a.y, i.z),
		vec3(a.x, i.y, a.z),
		vec3(a.x, i.y, i.z)
	};

	Interval result;
	result.min = result.max = Dot(axis, vertex[0]);

	for (int i = 1; i < 8; i++)
	{
		float projection = Dot(axis, vertex[i]);
		result.min = (projection < result.min) ? projection : result.min;
		result.min = (projection > result.max) ? projection : result.max;
	}

	return result;
}



bool OBBOBB(const OBB& obb1, const OBB& obb2)
{
	const float* o1 = obb1.orientation.asArray;
	const float* o2 = obb2.orientation.asArray;
	vec3 test[15] =
	{
		vec3(o1[0], o1[1], o1[2]),
		vec3(o1[3], o1[4], o1[5]),
		vec3(o1[6], o1[7], o1[8]),
		vec3(o2[0], o2[1], o2[2]),
		vec3(o2[3], o2[4], o2[5]),
		vec3(o2[6], o2[7], o2[8])
	};

	for (int i = 0; i < 3; i++)
	{
		test[6 + i * 3 + 0] = Cross(test[i], test[0]);
		test[6 + i * 3 + 1] = Cross(test[i], test[1]);
		test[6 + i * 3 + 2] = Cross(test[i], test[2]);
	}

	for (int i = 0; i < 15; i++)
	{
		if (!OverlapOnAxis(obb1, obb2, test[i]))
		{
			return false;
		}
	}

	return true;
}

bool OBBPlane(const OBB& obb, const Plane& plane)
{
	const float* o = obb.orientation.asArray;

	vec3 rotation[] =
	{
		vec3(o[0], o[1], o[2]),
		vec3(o[3], o[4], o[5]),
		vec3(o[6], o[7], o[8])
	};

	vec3 normal = plane.normal;

	float planeLength = obb.size.x * fabsf(Dot(normal, rotation[0])) +
		obb.size.y * fabsf(Dot(normal, rotation[1])) +
		obb.size.z * fabsf(Dot(normal, rotation[2]));

	float dot = Dot(plane.normal, obb.position);
	float distance = dot - plane.distance;

	return fabsf(distance) <= planeLength;
}

bool PlanePlane(const Plane& plane1, const Plane& plane2)
{
	vec3 d = Cross(plane1.normal, plane2.normal);
	return Dot(d, d) != 0;
}


bool Raycast(const Sphere& sphere, const Ray& ray, RaycastResult* outResult) {
	ResetRaycastResult(outResult);

	vec3 e = sphere.position - ray.origin;

	float radiusSqr = sphere.radius * sphere.radius;
	float eSqr = MagnitudeSqr(e);

	float a = Dot(e, ray.direction);

	float bSqr = eSqr - (a * a);
	float f = sqrt(radiusSqr - bSqr);

	float t = a - f;
	if (radiusSqr - (eSqr - (a * a)) < 0.0f)
	{
		return false;
	}
	else if (eSqr < radiusSqr)
	{
		t = a + f;
	}

	if (outResult != 0)
	{
		outResult->t = t;
		outResult->hit = true;
		outResult->point = ray.origin + ray.direction * t;
		outResult->normal = Normalized(outResult->point - sphere.position);
	}
	return true;
}

bool Raycast(const OBB& obb, const Ray& ray, RaycastResult* outResult) {
	ResetRaycastResult(outResult);

	const float* o = obb.orientation.asArray;
	const float* size = obb.size.asArray;

	vec3 X(o[0], o[1], o[2]);
	vec3 Y(o[3], o[4], o[5]);
	vec3 Z(o[6], o[7], o[8]);

	vec3 p = obb.position - ray.origin;
	vec3 f(
		Dot(X, ray.direction),
		Dot(Y, ray.direction),
		Dot(Z, ray.direction)
	);

	vec3 e(
		Dot(X, p),
		Dot(Y, p),
		Dot(Z, p)
	);

	float t[6] = { 0, 0, 0, 0, 0, 0 };
	for (int i = 0; i < 3; i++)
	{
		if (CMP(f[i], 0))
		{
			if (-e[i] - size[i] > 0 || -e[i] + size[i] < 0)
			{
				return -1;
			}
			f[i] = 0.00001f;
		}
		t[i * 2 + 0] = (e[i] + size[i]) / f[i];
		t[i * 2 + 1] = (e[i] - size[i]) / f[i];
	}

	float tmin = fmaxf(
		fmaxf(
			fminf(t[0], t[1]),
			fminf(t[2], t[3])
		),
		fminf(t[4], t[5])
	);

	float tmax = fminf(
		fminf(
			fmaxf(t[0], t[1]),
			fmaxf(t[2], t[3])
		),
		fmaxf(t[4], t[5])
	);

	if (tmax < 0)
	{
		return false;
	}

	if (tmin > tmax)
	{
		return false;
	}

	float t_result = tmin;
	if (tmin < 0.0f)
	{
		t_result = tmax;
	}

	if (outResult != 0)
	{
		outResult->t = t_result;
		outResult->hit = true;
		outResult->point = ray.origin + ray.direction * t_result;
		vec3 normals[] = {
			X, X * -1.0f,
			Y, Y * -1.0f,
			Z, Z * -1.0f
		};

		for (int i = 0; i < 6; i++)
		{
			if (CMP(t_result, t[i]))
			{
				outResult->normal = Normalized(normals[i]);
			}
		}
	}

	return true;
}

void ResetRaycastResult(RaycastResult* outResult) {
	if (outResult != 0)
	{
		outResult->t = -1;
		outResult->hit = false;
		outResult->normal = vec3(0, 0, 1);
		outResult->point = vec3(0, 0, 0);
	}
}

bool Raycast(const AABB& aabb, const Ray& ray, RaycastResult* outResult) {
	ResetRaycastResult(outResult);

	vec3 min = GetMin(aabb);
	vec3 max = GetMax(aabb);

	float t1 = (min.x - ray.origin.x) / ray.direction.x;
	float t2 = (max.x - ray.origin.x) / ray.direction.x;
	float t3 = (min.y - ray.origin.y) / ray.direction.y;
	float t4 = (max.y - ray.origin.y) / ray.direction.y;
	float t5 = (min.z - ray.origin.z) / ray.direction.z;
	float t6 = (max.z - ray.origin.z) / ray.direction.z;

	float tmin = fmaxf(
		fmaxf(
			fminf(t1, t2),
			fminf(t3, t4)
		),
		fminf(t5, t6)
	);

	float tmax = fminf(
		fminf(
			fmaxf(t1, t2),
			fmaxf(t3, t4)
		),
		fmaxf(t5, t6)
	);

	if (tmax < 0)
	{
		return false;
	}

	if (tmin > tmax)
	{
		return false;
	}

	float t_result = tmin;
	if (tmin < 0.0f)
	{
		return t_result = tmax;
	}

	if (outResult != 0)
	{
		outResult->t = t_result;
		outResult->hit = true;
		outResult->point = ray.origin + ray.direction * t_result;
		vec3 normals[] = {
			vec3(-1, 0, 0), vec3(1, 0, 0),
			vec3(0, -1, 0), vec3(0, 1, 0),
			vec3(0, 0, -1), vec3(0, 0, 1)
		};

		float t[] = { t1, t2, t3, t4, t5, t6 };
		for (int i = 0; i < 6; i++)
		{
			if (CMP(t_result, t[i]))
			{
				outResult->normal = normals[i];
			}
		}
	}

	return true;
}

bool Raycast(const Plane& plane, const Ray& ray, RaycastResult* outResult) {
	ResetRaycastResult(outResult);

	float nd = Dot(ray.direction, plane.normal);
	float pn = Dot(ray.origin, plane.normal);

	if (nd >= 0.0f)
	{
		return false;
	}

	float t = (plane.distance - pn) / nd;


	if (t >= 0.0f)
	{
		if (outResult != 0)
		{
			outResult->t = t;
			outResult->hit = true;
			outResult->point = ray.origin + ray.direction * t;
			outResult->normal = Normalized(plane.normal);
		}
		return true;
	}

	return false;
}

bool Linetest(const Sphere& sphere, const Line& line) {
	Point closest = ClosestPoint(line, sphere.position);
	float distSqr = MagnitudeSqr(sphere.position - closest);
	return distSqr < (sphere.radius * sphere.radius);
}

bool Linetest(const Plane& plane, const Line& line) {
	vec3 ab = line.end - line.start;

	float nA = Dot(plane.normal, line.start);
	float nAB = Dot(plane.normal, ab);

	if (nAB == 0.0f)
	{
		return false;
	}

	float t = (plane.distance - nA) / nAB;
	return t >= 0.0f && t <= 1.0f;
}

bool Linetest(const AABB& aabb, const Line& line) {
	Ray ray;
	ray.origin = line.start;
	ray.direction = Normalized(line.end - line.start);
	RaycastResult raycast;
	if (!Raycast(aabb, ray, &raycast))
	{
		return false;
	}

	float t = raycast.t;
	return t >= 0 && t * t <= LengthSq(line);
}

bool Linetest(const OBB& obb, const Line& line) {
	Ray ray;
	ray.origin = line.start;
	ray.direction = Normalized(line.end - line.start);
	RaycastResult raycast;
	if (!Raycast(obb, ray, &raycast))
	{
		return false;
	}

	float t = raycast.t;

	return t >= 0 && t * t <= LengthSq(line);
}

bool PointInTriangle(const Point& point, const Triangle& triangle) {
	// Move the triangle so that the point is  
	// now at the origin of the triangle
	vec3 a = triangle.a - point;
	vec3 b = triangle.b - point;
	vec3 c = triangle.c - point;

	vec3 normalPBC = Cross(b, c);
	vec3 normalPCA = Cross(c, a);
	vec3 normalPAB = Cross(a, b);

	if (Dot(normalPBC, normalPCA) < 0.0f)
	{
		return false;
	}

	if (Dot(normalPBC, normalPAB) < 0.0f)
	{
		return false;
	}

	return true;
}

vec3 Barycentric(const Point& point, const Triangle& triangle) {
	vec3 ap = point - triangle.a;
	vec3 bp = point - triangle.b;
	vec3 cp = point - triangle.c;

	vec3 ab = triangle.b - triangle.a;
	vec3 ac = triangle.c - triangle.a;
	vec3 bc = triangle.c - triangle.b;
	vec3 cb = triangle.b - triangle.c;
	vec3 ca = triangle.a - triangle.c;

	vec3 v = ab - Project(ab, cb);
	float a = 1.0f - (Dot(v, ap) / Dot(v, ab));

	v = bc - Project(bc, ac);
	float b = 1.0f - (Dot(v, bp) / Dot(v, bc));

	v = ca - Project(ca, ab);
	float c = 1.0f - (Dot(v, cp) / Dot(v, ca));

	return vec3(a, b, c);
}

Plane FromTriangle(const Triangle& triangle) {
	Plane result;
	result.normal = Normalized(Cross(triangle.b - triangle.a, triangle.c - triangle.a));
	result.distance = Dot(result.normal, triangle.a);
	return result;
}

Point ClosestPoint(const Triangle& t, const Point& p) {
	Plane plane = FromTriangle(t);
	Point closest = ClosestPoint(plane, p);
	if (PointInTriangle(closest, t))
	{
		return closest;
	}

	Point c1 = ClosestPoint(Line(t.a, t.b), p); // Line AB
	Point c2 = ClosestPoint(Line(t.b, t.c), p); // Line AB
	Point c3 = ClosestPoint(Line(t.c, t.a), p); // Line AB

	float magSqr1 = MagnitudeSqr(p - c1);
	float magSqr2 = MagnitudeSqr(p - c2);
	float magSqr3 = MagnitudeSqr(p - c3);

	if (magSqr1 < magSqr2 && magSqr1 < magSqr3)
	{
		return c1;
	}
	else if (magSqr2 < magSqr1 && magSqr2 < magSqr3)
	{
		return c2;
	}

	return c3;
}

bool TriangleSphere(const Triangle& triangle, const Sphere& sphere) {
	Point closest = ClosestPoint(triangle, sphere.position);
	float magSqr = MagnitudeSqr(closest - sphere.position);
	return magSqr <= sphere.radius * sphere.radius;
}

bool TriangleAABB(const Triangle& triangle, const AABB& aabb) {
	vec3 f0 = triangle.b - triangle.a;
	vec3 f1 = triangle.c - triangle.b;
	vec3 f2 = triangle.a - triangle.c;

	vec3 u0(1.0f, 0.0f, 0.0f);
	vec3 u1(0.0f, 1.0f, 0.0f);
	vec3 u2(0.0f, 0.0f, 1.0f);

	vec3 test[13] =
	{
		u0,
		u1,
		u2,
		Cross(f0, f1),
		Cross(u0, f0), Cross(u0, f1), Cross(u0, f2),
		Cross(u1, f0), Cross(u1, f1), Cross(u1, f2),
		Cross(u2, f0), Cross(u2, f1), Cross(u2, f2)
	};

	for (int i = 0; i < 13; i++)
	{
		if (!OverlapOnAxis(aabb, triangle, test[i]))
		{
			return false;
		}
	}

	return true;
}

bool TriangleOBB(const Triangle& triangle, const OBB& obb) {
	vec3 f0 = triangle.b - triangle.a;
	vec3 f1 = triangle.c - triangle.b;
	vec3 f2 = triangle.a - triangle.c;

	const float* orientation = obb.orientation.asArray;

	vec3 u0(orientation[0], orientation[1], orientation[2]);
	vec3 u1(orientation[3], orientation[4], orientation[5]);
	vec3 u2(orientation[6], orientation[7], orientation[8]);

	vec3 test[13] =
	{
		u0,
		u1,
		u2,
		Cross(f0, f1),
		Cross(u0, f0), Cross(u0, f1), Cross(u0, f2),
		Cross(u1, f0), Cross(u1, f1), Cross(u1, f2),
		Cross(u2, f0), Cross(u2, f1), Cross(u2, f2)
	};

	for (int i = 0; i < 13; i++)
	{
		if (!OverlapOnAxis(obb, triangle, test[i]))
		{
			return false;
		}
	}

	return true;
}

bool TriangleTriangle(const Triangle& triangle1, const Triangle& triangle2) {
	vec3 t1_f0 = triangle1.b - triangle1.a; // Triangle 1, Edge 0
	vec3 t1_f1 = triangle1.c - triangle1.b; // Triangle 1, Edge 1
	vec3 t1_f2 = triangle1.a - triangle1.c; // Triangle 1, Edge 2

	vec3 t2_f0 = triangle2.b - triangle2.a; // Triangle 2, Edge 0
	vec3 t2_f1 = triangle2.c - triangle2.b; // Triangle 2, Edge 1
	vec3 t2_f2 = triangle2.a - triangle2.c; // Triangle 2, Edge 2

	vec3 axisToTest[] =
	{
		Cross(t1_f0, t1_f1),
		Cross(t2_f0, t2_f1),
		Cross(t2_f0, t1_f0), Cross(t2_f0, t1_f1),
		Cross(t2_f0, t1_f2), Cross(t2_f1, t1_f0),
		Cross(t2_f1, t1_f1), Cross(t2_f1, t1_f2),
		Cross(t2_f2, t1_f0), Cross(t2_f2, t1_f1),
		Cross(t2_f2, t1_f2),
	};

	for (int i = 0; i < 11; i++)
	{
		if (!OverlapOnAxis(triangle1, triangle2, axisToTest[i]))
		{
			return false;
		}
	}

	return true;
}


bool TriangleTriangleRobust(const Triangle& triangle1, const Triangle& triangle2) {
	vec3 axisToTest[] =
	{
		SatCrossEdge(triangle1.a, triangle1.b, triangle1.b, triangle1.c),
		SatCrossEdge(triangle2.a, triangle2.b, triangle2.b, triangle2.c),
		SatCrossEdge(triangle2.a, triangle2.b, triangle1.a, triangle1.b),
		SatCrossEdge(triangle2.a, triangle2.b, triangle1.b, triangle1.c),
		SatCrossEdge(triangle2.a, triangle2.b, triangle1.c, triangle1.a),
		SatCrossEdge(triangle2.b, triangle2.c, triangle1.a, triangle1.b),
		SatCrossEdge(triangle2.b, triangle2.c, triangle1.b, triangle1.c),
		SatCrossEdge(triangle2.b, triangle2.c, triangle1.c, triangle1.a),
		SatCrossEdge(triangle2.c, triangle2.a, triangle1.a, triangle1.b),
		SatCrossEdge(triangle2.c, triangle2.a, triangle1.b, triangle1.c),
		SatCrossEdge(triangle2.c, triangle2.a, triangle1.c, triangle1.a),
	};


	for (int i = 0; i < 11; i++)
	{
		if (!OverlapOnAxis(triangle1, triangle2, axisToTest[i]))
		{
			if (!CMP(MagnitudeSqr(axisToTest[i]), 0)) {
				return false;
			}
		}
	}

	return true;
}

vec3 SatCrossEdge(const vec3& a, const vec3& b, const vec3& c, const vec3& d) {
	vec3 ab = a - b;
	vec3 cd = c - d;
	vec3 result = Cross(ab, cd);

	if (!CMP(MagnitudeSqr(result), 0))
	{
		return result;
	}
	else
	{
		vec3 axis = Cross(ab, c - a);
		result = Cross(ab, axis);
		if (!CMP(MagnitudeSqr(result), 0))
		{
			return result;
		}
	}

	return vec3();
}


bool Raycast(const Triangle& triangle, const Ray& ray, RaycastResult* outResult) {
	ResetRaycastResult(outResult);

	Plane plane = FromTriangle(triangle);
	RaycastResult planeResult;
	if (!Raycast(plane, ray, &planeResult))
	{
		return false;
	}

	float t = planeResult.t;

	Point result = ray.origin * ray.direction * t;
	vec3 barycentric = Barycentric(result, triangle);

	if (barycentric.x >= 0.0f && barycentric.x <= 1.0f &&
		barycentric.y >= 0.0f && barycentric.y <= 1.0f &&
		barycentric.z >= 0.0f && barycentric.z <= 1.0f)
	{
		if (outResult != 0)
		{
			outResult->t = t;
			outResult->hit = true;
			outResult->point = ray.origin + ray.direction * t;
			outResult->normal = plane.normal;
		}

		return true;
	}

	return false;
}

bool Linetest(const Triangle& triangle, const Line& line) {
	Ray ray;
	ray.origin = line.start;
	ray.direction = Normalized(line.end - line.start);

	RaycastResult raycast;
	if (!Raycast(triangle, ray, &raycast))
	{
		return false;
	}

	float t = raycast.t;
	return t >= 0.0f && t * t <= LengthSq(line);
}

void AccelerateMesh(Mesh& mesh) {
	if (mesh.accelerator != 0)
	{
		return;
	}

	vec3 min = mesh.vertices[0];
	vec3 max = mesh.vertices[0];

	for (int i = 1; i < mesh.numTriangles * 3; i++)
	{
		min.x = fminf(mesh.vertices[i].x, min.x);
		min.y = fminf(mesh.vertices[i].y, min.y);
		min.y = fminf(mesh.vertices[i].z, min.z);

		max.x = fmaxf(mesh.vertices[i].x, max.x);
		max.y = fmaxf(mesh.vertices[i].y, max.y);
		max.y = fmaxf(mesh.vertices[i].z, max.z);
	}

	mesh.accelerator = new BVHNode();
	mesh.accelerator->bounds = FromMinMax(min, max);
	mesh.accelerator->numTriangles = mesh.numTriangles;
	mesh.accelerator->triangles = new int[mesh.numTriangles];
	for (int i = 0; i < mesh.numTriangles; i++)
	{
		mesh.accelerator->triangles[i] = i;
	}

	SplitBVHNode(mesh.accelerator, mesh, 3);
}

void SplitBVHNode(BVHNode* node, const Mesh& model, int depth) {
	if (depth-- <= 0)
	{
		return;
	}

	if (node->children == 0)
	{
		if (node->triangles > 0)
		{
			node->children = new BVHNode[8];
			vec3 center = node->bounds.position;
			vec3 extents = node->bounds.size * 0.5f;

			node->children[0].bounds = AABB(center + vec3(-extents.x, +extents.y, -extents.z), extents);
			node->children[1].bounds = AABB(center + vec3(+extents.x, +extents.y, -extents.z), extents);
			node->children[2].bounds = AABB(center + vec3(-extents.x, +extents.y, +extents.z), extents);
			node->children[3].bounds = AABB(center + vec3(+extents.x, +extents.y, +extents.z), extents);
			node->children[4].bounds = AABB(center + vec3(-extents.x, -extents.y, -extents.z), extents);
			node->children[5].bounds = AABB(center + vec3(+extents.x, -extents.y, -extents.z), extents);
			node->children[6].bounds = AABB(center + vec3(-extents.x, -extents.y, +extents.z), extents);
			node->children[7].bounds = AABB(center + vec3(+extents.x, -extents.y, +extents.z), extents);
		}
	}

	if (node->children != 0 && node->numTriangles > 0)
	{
		for (int i = 0; i < 8; i++)
		{
			node->children[i].numTriangles = 0;
			for (int j = 0; j < node->numTriangles; j++)
			{
				Triangle t = model.triangles[node->triangles[j]];
				if (TriangleAABB(t, node->children[i].bounds))
				{
					node->children[i].numTriangles += 1;
				}
			}

			if (node->children[i].numTriangles == 0)
			{
				continue;
			}

			node->children[i].triangles = new int[node->children[i].numTriangles];
			int index = 0;
			for (int j = 0; j < node->numTriangles; j++)
			{
				Triangle t = model.triangles[node->triangles[j]];
				if (TriangleAABB(t, node->children[i].bounds))
				{
					node->children[i].triangles[index++] = node->triangles[j];
				}
			}
		}

		node->numTriangles = 0;
		delete[] node->triangles;
		node->triangles = 0;

		for (int i = 0; i < 8; i++)
		{
			SplitBVHNode(&node->children[i], model, depth);
		}
	}
}

void FreeBVH(BVHNode* node) {
	if (node->children != 0)
	{
		for (int i = 0; i < 8; i++)
		{
			FreeBVH(&node->children[i]);
		}
		delete[] node->children;
		node->children = 0;
	}

	if (node->numTriangles != 0 || node->triangles != 0)
	{
		delete[] node->triangles;
		node->triangles = 0;
		node->numTriangles = 0;
	}
}

bool MeshAABB(const Mesh& mesh, const AABB& aabb) {
	if (mesh.accelerator == 0)
	{
		for (int i = 0; i < mesh.numTriangles; i++)
		{
			if (TriangleAABB(mesh.triangles[i], aabb))
			{
				return true;
			}
		}
	}
	else
	{
		std::list<BVHNode*> toProcess;
		toProcess.push_front(mesh.accelerator);

		while (!toProcess.empty())
		{
			BVHNode* iterator = *(toProcess.begin());
			toProcess.erase(toProcess.begin());
			if (iterator->numTriangles >= 0)
			{
				for (int i = 0; i < iterator->numTriangles; i++)
				{

					if (TriangleAABB(mesh.triangles[iterator->triangles[i]], aabb))
					{
						return true;
					}
				}
			}

			if (iterator->children != 0)
			{
				for (int i = 8 - 1; i >= 0; --i)
				{
					if (AABBAABB(iterator->children[i].bounds, aabb))
					{
						toProcess.push_front(&iterator->children[i]);
					}
				}
			}
		}
	}
	return false;
}

bool Linetest(const Mesh& mesh, const Line& line) {
	if (mesh.accelerator == 0)
	{
		for (int i = 0; i < mesh.numTriangles; i++)
		{
			if (Linetest(mesh.triangles[i], line))
			{
				return true;
			}
		}
	}
	else
	{
		std::list<BVHNode*> toProcess;
		toProcess.push_front(mesh.accelerator);

		while (!toProcess.empty())
		{
			BVHNode* iterator = *(toProcess.begin());
			toProcess.erase(toProcess.begin());
			if (iterator->numTriangles >= 0)
			{
				for (int i = 0; i < iterator->numTriangles; i++)
				{

					if (Linetest(mesh.triangles[iterator->triangles[i]], line))
					{
						return true;
					}
				}
			}

			if (iterator->children != 0)
			{
				for (int i = 8 - 1; i >= 0; --i)
				{
					if (Linetest(iterator->children[i].bounds, line))
					{
						toProcess.push_front(&iterator->children[i]);
					}
				}
			}
		}
	}
	return false;
}

bool MeshSphere(const Mesh& mesh, const Sphere& sphere) {
	if (mesh.accelerator == 0)
	{
		for (int i = 0; i < mesh.numTriangles; i++)
		{
			if (TriangleSphere(mesh.triangles[i], sphere))
			{
				return true;
			}
		}
	}
	else
	{
		std::list<BVHNode*> toProcess;
		toProcess.push_front(mesh.accelerator);

		while (!toProcess.empty())
		{
			BVHNode* iterator = *(toProcess.begin());
			toProcess.erase(toProcess.begin());
			if (iterator->numTriangles >= 0)
			{
				for (int i = 0; i < iterator->numTriangles; i++)
				{

					if (TriangleSphere(mesh.triangles[iterator->triangles[i]], sphere))
					{
						return true;
					}
				}
			}

			if (iterator->children != 0)
			{
				for (int i = 8 - 1; i >= 0; --i)
				{
					if (SphereAABB(sphere, iterator->children[i].bounds))
					{
						toProcess.push_front(&iterator->children[i]);
					}
				}
			}
		}
	}
	return false;
}

bool MeshOBB(const Mesh& mesh, const OBB& obb) {
	if (mesh.accelerator == 0)
	{
		for (int i = 0; i < mesh.numTriangles; i++)
		{
			if (TriangleOBB(mesh.triangles[i], obb))
			{
				return true;
			}
		}
	}
	else
	{
		std::list<BVHNode*> toProcess;
		toProcess.push_front(mesh.accelerator);

		while (!toProcess.empty())
		{
			BVHNode* iterator = *(toProcess.begin());
			toProcess.erase(toProcess.begin());
			if (iterator->numTriangles >= 0)
			{
				for (int i = 0; i < iterator->numTriangles; i++)
				{

					if (TriangleOBB(mesh.triangles[iterator->triangles[i]], obb))
					{
						return true;
					}
				}
			}

			if (iterator->children != 0)
			{
				for (int i = 8 - 1; i >= 0; --i)
				{
					if (AABBOBB(iterator->children[i].bounds, obb))
					{
						toProcess.push_front(&iterator->children[i]);
					}
				}
			}
		}
	}
	return false;
}

bool MeshPlane(const Mesh& mesh, const Plane& plane) {
	if (mesh.accelerator == 0)
	{
		for (int i = 0; i < mesh.numTriangles; i++)
		{
			if (TrianglePlane(mesh.triangles[i], plane))
			{
				return true;
			}
		}
	}
	else
	{
		std::list<BVHNode*> toProcess;
		toProcess.push_front(mesh.accelerator);

		while (!toProcess.empty())
		{
			BVHNode* iterator = *(toProcess.begin());
			toProcess.erase(toProcess.begin());
			if (iterator->numTriangles >= 0)
			{
				for (int i = 0; i < iterator->numTriangles; i++)
				{

					if (TrianglePlane(mesh.triangles[iterator->triangles[i]], plane))
					{
						return true;
					}
				}
			}

			if (iterator->children != 0)
			{
				for (int i = 8 - 1; i >= 0; --i)
				{
					if (AABBPlane(iterator->children[i].bounds, plane))
					{
						toProcess.push_front(&iterator->children[i]);
					}
				}
			}
		}
	}
	return false;
}

bool MeshTriangle(const Mesh& mesh, const Triangle& triangle) {
	if (mesh.accelerator == 0)
	{
		for (int i = 0; i < mesh.numTriangles; i++)
		{
			if (TriangleTriangle(mesh.triangles[i], triangle))
			{
				return true;
			}
		}
	}
	else
	{
		std::list<BVHNode*> toProcess;
		toProcess.push_front(mesh.accelerator);

		while (!toProcess.empty())
		{
			BVHNode* iterator = *(toProcess.begin());
			toProcess.erase(toProcess.begin());
			if (iterator->numTriangles >= 0)
			{
				for (int i = 0; i < iterator->numTriangles; i++)
				{

					if (TriangleTriangle(mesh.triangles[iterator->triangles[i]], triangle))
					{
						return true;
					}
				}
			}

			if (iterator->children != 0)
			{
				for (int i = 8 - 1; i >= 0; --i)
				{
					if (AABBTriangle(iterator->children[i].bounds, triangle))
					{
						toProcess.push_front(&iterator->children[i]);
					}
				}
			}
		}
	}
	return false;
}

float MeshRay(const Mesh& mesh, const Ray& ray) {
	if (mesh.accelerator == 0)
	{
		for (int i = 0; i < mesh.numTriangles; i++)
		{
			RaycastResult raycast;
			Raycast(mesh.triangles[i], ray, &raycast);
			float result = raycast.t;
			if (result >= 0)
			{
				return result;
			}
		}
	}
	else
	{
		std::list<BVHNode*> toProcess;
		toProcess.push_front(mesh.accelerator);

		while (!toProcess.empty())
		{
			BVHNode* iterator = *(toProcess.begin());
			toProcess.erase(toProcess.begin());
			if (iterator->numTriangles >= 0)
			{
				for (int i = 0; i < iterator->numTriangles; i++)
				{
					RaycastResult raycast;
					Raycast(mesh.triangles[iterator->triangles[i]], ray, &raycast);
					float result = raycast.t;
					if (result >= 0)
					{
						return result;
					}
				}
			}

			if (iterator->children != 0)
			{
				for (int i = 8 - 1; i >= 0; --i)
				{
					RaycastResult raycast;
					Raycast(iterator->children[i].bounds, ray, &raycast);
					if (raycast.t >= 0)
					{
						toProcess.push_front(&iterator->children[i]);
					}
				}
			}
		}
	}
	return -1;
}

bool TrianglePlane(const Triangle& triangle, const Plane& plane) {
	float side1 = PlaneEquation(triangle.a, plane);
	float side2 = PlaneEquation(triangle.b, plane);
	float side3 = PlaneEquation(triangle.c, plane);

	if (CMP(side1, 0) && CMP(side2, 0) && CMP(side3, 0))
	{
		return true;
	}

	if (side1 > 0 && side2 > 0 && side3 > 0)
	{
		return false;
	}

	if (side1 < 0 && side2 < 0 && side3 < 0)
	{
		return false;
	}

	return true;
}

void Model::SetContent(Mesh* mesh) {
	content = mesh;
	if (content != 0)
	{
		vec3 min = mesh->vertices[0];
		vec3 max = mesh->vertices[0];

		for (int i = 1; i < mesh->numTriangles * 3; i++)
		{
			min.x = fminf(mesh->vertices[i].x, min.x);
			min.y = fminf(mesh->vertices[i].y, min.y);
			min.y = fminf(mesh->vertices[i].z, min.z);

			max.x = fmaxf(mesh->vertices[i].x, max.x);
			max.y = fmaxf(mesh->vertices[i].y, max.y);
			max.y = fmaxf(mesh->vertices[i].z, max.z);
		}

		bounds = FromMinMax(min, max);
	}
}

mat4 GetWorldMatrix(const Model& model)
{
	mat4 translation = Translation(model.position);
	mat4 rotation = Rotation(
		model.rotation.x,
		model.rotation.y,
		model.rotation.z
	);
	mat4 localMat = rotation * translation;
	mat4 parentMat;
	if (model.parent != 0)
	{
		parentMat = GetWorldMatrix(*model.parent);
	}
	return localMat * parentMat;
}

OBB GetOBB(const Model& model)
{
	mat4 world = GetWorldMatrix(model);
	AABB aabb = model.GetBounds();
	OBB obb;

	obb.size = aabb.size;
	obb.position = MultiplyPoint(aabb.position, world);
	obb.orientation = Cut(world, 3, 3);
	return obb;
}


float ModelRay(const Model& model, const Ray& ray)
{
	mat4 world = GetWorldMatrix(model);
	mat4 inv = Inverse(world);

	Ray local;
	local.origin = MultiplyPoint(ray.origin, inv);
	local.direction = MultiplyVector(ray.direction, inv);
	local.NormalizeDirection();

	if (model.GetMesh() != 0)
	{
		return MeshRay(*model.GetMesh(), local);
	}
	return -1;
}

bool Linetest(const Model& model, const Line& line)
{
	mat4 world = GetWorldMatrix(model);
	mat4 inv = Inverse(world);

	Line local;
	local.start = MultiplyPoint(line.start, inv);
	local.end = MultiplyPoint(line.end, inv);

	if (model.GetMesh() != 0)
	{
		return Linetest(*(model.GetMesh()), local);
	}
	return false;
}

bool ModelSphere(const Model& model, const Sphere& sphere) {
	mat4 world = GetWorldMatrix(model);
	mat4 inv = Inverse(world);

	Sphere local;
	local.position = MultiplyPoint(sphere.position, inv);
	if (model.GetMesh() != 0)
	{
		return MeshSphere(*(model.GetMesh()), local);
	}
	return false;
}

bool ModelAABB(const Model& model, const AABB& aabb) {
	mat4 world = GetWorldMatrix(model);
	mat4 inv = Inverse(world);

	OBB local;
	local.size = aabb.size;
	local.position = MultiplyPoint(aabb.position, inv);
	local.orientation = Cut(inv, 3, 3);

	if (model.GetMesh() != 0)
	{
		return MeshOBB(*(model.GetMesh()), local);
	}

	return false;
}

bool ModelOBB(const Model& model, const OBB& obb) {
	mat4 world = GetWorldMatrix(model);
	mat4 inv = Inverse(world);

	OBB local;
	local.size = obb.size;
	local.position = MultiplyPoint(obb.position, inv);
	local.orientation = obb.orientation * Cut(inv, 3, 3);

	if (model.GetMesh() != 0)
	{
		return MeshOBB(*(model.GetMesh()), local);
	}

	return false;
}

bool ModelPlane(const Model& model, const Plane& plane)
{
	mat4 world = GetWorldMatrix(model);
	mat4 inv = Inverse(world);

	Plane local;
	local.normal = MultiplyVector(plane.normal, inv);
	local.distance = plane.distance;

	if (model.GetMesh() != 0)
	{
		return MeshPlane(*(model.GetMesh()), local);
	}

	return false;
}

bool ModelTriangle(const Model& model, const Triangle& triangle)
{
	mat4 world = GetWorldMatrix(model);
	mat4 inv = Inverse(world);

	Triangle local;
	local.a = MultiplyPoint(triangle.a, inv);
	local.b = MultiplyPoint(triangle.b, inv);
	local.c = MultiplyPoint(triangle.c, inv);

	if (model.GetMesh() != 0)
	{
		return MeshTriangle(*(model.GetMesh()), local);
	}

	return false;
}

Point Intersection(Plane p1, Plane p2, Plane p3) {
	mat3 D(
		p1.normal.x, p2.normal.x, p3.normal.x,
		p1.normal.y, p2.normal.y, p3.normal.y,
		p1.normal.z, p2.normal.z, p3.normal.z
	);

	vec3 A(-p1.distance, -p2.distance, -p3.distance);

	mat3 Dx = D;
	mat3 Dy = D;
	mat3 Dz = D;
	Dx._11 = A.x; Dx._12 = A.y; Dx._13 = A.z;
	Dy._21 = A.x; Dy._22 = A.y; Dy._23 = A.z;
	Dz._31 = A.x; Dz._32 = A.y; Dz._33 = A.z;

	float detD = Determinant(D);
	if (CMP(detD, 0))
	{
		return Point();
	}

	float detDx = Determinant(Dx);
	float detDy = Determinant(Dy);
	float detDz = Determinant(Dz);

	return Point(detDx / detD, detDy / detD, detDz / detD);
}

void GetCorners(const Frustum& frustum, vec3* outCorners) {
	outCorners[0] = Intersection(frustum._near, frustum.top, frustum.left);
	outCorners[1] = Intersection(frustum._near, frustum.top, frustum.right);
	outCorners[2] = Intersection(frustum._near, frustum.bottom, frustum.left);
	outCorners[3] = Intersection(frustum._near, frustum.bottom, frustum.right);
	outCorners[4] = Intersection(frustum.top, frustum.top, frustum.left);
	outCorners[5] = Intersection(frustum.top, frustum.top, frustum.right);
	outCorners[6] = Intersection(frustum.top, frustum.bottom, frustum.left);
	outCorners[7] = Intersection(frustum.top, frustum.bottom, frustum.right);
}

bool Intersects(const Frustum& frustum, const Point& point)
{
	for (int i = 0; i < 6; i++)
	{
		vec3 normal = frustum.planes[i].normal;
		float distance = frustum.planes[i].distance;
		float side = Dot(point, normal) + distance;

		if (side < 0.0f)
		{
			return false;
		}
	}
	return true;
}

bool Intersects(const Frustum& frustum, const Sphere& sphere)
{
	for (int i = 0; i < 6; i++)
	{
		vec3 normal = frustum.planes[i].normal;
		float distance = frustum.planes[i].distance;
		float side = Dot(sphere.position, normal) + distance;

		if (side < -sphere.radius)
		{
			return true;
		}
	}
	return true;
}


float Classify(const AABB& aabb, const Plane& plane)
{
	float r = fabsf(aabb.size.x * plane.normal.x) +
		fabsf(aabb.size.y * plane.normal.y) +
		fabsf(aabb.size.z * plane.normal.z);

	float d = Dot(plane.normal, aabb.position) + plane.distance;
	if (fabsf(d) < r)
	{
		return 0.0f;
	}
	else if (d < 0.0f)
	{
		return d + r;
	}
	return d - r;
}

float Classify(const OBB& obb, const Plane& plane)
{
	float r = fabsf(obb.size.x * plane.normal.x) +
		fabsf(obb.size.y * plane.normal.y) +
		fabsf(obb.size.z * plane.normal.z);

	float d = Dot(plane.normal, obb.position) + plane.distance;
	if (fabsf(d) < r)
	{
		return 0.0f;
	}
	else if (d < 0.0f)
	{
		return d + r;
	}
	return d - r;
}

bool Intersects(const Frustum& frustum, const AABB& aabb)
{
	for (int i = 0; i < 6; i++)
	{
		if (Classify(aabb, frustum.planes[i]) < 0.0f)
		{
			return false;
		}
	}
	return true;
}

bool Intersects(const Frustum& frustum, const OBB& obb)
{
	for (int i = 0; i < 6; i++)
	{
		if (Classify(obb, frustum.planes[i]) < 0.0f)
		{
			return false;
		}
	}
	return true;
}

vec3 Unproject(const vec3& viewportPoint, const vec2& viewportOrigin, const vec2& viewportSize, const mat4& view, const mat4& projection) {
	float normalized[4] = {
		(viewportPoint.x - viewportOrigin.x) / viewportSize.x,
		(viewportPoint.y - viewportOrigin.y) / viewportSize.y,
		viewportPoint.z, 1.0f
	};

	float ndcSpace[4]
	{
		normalized[0], normalized[1],
		normalized[2], normalized[3],
	};

	ndcSpace[0] = ndcSpace[0] * 2.0f - 1.0f;
	ndcSpace[1] = 1.0f - ndcSpace[1] * 2.0f;

	if (ndcSpace[2] < 0.0f)
	{
		ndcSpace[2] = 0.0f;
	}

	if (ndcSpace[2] > 1.0f)
	{
		ndcSpace[2] = 1.0f;
	}

	mat4 invProjection = Inverse(projection);
	float eyeSpace[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
	// eyeSpace = MultiplyPoint(ndcSpace, invProjection);
	Multiply(eyeSpace, ndcSpace, 4, 4, invProjection.asArray, 4, 4);

	mat4 invView = Inverse(view);
	float worldSpace[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
	Multiply(worldSpace, eyeSpace, 4, 4, invView.asArray, 4, 4);

	if (!CMP(worldSpace[3], 0.0f))
	{
		worldSpace[0] /= worldSpace[3];
		worldSpace[1] /= worldSpace[3];
		worldSpace[2] /= worldSpace[3];
	}
	return vec3(worldSpace[0], worldSpace[1], worldSpace[2]);
}

Ray GetPickRay(const vec2& viewportPoint, const vec2& viewportOrigin, const vec2& viewportSize, const mat4& view, const mat4& projection) {
	vec3 nearPoint(viewportPoint.x, viewportPoint.y, 0.0f);
	vec3 farPoint(viewportPoint.x, viewportPoint.y, 1.0f);

	vec3 pNear = Unproject(nearPoint, viewportOrigin, viewportSize, view, projection);
	vec3 pFar = Unproject(farPoint, viewportOrigin, viewportSize, view, projection);

	vec3 normal = Normalized(pFar - pNear);
	vec3 origin = pNear;
	return Ray(origin, normal);
}

// Chapter 15

void ResetCollisionManifold(CollisionManifold* result)
{
	if (result != 0)
	{
		result->colliding = false;
		result->normal = vec3(0, 0, 1);
		result->depth = FLT_MAX;
		result->contacts.clear();
	}
}

std::vector<Point> GetVertices(const OBB& obb)
{
	std::vector<vec3> v;
	v.resize(8);

	vec3 C = obb.position;
	vec3 E = obb.size;
	const float* o = obb.orientation.asArray;

	vec3 A[] = {
		vec3(o[0], o[1], o[2]),
		vec3(o[3], o[4], o[5]),
		vec3(o[6], o[7], o[8]),
	};

	v[0] = C + A[0] * E[0] + A[1] * E[1] + A[2] * E[2];
	v[1] = C - A[0] * E[0] + A[1] * E[1] + A[2] * E[2];
	v[2] = C + A[0] * E[0] - A[1] * E[1] + A[2] * E[2];
	v[3] = C + A[0] * E[0] + A[1] * E[1] - A[2] * E[2];
	v[4] = C - A[0] * E[0] - A[1] * E[1] - A[2] * E[2];
	v[5] = C + A[0] * E[0] - A[1] * E[1] - A[2] * E[2];
	v[6] = C - A[0] * E[0] + A[1] * E[1] - A[2] * E[2];
	v[7] = C - A[0] * E[0] - A[1] * E[1] + A[2] * E[2];

	return v;
}

std::vector<Line> GetEdges(const OBB& obb)
{
	std::vector<Line> result;
	result.reserve(12);

	std::vector<Point> v = GetVertices(obb);
	int index[][2] =
	{
		{6,1},{6,3},{6,4},{2,7},{2,5},{2,0},
		{0,1},{0,3},{7,1},{7,4},{4,5},{5,3}
	};

	for (int j = 0; j < 12; j++)
	{
		result.push_back(Line(v[index[j][0]], v[index[j][1]]));
	}

	return result;
}

std::vector<Plane> GetPlanes(const OBB& obb)
{
	std::vector<vec3> v;
	v.resize(8);

	vec3 c = obb.position;
	vec3 e = obb.size;
	const float* o = obb.orientation.asArray;

	vec3 a[] = {
		vec3(o[0], o[1], o[2]),
		vec3(o[3], o[4], o[5]),
		vec3(o[6], o[7], o[8]),
	};

	std::vector<Plane> result;
	result.resize(6);

	result[0] = Plane(a[0], Dot(a[0], (c + a[0] * e.x)));
	result[1] = Plane(a[0] * -1.0f, -Dot(a[0], (c - a[0] * e.x)));
	result[2] = Plane(a[1], Dot(a[1], (c + a[1] * e.y)));
	result[3] = Plane(a[1] * -1.0f, -Dot(a[1], (c - a[1] * e.y)));
	result[4] = Plane(a[2], Dot(a[2], (c + a[2] * e.z)));
	result[5] = Plane(a[2] * -1.0f, -Dot(a[2], (c - a[2] * e.z)));

	return result;
}

bool ClipToPlane(const Plane& plane, const Line& line, Point* outPoint)
{
	vec3 ab = line.end - line.start;
	float nAB = Dot(plane.normal, ab);
	if (CMP(nAB, 0))
	{
		return false;
	}

	float nA = Dot(plane.normal, line.start);
	float t = (plane.distance - nA) / nAB;

	if (t >= 0.0f && t <= 1.0f)
	{
		if (outPoint != 0)
		{
			*outPoint = line.start + ab * t;
		}
		return true;
	}

	return false;
}

std::vector<Point> ClipEdgesToOBB(const std::vector<Line>& edges, const OBB& obb) {
	std::vector<Point> result;
	result.reserve(edges.size());
	Point intersection;

	std::vector<Plane> planes = GetPlanes(obb);

	for (int i = 0; i < planes.size(); i++)
	{
		for (int j = 0; j < edges.size(); j++)
		{
			if (ClipToPlane(planes[i], edges[j], &intersection))
			{
				if (PointInOBB(intersection, obb))
				{
					result.push_back(intersection);
				}
			}
		}
	}
	return result;
}

float PenetrationDepth(const OBB& obb1, const OBB& obb2, const vec3& axis, bool* outShouldFlip) {
	Interval i1 = GetInterval(obb1, axis);
	Interval i2 = GetInterval(obb2, axis);

	if (!((i2.min <= i1.max) && (i1.min <= i2.max)))
	{
		return 0.0f;
	}

	float len1 = i1.max - i1.min;
	float len2 = i2.max - i2.min;

	float min = fminf(i1.min, i2.min);
	float max = fmaxf(i1.max, i2.max);

	float length = max - min;
	if (outShouldFlip != 0)
	{
		*outShouldFlip = (i2.min < i1.min);
	}

	return (len1 + len2) - length;
}

CollisionManifold FindCollisionFeatures(const OBB& obbA, const OBB& obbB)
{
	CollisionManifold result;
	ResetCollisionManifold(&result);

	const float* o1 = obbA.orientation.asArray;
	const float* o2 = obbB.orientation.asArray;

	vec3 test[15] =
	{
		vec3(o1[0], o1[1], o1[2]),
		vec3(o1[3], o1[4], o1[5]),
		vec3(o1[6], o1[7], o1[8]),
		vec3(o2[0], o2[1], o2[2]),
		vec3(o2[3], o2[4], o2[5]),
		vec3(o2[6], o2[7], o2[8])
	};

	for (int i = 0; i < 3; i++)
	{
		test[6 + i * 3 + 0] = Cross(test[i], test[0]);
		test[6 + i * 3 + 1] = Cross(test[i], test[1]);
		test[6 + i * 3 + 2] = Cross(test[i], test[2]);
	}

	vec3* hitNormal = 0;
	bool shoudFlip;

	for (int i = 0; i < 15; i++)
	{
		if (MagnitudeSqr(test[i]) < 0.001f)
		{
			continue;
		}

		float depth = PenetrationDepth(obbA, obbB, test[i], &shoudFlip);

		if (depth <= 0.0f)
		{
			return result;
		}
		else if (depth < result.depth)
		{
			if (shoudFlip)
			{
				test[i] = test[i] * -1.0f;
			}
			result.depth = depth;
			hitNormal = &test[i];
		}
	}

	if (hitNormal == 0)
	{
		return result;
	}

	vec3 axis = Normalized(*hitNormal);

	std::vector<Point> c1 = ClipEdgesToOBB(GetEdges(obbB), obbA);
	std::vector<Point> c2 = ClipEdgesToOBB(GetEdges(obbA), obbB);

	result.contacts.reserve(c1.size() + c2.size());
	result.contacts.insert(result.contacts.end(), c1.begin(), c1.end());
	result.contacts.insert(result.contacts.end(), c2.begin(), c2.end());

	Interval i = GetInterval(obbA, axis);
	float distance = (i.max - i.min) * 0.5f - result.depth * 0.5f;
	vec3 pointOnPlane = obbA.position + axis * distance;

	for (int i = result.contacts.size() - 1; i >= 0; --i)
	{
		vec3 contact = result.contacts[i];
		result.contacts[i] = contact + (axis * Dot(axis, pointOnPlane - contact));

		for (int j = result.contacts.size() - 1; j > i; --j)
		{
			if (MagnitudeSqr(result.contacts[j] - result.contacts[i]) < 0.0001f)
			{
				result.contacts.erase(result.contacts.begin() + j);
				break;
			}
		}
	}

	result.colliding = true;
	result.normal = axis;

	return result;
}


CollisionManifold FindCollisionFeatures(const Sphere& sphereA, const Sphere& sphereB)
{
	CollisionManifold result;
	ResetCollisionManifold(&result);

	float r = sphereA.radius + sphereB.radius;
	vec3 d = sphereB.position - sphereA.position;

	if (MagnitudeSqr(d) - r * r > 0 ||
		MagnitudeSqr(d) == 0)
	{
		return result;
	}

	Normalize(d);
	result.colliding = true;
	result.normal = d;
	result.depth = fabsf(Magnitude(d) - r) * 0.5f;

	float distanceToIntersectionPoint = sphereA.radius - result.depth;
	Point contact = sphereA.position + d * distanceToIntersectionPoint;
	result.contacts.push_back(contact);

	return result;
}

CollisionManifold FindCollisionFeatures(const OBB& obb, const Sphere& sphere)
{
	CollisionManifold result;
	ResetCollisionManifold(&result);

	Point closestPoint = ClosestPoint(obb, sphere.position);
	float distanceSqr = MagnitudeSqr(closestPoint - sphere.position);

	if (distanceSqr > sphere.radius * sphere.radius)
	{
		return result;
	}

	vec3 normal;
	if (CMP(distanceSqr, 0.0f))
	{
		float magSqr = MagnitudeSqr(closestPoint - obb.position);
		if (CMP(magSqr, 0.0f))
		{
			return result;
		}

		normal = Normalized(closestPoint - obb.position);
	}
	else
	{
		normal = Normalized(sphere.position - closestPoint);
	}

	Point outsidePoint = sphere.position - normal * sphere.radius;
	float distance = Magnitude(closestPoint - outsidePoint);

	result.colliding = true;
	result.contacts.push_back(closestPoint + (outsidePoint - closestPoint) * 0.5f);
	result.normal = normal;
	result.depth = distance * 0.5f;

	return result;
}
