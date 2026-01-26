#include "2d_geometry.hpp"
#include "../matrices.hpp"
#include <cmath>
#include <cfloat>

#define CMP(x, y) \
 (fabsf((x) - (y)) <= FLT_EPSILON * \
 fmaxf(1.0f, fmaxf(fabsf(x), fabsf(y))))

#define CLAMP(number, minimum, maximum) \
 number = (number < minimum) ? minimum : ( \
 (number > maximum) ? maximum : number \
 )

#define OVERLAP(aMin, aMax, bMin, bMax) \
 ((bMin<= aMax) && (aMin<= bMax))

float Length(const Line2D& line)
{
    return Magnitude(line.end - line.start);
}

float LengthSqr(const Line2D& line)
{
    return MagnitudeSqr(line.end - line.start);
}


vec2 GetMin(const Rectangle2D& rect) 
{
    vec2 p1 = rect.origin;
    vec2 p2 = rect.origin + rect.size;

    return vec2(fminf(p1.x, p2.x), fminf(p1.y, p2.y));
}

vec2 GetMax(const Rectangle2D& rect)
{
    vec2 p1 = rect.origin;
    vec2 p2 = rect.origin + rect.size;

    return vec2(fmaxf(p1.x, p2.x), fmaxf(p1.y, p2.y));
}

Rectangle2D FromMinMax(const vec2& min, const vec2& max)
{
    return Rectangle2D(min, max - min);
}

bool PointOnLine(const Point2D& point, const Line2D& line)
{
    // Find the slope
    float dy = (line.end.y - line.start.y);
    float dx = (line.end.x - line.start.x);

    float M = dy / dx;
    // finding y intercept
    float B = line.start.y - M * line.start.x;
    
    return CMP(point.y, M * point.x +  B);
}

bool PointInCiricle(const Point2D& point, const Circle& c)
{
    Line2D line(point, c.position);
    if(LengthSqr(line) < c.radius * c.radius)
    {
        return true;
    } 
    return false;
}

bool PointInRectangle(const Point2D& point, const Rectangle2D& rectangle) 
{
    vec2 min = GetMin(rectangle);
    vec2 max = GetMax(rectangle);

    return min.x <= point.x &&
           min.y <= point.y &&
           point.x <= max.x &&
           point.y <= max.y; 
}

bool PointInOrientedRectangle(const Point2D& point, const OrientedRectangle& rectangle)
{
    vec2 rotatedVector = point - rectangle.position;
    float theta = -DEG2RAD(rectangle.rotation);
    float zRotation2x2[] = {
        cosf(theta), sinf(theta),
        -sinf(theta), cosf(theta)
    };

    Multiply(rotatedVector.asArray, vec2(rotatedVector.x, rotatedVector.y).asArray, 1, 2, zRotation2x2, 2, 2);
    Rectangle2D localRectangle(Point2D(), rectangle.halfExtents * 2.0f);
    vec2 localPoint = rotatedVector + rectangle.halfExtents;
    return PointInRectangle(localPoint, localRectangle);
}

bool PointInShape(const BoundingShape& shape, const Point2D& point)
{
    for(int i = 0; i < shape.numCircles; ++i)
    {
        if(PointInCiricle(point, shape.circles[i]));
        {
            return true;
        }
    }

    for(int i = 0; i < shape.numRectangles; ++i)
    {
        if(PointInRectangle(point, shape.rectangles[i]));
        {
            return true;
        }
    }
    return false;
}


bool LineCircle(const Line2D& line, const Circle& circle) 
{
    vec2 ab = line.end - line.start;
    float t = Dot(circle.position - line.start, ab) / Dot(ab, ab);
    if(t < 0.0f || t > 1.0f)
    {
        return false;
    }

    Point2D closesPoint = line.start + ab * t;
    Line2D circleToClosest(circle.position, closesPoint);
    return LengthSqr(circleToClosest) < circle.radius * circle.radius;
}

bool LineRectangle(const Line2D& line, const Rectangle2D& rect) 
{
    if(PointInRectangle(line.start, rect) || PointInRectangle(line.end, rect))
    {
        return true;
    } 

    vec2 norm = Normalized(line.end - line.start);
    norm.x = (norm.x != 0) ? 1.0f / norm.x : 0;
    norm.y = (norm.y != 0) ? 1.0f / norm.y : 0;

    vec2 min = (GetMin(rect) - line.start) * norm;
    vec2 max= (GetMin(rect) - line.start) * norm;

    float tmin = fmaxf(
        fminf(min.x, max.x),
        fminf(min.y, max.y)
    );

    float tmax = fminf(
        fmaxf(min.x, max.x),
        fmaxf(min.y, max.y)
    );

    if(tmax < 0 || tmin > tmax)
    {
        return false;
    }

    float t = (tmin < 0.0f) ? tmax : tmin;
    return t > 0.0f && t * t < LengthSqr(line);
}

bool LineOrientedRectangle(const Line2D& line, const OrientedRectangle& rect)
{
    float theta = -DEG2RAD(rect.rotation);
    float zRotation2x2[] = {
        cosf(theta), sinf(theta),
        -sinf(theta), cosf(theta)
    };
    Line2D localLine;

    vec2 rotatedVector = line.start - rect.position;
    Multiply(rotatedVector.asArray, vec2(rotatedVector.x, rotatedVector.y).asArray, 1, 2, zRotation2x2, 2, 2);
    localLine.start = rotatedVector + rect.halfExtents;

    rotatedVector = line.end - rect.position;
    Multiply(rotatedVector.asArray, vec2(rotatedVector.x, rotatedVector.y).asArray, 1, 2, zRotation2x2, 2, 2);
    localLine.end = rotatedVector + rect.halfExtents;

    Rectangle2D localRectangle(Point2D(), rect.halfExtents * 2.0f);
    return LineRectangle(localLine, localRectangle);
}

bool CircleCirle(const Circle& c1, const Circle& c2)
{
    Line2D line(c1.position, c2.position);
    float radiiSum = c1.radius + c2.radius;
    return LengthSqr(line) <= radiiSum * radiiSum;
}

bool CircleRectangle(const Circle& circle, const Rectangle2D& rectangle)
{
    vec2 min = GetMin(rectangle);
    vec2 max = GetMax(rectangle);

    Point2D closestPoint = circle.position;
    if(closestPoint.x < min.x)
    {
        closestPoint.x = min.x;
    }
    else if(closestPoint.x > max.x)
    {
        closestPoint.x = max.x;
    }

    closestPoint.y = (closestPoint.y < min.y) ?
        min.y : closestPoint.y;

    closestPoint.y = (closestPoint.y > max.y) ?
        max.y : closestPoint.y;

    Line2D line(circle.position, closestPoint);
    return LengthSqr(line) <= circle.radius * circle.radius;
}

bool CircleOrientedRectangle(const Circle& circle, const OrientedRectangle& rectangle)
{
    vec2 r = circle.position - rectangle.position;
    float theta = -DEG2RAD(rectangle.rotation);
    float zRotation2x2[] = {
        cosf(theta), sinf(theta),
        -sinf(theta), cosf(theta)
    };

    Multiply(r.asArray, vec2(r.x, r.y).asArray, 1, 2, zRotation2x2, 2, 2);
    Circle localCircle(r + rectangle.halfExtents, circle.radius);

    Rectangle2D localRectangle(Point2D(), rectangle.halfExtents * 2.0f);
    return CircleRectangle(localCircle, localRectangle);
}

bool RectangleRectangle(const Rectangle2D& rect1, const Rectangle2D& rect2)
{
    vec2 aMin = GetMin(rect1);
    vec2 aMax = GetMax(rect1);

    vec2 bMin = GetMin(rect2);
    vec2 bMax = GetMax(rect2);


    bool overX = ((bMin.x <= aMax.x) && (aMin.x <= bMax.x));
    bool overY = ((bMin.y <= aMax.y) && (aMin.y <= bMax.y));

    return overX && overY;
}

bool RectangleRectangleSAT(const Rectangle2D& rect1, const Rectangle2D& rect2)
{
    vec2 axisToTest[] = { vec2(1, 0), vec2(0, 1) };

    for(int i = 0; i < 2; i++)
    {
        // if intervals dont overlap, separating axis found, therefore, no collision
        if(!OverlapOnAxis(rect1, rect2, axisToTest[i]))
        {
            return false;
        }
    }

    // all intervals overlapped, collision found
    return true;
}

bool RectangleOrientedRectangle(const Rectangle2D& rect1, const OrientedRectangle& rect2)
{
    vec2 axisToTest[] = {
        vec2(1, 0), vec2(0, 1),
        vec2(), vec2()
    };

    float t = DEG2RAD(rect2.rotation);
    float zRotation[] = {
        cosf(t), sinf(t),
        -sinf(t), cosf(t)
    };

    vec2 axis = Normalized(vec2(0, rect2.halfExtents.x));
    Multiply(axisToTest[2].asArray, axis.asArray, 1, 2, zRotation, 2, 2);


    axis = Normalized(vec2(0, rect2.halfExtents.y));
    Multiply(axisToTest[3].asArray, axis.asArray, 1, 2, zRotation, 2, 2);

    for(int i = 0; i < 4; i++)
    {
        // if intervals dont overlap, separating axis found, therefore, no collision
        if(!OverlapOnAxis(rect1, rect2, axisToTest[i]))
        {
            return false;
        }
    }

    // all intervals overlapped, collision found
    return true;
}

bool OrientedRectangleOrientedRectangle(const OrientedRectangle& rect1, const OrientedRectangle& rect2)
{
    Rectangle2D local1(Point2D(), rect1.halfExtents * 2.0f);
    vec2 r = rect2.position - rect1.position;
    OrientedRectangle local2(rect2.position, rect2.halfExtents, rect2.rotation);
    local2.rotation = rect2.rotation - rect1.rotation;
    float t = -DEG2RAD(rect1.rotation);
    float z[] = {
        cosf(t), sinf(t),
        -sinf(t), cosf(t)
    };
    Multiply(r.asArray, vec2(r.x, r.y).asArray, 1, 2, z, 2, 2);
    local2.position = r + rect1.halfExtents;
    return RectangleOrientedRectangle(local1, local2);
}

Interval2D GetInterval(const Rectangle2D& rect, const vec2& axis)
{
    Interval2D result;
    vec2 min = GetMin(rect);
    vec2 max = GetMax(rect);

    // all vertices of rect
    vec2 verts[] = {
        vec2(min.x, min.y), vec2(min.x, max.y),
        vec2(max.x, max.y), vec2(max.x, min.y)
    };

    result.min = result.max = Dot(axis, verts[0]);
    for(int i = 0; i < 4; ++i)
    {
        float projection = Dot(axis, verts[i]);
        if(projection < result.min)
        {
            result.min = projection;
        }
        if(projection > result.max)
        {
            result.max = projection;
        }
    }
    return result;
}

bool OverlapOnAxis(const Rectangle2D& rect1, const Rectangle2D& rect2, const vec2& axis)
{
    Interval2D a = GetInterval(rect1, axis);
    Interval2D b = GetInterval(rect2, axis);
    return ((b.min <= a.max) && (a.min <= b.max));
}

Interval2D GetInterval(const OrientedRectangle& rect, const vec2& axis) 
{
    Rectangle2D r = Rectangle2D(Point2D(rect.position - rect.halfExtents), rect.halfExtents * 2.0f);

    vec2 min = GetMin(r);
    vec2 max = GetMax(r);
    vec2 verts[] = {
        min, max,
        vec2(min.x, max.y), vec2(max.x, min.y)
    };

    float t = -DEG2RAD(rect.rotation);
    float zRotation[] = {
        cosf(t), sinf(t),
        -sinf(t), cosf(t)
    };

    for(int i = 0; i < 4; i++)
    {
        vec2 r = verts[i] - rect.position;
        Multiply(r.asArray, vec2(r.x, r.y).asArray, 1, 2, zRotation, 2, 2);
        verts[i] = r + rect.position;
    }

    Interval2D result;
    result.min = result.max = Dot(axis, verts[0]);
    for(int i = 0; i < 4; i++)
    {
        float projection = Dot(axis, verts[i]);
        result.min = (projection < result.min) ? projection : result.min;
        result.max = (projection > result.max) ? projection : result.max;
    }
    return result;
}

bool OverlapOnAxis(const Rectangle2D& rect1, const OrientedRectangle& rect2, const vec2& axis)
{
    Interval2D a = GetInterval(rect1, axis);
    Interval2D b = GetInterval(rect2, axis);

    return ((b.min <= a.max) && (a.min > b.max));
}

Circle ContainingCircle(Point2D* pArray, int arrayCount)
{
    Point2D center;
    for(int i = 0; i < arrayCount; i++)
    {
        center = center + pArray[i];
    }

    center = center * (1.0f / (float)arrayCount);
    Circle result(center, 1.0f);
    result.radius = MagnitudeSqr(center - pArray[0]);
    for(int i = 1; arrayCount; i++)
    {
        float distance = MagnitudeSqr(center  - pArray[i]);
        if(distance > result.radius)
        {
            result.radius = distance;
        }
    }
    result.radius = sqrtf(result.radius);
    return result;
}

Rectangle2D ContainsRectangle(Point2D* pArray, int arrayCount)
{
    vec2 min = pArray[0];    
    vec2 max = pArray[0];    

    for(int i = 0; i < arrayCount; i++)
    {
        min.x = pArray[i].x < min.x ? pArray[i].x : min.x;
        min.y = pArray[i].y < min.y ? pArray[i].y : min.y;
        
        max.x = pArray[i].x > max.x ? pArray[i].x : max.x;
        max.y = pArray[i].y > max.y ? pArray[i].y : max.y;
    }

    return FromMinMax(min, max);
}
