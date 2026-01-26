#ifndef QUAD_TREE_HPP
#define QUAD_TREE_HPP

#include "../math/geometry/2d_geometry.hpp"
#include <vector>

struct QuadTreeData
{
    // game object
    void* object;
    Rectangle2D bounds;
    bool flag;
    inline QuadTreeData(void* o, const Rectangle2D& b) 
        : object(o), bounds(b), flag(false) {}
}; 

class QuadTreeNode
{
public:
    inline QuadTreeNode(const Rectangle2D& bounds)
        : nodeBounds(bounds), currentDepth(0){}
    
    bool IsLeaf();
    int NumObjects();
    void Insert(QuadTreeData& data);
    void Remove(QuadTreeData& data);
    void Update(QuadTreeData& data);
    void Shake();
    void Split();
    void Reset();
    std::vector<QuadTreeData*> Query(const Rectangle2D& area);
protected:
    std::vector<QuadTreeNode> children;
    std::vector<QuadTreeData*> contents;
    int currentDepth;
    static int maxDepth;
    static int maxObjectsPerNode;
    Rectangle2D nodeBounds;
};

typedef QuadTreeNode QuadTree;
#endif