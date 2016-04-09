#pragma once

#ifndef	BZF_ASTARNODE_H
#define	BZF_ASTARNODE_H

// YAGSBPL libraries
#include "yagsbpl_base.h"
#include "planners/A_star.h"

#include "BZDBCache.h"

#define SCALE	BZDBCache::tankRadius
// A node of the A* search graph
class AStarNode
{
public:
	AStarNode(void);
	~AStarNode(void);

	bool operator==(const AStarNode& n) const { return (x==n.x && y==n.y); }; // This must be defined for the node
	AStarNode(const float location[3]);
	AStarNode(int xi, int yi);
	static bool isAccessible(int x, int y);
inline int AStarNode::getX(void) const { return x; }
inline int AStarNode::getY(void) const { return y; }
inline void AStarNode::setX(int newX) { x = newX; }
inline void AStarNode::setY(int newY) { y = newY; }
	inline float AStarNode::getScaledX(void) const { return x * SCALE; }
	inline float AStarNode::getScaledY(void) const { return y * SCALE; }
private:
	int x, y; // Profiling observation: integer coordinates, hence operator==,
	          //  makes the search significantly faster (almost 10 folds than double)
};

// ============================================================
// Functions that describe the graph, placed inside a class
// Here the class "GraphFunctionContainer" is redefining the virtual functions declared in "SearchGraphDescriptorFunctionContainer"
// Thus the names of the functions are important here.

class GraphFunctionContainer : public SearchGraphDescriptorFunctionContainer<AStarNode,double>
{
public:

	int getHashBin(AStarNode& n);

	bool isAccessible(AStarNode& n);

	void getSuccessors(AStarNode& n, std::vector<AStarNode>* s, std::vector<double>* c); // Define a 8-connected graph

	double getHeuristics(AStarNode& n1, AStarNode& n2);

	// -------------------------------
	// constructors
	GraphFunctionContainer (float worldSize);

	static int Xmin, Xmax, Ymin, Ymax; // world size

private:

	// fixed transition costs vector
	std::vector<double> ConstCostVector;



};

#endif // BZF_ASTARNODE_H