#pragma once
#include <stdio.h>
#include <vector>

#include "planners/A_star.h"

class AStarNode {
public:
	int x, y;
	bool operator==(const AStarNode&n);
};

class AStarGraph : public SearchGraphDescriptorFunctionContainer<AStarNode, double>
{
	public:
		AStarGraph(int x, int y);
		int getHashBin(AStarNode& n);
		bool isAccessible(AStarNode& n);
		void getSuccessors(AStarNode& n, std::vector<AStarNode>* s, std::vector<double>* c);
		double getHeuristics(AStarNode& n1, AStarNode& n2);
		float getDistance(AStarNode& node, float* coordinates);
		void findBestMatch(AStarNode& node, float* trueValue);
		void findBestMatch(AStarNode& node, AStarNode& goal);
		static std::vector< AStarNode > generateSmoothedPath(std::vector< AStarNode > original);
		static bool pathIsClear(AStarNode& start, AStarNode& end);
	private:
		int xLength, yLength;
	
};

