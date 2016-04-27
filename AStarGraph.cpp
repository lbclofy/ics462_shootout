#include "AStarGraph.h"
#include <stdio.h>
#include "math.h"

#include "World.h"
#include "BZDBCache.h"
#include "ShotStrategy.h"
#include "TargetingUtils.h"

/**
These classes are for the purpose of using the YAGSBPL A* Search.
*/

/**
Represents a node in a graph.
Int used for simplicity.
*/
bool AStarNode::operator==(const AStarNode& n) { return (x == n.x && y == n.y); }

/**
Constructor. Class assumes that values range from -x to x and -y to y.
Integer values should be scaled before using them in constructor.
*/
AStarGraph::AStarGraph(int x, int y)
{
	xLength = x;
	yLength = y;
}

/**
Generate hash for coordiantes. This particular implementation will guarantee a unique number without wasting space. 
Hashes will range from 0-n, where n is the total number of nodes.
*/
int AStarGraph::getHashBin(AStarNode& n) 
{
	return ((n.y + yLength + 1) * ((xLength * 2) + 1)) + (n.x + xLength + 1);
}

/**
Determines if node can be accessed by tank.
Tank cannot go out of bounds or in obstacles.
*/
bool AStarGraph::isAccessible(AStarNode& n)
{
	const float scale = BZDBCache::tankRadius;
	const float position[3] = {n.x * scale, n.y * scale, 0.0f };
	return (World::getWorld()->inBuilding(position, scale / 2, 0.0f) == NULL && //not in obstacle
			n.x >= -xLength && n.x <= xLength && n.y >= -yLength && n.y <= yLength); //within graph
}

/**
Implementation kept the same as in example. 8-connected graph is reasonable.
*/
void AStarGraph::getSuccessors(AStarNode& n, std::vector<AStarNode>* s, std::vector<double>* c) // Define a 8-connected graph
{
	AStarNode tn;
	s->clear(); c->clear(); // Planner is supposed to clear these. Still, for safety we clear it again.
	for (int a = -1; a <= 1; a++)
		for (int b = -1; b <= 1; b++) {
			if (a == 0 && b == 0) continue;
			tn.x = n.x + a;
			tn.y = n.y + b;
			s->push_back(tn);
			c->push_back(sqrt((double)(a*a + b*b)));
		}
}

/**
Kept example implementation. Euclidean distance is guaranteed to be an underestimate or accurate.
*/
double AStarGraph::getHeuristics(AStarNode& n1, AStarNode& n2)
{
	int dx = abs(n1.x - n2.x);
	int dy = abs(n1.y - n2.y);
	return (sqrt((double)(dx*dx + dy*dy))); // Euclidean distance as heuristics
}

/**
Returns the distance between the node's coordinates and the target coordinates.
Distance is resized to BZFlag size.
*/
float AStarGraph::getDistance(AStarNode& node, float* coordinates) {
	const float scale = BZDBCache::tankRadius;
	float x = node.x * scale;
	float y = node.y * scale;
	return hypotf(coordinates[0] - x, coordinates[1] - y);
}

/**
Return the node that is accessible and best fits the true value.
If there are no accessible coordinates, simply leave the node unchanged.
*/
void AStarGraph::findBestMatch(AStarNode& node, float* trueValue) {
	std::vector<AStarNode> validNodes;
	std::vector<float> distances; // index will correspond to validNodes

	if (!isAccessible(node)) {
		for (int a = -1; a <= 1; a++) {
			AStarNode tempNode;
			for (int b = -1; b <= 1; b++) {
				tempNode.x = node.x + a;
				tempNode.y = node.y + b;
			}
			if (isAccessible(tempNode)) { //add valid nodes to vector and keep track of distance to true location
				validNodes.push_back(tempNode);
				float distance = getDistance(tempNode, trueValue);
				distances.push_back(distance);
			}
		}
		if (validNodes.size() > 0) { 
			int smallest = 0;
			AStarNode newTemp;
			for (int i = 0; i < validNodes.size(); i++) {
				if (distances[i] < distances[smallest]) {
					smallest = i;
				}
			}
			newTemp = validNodes[smallest]; // change node to closest accessible node
			node.x = newTemp.x;
			node.y = newTemp.y;
		}
	}
}

/**
Return the node that is accessible and is closest to the goal Node if the current node is unaccessible.
If the original node is accessible, do nothing.
If there are no accessible coordinates, simply leave the node unchanged.
*/
void AStarGraph::findBestMatch(AStarNode& node, AStarNode& goal) {
	const float scale = BZDBCache::tankRadius;
	float goalCoordinates[2] = {goal.x * scale, goal.y * scale};

	findBestMatch(node, goalCoordinates);
}

/**
Returns a path that is "smoothed." It essentially removes all nonessential nodes. 
If there is a straight path between two nodes on the path, then all the nodes in between them are ignored.
*/
std::vector< AStarNode > AStarGraph::generateSmoothedPath(std::vector< AStarNode > original) {
	std::vector< AStarNode > smoothed;

	for (int i = 0; i < original.size(); i++) {
		int until; //final node before the node can be reached in a straight line.
		until = i + 1;
		for (int i2 = (i + 1); i2 < original.size(); i2++) {
			until = i2;
			if (!pathIsClear(original[i], original[i2])) {
				break;
			}
		}
		smoothed.push_back(original[i]);
		i = until; //skip to next node in new path. Ignore all nodes in between
	}
	return smoothed;
}

/**
Check if tank can go straight from the start node to the end node.
Three rays are used to determine this: one from the original point and two that are perpendicular.
*/
bool AStarGraph::pathIsClear(AStarNode& start, AStarNode& end) {
	const float scale = BZDBCache::tankRadius;
	float startCoordinates[3] = { start.x * scale, start.y * scale, 0 };
	float endCoordinates[3] = { end.x * scale, end.y * scale, 0 };
	float direction[3] = {endCoordinates[0] - startCoordinates[0], endCoordinates[1] - startCoordinates[1], endCoordinates[2] - startCoordinates[2]};
	float length = hypotf(direction[0], direction[1]);
	float unitDirection[3] = { direction[0] / length, direction[1] / length, direction[2] };
	float perpendicular1[3] = { unitDirection[1], unitDirection[0] * -1.0f, unitDirection[2] }; //vectors perpendicular to vector from start node to end node
	float perpendicular2[3] = { unitDirection[1] * -1.0f, unitDirection[0], unitDirection[2] };
	float startSide1[3] = { startCoordinates[0] + (perpendicular1[0] * scale / 2), startCoordinates[1] + (perpendicular1[1] * scale / 2), startCoordinates[2] };
	float startSide2[3] = { startCoordinates[0] + (perpendicular2[0] * scale / 2), startCoordinates[1] + (perpendicular2[1] * scale / 2), startCoordinates[2] };
	float maxdistance = TargetingUtils::getTargetDistance(startCoordinates, endCoordinates);

	Ray tankRay(startCoordinates, unitDirection);
	Ray tankSideRay1(startSide1, unitDirection);
	Ray tankSideRay2(startSide2, unitDirection);

	return ShotStrategy::getFirstBuilding(tankRay, -0.5f, maxdistance) == NULL //true if none of the three rays hit any buildings
		&& ShotStrategy::getFirstBuilding(tankSideRay1, -0.5f, maxdistance) == NULL
		&& ShotStrategy::getFirstBuilding(tankSideRay2, -0.5f, maxdistance) == NULL;
}

