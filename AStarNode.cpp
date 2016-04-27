#include "AStarNode.h"
#include <math.h>
#include "common.h"
#include "BZDBCache.h"
#include "World.h"
#include "playing.h" // needed for controlPanel

#define BAD_FLAG_TRACE

int GraphFunctionContainer::Xmin, GraphFunctionContainer::Xmax;
int GraphFunctionContainer::Ymin, GraphFunctionContainer::Ymax;

AStarNode::AStarNode(void)
{
	x = y = 0;
}

AStarNode::~AStarNode(void)
{
}

// Takes a bzflag world location and returns the closest AStarNode
// if that node is not isAccessible (the location is next to the world boundaries or a building and
// the closest node is outside the world or inside the building), then return a neighbor node that isAccessible
AStarNode::AStarNode(const float location[3])
{
	x = GraphFunctionContainer::convertCoordinate(location[0]);
	y = GraphFunctionContainer::convertCoordinate(location[1]);
	int tempX = x;
	int tempY = y;
	if (AStarNode::isAccessible(x, y)) return;
	for (int a = -1; a <= 1; a++) {
		for (int b = -1; b <= 1; b++) {
			if (a == 0 && b == 0) continue;
			x = tempX + a;
			y = tempY + b;
			if (AStarNode::isAccessible(x, y)) return;
		}
	}
	char buffer[512];
	sprintf (buffer, "***AStarNode: could not find any isAccessible node for (%f, %f, %f)***", location[0], location[1], location[2]);
	controlPanel->addMessage(buffer);
}
AStarNode::AStarNode(int xi, int yi)
{
	x = xi;
	y = yi;
}
int GraphFunctionContainer::getHashBin(AStarNode& n)
{
	return ((n.getY() + abs(Ymin) + 1) * ((abs(Xmin) * 2) + 1)) + (n.getX() + abs(Xmin) + 1);
}

// returns true if the location (x, y) is inside the world boundaries and not in a building
bool AStarNode::isAccessible(int x, int y)
{
	// check for world boundaries
	if (x<GraphFunctionContainer::Xmin || x>GraphFunctionContainer::Xmax ||
		y<GraphFunctionContainer::Ymin || y>GraphFunctionContainer::Ymax)
		return false;
	// if not inside an obstacle
	float pos[3];
	pos[0] = x * SCALE;
	pos[1] = y * SCALE;
	pos[2] = 0.0f;

	return !(World::getWorld()->inBuilding(pos, BZDBCache::tankRadius/2, BZDBCache::tankHeight));
}

// static member function version of above
bool GraphFunctionContainer::isAccessible(AStarNode& n)
{
	return AStarNode::isAccessible(n.getX(), n.getY());
}

// return the neighbors of the AStarNode as a vector { NW, W, SW, N, S, NE, W, NW } in vector c
void GraphFunctionContainer::getSuccessors(AStarNode& n, std::vector<AStarNode>* s, std::vector<double>* c) // Define a 8-connected graph
{
	// Defining the initial transition costs
	std::vector<double> costVector;
	std::vector<Flag*> nearbyFlags;
	std::vector<Player*> nearbyEnemies;

	float maxAreaOfInfluence = 20.0f; // maximum area of influence (in front of tank)
	float minAreaOfInfluence = 3.0f; // minimum area of influence (not in front of tank)
	bool avoidBadFlags = currentStatus == OFFENSE || currentStatus == PATROL || !(player->getFlag() != NULL && player->getFlag() != Flags::Null && (player->getFlag()->flagTeam != NoTeam || player->getFlag()->flagQuality == FlagGood));
	bool avoidEnemies = currentStatus == RETURN; //only consider enemy influence if returning flag
	bool seekCover =  currentStatus == RETURN || currentStatus == OFFENSE;

	if(avoidBadFlags)
	for (int i = 0; i < numFlags; i++) { //get all nearby flags
		Flag* flag = &World::getWorld()->getFlag(i);
		TeamColor flagTeamColor = flag->type->flagTeam;
		if (flagTeamColor != NoTeam && flag->status != FlagOnTank && flag->type->flagQuality == FlagQuality::FlagBad
			&& hypotf(convertCoordinate(flag->position[0]) - n.getX(), convertCoordinate(flag->position[1]) - n.getY()) <= 3.0f) {
			nearbyFlags.push_back(flag);
		}
	}

	if(avoidEnemies)
	for (int t = 0; t <= World::getWorld()->getCurMaxPlayers(); t++){ //get all nearby players
		Player *p = 0;
		if (t < World::getWorld()->getCurMaxPlayers())
			p = World::getWorld()->getPlayer(t);
		if (!p || p->getId() == player->getId() || p->getColor() == player->getColor())
			continue;
		float distance = hypotf((float) convertCoordinate(p->getPosition()[0]) - n.getX(), (float) convertCoordinate(p->getPosition()[1]) - n.getY());
		if (distance < maxAreaOfInfluence + 2.0f) {
			nearbyEnemies.push_back(p);
		}
	}

	double multiplier = 1.0;
	if (avoidBadFlags)
		multiplier *= badFlagInfluence(nearbyFlags, n);
	if (avoidEnemies)
		multiplier *= enemyInfluence(nearbyEnemies, n, maxAreaOfInfluence, minAreaOfInfluence);
	if(seekCover)
		multiplier *= coverInfluence(n);

	// This function needn't account for obstacles or size of environment. That's done by "isAccessible"
	AStarNode tn;
	s->clear(); c->clear(); // Planner is supposed to clear these. Still, for safety we clear it again.
	for (int a=-1; a<=1; a++)
		for (int b=-1; b<=1; b++) {
			if (a==0 && b==0) continue;
			tn.setX(n.getX() + a);
			tn.setY(n.getY() + b);
			s->push_back(tn);
			if (avoidBadFlags || avoidEnemies || seekCover) {
				double cost = hypotf(a, b);
				cost *= multiplier;
				costVector.push_back(cost);
			}
		}
	if (avoidBadFlags || avoidEnemies || seekCover) {
		*c = costVector;
	} else {
		*c = ConstCostVector;
	}

}

/**
Calculates the influence of bad flags on the weight of the path.
A flag increases the weight of nearby nodes (at most 2 units away);
*/
double GraphFunctionContainer::badFlagInfluence(std::vector<Flag*> flagList, AStarNode n) {
	double multiplier = 1.0;
	for (Flag* flag : flagList) {
		double distance = hypotf(convertCoordinate(flag->position[0]) - n.getX(), convertCoordinate(flag->position[1]) - n.getY());
		if ( distance <= 2.0) {
			multiplier += 5.0 / (distance + 1.0);
#ifdef BAD_FLAG_TRACE
			char buffer[128];
			sprintf(buffer, "Bad Flag Found at (%f, %f), multiplier is now at %d", n.getScaledX(), n.getScaledY(), multiplier);
			controlPanel->addMessage(buffer);
#endif
		}
	}
	return multiplier;
}

/**
Calculates the influence of enemies on the weight of the path.
Greater influence in the direction that enemy is facing.
Influence within a certain distance from tank (minAOI).
Influence within a 45 degree angle in front of tank, up to a certain distance (maxAOI).
*/
double GraphFunctionContainer::enemyInfluence(std::vector<Player*> enemyList, AStarNode n, float maxAOI, float minAOI) {
	double multiplier = 1.0;

	for (Player* enemy : enemyList) {
		double distance = hypotf(convertCoordinate(enemy->getPosition()[0]) - n.getX(), convertCoordinate(enemy->getPosition()[1]) - n.getY());
		if (distance <= minAOI) {
			multiplier += minAOI / (distance + 1.0);
		}
		if (distance <= maxAOI) {
			float direction[3] = { n.getX() - convertCoordinate(enemy->getPosition()[0]), n.getY() - convertCoordinate(enemy->getPosition()[1]), 0.0f };
			float angleFrom = atan2(direction[1], direction[0]);
			float enemyAngle = enemy->getAngle();
			float angleMin = trueAngle(enemyAngle - M_PI / 16.0);
			if (trueAngle(angleFrom - angleMin) <= M_PI / 8.0) { // if position is in front of enemy tank (45 degree angle)
				multiplier += (maxAOI / 4.0) / (distance + 1.0);
			}
		}
	}
	return multiplier;
}

/**
Convert angle to equivalent value between 0.0 and 6.28
*/
float GraphFunctionContainer::trueAngle(float angle) {
	if (angle < 0.0f) return angle + (float)(2.0 * M_PI);
	if (angle > 2.0f * M_PI) return angle - (float)(2.0 * M_PI);
	return angle;
}

/**
Calculates the cover value for a specific node.
All calculated values are stored permanently into a hash map.
Values are calculated when needed and are readily available if called a second time.
*/
double GraphFunctionContainer::coverInfluence(AStarNode n) {
	if (coverValues[getHashBin(n)] == 0.0)
		coverValues[getHashBin(n)] = calculateCover(n);
	return coverValues[getHashBin(n)];
}

/**
Calculates the relative cover a position provides.
Values can range from 0.5 for safe cover to 1.5 for wide open areas.
*/
double GraphFunctionContainer::calculateCover(AStarNode n) {
	double multiplier = 1.5f;
	float distanceChecked = SCALE * 3.0f;

	const float sqrt2 = sqrt(2.0f);
	float unitDirections[][3] = { {0.0f, 1.0f, 0.0f }, {sqrt2, sqrt2, 0.0f }, { 1.0f, 0.0f, 0.0f }, { sqrt2, -sqrt2, 0.0f }, { 0.0f, -1.0f, 0.0f }, { -sqrt2, -sqrt2, 0.0f }, { -1.0f, 0.0f, 0.0f }, { -sqrt2, sqrt2, 0.0f } };
	for (float* direction : unitDirections) {
		float coordinates[3] = { n.getScaledX(), n.getScaledY(), 0.0f };
		Ray coverRay(coordinates, direction);
		if (ShotStrategy::getFirstBuilding(coverRay, -0.5f, distanceChecked) != NULL) {
			multiplier -= .125f;
		}
	}

	return multiplier;
}

/**
Converts an original coordinate to a scaled down version for use in the graph.
*/
int GraphFunctionContainer::convertCoordinate(float c) {
	return (c > 0.0) ? (int)floor(c / SCALE + 0.5f) : (int)ceil(c / SCALE - 0.5f);
}

// returns the Euclidean distance of the AStarNode to the goalNode
double GraphFunctionContainer::getHeuristics(AStarNode& n1, AStarNode& n2)
{
	// Start with Euclidean distance
	return hypot((double) n2.getX() - n1.getX(), (double) n2.getY() - n1.getY());
}

// -------------------------------
// constructors
GraphFunctionContainer::GraphFunctionContainer (float worldSize, int currentStatus, Player* player)
{ 
	int size = (int)worldSize/SCALE/2;
	Xmin = -size; Xmax = size; Ymin = -size; Ymax = size;
	this->currentStatus = currentStatus;
	this->player = player;

	// Defining the fixed transition costs
	double SQRT2 = sqrt(2.0);
	ConstCostVector.push_back(SQRT2); ConstCostVector.push_back(1.0);
	ConstCostVector.push_back(SQRT2); ConstCostVector.push_back(1.0);
	ConstCostVector.push_back(1.0); ConstCostVector.push_back(SQRT2);
	ConstCostVector.push_back(1.0); ConstCostVector.push_back(SQRT2);

}

std::unordered_map <int, double> GraphFunctionContainer::coverValues;
