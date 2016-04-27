/* bzflag
 * Copyright (c) 1993-2012 Tim Riker
 *
 * This package is free software;  you can redistribute it and/or
 * modify it under the terms of the license found in the file
 * named COPYING that should have accompanied this file.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

/*
 *
 */

#ifndef	BZF_ROBOT_PLAYER_H
#define	BZF_ROBOT_PLAYER_H

#include "common.h"

/* system interface headers */
#include <vector>

/* interface header */
#include "LocalPlayer.h"

/* local interface headers */
#include "Region.h"
#include "RegionPriorityQueue.h"
#include "ServerLink.h"

#include "ControlPanel.h"
#include "AStarNode.h" // needed for A* search

class RobotPlayer : public LocalPlayer {
  public:
			RobotPlayer(const PlayerId&,
				const char* name, ServerLink*,
				const char* _motto);

    float		getTargetPriority(const Player*) const;
    const Player*	getTarget() const;
    void		setTarget(const Player*);
    static void		setObstacleList(std::vector<BzfRegion*>*);

    void		restart(const float* pos, float azimuth);
    void		explodeTank();
	static void		init(void);
	bool		RobotPlayer::amAlive(float dt);
	bool		RobotPlayer::shotComing(float dt);
	void		RobotPlayer::doNothing(float dt);
	void		RobotPlayer::evade(float dt);
	void		RobotPlayer::followPath(float dt);

	bool		RobotPlayer::isFiringStatusReady(float dt);
	bool		RobotPlayer::hasShotTimerElapsed(float dt);
	bool		RobotPlayer::isShotCloseToTarget(float dt);
	bool		RobotPlayer::isBuildingInWay(float dt);
	bool		RobotPlayer::isTeammateInWay(float dt);
	void		RobotPlayer::setShortShotTimer(float dt);
	void		RobotPlayer::shootAndResetShotTimer(float dt);
	void		RobotPlayer::rotateShootAndResetShotTimer(float dt);

	bool		checkPursuit();
	bool		checkReturnFlag();
	bool		checkHoldingOwnFlag();
	bool		checkToBase();

	bool isTargetClose(float dt);

	bool		RobotPlayer::isHoldingFlag(float dt);
	bool		RobotPlayer::isFlagSticky(float dt);
	bool		RobotPlayer::isTeamFlag(float dt);
	bool		RobotPlayer::isMyTeamFlag(float dt);
	void		RobotPlayer::dropFlag(float dt);
	bool		RobotPlayer::isFlagGood(float dt);
	bool		RobotPlayer::isTargetVeryClose(float dt);
	bool		RobotPlayer::isTargetAFlag(float dt);
	bool		RobotPlayer::isTargetATank(float dt);

  private:
    void		doUpdate(float dt);
    void		doUpdateMotion(float dt);
    BzfRegion*		findRegion(const float p[2], float nearest[2]) const;
    float		getRegionExitPoint(
				const float p1[2], const float p2[2],
				const float a[2], const float targetPoint[2],
				float mid[2], float& priority);
     void		findPath(RegionPriorityQueue& queue,
				BzfRegion* region, BzfRegion* targetRegion,
				const float targetPoint[2], int mailbox);

     void		projectPosition(const Player *targ,const float t,float &x,float &y,float &z) const;
     void		getProjectedPosition(const Player *targ, float *projpos) const;

	 int		RobotPlayer::computeCenterOfMass(float neighborhoodSize, float cm[3]);
	 int		RobotPlayer::computeRepulsion(float neighborhoodSize, float repulse[3]);
	 int		RobotPlayer::computeAlign(float neighborhoodSize,
									  float avV[3], float* avAzimuth);
	 void		RobotPlayer::findHomeBase(TeamColor teamColor, float location[3]);
	 void RobotPlayer::findPatrolArea(TeamColor teamColor, float patrolArea[3]);
	 float* RobotPlayer::findPatrolPoint(float patrolArea[3]);
	 bool		RobotPlayer::myTeamHoldingOpponentFlag(void);
	 void		RobotPlayer::findOpponentFlag(float location[3]);
	 Player*		RobotPlayer::lookupLocalPlayer(PlayerId id);
	 void		RobotPlayer::aStarSearch(const float startPos[3], const float goalPos[3],
										 std::vector< std::vector< AStarNode > >& paths);
	 std::vector< AStarNode > RobotPlayer::generateSmoothedPath(std::vector< AStarNode > orig);
	 bool		RobotPlayer::pathIsClear(AStarNode& start, AStarNode& end);
	 void		RobotPlayer::seekFlag();
	 int		RobotPlayer::findClosestPartOfPath(float* position);
	 void		RobotPlayer::determineStatusAndGoal(float* goalPos);
	 bool		RobotPlayer::isClosestToOwnFlag();
	 bool		RobotPlayer::flagIsInBase();
	 bool		RobotPlayer::flagIsStolen();
	 bool		RobotPlayer::oneOfTwoClosestToOwnFlag();
	 bool		RobotPlayer::isBestInterceptCandidate();
	 void		RobotPlayer::findWallNextToBase(float* goal);
	 bool		RobotPlayer::inCorrectBaseSpot();

	 static const float		CohesionW;
	 static const float		SeparationW;
	 static const float		AlignW;
	 static const float		PathW;

  private:
    const Player*	target;
    std::vector<RegionPoint>	path;
    int		pathIndex;
    float		timerForShot;
    bool		drivingForward;
    static std::vector<BzfRegion*>* obstacleList;
	std::vector< std::vector< AStarNode > > paths; // planner result paths
	AStarNode pathGoalNode; // goal position for current planner result
	float shotAngle; // azimuth of incoming shot
	float targetdistance; // distance to target
	float targetdir[3]; // direction to target
	int currentStatus; //number indicating what the tank will do at the moment
	bool seekingFlag;
};

#endif // BZF_ROBOT_PLAYER_H

// Local Variables: ***
// mode:C++ ***
// tab-width: 8 ***
// c-basic-offset: 2 ***
// indent-tabs-mode: t ***
// End: ***
// ex: shiftwidth=2 tabstop=8

