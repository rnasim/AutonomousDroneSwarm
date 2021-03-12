#pragma once
#ifndef AIRSIMTEST_HPP
#define AIRSIMTEST_HPP

// Drone stuff
// Only works with airsim library

#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "common/common_utils/FileSystem.hpp"
#include <iostream>
#include <chrono>
#include <thread> 
#include <future>
#include <list>

using namespace msr::airlib;
using namespace std;

struct DroneRRTSTAR {
	msr::airlib::MultirotorRpcLibClient* client;
	BiRRTStar* rrt_graph;
	Coord* goal;

	DroneRRTSTAR() {
		client = NULL;
		rrt_graph = NULL;
		goal = NULL;
	}
	~DroneRRTSTAR() {

	}
};

struct CollisionDetails {
	int collision_index;
	int colliding_drone;
	int colliding_drone_safe_index;
	bool active_collision;

	CollisionDetails(int coll_ind, int coll_drone, int coll_drone_ind, bool active) {
		collision_index = coll_ind;
		colliding_drone = coll_drone;
		colliding_drone_safe_index = coll_drone_ind;
		active_collision = active;
	}
};

void singleDrone();
void droneTest();
std::vector<Vector3r> generatePath(std::vector<Node*> path, Vector3r loc);
std::vector<CollisionDetails*> droneCollisionCheck(int droneNum, std::vector<std::vector<Node*>> dronesPathList, std::vector<std::vector<Node*>> allDronesPaths);




#endif