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

using namespace msr::airlib;
using namespace std;

void droneTest();
std::vector<Vector3r> generatePath(std::vector<Node*> path, Vector3r loc);

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

#endif