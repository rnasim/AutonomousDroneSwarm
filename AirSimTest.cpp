#include "RRTStar.hpp"
#include "Graph.hpp"
#include "AirSimTest.hpp"

std::vector<Vector3r> generatePath(std::vector<Node*> path, Vector3r loc) {
	std::vector<Vector3r> drone_path;
	for (auto& node : path) {
		std::cout << "(" << node->coord->y - loc.x() << ", " << node->coord->x - loc.y() << ", " << -1 * node->coord->z - loc.z() << ")\n";
		drone_path.push_back(Vector3r(node->coord->y - loc.x(), node->coord->x - loc.y(), -1 * node->coord->z - loc.z()));
	}

	return drone_path;
}


bool distanceCheck(Node* A, Node* C, Node* D, Node* F) {

	float x;
	bool collision;
	x = pow(D->coord->x - A->coord->x, 2) + pow(D->coord->y - A->coord->y, 2) + pow(D->coord->z - A->coord->z, 2);
	x = sqrt(x);
	if (x < DRONESIZE * 2) {
		collision = true;
	}

	x = pow(F->coord->x - A->coord->x, 2) + pow(F->coord->y - A->coord->y, 2) + pow(F->coord->z - A->coord->z, 2);
	x = sqrt(x);
	if (x < DRONESIZE * 2) {
		collision = true;
	}

	x = pow(D->coord->x - C->coord->x, 2) + pow(D->coord->y - C->coord->y, 2) + pow(D->coord->z - C->coord->z, 2);
	x = sqrt(x);
	if (x < DRONESIZE * 2) {
		collision = true;
	}

	x = pow(F->coord->x - C->coord->x, 2) + pow(F->coord->y - C->coord->y, 2) + pow(F->coord->z - C->coord->z, 2);
	x = sqrt(x);
	if (x < DRONESIZE * 2) {
		collision = true;
	}

	return  false;
}

std::vector<CollisionDetails*> droneCollisionCheck(int droneNum, std::vector<std::vector<Node*>> dronesPathList) {
	std::vector<Node*> dronePath = dronesPathList.at(droneNum);

	std::vector<CollisionDetails*> collisions;

	//compare each drone segment to our current paths ones
	for (int i = 0; i < dronesPathList.size(); i++) {
		//skip comparing with itself
		if (i == droneNum) {
			continue;
		}
		for (int j = 0; j < dronesPathList.at(i).size()-1; j++) {
			bool collision;
			//loop through current drones path and check if each rectangle intersects or nah
			for (int k = 0; k < dronePath.size() - 1; k++) {
				collision = distanceCheck(dronePath.at(k), dronePath.at(k + 1), dronesPathList.at(i).at(j), dronesPathList.at(i).at(j + 1));

				if (collision) {
					collisions.push_back(new CollisionDetails(k, i, j, false));
				}
			}
		}
	}
	return collisions;
}


void fly(DroneRRTSTAR drone, int indexOfDrones[], int curr, std::vector<CollisionDetails*> droneIntersections) {
	std::cout << "Enter fly\n";
	DrivetrainType driveTrain = DrivetrainType::MaxDegreeOfFreedom;
	YawMode yaw_mode(true, 0);
	float velocity = 4.f;
	float moveTimeout = 30.0f;
	float lookahead = -1.0f;
	float adaptive_lookahead = 1.0f;
	string droneName = "Drone" + to_string(curr + 1);
	auto pos = drone.client->simGetObjectPose(droneName).position;

	for (int j = 0; j < drone.rrt_graph->getPath().size(); j++) {
		//check
		indexOfDrones[curr] = j;
		int flag = -1;
		for (int k = 0; k < droneIntersections.size(); k++) {
			if (droneIntersections.at(k)->collision_index == j) {
				if (indexOfDrones[droneIntersections.at(k)->colliding_drone] >= droneIntersections.at(k)->colliding_drone_safe_index) {
					droneIntersections.at(k)->active_collision = false;
				}
				if (droneIntersections.at(k)->active_collision) {
					flag = k;
					break;
				}
			}
		}

		if (flag < 0) {
			std::cout << "( " << drone.rrt_graph->getPath().at(j)->coord->y - pos.x() << ", " << drone.rrt_graph->getPath().at(j)->coord->x - pos.y() << ", " << drone.rrt_graph->getPath().at(j)->coord->z << ")\n";
			drone.client->moveToPositionAsync(drone.rrt_graph->getPath().at(j)->coord->y - pos.x(), drone.rrt_graph->getPath().at(j)->coord->x - pos.y(), -1*drone.rrt_graph->getPath().at(j)->coord->z, velocity, moveTimeout, driveTrain, yaw_mode, lookahead, adaptive_lookahead, droneName)->waitOnLastTask();
		}
		else {
			j--;
		}
	}
}

void droneTest() {
	// Array of Drones
	const int droneAmount = NUMDRONES;
	msr::airlib::MultirotorRpcLibClient* clients = new msr::airlib::MultirotorRpcLibClient[droneAmount];
	DroneRRTSTAR* drones = new DroneRRTSTAR[droneAmount];
	std::thread droneThreads[droneAmount];

	//
	std::vector<std::vector<CollisionDetails*>> pathIntersectionDetails;
	int indexOfDrones[NUMDRONES];
	std::vector<std::vector<Node*>> allDronePaths;
	pathIntersectionDetails.resize(NUMDRONES);
	//

	try {
		for (int i = 0; i < droneAmount; i++) {
			drones[i].client = &clients[i];
			drones[i].client->confirmConnection();
		}

		std::cout << "Press Enter to arm the drone" << std::endl; std::cin.get();
		for (int i = 0; i < droneAmount; i++) {
			string droneName = "Drone" + to_string(i + 1);
			drones[i].client->enableApiControl(true, droneName);
			drones[i].client->armDisarm(true, droneName);
		}
		
		std::cout << "Press Enter to set goal" << std::endl; std::cin.get();
		//for (int i = 0; i < 5; i++) {
		//	drones[i].goal = new Coord(25, 50, 5);
		//}

		//for (int i = 5; i < droneAmount; i++) {
		//	drones[i].goal = new Coord(50, 30, 5);
		//}

		drones[0].goal = new Coord(20, -45, 5);
		drones[1].goal = new Coord(20, -40, 5);
		drones[2].goal = new Coord(-20, -45, 5);
		drones[3].goal = new Coord(-20, -40, 5);
		drones[4].goal = new Coord(0, -45, 5);
		drones[5].goal = new Coord(20, 45, 5);
		drones[6].goal = new Coord(20, 40, 5);
		drones[7].goal = new Coord(-20, 45, 5);
		drones[8].goal = new Coord(-20, 40, 5);
		drones[9].goal = new Coord(0, 45, 5);

		std::cout << "Press Enter to takeoff" << std::endl; std::cin.get();
		float takeoffTimeout = 20;
		for (int i = 0; i < droneAmount; i++) {
			string droneName = "Drone" + to_string(i + 1);
			droneThreads[i] = std::thread(&MultirotorRpcLibClient::takeoffAsync, drones[i].client, takeoffTimeout, droneName);
		}

		for (int i = 0; i < droneAmount; i++) {
			droneThreads[i].join();
		}

		std::cout << "Press Enter to find starting location" << std::endl; std::cin.get();
		for (int i = 0; i < droneAmount; i++) {
			string droneName = "Drone" + to_string(i + 1);
			msr::airlib::Pose pose = drones[i].client->simGetObjectPose(droneName);
			auto pos = drones[i].client->getMultirotorState().getPosition();
			std::cout << "Drone " << i + 1 << std::endl;
			std::cout << "Unreal: (" << pose.position.x() << ", " << pose.position.y() << ", " << pose.position.z() << ")" << std::endl;
			std::cout << "Local: (" << pos.x() << ", " << pos.y() << ", " << pos.z() << ")" << std::endl;
		}

		std::cout << "Press Enter to calculate RRTStar" << std::endl; std::cin.get();

		std::vector<std::future<void>> futureResults;
		for (int i = 0; i < droneAmount; i++) {
			string droneName = "Drone" + to_string(i + 1);
			auto pos = drones[i].client->simGetObjectPose(droneName).position;
			Coord start = Coord(pos.y(), pos.x(), -1*pos.z());
			Coord end = *drones[i].goal;
			drones[i].rrt_graph = new BiRRTStar(start, end);
			futureResults.emplace_back(std::async(std::launch::async, &BiRRTStar::CallRRTStar, drones[i].rrt_graph));
		}
		for (int i = 0; i < droneAmount; i++) {
			futureResults.at(i).get();
		}
		for (int i = 0; i < droneAmount; i++) {
			std::cout << "Graph " << i + 1 << std::endl;
			if (drones[i].rrt_graph) {
				allDronePaths.push_back(drones[i].rrt_graph->getPath());
				for (auto& it : drones[i].rrt_graph->getPath()) {
					std::cout << it->node_number << " ";
				}
			}
			std::cout << std::endl;
		}

		std::cout << "Press Enter to check drone path collisions" << std::endl; std::cin.get();
		for (int i = 0; i < droneAmount; i++) {
			pathIntersectionDetails.push_back(droneCollisionCheck(i, allDronePaths));
		}

		std::cout << "Press Enter to move drones" << std::endl; std::cin.get();
		DrivetrainType driveTrain = DrivetrainType::MaxDegreeOfFreedom;
		YawMode yaw_mode(true, 0);
		float velocity = 4.f;
		float moveTimeout = 30.0f;
		float lookahead = -1.0f;
		float adaptive_lookahead = 1.0f;

		for (int i = 0; i < droneAmount; i++) {
			//string droneName = "Drone" + to_string(i + 1);
			//auto pos = drones[i].client->simGetObjectPose(droneName).position;
			
			// Holds the current index of the drone path
			indexOfDrones[i] = 0;
			droneThreads[i] = std::thread(fly, drones[i], indexOfDrones, i, pathIntersectionDetails.at(i));

			//std::vector<Vector3r> v = generatePath(drones[i].rrt_graph->getPath(), pos);
			//droneThreads[i] = std::thread(&MultirotorRpcLibClient::moveOnPathAsync, drones[i].client, v, velocity, moveTimeout, driveTrain, yaw_mode, lookahead, adaptive_lookahead, droneName);
		}
		for (int i = 0; i < droneAmount; i++) {
			droneThreads[i].join();
		}

		std::cout << "Press Enter to show drone location" << std::endl; std::cin.get();
		int y = 0;
		for (int i = 0; i < droneAmount; i++) {
			string droneName = "Drone" + to_string(i + 1);
			msr::airlib::Pose pose = drones[i].client->simGetObjectPose(droneName);
			auto pos = drones[i].client->getMultirotorState().getPosition();
			std::cout << "Drone " << i + 1 << std::endl;
			std::cout << "Unreal: (" << pose.position.x() << ", " << pose.position.y() << ", " << pose.position.z() << ")" << std::endl;
			std::cout << "Local: (" << pos.x() << ", " << pos.y() << ", " << pos.z() << ")" << std::endl;
		}

		std::cout << "Press Enter to disarm" << std::endl; std::cin.get();
		for (int i = 0; i < droneAmount; i++) {
			string droneName = "Drone" + to_string(i + 1);
			drones[i].client->armDisarm(false, droneName);
			drones[i].client->enableApiControl(false, droneName);
			drones[i].client->reset();
		}

		// Garbage Collection
		delete[] clients;
		delete[] drones;
		//delete rrtTree;
	}
	catch (rpc::rpc_error& e) {
		std::string msg = e.get_error().as<std::string>();
		std::cout << "Exception raised by the API, something went wrong." << std::endl << msg << std::endl;
	}
}
