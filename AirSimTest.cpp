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


//bool distanceCheck(Node* A, Node* C, Node* D, Node* F) {
//
//	float x;
//	int detection_size = 2;
//
//	x = findDistance(A->coord, D->coord);
//	if (x < DRONESIZE * detection_size) {
//		return true;
//	}
//
//	x = findDistance(A->coord, F->coord);
//	if (x < DRONESIZE * detection_size) {
//		return true;
//	}
//
//	x = findDistance(C->coord, D->coord);
//	if (x < DRONESIZE * detection_size) {
//		return true;
//	}
//
//	x = findDistance(C->coord, F->coord);
//	if (x < DRONESIZE * detection_size) {
//		return true;
//	}
//
//	return false;
//}

bool distanceCheck(Node* A, Node* D) {

	float x;
	int detection_size = 25;

	x = findDistance(A->coord, D->coord);
	if (x < DRONESIZE * detection_size) {
		return true;
	}

	return false;
}

std::vector<CollisionDetails*> droneCollisionCheck(int droneNum, std::vector<std::vector<Node*>> dronesPathList, std::vector<std::vector<Node*>> allDronesPaths) {
	std::vector<CollisionDetails*> collisions;

	//compare each drone segment to our current paths ones
	std::cout << "dronepathlist size" << dronesPathList.size() <<"   Drone num:"<< droneNum << std::endl;
	for (int i = 0; i < allDronesPaths.size(); i++) {
		std::vector<Node*> dronePath = allDronesPaths.at(droneNum);
		//skip comparing with itself
		if (i == droneNum) {
			continue;
		}
		for (int j = 0; j < allDronesPaths.at(i).size()-1; j++) {
			bool collision;
			//loop through current drones path and check if each rectangle intersects or nah
			for (int k = 0; k < dronePath.size() - 1; k++) {
				//std::cout << "Enter Third Loop\n";
				//collision = distanceCheck(dronePath.at(k), dronePath.at(k + 1), dronesPathList.at(i).at(j), dronesPathList.at(i).at(j + 1));
				collision = distanceCheck(dronePath.at(k), allDronesPaths.at(i).at(j));
				if (collision) {
					//std::cout << "Collision\n";
					collisions.push_back(new CollisionDetails(k, i, j+1, false));
					break;
				}
			}
		}
	}
	/*std::cout << "Collisions size: " << collisions.size() << std::endl;
	std::cout << "Drone " << droneNum << std::endl;
	for (int j = 0; j < collisions.size(); j++) {
		std::cout << "Collding drone: " << collisions.at(j)->colliding_drone << std::endl;
		std::cout << "Collding drone safe index: " << collisions.at(j)->colliding_drone_safe_index << std::endl;
		std::cout << "Collision Index: " << collisions.at(j)->collision_index << std::endl;
	}*/
	return collisions;
}


void fly(DroneRRTSTAR drone, int indexOfDrones[], int curr, std::vector<std::vector<CollisionDetails*>> droneIntersections, int priority[]) {
	DrivetrainType driveTrain = DrivetrainType::MaxDegreeOfFreedom;
	YawMode yaw_mode(true, 0);
	float velocity = 3.f;
	float moveTimeout = 30.0f;
	float lookahead = -1.0f;
	float adaptive_lookahead = 1.0f;
	string droneName = "Drone" + to_string(curr + 1);
	auto pos = drone.client->simGetObjectPose(droneName).position;
	std::cout << "DroneINTERSECTION SIZE: " << droneIntersections.at(curr).size() << std::endl;
	indexOfDrones[curr] = 0;
	
	for (int j = 0; j < drone.rrt_graph->getPath().size(); j++) {
		
		//check
		//int waiting_for_drone;
		int flag = -1;
		for (int k = 0; k < droneIntersections.at(curr).size(); k++) {
			if (droneIntersections.at(curr).at(k)->collision_index < j + 5) {
				
				/*if (indexOfDrones[droneIntersections.at(curr).at(k)->colliding_drone] >= droneIntersections.at(curr).at(k)->colliding_drone_safe_index) {
					droneIntersections.at(curr).at(k)->active_collision = false;	
				}*/

				if ((indexOfDrones[droneIntersections.at(curr).at(k)->colliding_drone] <= droneIntersections.at(curr).at(k)->colliding_drone_safe_index) &&
					(droneIntersections.at(curr).at(k)->colliding_drone_safe_index - indexOfDrones[droneIntersections.at(curr).at(k)->colliding_drone] <= 8)) {
					//&&indexOfDrones[droneIntersections.at(curr).at(k)->colliding_drone] <= droneIntersections.at(curr).at(k)->colliding_drone_safe_index) {
					std::cout << "collision True\n";
					droneIntersections.at(curr).at(k)->active_collision = true;
					flag = k;
				}
				else {
					std::cout << "collision False\n";
					std::cout << "Flag = " << flag << std::endl;
					droneIntersections.at(curr).at(k)->active_collision = false;
					break;
				}

				//for (int i = 0; i < droneIntersections.at(droneIntersections.at(curr).at(k)->colliding_drone).size(); i++) {
				//	if (droneIntersections.at(droneIntersections.at(curr).at(k)->colliding_drone).at(i)->active_collision) {
				//		flag = -2;
				//		break;
				//	}
				//}
				//if (flag == -2) {
				//	break;
				//}

				//if (droneIntersections.at(curr).at(k)->active_collision) {
				//	waiting_for_drone = droneIntersections.at(curr).at(k)->colliding_drone;
				//	std::cout << "Index of Colliding Drone: " << indexOfDrones[droneIntersections.at(curr).at(k)->colliding_drone] << std::endl;
				//	flag = k;
				//	break;
				//}
			}
		}

		if (flag < 0 || curr == priority[0]) {
			std::cout << "( " << drone.rrt_graph->getPath().at(j)->coord->y - pos.x() << ", " << drone.rrt_graph->getPath().at(j)->coord->x - pos.y() << ", " << drone.rrt_graph->getPath().at(j)->coord->z << ")\n";
			drone.client->moveToPositionAsync(drone.rrt_graph->getPath().at(j)->coord->y - pos.x(), drone.rrt_graph->getPath().at(j)->coord->x - pos.y(), -1*drone.rrt_graph->getPath().at(j)->coord->z, velocity, moveTimeout, driveTrain, yaw_mode, lookahead, adaptive_lookahead, droneName)->waitOnLastTask();
			if (j == drone.rrt_graph->getPath().size() - 1) {
				priority[0] = curr + 1;
			}
			indexOfDrones[curr] = j;
			
		}
		else {
			std::cout << "Drone " << curr + 1 << " is waiting for \n"; //<< waiting_for_drone + 1<< std::endl;
			std::this_thread::sleep_for(std::chrono::duration<double>(2.5));
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
	static int indexOfDrones[NUMDRONES];
	int priority[1];
	std::vector<std::vector<Node*>> allDronePaths;
	//pathIntersectionDetails.resize(NUMDRONES);
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
		drones[1].goal = new Coord(20, -30, 5);
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
		std::vector<std::vector<Node*>> flyingDrones;
		for (int i = 0; i < droneAmount; i++) {
			pathIntersectionDetails.push_back(droneCollisionCheck(i, flyingDrones, allDronePaths));
			flyingDrones.push_back(allDronePaths.at(i));
		}
		
		std::cout << "pathhinterseacoitpahtohawoipj " << pathIntersectionDetails.size() << std::endl;
		//for (int i = 0; i < pathIntersectionDetails.size(); i++) {
		//	std::cout << "Drone " << i + 1 << std::endl;
		//	for (int j = 0; j < pathIntersectionDetails.at(i).size(); j++) {
		//		std::cout << "Collding drone: " << pathIntersectionDetails.at(i).at(j)->colliding_drone << std::endl;
		//		std::cout << "Collding drone safe index: " << pathIntersectionDetails.at(i).at(j)->colliding_drone_safe_index << std::endl;
		//		std::cout << "Collision Index: " << pathIntersectionDetails.at(i).at(j)->collision_index << std::endl;
		//	}
		//}

		std::cout << "Press Enter to move drones" << std::endl; std::cin.get();
		DrivetrainType driveTrain = DrivetrainType::MaxDegreeOfFreedom;
		YawMode yaw_mode(true, 0);
		float velocity = 4.f;
		float moveTimeout = 30.0f;
		float lookahead = -1.0f;
		float adaptive_lookahead = 1.0f;

		priority[0] = 0;
		for (int i = 0; i < droneAmount; i++) {
			//string droneName = "Drone" + to_string(i + 1);
			//auto pos = drones[i].client->simGetObjectPose(droneName).position;
			
			// Holds the current index of the drone path
			indexOfDrones[i] = 0;
			droneThreads[i] = std::thread(fly, drones[i], indexOfDrones, i, pathIntersectionDetails, priority);

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
