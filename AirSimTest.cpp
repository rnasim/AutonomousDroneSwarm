#include "RRTStar.hpp"
#include "Graph.hpp"
#include "AirSimTest.hpp"



std::vector<Vector3r> generatePath(std::vector<Node*> path, Vector3r loc) {
	std::vector<Vector3r> drone_path;
	for (auto& node : path) {
		if (node->node_number == 0) {
			std::cout << "Starting location: ";
			std::cout << "(" << node->coord->x << ", " << node->coord->y << ", " << -1 * node->coord->z << ")\n";
			continue;
			
		}
		node->printNode();
		drone_path.push_back(Vector3r(node->coord->x - loc.x(), node->coord->y - loc.y(), -1 * node->coord->z - loc.z()));

	}
	
	return drone_path;
}

void droneTest() {
	// Array of Drones
	const int droneAmount = NUMDRONES;
	msr::airlib::MultirotorRpcLibClient* clients = new msr::airlib::MultirotorRpcLibClient[droneAmount];
	DroneRRTSTAR* drones = new DroneRRTSTAR[droneAmount];
	std::thread droneThreads[droneAmount];

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
		//std::cout << "Press Enter to move og location" << std::endl; std::cin.get();
		//int y = 0;
		//for (int i = 0; i < droneAmount; i++) {
		//	string droneName = "Drone" + to_string(i + 1);
		//	msr::airlib::Pose pose = drones[i].client->simGetObjectPose(droneName);
		//	std::cout << "(" << pose.position.x() << ", " << pose.position.y() << ", " << pose.position.z() << ")" << std::endl;
		//	//pose.position = Vector3r(0, y, 0);
		//	//drones[i].client->simSetObjectPose(droneName, pose, true);
		//	//y += 4;
		//}
		
		std::cout << "Press Enter to set goal" << std::endl; std::cin.get();
		for (int i = 0; i < droneAmount; i++) {
			drones[i].goal = new Coord(50, 25, 5);
		}

		//drones[0].goal = new Coord(50, 25, 5);
		//drones[1].goal = new Coord(30, 10, 5);
		//drones[2].goal = new Coord(30, 50, 5);

		std::cout << "Press Enter to takeoff" << std::endl; std::cin.get();
		float takeoffTimeout = 20;
		for (int i = 0; i < droneAmount; i++) {
			string droneName = "Drone" + to_string(i + 1);
			droneThreads[i] = std::thread(&MultirotorRpcLibClient::takeoffAsync, drones[i].client, takeoffTimeout, droneName);
		}
		for (int i = 0; i < droneAmount; i++) {
			droneThreads[i].join();
		}

		std::cout << "Press Enter to calculate RRTStar" << std::endl; std::cin.get();

		//std::vector<Node*> path;
		//for (int i = 0; i < droneAmount; i++) {
		//	auto pos = drones[i].client->getMultirotorState().getPosition();
		//	
		//	Coord start = Coord(pos.x(), pos.y(), pos.z());
		//	Coord end = Coord(-50, 25, 5);
		//	
		//	BiRRTStar brrt = BiRRTStar(start, end);
		//	path = brrt.CallRRTStar();
		//	for (auto& it : path) {
		//		std::cout << it->node_number << " ";
		//	}
		//	std::cout << std::endl;
		//}

		// Code from TestMany
		//std::vector<Coord> startCoords;
		//std::vector<Coord> goalCoords;
		//for (int i = 0; i < 3; i++) {
		//	float offset = i * 20;
		//	Coord startCoord = Coord(0 + offset, 0 + offset, 0 + offset);				// 0,0,0					20,20,20					40,40,40
		//	Coord goalCoord = Coord(23.12f + offset, -45.37f + offset, 23.5f + offset); // 23.12, -45.37, 23.5		43.12, -25.37, 43.5			63.12, -5.37, 63.5
		//	startCoords.push_back(startCoord);
		//	goalCoords.push_back(goalCoord);
		//}

		//Graph* graph = rrtStarMany(startCoords, goalCoords);
		//if (graph) {
		//	graph->printGraph();
		//	graph->printPathMany();
		//}

		// End Code from TestMany

		std::vector<std::future<void>> futureResults;
		for (int i = 0; i < droneAmount; i++) {
			string droneName = "Drone" + to_string(i + 1);
			int offset = -i * 20;
			auto pos = drones[i].client->simGetObjectPose(droneName).position;
			//auto pos = drones[i].client->getMultirotorState().getPosition();
			Coord start = Coord(pos.x(), pos.y(), pos.z());
			Coord end = *drones[i].goal;
			drones[i].rrt_graph = new BiRRTStar(start, end);
			futureResults.emplace_back(std::async(std::launch::async, &BiRRTStar::CallRRTStar, drones[i].rrt_graph));
			//Coord startingCoord = Coord(pos.x(), pos.y(), pos.z());
			//Coord goalCoord = Coord(23.12f, -58.37f, 23.5f);
			//futureResults.emplace_back(std::async(std::launch::async, rrtStar, startingCoord, goalCoord));
		}
		for (int i = 0; i < droneAmount; i++) {
			//drones[i].rrt_graph = futureResults.at(i).get();
			futureResults.at(i).get();
		}
		for (int i = 0; i < droneAmount; i++) {
			std::cout << "Graph " << i + 1 << std::endl;
			if (drones[i].rrt_graph) {
				for (auto& it : drones[i].rrt_graph->getPath()) {
					std::cout << it->node_number << " ";
				}
			}
			std::cout << std::endl;
		}

		std::cout << "Press Enter to move drones" << std::endl; std::cin.get();
		DrivetrainType driveTrain = DrivetrainType::MaxDegreeOfFreedom;
		YawMode yaw_mode(true, 0);
		float velocity = 10.f;
		float moveTimeout = 30.0f;
		float lookahead = -1.0f;
		float adaptive_lookahead = 1.0f;
		for (int i = 0; i < droneAmount; i++) {
			string droneName = "Drone" + to_string(i + 1);
			auto pos = drones[i].client->simGetObjectPose(droneName).position;
			std::vector<Vector3r> v = generatePath(drones[i].rrt_graph->getPath(), pos);
			droneThreads[i] = std::thread(&MultirotorRpcLibClient::moveOnPathAsync, drones[i].client, v, velocity, moveTimeout, driveTrain, yaw_mode, lookahead, adaptive_lookahead, droneName);
		}
		for (int i = 0; i < droneAmount; i++) {
			droneThreads[i].join();
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
