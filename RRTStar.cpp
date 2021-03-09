#include "RRTStar.hpp"
#include "Graph.hpp"

// Constructor
BiRRTStar::BiRRTStar(Coord start_coord, Coord end_coord)
{
	start_node = new Node(0, start_coord);
	end_node = new Node(1, end_coord);
};

void BiRRTStar::CallRRTStar() {
	//Initializes obstacles here
	Obstacles o = Obstacles(MAPMINX, MAPMINY, MAPMINZ, MAPMAXX, MAPMAXY, MAPMAXZ);
	o.initObstacles();
	addObstacles(&o);

	Graph* graphA = new Graph(start_node);
	start_node->coord->printCoord();
	Graph* graphB = new Graph(end_node);
	end_node->coord->printCoord();

	int i = 2;
	while (i < NUMNODES + 1) {
		//if (graphA->isInit()) {
		//	std::cout << "Graph A! \n";
		//}
		//else {
		//	std::cout << "Graph B! \n";
		//}

		Coord random_coord = Coord();	// Random Position
		Node* nearest_node = nearestNode(&random_coord, graphA); // Nearest Node from the random point

		// Creating coordinate and node step size away from the nearest coord
		Coord step = stepNode(nearest_node->coord, &random_coord);

		if (o.collisionCheck(nearest_node->coord, &step)) { // If obstacle is in between two nodes return true
			//std::cout << "collision here!\n";
			//step.printCoord();
			continue;
		}

		float cost = findDistance(nearest_node->coord, &step);
		if (cost > findDistance(nearest_node->coord, &random_coord)) {
			continue;
		}

		Node* new_node = new Node(i, step);
		graphA->addEdge(nearest_node, new_node, cost);

		//if (i % 100 == 0) {
		new_node->printNode();
		//}

		std::vector<Node*> neighbors = nearestNeighbors(new_node, graphA);
		Node* bestNode = chooseBestParent(new_node, neighbors);
		if (bestNode) {
			graphA->rewireEdge(bestNode, new_node);
		}

		Node* nearest_vertex = nearestNode(new_node->coord, graphB); // Nearest Node from opposite graph
		if (!o.collisionCheck(new_node->coord, nearest_vertex->coord)) {
			connect(new_node, nearest_vertex, graphA);
			return;
		}

		std::swap(graphA, graphB);
		i++;
	}
	std::cout << "Goal was not found" << std::endl;

	// Return empty if no path found
	return;
}

void BiRRTStar::addObstacles(Obstacles* o) { obs = o; }

// Creates a new coordinate step size away from the selected Node
Coord BiRRTStar::stepNode(Coord* coord, Coord* random_coord)
{
	//Need to handle case when coord+step_size is greater than random point
	float x_new, y_new, z_new, hypotonouse;
	z_new = ((STEPSIZE)*sqrt(2.0f)) / 2.0f;
	hypotonouse = ((STEPSIZE)*sqrt(2.0f)) / 2.0f;
	x_new = ((hypotonouse)*sqrt(2.0f)) / 2.0f;
	y_new = ((hypotonouse)*sqrt(2.0f)) / 2.0f;


	// Checks if random coordinate is at a location less than the nearest coordinate
	if (random_coord->x < coord->x) { x_new = -x_new; }
	if (random_coord->y < coord->y) { y_new = -y_new; }
	if (random_coord->z < coord->z) { z_new = -z_new; }

	Coord newCoord = Coord(coord->x + x_new, coord->y + y_new, coord->z + z_new);

	return newCoord;
}

Coord BiRRTStar::extend(Coord* point, Coord* vector_dir, int t)
{
	float x = point->x + (t * STEPSIZE) * vector_dir->x;
	float y = point->y + (t * STEPSIZE) * vector_dir->y;
	float z = point->z + (t * STEPSIZE) * vector_dir->z;

	Coord newCoord = Coord(x, y, z);

	return newCoord;
}

Node* BiRRTStar::nearestNode(Coord* random_coord, Graph* graph)
{

	hashTable* hash = graph->getHashTable();
	int loc = hash->search(random_coord);
	/* Uncomment this section for recursive node search. Still not optimized, not sure if it can be optimized further*/
	//int lowerIndex = loc - ((float)RADIUS / (float)hash->getBucketRange());
	//int upperIndex = loc + ((float)RADIUS / (float)hash->getBucketRange()) + 1;

	//if (upperIndex >= hash->getNumBuckets()) {
	//	upperIndex = hash->getNumBuckets();
	//}
	//if (lowerIndex < 0) {
	//	lowerIndex = 0;
	//}
	//std::cout << "Lowerindex = " << lowerIndex << "\tHigh Index = " << upperIndex << "\tx coord = " << random_coord->x << std::endl;

	/*END SECTION*/

	std::vector<Node*>* table = hash->getTable();
	Node* nearest_node = graph->getFirstNode();

	/* Uncomment this section for naive node search. Fastest run time*/

	int lowerIndex = 0;
	int upperIndex = NUMNODES;

	/*END SECTION*/
	for (int i = lowerIndex; i < upperIndex; i++) {
		for (int j = 0; j < table[i].size(); j++) {
			if (findDistance(random_coord, nearest_node->coord) > findDistance(random_coord, table[i].at(j)->coord))
			{
				if (findDistance(random_coord, table[i].at(j)->coord) != 0) {
					nearest_node = table[i].at(j);
				}
			}
		}
	}


	if (!hash->inBucket(nearest_node, lowerIndex, upperIndex)) {
		/*	return nearestNode(random_coord, graph, 1, lowerIndex, upperIndex);*/
		//std::cout << "HELLO RECURSION" << std::endl;
		float offset = 100;
		int lowerIndexLowBound = loc - (((float)RADIUS + offset) / (float)hash->getBucketRange());
		int upperIndexHighBound = loc + (((float)RADIUS + offset) / (float)hash->getBucketRange()) + 1;

		int& LILB = lowerIndexLowBound;
		int& UIHB = upperIndexHighBound;

		int& LIHB = lowerIndex;
		int& UILB = upperIndex;

		return nearestNode(random_coord, hash, LIHB, UILB, nearest_node, table, LILB, UIHB, offset);

	}

	return nearest_node;
}

Node* BiRRTStar::nearestNode(Coord* random_coord, hashTable* hash, int& lowerIndexHighBound, int& upperIndexLowBound, Node* nearest_node, std::vector<Node*>* table, int& lowerIndexLowBound, int& upperIndexHighBound, float offset) {

	//std::cout << "HELLO RECURSION1" << std::endl;
	//std::cout << "HELLO RECURSION2" << std::endl;

	if (upperIndexHighBound >= hash->getNumBuckets()) {
		upperIndexHighBound = hash->getNumBuckets();
	}
	if (lowerIndexLowBound <= 0) {
		lowerIndexLowBound = 0;
	}
	//std::cout << "Lowerindex = " << lowerIndex << "\tHigh Index = " << upperIndex << "\tx coord = " << random_coord->x << std::endl;

	//lowerIndex = 0;
	//upperIndex = NUMNODES - 1;


	//for (int i = lowerIndex; i < upperIndex; i++) {
	//	///std::cout << "HELLO RECURSION3 = " << i << std::endl;

	//	for (int j = 0; j < table[i].size(); j++) {
	//		if (findDistance(random_coord, nearest_node->coord) > findDistance(random_coord, table[i].at(j)->coord))
	//		{
	//			if (findDistance(random_coord, table[i].at(j)->coord) != 0) {
	//			///	std::cout << "HELLO RECURSION4" << std::endl;
	//				nearest_node = table[i].at(j);
	//			}
	//		}
	//	}
	//}

	int i = 0;
	while (i < offset) {
		if (lowerIndexLowBound < (lowerIndexHighBound - i)) {
			for (int j = 0; j < table[lowerIndexHighBound - i].size(); j++) {
				if (findDistance(random_coord, nearest_node->coord) > findDistance(random_coord, table[lowerIndexHighBound - i].at(j)->coord))
				{
					if (findDistance(random_coord, table[lowerIndexHighBound - i].at(j)->coord) != 0) {
						nearest_node = table[lowerIndexHighBound - i].at(j);
					}

				}
			}
		}

		if ((upperIndexLowBound + i) < upperIndexHighBound) {
			for (int j = 0; j < table[upperIndexLowBound + i].size(); j++) {
				if (findDistance(random_coord, nearest_node->coord) > findDistance(random_coord, table[upperIndexLowBound + i].at(j)->coord))
				{
					if (findDistance(random_coord, table[upperIndexLowBound + i].at(j)->coord) != 0) {
						nearest_node = table[upperIndexLowBound + i].at(j);
					}

				}
			}
		}
		i++;
	}

	if (!hash->inBucket(nearest_node, lowerIndexLowBound, upperIndexHighBound)) {
		/* Bound variables now switch here. lowerIndexHighBound will become the new low bound. Same for upperIndex	*/
		//std::cout << "HELLO RECURSION 2" << std::endl;
		//std::cout << "Lowerindex = " << lowerIndexLowBound << "\tHigh Index = " << upperIndexHighBound << "\tx coord = " << nearest_node->coord->x << std::endl;
		float diff;
		if (lowerIndexHighBound - lowerIndexLowBound >= upperIndexHighBound - upperIndexLowBound) {
			diff = lowerIndexHighBound - lowerIndexLowBound;

		}
		else {
			diff = upperIndexHighBound - upperIndexLowBound;

		}
		lowerIndexHighBound = lowerIndexLowBound - diff;
		//std::cout << "Diff = " << diff << std::endl;
		upperIndexLowBound = upperIndexHighBound + diff;
		//std::cout << "Lowerindex HIGH = " << lowerIndexHighBound << "\tHigh Index LOW = " << upperIndexLowBound << "\tx coord = " << nearest_node->coord->x << std::endl;

		if (upperIndexLowBound >= hash->getNumBuckets()) {
			upperIndexLowBound = hash->getNumBuckets();
		}

		if (lowerIndexHighBound < 0) {
			lowerIndexHighBound = 0;
		}

		return nearestNode(random_coord, hash, lowerIndexLowBound, upperIndexHighBound, nearest_node, table, lowerIndexHighBound, upperIndexLowBound, diff);
	}

	return nearest_node;
}

std::vector<Node*> BiRRTStar::nearestNeighbors(Node* new_node, Graph* graph) {

	hashTable* hash = graph->getHashTable();
	std::vector<Node*> neighbors;


	int loc = hash->search(new_node->coord);
	int lowerIndex = loc - ((float)RADIUS / (float)hash->getBucketRange());
	int upperIndex = loc + ((float)RADIUS / (float)hash->getBucketRange()) + 1;

	//std::cout << "lower:" << lowerIndex << ", upper: " << upperIndex << "\n";

	if (upperIndex >= hash->getNumBuckets()) {
		upperIndex = hash->getNumBuckets();
	}
	if (lowerIndex < 0) {
		lowerIndex = 0;
	}

	std::vector<Node*>* table = hash->getTable();

	for (int i = lowerIndex; i < upperIndex; i++) {
		for (int j = 0; j < table[i].size(); j++) {

			if (new_node->parent == table[i].at(j)) {
				//std::cout << "Dont need to add parent as neighbor" << endl;
				continue;
			}
			if (new_node == table[i].at(j)) {
				//std::cout << "Cant be your own neighbor" << endl;
				continue;
			}
			//if distance between node in adj_list and node param < radius
			if (findDistance(new_node->coord, table[i].at(j)->coord) <= RADIUS) {
				neighbors.push_back(table[i].at(j));

			}
		}
	}


	//
	//std::vector<Node*> list = graph->getAdjList();
	//Coord* coord = new_node->coord;
	//std::vector<Node*> neighbors;
	//for (auto& node : list) {
	//	if (!node) { continue; }
	//	if (new_node->parent == node) {
	//		//std::cout << "Dont need to add parent as neighbor" << endl;
	//		continue;
	//	}
	//	if (new_node == node) {
	//		//std::cout << "Cant be your own neighbor" << endl;
	//		continue;
	//	}
	//	//if distance between node in adj_list and node param < radius
	//	if (findDistance(coord, node->coord) <= RADIUS) {
	//		neighbors.push_back(node);
	//	}
	//}

	return neighbors;
}

Node* BiRRTStar::chooseBestParent(Node* new_node, std::vector<Node*> neighbors) {
	for (auto& node_neighbor : neighbors) {
		float cost_new = findDistance(new_node->coord, node_neighbor->coord);
		if (cost_new + node_neighbor->weight < new_node->weight) {
			if (obs->collisionCheck(node_neighbor->coord, new_node->coord)) { // If obstacle is in between two nodes return true
				//std::cout << "collision here when rewiring!\n";
				continue;
			}
			return node_neighbor;
		}
	}
	return NULL;
}

void BiRRTStar::connect(Node* node_src, Node* node_dest, Graph* graph)
{
	// Get path for now CHANGE LATER
	std::vector<Node*> part_1, part_2;

	//path_single.push_back(node);

	// Direction Vector
	float x = node_dest->coord->x - node_src->coord->x;
	float y = node_dest->coord->y - node_src->coord->y;
	float z = node_dest->coord->z - node_src->coord->z;

	float unit_vector = sqrt(x * x + y * y + z * z);

	Coord vector_dir = Coord(x / unit_vector, y / unit_vector, z / unit_vector);
	int t = 1;

	Node* new_node;
	Node* step_node;
	new_node = node_src;
	while (new_node != node_dest) {
		float cost = findDistance(new_node->coord, node_dest->coord);
		if (cost < GOALRADIUS) {
			part_1 = makePath(new_node);
			part_2 = makePath(node_dest);

			cost = 0;
			for (int i = 0; i < part_1.size() - 1; i++) {
				graph->rewireEdge(part_1.at(i), part_1.at(i + 1));
			}
			for (int i = 0; i < part_2.size() - 1; i++) {
				graph->rewireEdge(part_2.at(i), part_2.at(i + 1));
			}

			for (auto& it : part_1) {
				if (it->parent) {
					cost += it->segment;
				}
			}
			for (auto& it : part_2) {
				if (it->parent) {
					cost += it->segment;
				}
			}

			cost += findDistance(new_node->coord, node_dest->coord);

			float total_cost = cost;

			// Debug
			//std::cout << "\nPart 1 starts at " << new_node->node_number << "\n";
			//for (auto& it : part_1) {
			//	std::cout << it->node_number << " ";
			//}
			//std::cout << "\nPart 2 starts at " << node_dest->node_number << "\n";
			//for (auto& it : part_2) {
			//	std::cout << it->node_number << " ";
			//}

			// End Debug
			if (graph->isInit()) {
				std::cout << "\nGraph A found path connection! \n";
				for (auto it = part_1.rbegin(); it != part_1.rend(); ++it) {
					path_single.push_back(*it);
				}
				for (auto it = part_2.begin(); it != part_2.end(); ++it) {
					path_single.push_back(*it);
				}
			}
			else {
				std::cout << "Graph B found path connection! \n";
				for (auto it = part_2.rbegin(); it != part_2.rend(); ++it) {
					path_single.push_back(*it);
				}
				for (auto it = part_1.begin(); it != part_1.end(); ++it) {
					path_single.push_back(*it);
				}
			}

			cost = findDistance(new_node->coord, node_dest->coord);

			graph->addEdge(new_node, node_dest, cost);
			std::cout << "Total cost: " << total_cost << std::endl;
			break;
		}

		Coord step = extend(node_src->coord, &vector_dir, t);
		step_node = new Node(new_node->node_number + 1, step);
		graph->addEdge(new_node, step_node, findDistance(new_node->coord, step_node->coord));
		new_node = step_node;
		t++;
	}

	std::cout << "There is a CONNECTION!\n" << std::endl;
}

bool BiRRTStar::inGoalRadiusSingle(Coord* node_coord) {
	if (findDistance(node_coord, end_node->coord) < GOALRADIUS) {
		return true;
	}
	else
		return false;
}

std::vector<Node*> BiRRTStar::makePath(Node* last_node) {
	std::vector<Node*> path;
	Node* node = last_node;
	Node* check = last_node->parent;
	while (node->parent) {
		if (check->parent) {
			if (!obs->collisionCheck(node->coord, check->parent->coord)) {
				check = check->parent;
			}
			else {
				path.push_back(node);
				node = check;
				check = node->parent;
			}
		}
		else {
			path.push_back(node);
			node = check;
		}
	}
	path.push_back(node);

	return path;

	//std::vector<Node*> path;
	//Node* node = last_node;
	//while (node->parent) {
	//	path.push_back(node);
	//	node = node->parent;
	//}
	//path.push_back(node);

	//return path;
}

	/**********************************/
	/**********************************/

	// Getter Functions

std::vector<Node*> BiRRTStar::getPath() {
	return path_single;
}

void BiRRTStar::printPathSingle() {
	for (auto& node : path_single) {
		std::cout << node->node_number << " ";
	}
	std::cout << std::endl;
	//std::cout << "Total Cost: " << end_node->weight << std::endl;
}

//void BiRRTStar::printPathMany() {
//	for (int i = 0; i < NUMDRONES; i++) {
//		std::vector<Node*> path = path_many.at(i);
//		std::cout << "Path " << i << ": ";
//		if (path.empty()) {
//			std::cout << "No path found :(" << std::endl;
//			continue;
//		}
//		for (auto& node : path) {
//			if (!node) { continue; }
//			std::cout << node->node_number << " ";
//		}
//		std::cout << std::endl;
//		std::cout << "Total Cost: " << end_nodes.at(i)->weight << std::endl;
//	}
//}
