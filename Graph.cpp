#include "Graph.hpp"
using namespace std;

// Constructor

// Two Graphs
Graph::Graph(Node* start_node)
{
	// true if Init (even) // false if Goal (odd)
	if (start_node->node_number % 2 == 0) {
		init = true;
	} else { 
		init = false;
	}

	firstNode = start_node;

	nodeHash = new hashTable();
	nodeHash->insert(start_node);

	adj_list.push_back(start_node);
};

Graph::~Graph() {
	for (auto& iter : adj_list) {
		delete iter;
	}
}

// Setter Functions

void Graph::addEdge(Node* node_src, Node* node_dest, float weight)
{
	float total_weight = weight + node_src->weight;
	node_dest->segment = findDistance(node_src->coord, node_dest->coord);
	node_dest->weight = total_weight;
	node_dest->parent = node_src;
	nodeHash->insert(node_dest);
	
	//delete this after hash is complete
	adj_list.push_back(node_dest);
}

void Graph::rewireEdge(Node* node_src, Node* node_dest) {
	float total_weight = findDistance(node_src->coord, node_dest->coord) + node_src->weight;
	node_dest->weight = total_weight;
	node_dest->segment = findDistance(node_src->coord, node_dest->coord);
	node_dest->parent = node_src;
}


// Getter Functions

////Double check this idk if it works
//Coord* Graph::getCellCoords(Node* node) {
//	
//	float x = (int)node->coord->x / CELLSIZE;
//	float y = (int)node->coord->y / CELLSIZE;
//	float z = (int)node->coord->z / CELLSIZE;
//
//	Coord* cellCoord = new Coord(x, y, z);
//
//	return cellCoord;
//}


hashTable* Graph::getHashTable() {
	return nodeHash;
}

int Graph::getNumNodes() {
	return num_nodes;
}

bool Graph::isInit() {
	return init;
}

//std::vector<Node*> Graph::getAdjList() {
//	return adj_list;
//}

Node* Graph::getFirstNode() {
	return firstNode;
}

// Debug Functions

void Graph::printGraph()
{
	cout << "Size of Tree: " << adj_list.size() << endl;
	cout << "Adjacency List" << endl;
	for (auto& node : adj_list) {
		node->printNode();
	}
	//for (auto& node : adj_list)
	//{
	//	if (!node) { continue; }
	//	cout << "(" << node->node_number << ", " << nodeChild->node_number << ", " << nodeChild->weight << ") " << endl;

	//	//for (auto& nodeChild : node->connectedNodes)
	//	//{
	//	//	if (!nodeChild) { continue; }
	//	//	cout << "(" << node->node_number << ", " << nodeChild->node_number << ", " << nodeChild->weight << ") " << endl;
	//	//}
	//}
}

//void Graph::printCellPop() {
//	for (int i = 0; i < NUMCELLSX; i++) {
//		for (int j = 0; j < NUMCELLSY; j++) {
//			for (int k = 0; k < NUMCELLSZ; k++) {
//				cout << "cell[" << i << "][" << j << "][" << k << "] population = " << cells[i][j][k].size() << "\n";
//
//			}
//		}
//	}
//}

// End Debug Functions
