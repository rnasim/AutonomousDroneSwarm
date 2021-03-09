#ifndef RRTSTAR_HPP
#define RRTSTAR_HPP

#include "Graph.hpp"

class BiRRTStar {
	Obstacles* obs;

    // Single Query // RRT is a single query path planner
    Node* start_node;
    Node* end_node;

    std::vector<Node*> path_single;

    /**********************************/
    /**********************************/
public:
    BiRRTStar(Coord start_coord, Coord end_coord);

    void CallRRTStar();

    /* Utility Functions */

    // add Obstacles
    void addObstacles(Obstacles* o);

    // Returns Coord stepsize away from nearest_node coord
    Coord stepNode(Coord* coord, Coord* random_coord);

    // Extends to targeted coordinate
    Coord extend(Coord* coord, Coord* random_coord, int t);

    // Finds node closest to the coord
    Node* nearestNode(Coord* random_coord, Graph* graph);

    // Returns vector of nodes closest to node in radius r
    std::vector<Node*> nearestNeighbors(Node* new_node, Graph* graph);
    Node* nearestNode(Coord* random_coord, hashTable* hash, int& lowerIndexHighBound, int& upperIndexLowBound, Node* nearest_node, std::vector<Node*>* table, int& lowerIndexLowBound, int& upperIndexHighBound, float offset);

    // Choose best parent with least cost from neighbors
    Node* chooseBestParent(Node* new_node, std::vector<Node*> neighbors);

    // Connect
    void connect(Node* node_src, Node* node_dest, Graph* graph);

    // Returns true if coord is in goal radius
    bool inGoalRadiusSingle(Coord* node_coord);

    // Generates final path if path is found
    std::vector<Node*> makePath(Node* node);

    /**********************************/
    /**********************************/

    // Getter Functions

    // Returns path found if a path is found
    std::vector<Node*> getPath();

    void printPathSingle();

    /**********************************/
    /**********************************/
};


#endif