#ifndef DATA_HPP
#define DATA_HPP

#include <cmath>
#include <iostream>

#include "Constants.hpp"

struct Coord {
    float x, y, z;

    Coord()
    {
        x = float((rand() % MAPSIZEX) - abs(MAPMINX));
        y = float((rand() % MAPSIZEY) - abs(MAPMINY));
        z = float((rand() % MAPSIZEZ));
    }

    Coord(float x_new, float y_new, float z_new)
    {
        x = x_new;
        y = y_new;
        z = z_new;
    }

    void printCoord() {
        std::cout << "Coord (X,Y,Z) = (" << x << "," << y << "," << z << ")\n" << std::endl;
    }
};


struct Node {
    int node_number;
    float weight;
    float segment;
    bool visited;
    bool in_use;

    Coord* coord;
    //Coord* cell_coord;
    Node* parent;

    Node()
    {
        node_number = 0;
        coord = new Coord();
        weight = 0; // Temp value
        segment = 0;
        visited = false;
        in_use = false;
        parent = NULL;

        //cell_coord = NULL;
    }

    Node(Coord new_coord)
    {
        node_number = 0;
        weight = 0;
        segment = 0;
        coord = new Coord(new_coord.x, new_coord.y, new_coord.z);
        visited = false;
        in_use = false;
        parent = NULL;

        //cell_coord = NULL;
    }

    Node(int node_num, Coord new_coord)
    {
        node_number = node_num;
        segment = 0;
        weight = 0;
        coord = new Coord(new_coord.x, new_coord.y, new_coord.z);
        visited = false;
        in_use = false;
        parent = NULL;

        //cell_coord = NULL;
    }

    ~Node()
    {
        delete coord;
        //delete cell_coord;
    }

    void printNode() {
        std::cout << "Node: " << node_number << std::endl;
        std::cout << "Coord(X,Y,Z) = (" << coord->x << "," << coord->y << "," << coord->z << ")\n\n";
        //if(cell_coord != NULL)
            //cout << "Cell (X,Y,Z) = (" << cell_coord->x << "," << cell_coord->y << "," << cell_coord->z << ")\n" << endl;
    }
};

static float findDistance(Coord* coord_src, Coord* coord_dest);

// Finds distance between two nodes
static float findDistance(Coord* coord_src, Coord* coord_dest)
{

    float dist_x = pow((coord_dest->x - coord_src->x), 2);
    float dist_y = pow((coord_dest->y - coord_src->y), 2);
    float dist_z = pow((coord_dest->z - coord_src->z), 2);

    float total_distance = sqrt((dist_x + dist_y + dist_z));
    /*std::cout << "dx: " << dist_x << " dy: " << dist_y << " dz: " << dist_z << std::endl;
    std::cout << "Total Distance: " << total_distance << std::endl;*/
    return total_distance;
}

#endif

