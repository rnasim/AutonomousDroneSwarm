#ifndef OBSTACLES_HPP
#define OBSTACLES_HPP

#include <stdlib.h>
#include <vector>
#include <stack>
#include <list>
#include <time.h>
#include <cmath>
#include <iostream>
#include <assert.h>
#include<fstream>
#include <string>

#include "Data.hpp"

//ai + bj + ck = d
struct Plane {
	float a;
	float b;
	float c;
	float d;

	Plane* side1;
	Plane* side2;
	Plane* side3;
	Plane* side4;


	Plane(float a1, float b1, float c1, float d1) {
		a = a1;
		b = b1;
		c = c1;
		d = d1;
		
	}
};


struct Obstacle {
	//f=front, b=back, l=left, r=right, u=up, d=down
	//vertices
	Coord* ful;
	Coord* fur;
	Coord* fdl;
	Coord* fdr;
	Coord* bul;
	Coord* bur;
	Coord* bdl;
	Coord* bdr;

	//Planes for each face
	Plane* pUp;
	Plane* pDown;
	Plane* pLeft;
	Plane* pRight;
	Plane* pFront;
	Plane* pBack;

    Obstacle(Coord* a, Coord* b, Coord* c, Coord* d, Coord* e, Coord* f, Coord* g, Coord* h)
    {
		ful = a;
		fur = b;
		fdl = c;
		fdr = d;
		bul = e;
		bur = f;
		bdl = g;
		bdr = h;
    }
};

class Obstacles {
	float xMapMin;
	float yMapMin;
	float zMapMin;

	float xMapMax;
	float yMapMax;
	float zMapMax;

	int numObstacles;
	std::vector<Obstacle*> obstacleList;
public:
	Obstacles(float xMin,float xMax,float yMin, float yMax, float zMin, float zMax);
	void initObstacles();
	void addObstacle(Coord* a, Coord* b, Coord* c, Coord* d, Coord* e, Coord* f, Coord* g, Coord* h);
	bool checkInMap(Coord* coord);
	bool collisionCheck(Coord* A, Coord* B);
	Coord* crossProduct(Coord* A, Coord* B);
	bool checkObstacleInt(Coord* A, Coord* B, Obstacle* ob);
	bool pointInPlane(Coord* A, Coord* B, Plane* plane);
	bool checkBox(Coord* node_to_check, Obstacle* ob);

};


#endif