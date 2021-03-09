#include "Obstacles.hpp"
#include "Graph.hpp"

Obstacles::Obstacles(float xMin, float xMax, float yMin, float yMax, float zMin, float zMax) {
	xMapMin = xMin;
	yMapMin = yMin;
	zMapMin = zMin;

	xMapMax = xMax;
	yMapMax = yMax;
	zMapMax = zMax;

	numObstacles = 0;
}

void Obstacles::initObstacles() {
	Coord* a = new Coord();
	Coord* b = new Coord();
	Coord* c = new Coord();
	Coord* d = new Coord();
	Coord* e = new Coord();
	Coord* f = new Coord();
	Coord* g = new Coord();
	Coord* h = new Coord();

	std::ifstream myReadFile;
	myReadFile.open("obstacles.txt");
	std::string output;
	int lineCounter = 0;
	float x, y, z = 0;
	if (myReadFile.is_open()) {

		while (!myReadFile.eof()) {
			getline(myReadFile, output);

			//skip first line
			if (lineCounter == 0) {
				lineCounter++;
				continue;
			}

			int counter = 0;
			int comma1;
			for (int i = 0; i < output.size(); i++) {
				if (output[i] == ',') {
					if (counter == 0) {
						x = stof(output.substr(0, i));
						comma1 = i;
						counter++;
					}
					else if (counter == 1) {
						std::string letter = output.substr(comma1 + 1, i);
						y = stof(letter);
						z = stof(output.substr(i + 1, output.size()));
						break;
					}
				}
			}

			int lineNum = (lineCounter - 1) % 9;
			//skip/print obstacle titles
			if (lineNum == 0) {
				lineCounter++;
				std::cout << "adding the obstacle:" << output << "\n";

				continue;
			}
			else if (lineNum == 1) {
				a = new Coord(x, y, z);
				a->printCoord();

			}
			else if (lineNum == 2) {
				b = new Coord(x, y, z);
				b->printCoord();
			}
			else if (lineNum == 3) {
				c = new Coord(x, y, z);
				c->printCoord();
			}
			else if (lineNum == 4) {
				d = new Coord(x, y, z);
				d->printCoord();
			}
			else if (lineNum == 5) {
				e = new Coord(x, y, z);
				e->printCoord();
			}
			else if (lineNum == 6) {
				f = new Coord(x, y, z);
				f->printCoord();
			}
			else if (lineNum == 7) {
				g = new Coord(x, y, z);
				g->printCoord();
			}
			else if (lineNum == 8) {
				h = new Coord(x, y, z);
				h->printCoord();
				try {
					addObstacle(a, b, c, d, e, f, g, h);
				}
				catch (const std::exception e) {
				}
			}


			lineCounter++;

		}
	}
	myReadFile.close();
}


//Check the plane equations and make sure we're using the right vectors for each face. I haven't checked those yet
void Obstacles::addObstacle(Coord* a, Coord* b, Coord* c, Coord* d, Coord* e, Coord* f, Coord* g, Coord* h) {
	Obstacle* obstacle = new Obstacle(a, b, c, d, e, f, g, h);

	numObstacles++;


	//calculate plane equations
	Coord* ful = a;
	Coord* fur = b;
	Coord* fdl = c;
	Coord* fdr = d;
	Coord* bul = e;
	Coord* bur = f;
	Coord* bdl = g;
	Coord* bdr = h;


	Coord* directVect1;
	Coord* directVect2;

	float xo, yo, zo, a1, b1, c1, d1;
	Coord* face;

	//std::cout << "vvv PLANE EQUATIONS vvv\n";
	//std::cout << "-----------------------\n";
	//----------------------------------------up face plane equation------------------------------------------// (checked gives correct plane equation)
	xo = ful->x;
	yo = ful->y;
	zo = ful->z;

	//line1 eqn (ful -> fur)
	directVect1 = new Coord(fur->x - ful->x, fur->y - ful->y, fur->z - ful->z);

	//line2 eqn (ful -> bul)
	directVect2 = new Coord(bul->x - ful->x, bul->y - ful->y, bul->z - ful->z);

	face = crossProduct(directVect1, directVect2);	//creates normal vector of two vectors
	a1 = face->x;
	b1 = face->y;
	c1 = face->z;
	d1 = a1 * xo + b1 * yo + c1 * zo;		//Double check this line
	Plane* plane = new Plane(a1, b1, c1, d1);
	obstacle->pUp = plane;

	//std::cout << a1 << "x + " << b1 << "y + " << c1 << "z = " << d1 << "\n";
	//----------------------------------------down face plane equation------------------------------------------// (This is pointing in the wrong direction)
	xo = fdl->x;
	yo = fdl->y;
	zo = fdl->z;

	//line1 eqn (fdl -> fdr)
	directVect2 = new Coord(fdr->x - fdl->x, fdr->y - fdl->y, fdr->z - fdl->z);

	//line2 eqn (fdl -> bdl)
	directVect1 = new Coord(bdl->x - fdl->x, bdl->y - fdl->y, bdl->z - fdl->z);

	face = crossProduct(directVect1, directVect2);	//creates normal vector of two vectors
	a1 = face->x;
	b1 = face->y;
	c1 = face->z;
	d1 = a1 * xo + b1 * yo + c1 * zo;		//Double check this line
	plane = new Plane(a1, b1, c1, d1);
	obstacle->pDown = plane;
	//std::cout << a1 << "x + " << b1 << "y + " << c1 << "z = " << d1 << "\n";


	//----------------------------------------left face plane equation------------------------------------------// (checked gives correct plane equation)
	xo = fdl->x;
	yo = fdl->y;
	zo = fdl->z;

	//line1 eqn (fdl -> bdl)
	directVect2 = new Coord(bdl->x - fdl->x, bdl->y - fdl->y, bdl->z - fdl->z);

	//line5 eqn (fdl -> ful)
	directVect1 = new Coord(ful->x - fdl->x, ful->y - fdl->y, ful->z - fdl->z);

	face = crossProduct(directVect1, directVect2);	//creates normal vector of two vectors
	a1 = face->x;
	b1 = face->y;
	c1 = face->z;
	d1 = a1 * xo + b1 * yo + c1 * zo;		//Double check this line
	plane = new Plane(a1, b1, c1, d1);
	obstacle->pLeft = plane;
	//std::cout << a1 << "x + " << b1 << "y + " << c1 << "z = " << d1 << "\n";


	//----------------------------------------right face plane equation------------------------------------------// (checked gives correct plane equation)
	xo = fdr->x;
	yo = fdr->y;
	zo = fdr->z;

	//line1 eqn (fdr -> bdr)
	directVect1 = new Coord(bdr->x - fdr->x, bdr->y - fdr->y, bdr->z - fdr->z);

	//line2 eqn (fdr -> fur)
	directVect2 = new Coord(fur->x - fdr->x, fur->y - fdr->y, fur->z - fdr->z);

	face = crossProduct(directVect1, directVect2);	//creates normal vector of two vectors
	a1 = face->x;
	b1 = face->y;
	c1 = face->z;
	d1 = a1 * xo + b1 * yo + c1 * zo;		//Double check this line
	plane = new Plane(a1, b1, c1, d1);
	obstacle->pRight = plane;
	//std::cout << a1 << "x + " << b1 << "y + " << c1 << "z = " << d1 << "\n";

	//----------------------------------------front face plane equation------------------------------------------// (checked gives correct plane equation)
	xo = fdl->x;
	yo = fdl->y;
	zo = fdl->z;

	//line1 eqn (fdl -> fdr)
	directVect1 = new Coord(fdr->x - fdl->x, fdr->y - fdl->y, fdr->z - fdl->z);

	//line2 eqn (fdl -> ful)
	directVect2 = new Coord(ful->x - fdl->x, ful->y - fdl->y, ful->z - fdl->z);

	face = crossProduct(directVect1, directVect2);	//creates normal vector of two vectors
	a1 = face->x;
	b1 = face->y;
	c1 = face->z;
	d1 = a1 * xo + b1 * yo + c1 * zo;		//Double check this line
	plane = new Plane(a1, b1, c1, d1);
	obstacle->pFront = plane;

	//std::cout << a1 << "x + " << b1 << "y + " << c1 << "z = " << d1 << "\n";

	//----------------------------------------back face plane equation------------------------------------------// (checked gives correct plane equation)
	xo = bdl->x;
	yo = bdl->y;
	zo = bdl->z;

	//line1 eqn (bdl -> bdr)
	directVect2 = new Coord(bdr->x - bdl->x, bdr->y - bdl->y, bdr->z - bdl->z);

	//line2 eqn (bdl -> bul)
	directVect1 = new Coord(bul->x - bdl->x, bul->y - bdl->y, bul->z - bdl->z);

	face = crossProduct(directVect1, directVect2);	//creates normal vector of two vectors
	a1 = face->x;
	b1 = face->y;
	c1 = face->z;
	d1 = a1 * xo + b1 * yo + c1 * zo;		//Double check this line
	plane = new Plane(a1, b1, c1, d1);
	obstacle->pBack = plane;
	//std::cout << a1 << "x + " << b1 << "y + " << c1 << "z = " << d1 << "\n";

	//std::cout << "-----------------------";
	//std::cout << "\n";
	//std::cout << "\n";
	//pUp
	obstacle->pUp->side1 = obstacle->pLeft;
	obstacle->pUp->side2 = obstacle->pRight;
	obstacle->pUp->side3 = obstacle->pBack;
	obstacle->pUp->side4 = obstacle->pFront;

	//pLeft
	obstacle->pLeft->side1 = obstacle->pUp;
	obstacle->pLeft->side2 = obstacle->pDown;
	obstacle->pLeft->side3 = obstacle->pBack;
	obstacle->pLeft->side4 = obstacle->pFront;

	//pRight
	obstacle->pRight->side1 = obstacle->pUp;
	obstacle->pRight->side2 = obstacle->pDown;
	obstacle->pRight->side3 = obstacle->pBack;
	obstacle->pRight->side4 = obstacle->pFront;

	//pDown
	obstacle->pDown->side1 = obstacle->pLeft;
	obstacle->pDown->side2 = obstacle->pRight;
	obstacle->pDown->side3 = obstacle->pBack;
	obstacle->pDown->side4 = obstacle->pFront;

	//pFront
	obstacle->pFront->side1 = obstacle->pLeft;
	obstacle->pFront->side2 = obstacle->pRight;
	obstacle->pFront->side3 = obstacle->pUp;
	obstacle->pFront->side4 = obstacle->pDown;

	//pBack
	obstacle->pBack->side1 = obstacle->pLeft;
	obstacle->pBack->side2 = obstacle->pRight;
	obstacle->pBack->side3 = obstacle->pUp;
	obstacle->pBack->side4 = obstacle->pDown;


	obstacleList.push_back(obstacle);
}

//equivalent to A x B
Coord* Obstacles::crossProduct(Coord* A, Coord* B) {
	Coord* Out = new Coord();
	Out->x = (A->y * B->z) - (A->z * B->y);
	Out->y = -(A->x * B->z) + (A->z * B->x);
	Out->z = (A->x * B->y) - (A->y * B->x);

	return Out;
}

bool Obstacles::collisionCheck(Coord* A, Coord* B) {
	Coord* intersect;

	//check if points are in map (checkInMap returns true if in map)
	bool status;
	status = checkInMap(A);
	if (status == false) {
		//std::cout << "below is A\n";
		//A->printCoord();
		//std::cout << "not in map\n";
		return true;
	}

	status = checkInMap(B);
	if (status == false) {
		//std::cout << "below is B\n";
		//B->printCoord();
		//std::cout << "not in map\n";
		return true;
	}



	//loop through obstacles
	for (int i = 0; i < numObstacles; i++) {

		//check if the obstacle is between A and B inclusive
		Obstacle* ob = obstacleList.at(i);
		status = checkObstacleInt(A, B, ob);
		if (checkBox(B, ob) || checkBox(A, ob)) {
			//std::cout << "Node is in box " << std::endl;
			return true;
		}
		if (status == true) {
			//std::cout << "HERE " << std::endl;
			return true;
		}
	}

	return false;
}

bool Obstacles::checkBox(Coord* node_to_check, Obstacle* ob) {
	//Check if coord is inbetween these x, y, z values
	// fdl <= x,y,z <= bur
	// if true node is inside box
	// else node is not inside box
	if (ob->fdl->x <= node_to_check->x && node_to_check->x <= ob->bur->x &&
		ob->fdl->y <= node_to_check->y && node_to_check->y <= ob->bur->y &&
		ob->fdl->z <= node_to_check->z && node_to_check->z <= ob->bur->z) {
		//std::cout << "Node is in box " << std::endl;
		//node_to_check->printCoord();
		return true;
	}
	else { return false; }
}


//creates a plane equation and finds point of intersection (if A and B are on different sides of the plane, the if should be negative)
bool Obstacles::checkObstacleInt(Coord* A, Coord* B, Obstacle* ob) {
	bool result;
	////std::cout<< "UPFACE " << ((ob->pUp->a * A->x) + (ob->pUp->b * A->y) + (ob->pUp->c * A->z) - ob->pUp->d) * ((ob->pUp->a * B->x) + (ob->pUp->b * B->y) + (ob->pUp->c * B->z) - ob->pUp->d) << "\n";
	if (((ob->pUp->a * A->x) + (ob->pUp->b * A->y) + (ob->pUp->c * A->z) - ob->pUp->d) * ((ob->pUp->a * B->x) + (ob->pUp->b * B->y) + (ob->pUp->c * B->z) - ob->pUp->d) <= 0.0f) {
		//std::cout << "Possible Collision: UP face\n";
		result = pointInPlane(A, B, ob->pUp);
		if (result == true) {
			//std::cout << "COLLISION IN UP face\n";

			return true;
		}
	}

	////std::cout << "downFACE " << ((ob->pDown->a * A->x) + (ob->pDown->b * A->y) + (ob->pDown->c * A->z) - ob->pDown->d) * ((ob->pDown->a * B->x) + (ob->pDown->b * B->y) + (ob->pDown->c * B->z) - ob->pDown->d) << "\n";
	if (((ob->pDown->a * A->x) + (ob->pDown->b * A->y) + (ob->pDown->c * A->z) - ob->pDown->d) * ((ob->pDown->a * B->x) + (ob->pDown->b * B->y) + (ob->pDown->c * B->z) - ob->pDown->d) <= 0.0f) {
		//std::cout << "Possible Collision: DOWN face\n";
		result = pointInPlane(A, B, ob->pDown);
		if (result == true) {
			//std::cout << "COLLISION IN DOWN face\n";

			return true;
		}
	}

	////std::cout << "leftFACE " << ((ob->pLeft->a * A->x) + (ob->pLeft->b * A->y) + (ob->pLeft->c * A->z) - ob->pLeft->d) * ((ob->pLeft->a * B->x) + (ob->pLeft->b * B->y) + (ob->pLeft->c * B->z) - ob->pLeft->d) << "\n";
	if (((ob->pLeft->a * A->x) + (ob->pLeft->b * A->y) + (ob->pLeft->c * A->z) - ob->pLeft->d) * ((ob->pLeft->a * B->x) + (ob->pLeft->b * B->y) + (ob->pLeft->c * B->z) - ob->pLeft->d) <= 0.0f) {
		//std::cout << "Possible Collision: LEFT face\n";
		result = pointInPlane(A, B, ob->pLeft);
		if (result == true) {
			//std::cout << "COLLISION IN LEFT face\n";

			return true;
		}
	}

	////std::cout << "rightFACE " << ((ob->pRight->a * A->x) + (ob->pRight->b * A->y) + (ob->pRight->c * A->z) - ob->pRight->d) * ((ob->pRight->a * B->x) + (ob->pRight->b * B->y) + (ob->pRight->c * B->z) - ob->pRight->d) << "\n";
	if (((ob->pRight->a * A->x) + (ob->pRight->b * A->y) + (ob->pRight->c * A->z) - ob->pRight->d) * ((ob->pRight->a * B->x) + (ob->pRight->b * B->y) + (ob->pRight->c * B->z) - ob->pRight->d) <= 0.0f) {
		//std::cout << "Possible Collision: RIGHT face\n";
		result = pointInPlane(A, B, ob->pRight);
		if (result == true) {
			//std::cout << "COLLISION IN RIGHT face\n";

			return true;
		}
	}


	////std::cout << "frontFACE " << ((ob->pFront->a * A->x) + (ob->pFront->b * A->y) + (ob->pFront->c * A->z) - ob->pFront->d) * ((ob->pFront->a * B->x) + (ob->pFront->b * B->y) + (ob->pFront->c * B->z) - ob->pFront->d)  << "\n";
	if (((ob->pFront->a * A->x) + (ob->pFront->b * A->y) + (ob->pFront->c * A->z) - ob->pFront->d) * ((ob->pFront->a * B->x) + (ob->pFront->b * B->y) + (ob->pFront->c * B->z) - ob->pFront->d) <= 0.0f) {
		//std::cout << "Possible Collision: FRONT face\n";
		result = pointInPlane(A, B, ob->pFront);
		if (result == true) {
			//std::cout << "COLLISION IN FRONT face\n";

			return true;
		}
	}

	////std::cout << "backFACE " << ((ob->pBack->a * A->x) + (ob->pBack->b * A->y) + (ob->pBack->c * A->z) - ob->pBack->d) * ((ob->pBack->a * B->x) + (ob->pBack->b * B->y) + (ob->pBack->c * B->z) - ob->pBack->d) << "\n";
	if (((ob->pBack->a * A->x) + (ob->pBack->b * A->y) + (ob->pBack->c * A->z) - ob->pBack->d) * ((ob->pBack->a * B->x) + (ob->pBack->b * B->y) + (ob->pBack->c * B->z) - ob->pBack->d) <= 0.0f) {
		//std::cout << "Possible Collision: BACK face\n";
		result = pointInPlane(A, B, ob->pBack);
		if (result == true) {
			//std::cout << "COLLISION IN BACK face\n";

			return true;
		}
	}
	//CheckTopFace
		//find intersect of pUp
		//get the 4 vectors formed by top corners 
		//take the determinant of all four vectors  https://stackoverflow.com/questions/1560492/how-to-tell-whether-a-point-is-to-the-right-or-left-side-of-a-line 
			//0 means it is on the line
			//1 means it is on one side
			//-1 means it is on the other side
			//if the point is on the cube side of all of the vectors then it is inside the object

	//repeat for all faces

	return NULL;
}

//returns true if within map
bool Obstacles::checkInMap(Coord* c) {
	if (c->x >= MAPMAXX - MINOBSTDIST || c->x < MAPMINX + MINOBSTDIST ||
		c->y >= MAPMAXY - MINOBSTDIST || c->y < MAPMINY + MINOBSTDIST ||
		c->z >= MAPMAXZ - MINOBSTDIST || c->z < MAPMINZ + MINOBSTDIST)
	{
		return false;
	}
	return true;
}

bool Obstacles::pointInPlane(Coord* A, Coord* B, Plane* plane) {

	//std::cout << "**running pointInPlane***\n";
	Coord* AB;
	Coord* AP;
	float x, y, z;
	float a, b, c;
	float t;
	Coord* p = new Coord();							//point of intersection
	float x1, x2, y1, y2, z1, z2;

	AB = new Coord(B->x - A->x, B->y - A->y, B->z - A->z);

	x1 = A->x;
	x2 = AB->x;
	y1 = A->y;
	y2 = AB->y;
	z1 = A->z;
	z2 = AB->z;

	t = ((plane->d) - plane->a * x1 - plane->b * y1 - plane->c * z1) / (plane->a * x2 + plane->b * y2 + plane->c * z2);	// plane equation: a(x - xo) + b(y - yo) + c(z - zo) = 0;

	p->x = x1 + (x2 * t);
	p->y = y1 + (y2 * t);
	p->z = z1 + (z2 * t);

	////std::cout << "plane intersect at:" << p->x << "," << p->y << "," << p->z << "\n";

	//check if our point of intersection p is on the inside of all surrounding planes
	/*if ((plane->side1->a * p->x) + (plane->side1->b * p->y) + (plane->side1->c * p->z) - plane->side1->d < 0) {
		//std::cout << "SIDE: "<< plane->side1 << "\n";
		//std::cout << "here1\n";
		return true;
	}
	if ((plane->side2->a * p->x) + (plane->side2->b * p->y) + (plane->side2->c * p->z) - plane->side2->d < 0) {
	//std::cout << "here2\n";

		return true;
	}
	if ((plane->side3->a * p->x) + (plane->side3->b * p->y) + (plane->side3->c * p->z) - plane->side3->d < 0) {
	//std::cout << "here3\n";
		return true;

	}
	if ((plane->side4->a * p->x) + (plane->side4->b * p->y) + (plane->side4->c * p->z) - plane->side4->d < 0) {
	//std::cout << "here4\n";

		return true;
	}*/
	//std::cout << "Point of intersection:(" << p->x << "," << p->y << "," << p->z << ")\n";
	////std::cout << "side1 plane equation:" << plane->side1->a << "x + " << plane->side1->b << "y + " << plane->side1->c << "z =" << plane->side1->d << "\n";
	//std::cout << "CHECK SIDE1: " << (plane->side1->a * p->x) + (plane->side1->b * p->y) + (plane->side1->c * p->z) - plane->side1->d << "\n"; //<= 0.f  
	//std::cout << "CHECK SIDE3: " << (plane->side3->a * p->x) + (plane->side3->b * p->y) + (plane->side3->c * p->z) - plane->side3->d << "\n"; //<= 0.f 
	//std::cout << "CHECK SIDE2: " << (plane->side2->a * p->x) + (plane->side2->b * p->y) + (plane->side2->c * p->z) - plane->side2->d << "\n"; //<= 0.f 
	//std::cout << "CHECK SIDE4: " << (plane->side4->a * p->x) + (plane->side4->b * p->y) + (plane->side4->c * p->z) - plane->side4->d << "\n"; //<= 0.f)
	//check if our point of intersection p is on the right of each edge of the face
	if ((plane->side1->a * p->x) + (plane->side1->b * p->y) + (plane->side1->c * p->z) - plane->side1->d <= 0.f && (plane->side3->a * p->x) + (plane->side3->b * p->y) + (plane->side3->c * p->z) - plane->side3->d <= 0.f && (plane->side2->a * p->x) + (plane->side2->b * p->y) + (plane->side2->c * p->z) - plane->side2->d <= 0.f && (plane->side4->a * p->x) + (plane->side4->b * p->y) + (plane->side4->c * p->z) - plane->side4->d <= 0.f) {
		//std::cout << "***COLLISION DETECTED***\n";
		return true;
	}
	/*if ( ((plane->side1->a * p->x) + (plane->side1->b * p->y) + (plane->side1->c * p->z) - plane->side1->d ) * ((plane->side3->a * p->x) + (plane->side3->b * p->y) + (plane->side3->c * p->z) - plane->side3->d) > 0) {
		//std::cout << "here1\n";
		return true;
	}
	if ((plane->side2->a * p->x) + (plane->side2->b * p->y) + (plane->side2->c * p->z) - plane->side2->d * ((plane->side4->a * p->x) + (plane->side4->b * p->y) + (plane->side4->c * p->z) - plane->side4->d) > 0) {
		//std::cout << "here2\n";

		return true;
	}*/
	/*if ( (plane->side3->a * p->x) + (plane->side3->b * p->y) + (plane->side3->c * p->z) - plane->side3->d < 0) {
		//std::cout << "here3\n";
		return true;

	}*/
	/*if ((plane->side4->a * p->x) + (plane->side4->b * p->y) + (plane->side4->c * p->z) - plane->side4->d < 0) {
		//std::cout << "here4\n";

		return true;
	}*/
	//std::cout << "FALSE ALARM: there is NO intersection in this face\n";
	//std::cout << "\n";
	return false;
}