#ifndef HASH_HPP
#define HASH_HPP

#include <stdlib.h>
#include <vector>
#include <stack>
#include <list>
#include <time.h>
#include <cmath>
#include <iostream>
#include <assert.h>
#include <vector>
#include "Data.hpp"


struct hashLocation {
	int bucketIndex;
	int tableIndex;

	hashLocation(int b, int t) {
		bucketIndex = b;
		tableIndex = t;
	}
};


class hashTable {
public:
	hashTable();
	void insert(Node* node);
	bool inBucket(Node* node, int min, int max);
	hashLocation* search(Node* node);
	void remove(Node* node);
	void printTable();
	void printBucket(int i);
	int search(Coord* coord);

	//Getter Functions
	float getBucketRange();
	int getNumBuckets();
	int getNumValues();
	std::vector<Node*>* getTable();


private:
	int numBuckets;
	float bucketRange;
	int numValues;
	std::vector<Node*>* table;

	



};




#endif