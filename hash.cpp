#include "hash.hpp"

hashTable::hashTable() {
	numBuckets = NUMNODES;
	bucketRange = ((float)MAPSIZEX / (float)numBuckets);
	table = new std::vector<Node*>[numBuckets];
	

	//std::cout << "bucketRange is: " << bucketRange << ", numBuckets is: " << numBuckets << ", MAPSIZE X is: " << MAPSIZEX <<"\n";

}

void hashTable::insert(Node* node) {
	float x = node->coord->x;
	x += abs(MAPMINX);
	int bucketIndex = (int) x/bucketRange;
	//std::cout <<"bucketIndex is: " << bucketIndex << "\n";

	table[bucketIndex].push_back(node);
}

bool hashTable::inBucket(Node* node, int min, int max) {
	float x = node->coord->x;
	x += abs(MAPMINX);
	int bucketIndex = (int)x / bucketRange;
	if (min <= bucketIndex && bucketIndex <= max) {
		return true;
	}
	return false;
}
hashLocation* hashTable::search(Node* node) {
	float x = node->coord->x;
	hashLocation* h;
	int tableIndex = (x+abs(MAPMINX)) / bucketRange;

	for (int i = 0; i < table[tableIndex].size(); i++) {
		if (table[tableIndex].at(i) == node) {
			h = new hashLocation(i, tableIndex);
			return h;
		}
	}
	return NULL;
}


int hashTable::search(Coord* coord) {
	float x = coord->x;
	int tableIndex = (x + abs(MAPMINX)) / bucketRange;	
	
	return tableIndex;
}

void hashTable::remove(Node* node) {
	float x = node->coord->x;
	int bucketIndex = x / bucketRange;
	for (int i = 0; i < table[bucketIndex].size(); i++) {
		if (node == table[bucketIndex].at(i)) {
			table[bucketIndex].erase(i+table[bucketIndex].begin());
		}
	}
}

void hashTable::printBucket(int i) {
	std::cout << "bucket " << i << " contains:\n";
	for (int j = 0; j <table[i].size() ; j++) {
		std::cout << &table[i] << " x Coord is: " << table[i].at(j)->coord->x << ", ";
	}
	std::cout << "\n";
}

void hashTable::printTable() {
	for (int i = 0; i < numBuckets; i++) {
		if (table[i].size() != 0) {
			printBucket(i);
		}
	}
}




//Getter Functions
float hashTable::getBucketRange() {
	return bucketRange;
}
int hashTable::getNumBuckets() {
	return numBuckets;
}
int hashTable::getNumValues() {
	return numValues;
}
std::vector<Node*>* hashTable::getTable() {
	return table;
}
