#include "RRTStar.hpp"
#include "Graph.hpp"
#include "AirSimTest.hpp"
// Memory Leak Check
#include <crtdbg.h>

#ifdef _DEBUG
#define DBG_NEW new ( _NORMAL_BLOCK , __FILE__ , __LINE__ )
// Replace _NORMAL_BLOCK with _CLIENT_BLOCK if you want the
// allocations to be of _CLIENT_BLOCK type
#else
#define DBG_NEW new
#endif

void singleRRT() {
    Coord start = Coord(-110, 130, 10);
    Coord end = Coord(110, -130, 10);
    BiRRTStar brrt = BiRRTStar(start, end);
    brrt.CallRRTStar();

    //for (auto& it : brrt.getPath()) {
    //    it->printNode();s
    //}
    for (auto& it : brrt.getPath()) {
        std::cout << it->node_number << " ";
    }
    std::cout << std::endl;
    
}

void print(int i) {
    std::cout << "Lol " << i << std::endl;
}

void threadLol() {
    std::thread droneThreads[5];
    for (int i = 0; i < 5; i++) {
        droneThreads[i] = std::thread(print, i);
    }

    

    for (int i = 0; i < 5; i++) {
        droneThreads[i].join();
    }
}


int main()
{
    //srand((unsigned)time(NULL));
    srand(SEED);
    //threadLol();
    //singleRRT();
    //singleDrone();
    droneTest();
    //std::vector<CollisionDetails*> collisions;
    //std::vector<Node*> path_1, path_2;
    //std::vector<std::vector<Node*>> allDronePaths;

    //path_1.push_back(new Node(0, Coord(0, 0, 0)));
    //path_1.push_back(new Node(1, Coord(25, 25, 25)));
    //path_1.push_back(new Node(2, Coord(40, 50, 5)));
    //

    //path_2.push_back(new Node(0, Coord(10, 0, 0)));
    //path_2.push_back(new Node(1, Coord(24, 24, 24)));
    //path_2.push_back(new Node(2, Coord(35, 35, 35)));
    //

    //allDronePaths.push_back(path_1);
    //allDronePaths.push_back(path_2);

    //for (int i = 0; i < allDronePaths.size(); i++) {
    //    droneCollisionCheck(i, allDronePaths);
    //}

    _CrtDumpMemoryLeaks();
    return 0;
}