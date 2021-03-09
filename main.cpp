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


    _CrtDumpMemoryLeaks();
    return 0;
}