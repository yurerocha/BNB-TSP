#include <iostream>
using namespace std;

#include <memory>
#include <string>
#include <vector>
#include "BNB.h"
#include "data.h"
#include "hungarian.h"

int main(int argc, char** argv) {
	auto files = std::vector<string>({"bayg29", "bays29", "burma14", "fri26", 
									  "gr17", "gr21", "gr24", "ulysses16", 
									  "ulysses22"});
	for(auto f : files) {
		auto inst = "input/" + f + ".tsp";
		std::cout << "Read instance: " << inst << std::endl;
		// auto pData = std::make_shared<Data>(argc, argv[1]);
		auto pData = std::make_shared<Data>(argc, inst.c_str());
		pData->readData();

		double **cost = new double*[pData->getDimension()];
		for(int i = 0; i < pData->getDimension(); i++) {
			cost[i] = new double[pData->getDimension()];
			for(int j = 0; j < pData->getDimension(); j++) {
				cost[i][j] = pData->getDistance(i,j);
			}
		}

		auto bnb = std::make_shared<BNB>(pData, cost);
		std::cout << "Branching strategy: " << argv[2] << std::endl;
		if(string(argv[2]) == "dfs") {
			bnb->runLB();
		} else if(string(argv[2]) == "bfs") {
			bnb->run(false);
		} else if(string(argv[2]) == "lb") {
			bnb->runLB();
		}

		// hungarian_problem_t p;
		// int mode = HUNGARIAN_MODE_MINIMIZE_COST;
		// double obj_value = hungarian_solve(&p);
		// cout << "Obj. value: " << obj_value << endl;

		// cout << "Assignment" << endl;
		// hungarian_print_assignment(&p);

		// hungarian_free(&p);
		for(int i = 0; i < pData->getDimension(); i++) delete [] cost[i];
		delete [] cost;
		// delete pData.get();
	}

	return 0;
}
