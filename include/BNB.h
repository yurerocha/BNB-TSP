#ifndef BRANCH_AND_BOUND_H
#define BRANCH_AND_BOUND_H

#include <iomanip>
#include <limits>
#include <queue>
#include <utility>
#include <vector>
#include "data.h"
#include "hungarian.h"
#include "Timer.h"
#include "Utils.h"

class BNB {
public:
    BNB(const std::shared_ptr<Data>& pData, double** pCost);

    /**
     * @brief Run Branch and Bound algorithm with DFS or BFS branching strategy.
    */
    void run(bool isBFS);

    /**
     * @brief Run Branch and Bound algorithm with lower bound branching 
     * strategy.
    */
    void runLB();
    void getSolutionHungarian(Node& rNode);

    void resetCosts();

    void log(std::ostream& rOs, double lb, double up, double time) const;

    // ~BNB();
private:
    std::shared_ptr<Data> mpData;
    double** mpCost;
    Vec2D<double> mpCostCopy;
};

#endif