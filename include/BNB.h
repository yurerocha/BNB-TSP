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
     * @brief Run branch-and-bound algorithm with DFS or BFS branching strategy.
    */
    void run(bool isDFS);

    auto branchingStrategy(const std::list<Node>& tree, bool isDFS) const {
        if(isDFS) {
            // tree.end() points to last pos + 1
            return std::make_pair(tree.back(), --tree.end());
        } else {
            return std::make_pair(tree.front(), tree.begin());
        }
    };

    /**
     * @brief Run branch-and-bound algorithm with lower bound branching 
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