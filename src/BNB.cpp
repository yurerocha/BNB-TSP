#include "BNB.h"

BNB::BNB(const std::shared_ptr<Data>& pData, double** pCost) 
    : mpData(pData), mpCost(pCost) {
    // hungarian_init(&mHungarian, mpCost, mpData->getDimension(), 
    //                mpData->getDimension(), mode);
    mpCostCopy = Vec2D<double>(mpData->getDimension());
    for(int i = 0; i < mpData->getDimension(); ++i){
        mpCostCopy[i] = std::vector<double>(mpData->getDimension());
        for(int j = 0; j < mpData->getDimension(); ++j){
            mpCostCopy[i][j] = mpCost[i][j];
        }
    }
}

void BNB::run(bool isDFS) {
    Timer timer;
    timer.start();

    std::cout << "Start execution of BNB algorithm (lb ub gap time)" << std::endl;
    Node root; // no raiz
    getSolutionHungarian(root); // resolver AP_TSP a partir da instancia original
    double lower_bound = root.lower_bound;

    /* criacao da arvore */
    auto tree = std::list<Node>();
    tree.push_back(root);

    // gerar solução inicial.
    double upper_bound = solveGreedyTSP(mpData);
    while(!tree.empty()) {
        // escolher um dos nos da arvore
        auto [node, itNode] = branchingStrategy(tree, isDFS);

        getSolutionHungarian(node);
        if(isg(node.lower_bound, upper_bound)) {
            tree.erase(itNode);
            continue;
        }
        if(node.feasible) {
            // upper_bound = std::min(upper_bound, node.lower_bound);
            if(isl(node.lower_bound, upper_bound)) {
                upper_bound = node.lower_bound;
                log(std::cout, lower_bound, upper_bound, timer.count());
            }
        } else {
            /* Adicionando os filhos */
            // iterar por todos os arcos do subtour escolhido
            for(uint i = 0; i < node.subtours[node.chosen].size() - 1; ++i) {
                Node n;
                n.forbidden_arcs = node.forbidden_arcs;
                std::pair<int,int> forbidden_arc = {
                    node.subtours[node.chosen][i],
                    node.subtours[node.chosen][i + 1]
                };
                n.forbidden_arcs.push_back(forbidden_arc);
                tree.push_back(n); // inserir novos nos na arvore
            }
        }
        tree.erase(itNode);
    }
}

void BNB::runLB() {
    Timer timer;
    timer.start();

    std::cout << "Start execution of BNB algorithm (lb ub gap)" << std::endl;
    Node root; // no raiz
    getSolutionHungarian(root); // resolver AP_TSP a partir da inst original
    double lower_bound = root.lower_bound;
    /* criacao da arvore */
    auto comp = [](const Node& n1, const Node& n2) {
                    return isg(n1.lower_bound, n2.lower_bound);
                };
    std::priority_queue<Node, std::vector<Node>, decltype(comp)> tree(comp);

    tree.push(root);
    // gerar solução inicial.
    double upper_bound = solveGreedyTSP(mpData);
    while(!tree.empty()) {
        // escolher um dos nos da arvore
        auto node = tree.top();

        getSolutionHungarian(node);
        if(isg(node.lower_bound, upper_bound)) {
            tree.pop();
            continue;
        }
        if(node.feasible) {
            // upper_bound = std::min(upper_bound, node.lower_bound);
            if(isl(node.lower_bound, upper_bound)) {
                upper_bound = node.lower_bound;
                log(std::cout, lower_bound, upper_bound, timer.count());
            }
        } else {
            /* Adicionando os filhos */
            // iterar por todos os arcos do subtour escolhido
            for(uint i = 0; i < node.subtours[node.chosen].size() - 1; ++i) {
                Node n;
                n.forbidden_arcs = node.forbidden_arcs;
                std::pair<int,int> forbidden_arc = {
                    node.subtours[node.chosen][i],
                    node.subtours[node.chosen][i + 1]
                };
                n.forbidden_arcs.push_back(forbidden_arc);
                tree.push(n); // inserir novos nos na arvore
            }
        }
        tree.pop();
    }
}

void BNB::getSolutionHungarian(Node& rNode) {
    // std::cout << "Solve TSP via Hungarian algorithm" << std::endl;
    for(const auto& forb_arc : rNode.forbidden_arcs) {
        mpCost[forb_arc.first][forb_arc.second] = INFINITE;
        // std::cout << forb_arc.first << " " << forb_arc.second << std::endl;
    }
    hungarian_problem_t p;
    int mode = HUNGARIAN_MODE_MINIMIZE_COST;
    hungarian_init(&p, mpCost, mpData->getDimension(), 
                   mpData->getDimension(), mode);
    rNode.lower_bound = hungarian_solve(&p);
    // hungarian_print_status(&p);
    rNode.subtours = findSubtours(mpData, p);
    hungarian_free(&p);
    rNode.feasible = isFeasible(mpData, rNode.subtours);
    rNode.chosen = choose(rNode.subtours);

    for(const auto& forb_arc : rNode.forbidden_arcs) {
        mpCost[forb_arc.first][forb_arc.second] 
                                  = mpCostCopy[forb_arc.first][forb_arc.second];
        // std::cout << forb_arc.first << " " << forb_arc.second << std::endl;
    }
}

void BNB::log(std::ostream& rOs, double lb, double up, double time) const {
    rOs << std::fixed << std::setprecision(1) << lb << "\t"
        << up << "\t"
        << gap(lb, up) << "\t"
        << time << std::endl;
}

// BNB::~BNB() {
//   hungarian_free(&hungarian);
// }
