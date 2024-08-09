/*******************************************
 * Author: Shuai Zhou.
 * All Rights Reserved.
 *******************************************/

#include "Priority-sipp.hpp"

int rzq::P_SIPP::Solve(std::vector<long> start, std::vector<long> goal, rzq::basic::Grid *graph) {
    _graph = graph;
    _start = start;
    _goal = goal;
    runtime = 0;
    soc = 0;
    makespan = 0;
    rzq::basic::CostVector wait_cost(1,2); // all values=1, length=2.
    std::vector< std::vector<long> > node_constraints; // keep updating each time
    std::vector< std::vector<long> > edge_constraints;
    double timelimit = 30;
    for (int i = 0; i < start.size(); i++){
        // to be edited
        int cost = 1;
        rzq::basic::Grid cost1;
        cost1.Resize(_graph->GetRowNum(),_graph->GetColNum(),cost);
        std::vector<rzq::basic::Grid> cost_grids;
        cost_grids.push_back(cost1);
        //
        rzq::basic::GridkConn g; // this is the graph (impl as a grid) that represents the workspace
        g.Init(*_graph, cost_grids);
        long vo = start[i];
        long vd = goal[i];
        rzq::search::MOSPPResult res;
        rzq::search::RunMOSIPPGrid(g, vo, vd, timelimit, wait_cost, node_constraints, edge_constraints, &res);
        // update value
        if (!res.success) {
            return 0;
        } else {
            for (auto iter : res.paths) {
                long k = iter.first; // id of a Pareto-optipmal solution
                for (auto xx : res.times[k]) {
                    runtime += xx;
                }
                for (auto yy : res.costs[k]) {
                    runtime += yy;
                    if (yy > makespan) {
                        makespan = yy;
                    }
                }
                // add constrain
                for (auto xx : res.paths[k]) {
                    std::cout << xx << ", ";
                }
            }
        }
    }
    return 1;
}
