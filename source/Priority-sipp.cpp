/*******************************************
 * Author: Shuai Zhou.
 * All Rights Reserved.
 *******************************************/

#include "Priority-sipp.hpp"
#include <chrono>

int rzq::P_SIPP::Solve(std::vector<long> start, std::vector<long> goal, rzq::basic::Grid *graph, long max) {
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
    auto start_time = std::chrono::steady_clock::now();
    for (int i = 0; i < start.size(); i++){
        auto current_time = std::chrono::steady_clock::now();
        double elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
        if (elapsed_time > timelimit) {
            return 0; // 超时返回0
        }
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
                for (auto yy : res.costs[k]) {
                    soc += yy;
                    if (yy > makespan) {
                        makespan = yy;
                    }
                }
                // add constrain
                auto& path = res.paths[k];
                auto& times = res.times[k];
                // first node constrain
                for (long t = times[0]; t <= times[1]; ++t) {
                    node_constraints.emplace_back(std::vector<long>({path[0], t}));
                }

                // tail node constrain
                for (long t = times.back() - cost; t <= max; ++t) {
                    node_constraints.emplace_back(std::vector<long>({path.back(), t}));
                }

                // 中间节点约束
                for (size_t i = 1; i < path.size() - 1; ++i) {
                    for (long t = times[i] - cost; t <= times[i + 1]; ++t) {
                        node_constraints.emplace_back(std::vector<long>({path[i], t}));
                    }
                }

            }
        }
    }
    auto current_time = std::chrono::steady_clock::now();
    runtime = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
    return 1;
}
