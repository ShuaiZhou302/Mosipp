
#ifndef MOSIPP_PRIORITY_SIPP_HPP
#define MOSIPP_PRIORITY_SIPP_HPP
#include "mosipp.hpp"
#include "graph.hpp"
#include <set>
#include <unordered_map>
#include <limits>
#include <list>
#include "mospp_util.hpp"

namespace rzq{
class P_SIPP {
public:
    int Solve(std::vector<long> start, std::vector<long> goal, basic::Grid* graph, long max);
    double get_runtime(){return runtime;}
    long get_soc(){return soc;}
    long get_makespan(){return makespan;}

private:
    basic::Grid* _graph;
    std::vector<long> _start;
    std::vector<long> _goal;
    long soc;
    long makespan;
    double runtime;




};
}
#endif //MOSIPP_PRIORITY_SIPP_HPP
