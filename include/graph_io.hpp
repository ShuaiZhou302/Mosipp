
#ifndef ZHONGQIANGREN_BASIC_GRAPH_IO_H_
#define ZHONGQIANGREN_BASIC_GRAPH_IO_H_

#include <unordered_map>
#include <unordered_set>
#include <string>
#include <vector>
#include <iostream>

#include "graph.hpp"
// #include "emoa.hpp"

namespace raplab{


// int LoadSparseGraphDIMAC(std::string dist_data_fname, std::string time_data_fname, int cost_dim, Roadmap* out) ;

int LoadStartGoal(std::string benchmark_table_fname, std::vector<int>* sources, std::vector<int>* goals);

int LoadCoordDIMAC(std::string coord_file, std::vector< std::vector<double> >* output) ;

int LoadMap_MovingAI(std::string map_file_path, std::vector<std::vector<double> >* output) ;

int LoadScenarios(std::string filePath, int n, std::vector<long>* starts,
                                                              std::vector<long>* goals);

} // end namespace raplab


#endif  // ZHONGQIANGREN_BASIC_GRAPH_IO_H_
