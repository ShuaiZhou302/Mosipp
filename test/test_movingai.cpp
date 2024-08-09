
/*******************************************
 * Author: Shuai Zhou.
 * All Rights Reserved.
 *******************************************/
#include <fstream>
#include <iostream>
#include <vector>
#include "mosipp.hpp"
#include <string>
#include <sstream>
#include <iomanip> // Include this header for std::fixed and std::setprecision
#include <fstream> // Include this header for file operations


int emptylimit = 450;
int roomlimit = 5000;
int denlimit = 900;
int warehouse = 3000;

int LoadMap_MovingAI(std::string map_file_path, rzq::basic::Grid* output) {
    // 打开文件
    std::ifstream ifs(map_file_path);
    if (!ifs.is_open()) {
        std::cout << "Fail to open " << map_file_path << std::endl;
        return 0;
    }

    std::string line;
    bool read = false;
    std::vector<std::vector<int>> grid;

    // 读取文件并生成栅格地图
    while (std::getline(ifs, line)) {
        if (line.find_first_of(".@TG") != std::string::npos) {
            read = true;
        }
        if (read) {
            std::vector<int> row;
            for (char pixel : line) {
                if (pixel == '.' || pixel == 'G') {
                    row.push_back(0);  // 空闲区域
                } else {
                    row.push_back(1);  // 障碍物
                }
            }
            grid.push_back(row);
        }
    }
    ifs.close();

    // 设置 rzq::basic::Grid 的大小
    int rows = grid.size();
    int cols = grid.empty() ? 0 : grid[0].size();
    output->Resize(rows, cols);

    // 将 grid 的值设置到 rzq::basic::Grid
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            if (grid[y][x] == 1) {
                output->Set(y, x, 1);  // 设置障碍物
            }
        }
    }

    return 1;
}
int LoadScenarios(std::string filePath, int n, std::vector<long>* starts, std::vector<long>* goals) {
    std::ifstream infile(filePath);
    std::string line;
    int lineCount = 0;
    bool firstLine = true;

    // 检查文件是否成功打开
    if (!infile.is_open()) {
        std::cerr << "Failed to open file: " << filePath << std::endl;
        return -1;
    }

    while (std::getline(infile, line) && lineCount < n) {
        if (firstLine) {
            firstLine = false; // Skip the version line
            continue;
        }

        std::istringstream iss(line);
        int bucket, mapWidth, mapHeight, startX, startY, goalX, goalY;
        std::string mapFile;
        double optimalLength;

        if (!(iss >> bucket >> mapFile >> mapWidth >> mapHeight >> startX >> startY >> goalX >> goalY >> optimalLength)) {
            std::cerr << "Failed to parse line: " << line << std::endl;
            break;
        }

        // Convert (x, y) to node ID
        long startID = startY * mapWidth + startX;
        long goalID = goalY * mapWidth + goalX;

        // 添加一些调试信息
        //std::cout << "Parsed startID: " << startID << ", goalID: " << goalID << std::endl;

        starts->push_back(startID);
        goals->push_back(goalID);

        lineCount++;
    }

    /* 检查是否正确读取了指定数量的行
    if (lineCount < n) {
        std::cerr << "Warning: Only " << lineCount << " lines were read. Expected: " << n << std::endl;
    }*/

    return 1;
}
int Test_P_SIPP() {
    std::cout << "####### Moving ai test Begin #######" << std::endl;
    std::string MapPath = "C:/Users/David Zhou/Documents/GitHub/public_LSRP/data/maps/den520d.map";
    std::ofstream output_file("C:/Users/David Zhou/Documents/GitHub/public_LSRP/-den520-baseline1.txt");

    if (!output_file.is_open()) {
        std::cerr << "Failed to open the file." << std::endl;
        return -1;
    }

    for (int n = ; n <= 1000; n += 50) {
        int success_count = 0;
        output_file << "n: " << n << std::endl;

        for (int i = 1; i <= 25; ++i) {
            std::string ScenPath = R"(C:/Users/David Zhou/Documents/GitHub/public_LSRP/data/scen/den520d-scen-random/den520d-random-)" + std::to_string(i) + ".scen";
            raplab::Grid2d g;
            std::vector<std::vector<double>> occupancy_grid;
            raplab::LoadMap_MovingAI(MapPath, &occupancy_grid);
            g.SetOccuGridPtr(&occupancy_grid);
            double time_limit = 30;

            std::vector<long> starts;
            std::vector<long> goals;
            raplab::LoadScenarios(ScenPath, n, &starts, &goals);
            raplab::Lsrp planner;
            planner.SetGraphPtr(&g);
            std::vector<double> duration(starts.size(), 1);
            //std::vector<double> duration(starts.size());
            //std::vector<double> settings = {0.8, 2.2, 2.7, 4.3, 5}; // 设置新的序列
            //for (size_t j = 0; j < duration.size(); ++j) {
            //    duration[j] = settings[j % settings.size()]; // 循环赋值
            //}
            /*
            for (size_t j = 0; j < duration.size(); ++j) {
                duration[j] = (j % 5) + 1;
            }*/
            planner.Setduration(duration);
            planner.Solve(starts, goals, time_limit, 5.0);
            auto soc = planner.re_soc();
            auto makespan = planner.re_makespan();
            auto runtime = planner.GetRuntime();

            if (runtime <= 30) {
                ++success_count;
                output_file << "Solution found: true" << std::endl;
                output_file << "Runtime: " << runtime << std::endl;
                output_file << "Makespan: " << makespan << std::endl;
                output_file << "Soc: " << std::fixed << std::setprecision(2) << soc << std::endl;
            } else {
                output_file << "Solution found: false" << std::endl;
                output_file << "Runtime: 0" << std::endl;
                output_file << "Makespan: 0" << std::endl;
                output_file << "Soc: 0" << std::endl;
            }
        }

        double success_rate = (success_count / 25.0) * 100;
        output_file << "Success rate: " << success_rate << "%" << std::endl;
    }

    output_file.close();
    std::cout << "####### Moving ai test End #######" << std::endl;
    return 1;
}
int main(){
    return 0;
};


