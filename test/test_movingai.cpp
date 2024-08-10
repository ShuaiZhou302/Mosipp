
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
#include "Priority-sipp.hpp"


int emptylimit = 500;
int roomlimit = 5500;
int denlimit = 1000;
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
    std::string MapPath = "/home/david/文档/GitHub/Mosipp/data/maps/warehouse-10-20-10-2-1.map";
    std::ofstream output_file("/home/david/文档/GitHub/Mosipp/data/result/p_sipp-warehouse-baseline1-results.txt");

    rzq::basic::Grid g;
    LoadMap_MovingAI(MapPath, &g);

    if (!output_file.is_open()) {
        std::cerr << "Failed to open the file." << std::endl;
        return -1;
    }

    for (int n = 150; n <= 600; n += 50) {
        int success_count = 0;
        output_file << "n: " << n << std::endl;

        for (int i = 1; i <= 25; ++i) {
            std::string ScenPath = R"(/home/david/文档/GitHub/Mosipp/data/scen/warehouse-161-63-scen-random/warehouse-10-20-10-2-1-random-)" + std::to_string(i) + ".scen";
            std::vector<long> starts;
            std::vector<long> goals;
            LoadScenarios(ScenPath, n, &starts, &goals);
            rzq::P_SIPP planner;
            int result = planner.Solve(starts,goals,&g,denlimit);

            if (result) {
                ++success_count;
                output_file << "Solution found: true" << std::endl;
                output_file << "Runtime: " << planner.get_runtime() << std::endl;
                output_file << "Makespan: " << planner.get_makespan() << std::endl;
                output_file << "Soc: " << std::fixed << std::setprecision(2) << planner.get_soc() << std::endl;
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
int simple_TEST()
{
    // static environment
  rzq::basic::Grid static_world; // static obstacles appears in this 2d grid.
  int r = 3; // rows (y)
  int c = 3; // columns (x)
  static_world.Resize(r,c);
    static_world.Set(1,0,1); // set grid[y=1,x=1] = 1, a static obstacle.
    static_world.Set(2,0,1); // set grid[y=1,x=1] = 1, a static obstacle.
    static_world.Set(1,2,1); // set grid[y=1,x=1] = 1, a static obstacle.
    static_world.Set(2,2,1); // set grid[y=1,x=1] = 1, a static obstacle.
  std::vector<long> starts{1,0};
  std::vector<long> goals{4,7};
  rzq::P_SIPP planner;
  int result = planner.Solve(starts,goals,&static_world,10);
  std::cout<<result<<std::endl;
  return 1;
}
int main(){
    Test_P_SIPP();
    //simple_TEST();
    return 0;
};


