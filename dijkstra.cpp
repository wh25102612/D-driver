#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <limits>

// 节点结构体
struct Node {
    int x, y;           // 节点坐标
    double cost;        // 到达该节点的成本
    int parent_x, parent_y; // 父节点坐标
    bool visited;       // 节点是否被访问过的标志

    // 重载<运算符，使得在优先队列中以cost为比较基准
    bool operator < (const Node& rhs) const {
        return cost > rhs.cost;
    }
};

class Dijkstra {
private:
    std::vector<std::vector<bool>> obstacle_map; // 障碍物地图
    double resolution; // 网格大小
    std::vector<std::vector<Node>> grid_map; // 网格地图
    int width, height; // 网格宽度和高度

    const int dx[8] = {1, 0, -1, 0, 1, -1, 1, -1};
    const int dy[8] = {0, 1, 0, -1, 1, 1, -1, -1};
    const double dcost[8] = {1, 1, 1, 1, std::sqrt(2), std::sqrt(2), std::sqrt(2), std::sqrt(2)};

public:
    // 构造函数，初始化网格地图和障碍物地图
    Dijkstra(int width, int height, double resolution)
        : width(width), height(height), resolution(resolution) {
        grid_map = std::vector<std::vector<Node>>(width, std::vector<Node>(height));
        obstacle_map = std::vector<std::vector<bool>>(width, std::vector<bool>(height, false));

        for (int i = 0; i < width; ++i) {
            for (int j = 0; j < height; ++j) {
                grid_map[i][j].x = i;
                grid_map[i][j].y = j;
                grid_map[i][j].cost = std::numeric_limits<double>::infinity(); // 初始化为无穷大
                grid_map[i][j].parent_x = -1;
                grid_map[i][j].parent_y = -1;
                grid_map[i][j].visited = false; // 初始化为未访问
            }
        }
    }

    // 设置障碍物
    void setObstacle(int x, int y) {
        obstacle_map[x][y] = true;
    }

    // Dijkstra规划函数
    std::vector<Node> planning(int start_x, int start_y, int goal_x, int goal_y) {
        std::priority_queue<Node> open_list;
        grid_map[start_x][start_y].cost = 0.0;  // 设置起点成本为0
        open_list.push(grid_map[start_x][start_y]);

        while (!open_list.empty()) {
            Node current = open_list.top();
            open_list.pop();

            // 如果当前节点是目标节点，搜索结束
            if (current.x == goal_x && current.y == goal_y) {
                // 从目标节点回溯到起始节点，得到路径
                std::vector<Node> path;
                while (current.x != start_x || current.y != start_y) {
                    path.push_back(current);
                    current = grid_map[current.parent_x][current.parent_y];
                }
                path.push_back(grid_map[start_x][start_y]);
                return path;
            }

            for (int i = 0; i < 8; ++i) {
                int nx = current.x + dx[i];
                int ny = current.y + dy[i];

                // 检查节点是否在地图内以及是否是障碍物或已访问过
                if (nx >= 0 && ny >= 0 && nx < width && ny < height && !obstacle_map[nx][ny] && !grid_map[nx][ny].visited) {
                    double new_cost = current.cost + dcost[i];
                    if (grid_map[nx][ny].cost > new_cost) {
                        grid_map[nx][ny].cost = new_cost;
                        grid_map[nx][ny].parent_x = current.x;
                        grid_map[nx][ny].parent_y = current.y;
                        open_list.push(grid_map[nx][ny]);
                    }
                }
            }
            // 标记当前节点为已访问
            grid_map[current.x][current.y].visited = true;
        }

        // 如果没有找到路径，返回一个空的路径
        return {};
    }
};

int main() {
    // 创建一个20x20的网格，每个格子大小为1
    Dijkstra planner(20, 20, 1);

    // 添加障碍物
    for (int i = 5; i <= 15; ++i) {
        planner.setObstacle(i, 10);
    }

    // 使用Dijkstra算法规划从(0,0)到(19,19)的路径
    std::vector<Node> path = planner.planning(0, 0, 19, 19);

    // 打印路径
    std::cout << "Path:" << std::endl;
    for (const Node& node : path) {
        std::cout << "(" << node.x << ", " << node.y << ")" << std::endl;
    }

    return 0;
}
