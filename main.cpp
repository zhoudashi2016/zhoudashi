#include <iostream>
#include <vector>
#include <unordered_set>
#include <queue>
#include <cmath>
#include <opencv2/opencv.hpp>

using namespace std;

struct MyPoint {
    int x;
    int y;
};

struct Path {
    double length;
    MyPoint startPoint;
    MyPoint endPoint;
};

double getDistance(MyPoint p1, MyPoint p2) {
    return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
}

void addEdge(vector<vector<double>>& graph, MyPoint p1, MyPoint p2) {
    double distance = getDistance(p1, p2);
    graph[p1.x][p1.y] = distance;
    graph[p2.x][p2.y] = distance;
}

vector<Path> searchPaths(vector<MyPoint>& A, vector<MyPoint>& B, double maxLength) {
    int nodeCount = A.size() + B.size();
    vector<vector<double>> graph(nodeCount, vector<double>(nodeCount, -1));

    // 添加边
    for (const auto& p1 : A) {
        for (const auto& p2 : B) {
            addEdge(graph, p1, p2);
        }
    }

    vector<double> shortestPath(nodeCount, -1);
    vector<int> previousNode(nodeCount, -1);
    priority_queue<pair<double, int>> q;

    // 初始化起点
    for (const auto& p : A) {
        shortestPath[p.x] = 0;
        q.emplace(0, p.x);
    }

    // Dijkstra算法
    while (!q.empty()) {
        int u = q.top().second;
        q.pop();

        for (int v = 0; v < nodeCount; v++) {
            if (graph[u][v] <= 0) // 跳过没有边连接的节点
                continue;

            if (shortestPath[v] == -1 || shortestPath[v] > shortestPath[u] + graph[u][v]) {
                shortestPath[v] = shortestPath[u] + graph[u][v];
                previousNode[v] = u;
                q.emplace(-shortestPath[v], v);
            }
        }
    }

    vector<Path> paths;
    for (const auto& p : B) {
        if (shortestPath[p.x] > 0 && shortestPath[p.x] <= maxLength) {
            MyPoint startPoint = A[previousNode[p.x]];
            MyPoint endPoint = p;
            double length = shortestPath[p.x];
            paths.push_back({ length, startPoint, endPoint });
        }
    }

    return paths;
}

void displayPathGraph(const std::vector<Path>& paths) {
    // 创建路径图
    cv::Mat pathGraph(800, 800, CV_8UC3, cv::Scalar(255, 255, 255));

    // 在路径图上绘制路径
    for (const Path& path : paths) {
        cv::Point startPoint(path.startPoint.x, path.startPoint.y);
        cv::Point endPoint(path.endPoint.x, path.endPoint.y);
        cv::Scalar color(0, 0, 255); // 使用红色表示路径
        cv::line(pathGraph, startPoint, endPoint, color, 2);
    }

    // 创建显示窗口
    cv::namedWindow("Path Graph", cv::WINDOW_NORMAL);

    // 显示路径图窗口并调整大小以适应路径图
    cv::imshow("Path Graph", pathGraph);
    cv::resizeWindow("Path Graph", pathGraph.cols, pathGraph.rows);

    // 等待用户按下按键
    cv::waitKey(0);
}



int main() {
    vector<MyPoint> A = { {1, 2}, {3, 3}, {1, 1} };
    vector<MyPoint> B = { {0, 0}, {1, 4}, {2, 0} };

    double maxLength = 10.0;

    vector<Path> paths = searchPaths(A, B, maxLength);

    if (paths.empty()) {
        cout << "No paths found." << endl;
    } else {
        for (const auto& path : paths) {
            cout << "Path Length: " << path.length << endl;
            cout << "Start Point: (" << path.startPoint.x << ", " << path.startPoint.y << ")" << endl;
            cout << "End Point: (" << path.endPoint.x << ", " << path.endPoint.y << ")" << endl;
        }
    }

    // 获取路径集合
    std::vector<Path> pathList = searchPaths(A, B, maxLength);

    // 显示路径图
    displayPathGraph(pathList);

    return 0;
}