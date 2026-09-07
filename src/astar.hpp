// astar.hpp
//
// A* motion planning on a small fixed-size occupancy grid. The grid stands
// in for a simple obstacle map the mission computer would plan a path
// through. No dynamic allocation for the grid itself; the frontier and
// path use standard containers (vector, priority_queue).

#ifndef AUTONOMY_PLATFORM_ASTAR_HPP
#define AUTONOMY_PLATFORM_ASTAR_HPP

#include <vector>
#include <cmath>

using namespace std;

struct GridCell {
    int x;
    int y;
};

bool GridCellEquals(const GridCell &a, const GridCell &b) {
    return a.x == b.x && a.y == b.y;
}

// One entry on the A* open list (frontier), ordered by f-score (g + heuristic).
struct AstarOpenEntry {
    float f_score;
    int x;
    int y;
};

// Finds and removes the lowest f-score entry from the open list. The open
// list stays a plain vector and is scanned linearly rather than using a
// priority_queue with a custom comparator, keeping everything here plain
// functions and containers instead of operator overloads or functors. This
// is fine for the small grids this planner runs on.
AstarOpenEntry AstarPopLowestFScore(vector<AstarOpenEntry> &open_list) {
    int best_index = 0;
    for (int i = 1; i < (int)open_list.size(); i = i + 1) {
        if (open_list[i].f_score < open_list[best_index].f_score) {
            best_index = i;
        }
    }
    AstarOpenEntry best = open_list[best_index];
    open_list.erase(open_list.begin() + best_index);
    return best;
}

// A fixed-size occupancy grid: 0 = free, 1 = obstacle.
class OccupancyGrid {
public:
    OccupancyGrid(int width, int height) {
        width_ = width;
        height_ = height;
        for (int i = 0; i < width * height; i = i + 1) {
            cells_[i] = 0;
        }
    }

    void SetObstacle(int x, int y) {
        if (InBounds(x, y)) {
            cells_[y * width_ + x] = 1;
        }
    }

    bool IsObstacle(int x, int y) const {
        if (!InBounds(x, y)) {
            return true;
        }
        return cells_[y * width_ + x] == 1;
    }

    bool InBounds(int x, int y) const {
        return x >= 0 && x < width_ && y >= 0 && y < height_;
    }

    int GetWidth() const {
        return width_;
    }

    int GetHeight() const {
        return height_;
    }

private:
    static const int kMaxCells = 64 * 64;
    int cells_[kMaxCells];
    int width_;
    int height_;
};

float AstarHeuristic(int x1, int y1, int x2, int y2) {
    float dx = (float)(x1 - x2);
    float dy = (float)(y1 - y2);
    return sqrtf(dx * dx + dy * dy);
}

// Finds a shortest path from start to goal on the given grid using A* with
// 4-connected moves. Returns true and fills out_path (start to goal,
// inclusive) if a path exists, false otherwise.
bool AstarFindPath(const OccupancyGrid &grid, GridCell start, GridCell goal, vector<GridCell> &out_path) {
    out_path.clear();

    if (grid.IsObstacle(start.x, start.y) || grid.IsObstacle(goal.x, goal.y)) {
        return false;
    }

    int width = grid.GetWidth();
    int height = grid.GetHeight();

    vector<float> g_score(width * height, 1e9f);
    vector<int> came_from_x(width * height, -1);
    vector<int> came_from_y(width * height, -1);
    vector<bool> visited(width * height, false);

    vector<AstarOpenEntry> open_list;

    int start_index = start.y * width + start.x;
    g_score[start_index] = 0.0f;

    AstarOpenEntry start_entry;
    start_entry.f_score = AstarHeuristic(start.x, start.y, goal.x, goal.y);
    start_entry.x = start.x;
    start_entry.y = start.y;
    open_list.push_back(start_entry);

    int move_dx[4] = {1, -1, 0, 0};
    int move_dy[4] = {0, 0, 1, -1};

    while (!open_list.empty()) {
        AstarOpenEntry current = AstarPopLowestFScore(open_list);

        int current_index = current.y * width + current.x;
        if (visited[current_index]) {
            continue;
        }
        visited[current_index] = true;

        if (current.x == goal.x && current.y == goal.y) {
            vector<GridCell> reversed_path;
            int cx = goal.x;
            int cy = goal.y;
            while (!(cx == start.x && cy == start.y)) {
                GridCell cell;
                cell.x = cx;
                cell.y = cy;
                reversed_path.push_back(cell);
                int index = cy * width + cx;
                int prev_x = came_from_x[index];
                int prev_y = came_from_y[index];
                cx = prev_x;
                cy = prev_y;
            }
            GridCell start_cell;
            start_cell.x = start.x;
            start_cell.y = start.y;
            reversed_path.push_back(start_cell);

            int path_size = (int)reversed_path.size();
            for (int i = path_size - 1; i >= 0; i = i - 1) {
                out_path.push_back(reversed_path[i]);
            }
            return true;
        }

        for (int move = 0; move < 4; move = move + 1) {
            int neighbor_x = current.x + move_dx[move];
            int neighbor_y = current.y + move_dy[move];

            if (grid.IsObstacle(neighbor_x, neighbor_y)) {
                continue;
            }

            int neighbor_index = neighbor_y * width + neighbor_x;
            float tentative_g = g_score[current_index] + 1.0f;

            if (tentative_g < g_score[neighbor_index]) {
                g_score[neighbor_index] = tentative_g;
                came_from_x[neighbor_index] = current.x;
                came_from_y[neighbor_index] = current.y;

                AstarOpenEntry neighbor_entry;
                neighbor_entry.f_score = tentative_g + AstarHeuristic(neighbor_x, neighbor_y, goal.x, goal.y);
                neighbor_entry.x = neighbor_x;
                neighbor_entry.y = neighbor_y;
                open_list.push_back(neighbor_entry);
            }
        }
    }

    return false;
}

#endif // AUTONOMY_PLATFORM_ASTAR_HPP
