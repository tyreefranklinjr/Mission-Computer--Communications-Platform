// test_astar.cpp
//
// AI-assisted (Claude): this test was developed with AI assistance.
//
// Checks that A* finds a correct shortest path on a small known grid with
// obstacles, and correctly reports failure when no path exists.

#include <cstdio>

using namespace std;

#include "../src/astar.hpp"

int g_failures = 0;

void Check(bool condition, const char *message) {
    if (!condition) {
        printf("FAIL: %s\n", message);
        g_failures = g_failures + 1;
    } else {
        printf("PASS: %s\n", message);
    }
}

int main() {
    // 5x5 grid, a vertical wall at x=2 from y=0..3 with a single gap at y=4.
    //   . . # . .
    //   . . # . .
    //   . . # . .
    //   . . # . .
    //   . . . . .   <- gap at (2,4)
    OccupancyGrid grid(5, 5);
    grid.SetObstacle(2, 0);
    grid.SetObstacle(2, 1);
    grid.SetObstacle(2, 2);
    grid.SetObstacle(2, 3);

    GridCell start;
    start.x = 0;
    start.y = 0;
    GridCell goal;
    goal.x = 4;
    goal.y = 0;

    vector<GridCell> path;
    bool found = AstarFindPath(grid, start, goal, path);

    Check(found, "A* finds a path around the wall through the gap");
    if (found) {
        Check(GridCellEquals(path.front(), start), "path starts at the start cell");
        Check(GridCellEquals(path.back(), goal), "path ends at the goal cell");

        bool passes_through_gap = false;
        for (int i = 0; i < (int)path.size(); i = i + 1) {
            if (path[i].x == 2 && path[i].y == 4) {
                passes_through_gap = true;
            }
        }
        Check(passes_through_gap, "path correctly routes through the only gap in the wall");

        // Shortest possible path detours down to the gap at (2,4) and back
        // up to (4,0): 4 steps down + 2 across to reach the gap, then 2
        // across + 4 up to reach the goal, plus the starting cell itself.
        Check((int)path.size() <= 13, "path length is the expected shortest length or shorter");

        for (int i = 0; i < (int)path.size(); i = i + 1) {
            Check(!grid.IsObstacle(path[i].x, path[i].y), "no path cell sits on an obstacle");
        }
    }

    // A fully blocked grid should correctly report no path found.
    OccupancyGrid blocked_grid(3, 3);
    blocked_grid.SetObstacle(1, 0);
    blocked_grid.SetObstacle(1, 1);
    blocked_grid.SetObstacle(1, 2);
    GridCell blocked_start;
    blocked_start.x = 0;
    blocked_start.y = 1;
    GridCell blocked_goal;
    blocked_goal.x = 2;
    blocked_goal.y = 1;
    vector<GridCell> blocked_path;
    bool blocked_found = AstarFindPath(blocked_grid, blocked_start, blocked_goal, blocked_path);
    Check(!blocked_found, "A* correctly reports no path when the goal is fully walled off");

    if (g_failures == 0) {
        printf("\nAll test_astar checks passed.\n");
        return 0;
    }
    printf("\n%d test_astar check(s) failed.\n", g_failures);
    return 1;
}
