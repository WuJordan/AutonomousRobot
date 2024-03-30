#pragma once

// #include <iostream>

#include "Tuple.hpp"
#include "Graph.hpp"

namespace mtrn3100 {

// COMPLETE THIS FUNCTION.
template <typename N, typename E>
char* graph2ascii(Graph<N, E> const& g) {
    // 3 x 3 maze.
    int const numRows = 3;
    int const numCols = 3;

    // Helper function to convert between (rows, cols) and number of chars.
    auto row2charPos = [](int const r) { return r * 2 + 1; };
    auto col2charPos = [](int const c) { return c * 4 + 2; };

    int const numCharRows = row2charPos(numRows);      // 7 characters.
    int const numCharCols = col2charPos(numCols);      // 14 characters.
    char* maze = new char[numCharCols * numCharRows];  // 98 bytes is needed to represent 3 x 3 maze.

    // Helper function to access the maze with a 2D model.
    auto maze2d = [&maze, &numCharCols](unsigned const r, unsigned const c) -> char& {
        return maze[r * numCharCols + c];
    };

    // Initialise the maze values.
    for (int i = 0; i < numCharCols * numCharRows; i++) {
        maze[i] = ' ';
        // maze[i] = '*';
    }

    // Do new lines.
    for (int r = 0; r < numCharRows; r++) {
        maze2d(r, numCharCols - 1) = '\n';
    }

    // Terminate the string.
    maze2d(numCharRows - 1, numCharCols - 1) = '\0';

    // Do external walls.
    for (int i = 0; i < numCharRows; i++) {
        if (i % 2 == 0) {
            // Horizontal walls
            for (int j = 0; j < numCharCols - 1; j++) {
                if (j % 4 == 0) {
                    maze2d(i, j) = ' ';
                }
                else {
                    maze2d(i, j) = '-';
                }
            }
        }
        else {
            // Vertical walls
            for (int j = 0; j < numCharCols - 1; j++) {
                if (j % 4 == 0) {
                    maze2d(i, j) = '|';
                }
            }
        }
    }

    // Do internal walls.
    for (auto const& edge : g.edges()) {
        auto v1 = mtrn3100::get<0>(edge.value);
        auto v2 = mtrn3100::get<1>(edge.value);
        // std::cout << v1 << " " << v2 << std::endl;
        int r1 = (v1 - 1) / numCols;
        int r2 = (v2 - 1) / numCols;
        int c1 = (v1 - 1) % numCols;
        int c2 = (v2 - 1) % numCols;
        if (r1 == r2) {
            // Remove a wall
            if (c1 < c2) {
                // Remove wall right of c1
                maze2d(row2charPos(r1), col2charPos(c1) + 2) = ' ';
            }
        }
        else {
            // Remove a floor
            if (r1 < r2) {
                // Remove floor above r2
                for (int i = 0; i < 3; i++) {
                    int num = (row2charPos(r2) - 1) * numCharCols + 1 + 4 * c1 + i;
                    maze[num] = ' ';
                }

            }
        }
    }

    return maze;
}

}  // namespace mtrn3100