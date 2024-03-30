#pragma once

#include "Tuple.hpp"
#include "Graph.hpp"

namespace mtrn3100 {

// COMPLETE THIS FUNCTION.
template <typename N, typename E>
char* graph2ascii(Graph<N, E> const& g) {
    // This is how the nodes are numbered.
    //  --- --- --- --- --- --- --- --- ---
    // | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 |
    //  --- --- --- --- --- --- --- --- ---
    // | 10| 11| 12| 13| 14| 15| 16| 17| 18|
    //  --- --- --- --- --- --- --- --- ---
    // | 19| 20| 21| 22| 23| 24| 25| 26| 27|
    //  --- --- --- --- --- --- --- --- ---
    // | 28| 29| 30| 31| 32| 33| 34| 35| 36|
    //  --- --- --- --- --- --- --- --- ---
    // | 37| 38| 39| 40| 41| 42| 43| 44| 45|
    //  --- --- --- --- --- --- --- --- ---
    int const numRows = 5;
    int const numCols = 9;

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
        // maze[i] = ' ';
        maze[i] = '*';
    }

    // Do new lines.
    for (int r = 0; r < numCharRows; r++) {
        maze2d(r, numCharCols - 1) = '\n';
    }

    // Terminate the string.
    maze2d(numCharRows - 1, numCharCols - 1) = '\0';


    // Put numbers in each cell
    // int num = 1;
    // for (int i = 0; i < 3; i++) {
    //     int r = row2charPos(i);
    //     for (int j = 0; j < 3; j++) {
    //         int c = col2charPos(j);
    //         maze2d(r,c) = num + '0';
    //         num++;
    //     }
    // }


    // Do external walls.
    for (int i = 0; i < numCharRows; i++) {
        if (i % 2 == 0) {
            // Horizontal walls
            for (int j = 0; j < numCharCols - 1; j++) {
                if (j % 4 == 0) {
                    maze2d(i, j) = '*';
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
        // Serial.print("V1: ");
        // Serial.print(v1);
        // Serial.print(", V2: ");
        // Serial.println(v2);
        int r1 = (v1 - 1) / numCols;
        int r2 = (v2 - 1) / numCols;
        int c1 = (v1 - 1) % numCols;
        int c2 = (v2 - 1) % numCols;
        // Serial.print("r1: ");
        // Serial.print(r1);
        // Serial.print(", r2: ");
        // Serial.print(r2);
        // Serial.print(", c1: ");
        // Serial.print(c1);
        // Serial.print(", c2: ");
        // Serial.println(c2);
        // std::cout << "r1: " << r1 << ", r2: " << r2 << " | c1: " << c1 << ", c2: " << c2 << std::endl;
        if (r1 == r2) {
            // Remove a wall
            if (c1 < c2) {
                // Remove wall right of c1
                maze2d(row2charPos(r1), col2charPos(c1) + 2) = '*';
            }
            else {
                // Remove wall left of c2
                // maze2d(row2charPos(r1), col2charPos(c2) - 2) = ' ';
            }
        }
        else {
            // Remove a floor
            if (r1 < r2) {
                // Remove floor above r2
                for (int i = 0; i < 3; i++) {
                    int num = (row2charPos(r2) - 1) * numCharCols + 1 + 4 * c1 + i;
                    maze[num] = '*';
                }

            }
            else {
                // Remove floor above r1
            }
        }
    }

    // For debugging. Don't forget to include iostream.
    // std::cout << maze << std::endl;

    return maze;
}

}  // namespace mtrn3100