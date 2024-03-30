#pragma once
#include "Graph.hpp"
#include "LinkedList.hpp"

namespace mtrn3100
{


    template <typename N>
    LinkedList<LinkedList<N>> bfs_multiple(Graph const &g, N const &src, N const &dst)
    {
    LinkedList<LinkedList<N>> paths;
    LinkedList<LinkedList<N>> queue;
    queue.push_back({src});

    int shortest_path = 500;

    
    LinkedList<N> visited_nodes; // Keeps track of visited nodes

    while (!queue.empty())
    {
        LinkedList<N> path = queue.pop_front();
        for (auto iter = path.begin(); iter != path.end(); ++iter) {
            N n = iter->value;
            // Serial.print(n);
            // Serial.print(" ");
        }
        // Serial.println();
        // Serial.print("Remaining memory: ");
        //             Serial.print(remainingMemory());
        //             Serial.println(" bytes");
        auto last_node = path.back();

        bool already_visited = false;
        for (auto iter = visited_nodes.begin(); iter != visited_nodes.end(); ++iter) {
            if (iter->value == last_node) {
                already_visited = true;
                continue;
            }
        }

        if (already_visited) {
            continue;  // Skip already visited nodes
        }

        visited_nodes.push_back(last_node); // Mark the node as visited
        if (last_node == dst) {
          if (path.size() <= shortest_path) {
            if (path.size() < shortest_path) {

                paths.clear(); // Clear previous paths since a shorter one is found
                shortest_path = path.size();
            }
            paths.push_back(path);
          }
        }
        else {
          LinkedList<N> neighbours = g.nodes(last_node);
          // Serial.print("Neighbours of ");
          // Serial.print(last_node);
          // Serial.print(": ");
          for (auto iter = neighbours.begin(); iter != neighbours.end(); ++iter) {
            N n = iter->value;
            // Serial.print(n);
            // Serial.print(" ");
            if (!path.contains(n)) {
              LinkedList<N> new_path(path);
              new_path.push_back(n);
              queue.push_back(new_path);
            }
          }
          // Serial.println();
        }
    }
    return paths;
  }

} // namespace mtrn3100
