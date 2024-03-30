#pragma once
#include "LinkedList.hpp"

namespace mtrn3100 {
const int MAX_VERTICES = 45; // Adjust this based on your needs

class Graph {
public:
    Graph(int vertices);
    void addEdge(int v, int w);
    void printGraph();
    LinkedList<int> nodes(int index);

private:
    int numVertices;
    int adjacencyList[MAX_VERTICES][MAX_VERTICES];
};

Graph::Graph(int vertices) {
    numVertices = vertices;
    for (int i = 0; i < numVertices; ++i) {
        for (int j = 0; j < numVertices; ++j) {
            adjacencyList[i][j] = 0; // Initialize all elements to 0
        }
    }
}

void Graph::addEdge(int v, int w) {
    adjacencyList[v - 1][w - 1] = 1;
    adjacencyList[w - 1][v - 1] = 1; // For undirected edges
}

LinkedList<int> Graph::nodes(int index) {
    LinkedList<int> neighbors; // Linked list to store neighbors
    for (int w = 0; w < numVertices; ++w) {
        if (adjacencyList[index - 1][w] == 1) {
            neighbors.push_back(w + 1); // Add neighbor to linked list
        }
    }
    return neighbors;
}

void Graph::printGraph() {
    for (int v = 0; v < numVertices; ++v) {
        Serial.print("Vertex " + String(v + 1) + " -> ");
        for (int w = 0; w < numVertices; ++w) {
            if (adjacencyList[v][w] == 1) {
                Serial.print(w + 1);
                Serial.print(" ");
            }
        }
        Serial.println();
    }
}

}