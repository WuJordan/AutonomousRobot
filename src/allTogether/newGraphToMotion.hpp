#include "Graph.hpp"
#include "LinkedList.hpp"

typedef mtrn3100::LinkedList<mtrn3100::LinkedList<int>, mtrn3100::util::less<mtrn3100::LinkedList<int>>> DoubleLinked;

namespace mtrn3100 {

void moveUp(char* plan, int &numSteps, int &heading, int &row);

void moveDown(char* plan, int &numSteps, int &heading, int &row);

void moveLeft(char* plan, int &numSteps, int &heading, int &col);

void moveRight(char* plan, int &numSteps, int &heading, int &col);

void goForward(char* plan, int &numSteps);

void leftForward(char* plan, int &numSteps);

void rightForward(char* plan, int &numSteps);

void reverseForward(char* plan, int &numSteps);

String makeMotionPlan(DoubleLinked paths, int start, int finish, int facing, int planLength) {
    // TODO COMMENTS !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // Maze size
    int numRows = 5;
    int numCols = 9;
    const String headings = "NESW"; //array of char with the different heading types
    char plans[paths.size()][planLength]; // Array to store all motion plans
    int planCount = 0;
    
    for (auto path = paths.begin(); path != paths.end(); path++) {
        // Set robot at starting position and heading
        int curRow = (start - 1) / numCols;
        int curCol = (start - 1) % numCols;
        int numSteps = 0;
        int heading = facing;
        
        auto list = path->value;
        Serial.println("A path");

        // Make array to store motion plan
        char plan[planLength];
        for (int i = 0; i < planLength; i++) {
          plan[i] = ' ';
        }
        // memset(plan, ' ', sizeof(plan));
        plan[sizeof(plan) - 1] = '\0'; // Add null terminator at the end

        for (auto step = list.begin(); step != list.end(); step++) {
          int destRow = (step->value - 1) / numCols;
          int destCol = (step->value - 1) % numCols;

          if (destRow < curRow) {
              moveUp(plan, numSteps, heading, curRow);
          }
          else if (destRow > curRow) {
              moveDown(plan, numSteps, heading, curRow);
          }
          else if (destCol < curCol) {
              moveLeft(plan, numSteps, heading, curCol);
          }
          else if (destCol > curCol) {
              moveRight(plan, numSteps, heading, curCol);
          }
        }
        // Store plan
        plan[numSteps] = '\0';
        Serial.println(plan);
        strcpy(plans[planCount], plan);
        // delete[] plan;
        planCount++;
    }

    // Find plan with shortest number of steps
    int size = planLength;
    int index = -1;
    for (int i = 0; i < paths.size(); i++) {
      if (strlen(plans[i]) < size) {
        size = strlen(plans[i]);
        index = i;
      }
    }
    
    return plans[index];
}

void moveUp(char* plan, int &numSteps, int &heading, int &row) {
    switch (heading) {
        case 0: // N
            goForward(plan, numSteps);
            break;
        case 1: // E
            leftForward(plan, numSteps);
            break;
        case 2: // S
            reverseForward(plan, numSteps);
            break;
        case 3: // W
            rightForward(plan, numSteps);
            break;
    }
    heading = 0;
    row--;
}

void moveDown(char* plan, int &numSteps, int &heading, int &row) {
    switch (heading) {
        case 0: // N
            reverseForward(plan, numSteps);
            break;
        case 1: // E
            rightForward(plan, numSteps);
            break;
        case 2: // S
            goForward(plan, numSteps);
            break;
        case 3: // W
            leftForward(plan, numSteps);
            break;
    }
    heading = 2;
    row++;
}

void moveLeft(char* plan, int &numSteps, int &heading, int &col) {
    switch (heading) {
        case 0: // N
            leftForward(plan, numSteps);
            break;
        case 1: // E
            reverseForward(plan, numSteps);
            break;
        case 2: // S
            rightForward(plan, numSteps);
            break;
        case 3: // W
            goForward(plan, numSteps);
            break;
    }
    heading = 3;
    col--;
}

void moveRight(char* plan, int &numSteps, int &heading, int &col) {
    switch (heading) {
        case 0: // N
            rightForward(plan, numSteps);
            break;
        case 1: // E
            goForward(plan, numSteps);
            break;
        case 2: // S
            leftForward(plan, numSteps);
            break;
        case 3: // W
            reverseForward(plan, numSteps);
            break;
    }
    heading = 1;
    col++;
}

void goForward(char* plan, int &numSteps) {
    plan[numSteps] = 'F';
    numSteps++;
}

void leftForward(char* plan, int &numSteps) {
    plan[numSteps] = 'L';
    numSteps++;
    goForward(plan, numSteps);
}

void rightForward(char* plan, int &numSteps) {
    plan[numSteps] = 'R';
    numSteps++;
    goForward(plan, numSteps);
}

void reverseForward(char* plan, int &numSteps) {
    plan[numSteps] = 'L';
    numSteps++;
    leftForward(plan, numSteps);
}

}