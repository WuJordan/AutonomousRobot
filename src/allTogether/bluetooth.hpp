#pragma once

namespace mtrn3100 {


void recvWithStartEndMarkers() { //receives the strings and seperates the strings by new line using < as start marker and > as end marker
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
 
    while (Serial3.available() > 0 && newData == false) {
        rc = Serial3.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
    
    if (line < 11) {
        updateMaze();
    } else if (line == 12) {
        updateStartDest();
    }
    
}

void updateMaze() { // adds the received data into the char array maze with \n and \0
    if (newData == true) {
        strcat(maze, receivedChars);
        if (line == 11) {
            strcat(maze, "\0"); // terminate the string
            Serial.println("end");
        } else {
            strcat(maze, "\n");
        }
        newData = false;
        line++;
    }
}

void updateStartDest () {
    if (newData == true) {
        Serial.println(receivedChars);
        char startChar[] = "00";
        char destChar[] = "00";
        startChar[0] = receivedChars[0];
        startChar[1] = receivedChars[1];
        destChar[0] = receivedChars[2];
        destChar[1] = receivedChars[3];
        start = atoi(startChar);
        dest = atoi(destChar);
        if (receivedChars[4] == 'N') {
            heading = 0;
        } 
        else if (receivedChars[4] == 'E') {
            heading = 1;
        }
        else if (receivedChars[4] == 'S') {
            heading = 2;
        }
        else if (receivedChars[4] == 'W') {
            heading = 3;
        }
        line++;
    }
}
}