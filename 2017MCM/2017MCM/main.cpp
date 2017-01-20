//
//  main.cpp
//  2017MCM
//
//  Created by Shi Yue on 19/1/2017.
//  Copyright © 2017 Shi Yue. All rights reserved.
//
const int MAX_EDGES_PER_VERTEX = 4

#include <iostream>

class Vertex {
private:
    double mile;
    Edge *edges[MAX_EDGES_PER_VERTEX];
public:
    Vertex(double mile) {
        this->mile = mile;
        for (int i = 0; i < MAX_EDGES_PER_VERTEX; i++) {
            edges[i] = NULL;
        }
    }
    double getMile() {
        return this->mile;
    }
    void setEdge(Edge *e) {
        for (int i = 0; i < MAX_EDGES_PER_VERTEX; i++) {
            if (edges[i] == NULL;) {
                edges[i] = e;
            }
        }
    }
    Edge *getEdge(int index) {
        return this->edges[i];
    }
};

class Edge {
private:
    int routeID;
    char type;
    Vertex *startMilepost;
    Vertex *endMilepost;
    int numLaneDec;
    int numLaneInc;
public:
    Edge(int routeID, char type, Vertex *startMilepost, Vertex *endMilepost, int numLaneDec, int numLaneInc) {
        this->routeID = routeID;
        this->type = type;
        this->startMilepost = startMilepost;
        this->endMilepost = endMilepost;
        this->numLaneDec = numLaneDec;
        this->numLaneInc = numLaneInc;
    }
};


int main( ) {

    return 0;
}
