//
//  main.cpp
//  2017MCM-C
//
//  Created by Shi Yue on 20/1/2017.
//  Copyright © 2017 Shi Yue. All rights reserved.
//

#include <iostream>
#include <algorithm>
#include <cmath>

//Time
const double DT = 0.1;
const double END_TIME = 100.0;
//Road
const int NUM_LANE = 4;
const int ROAD_LENGTH = 1000;
const int BLOCK_LENGTH = 5;
const int NUM_BLOCKS_PER_LANE = ROAD_LENGTH / BLOCK_LENGTH;
const double LANE_V_LIMIT[5] = {70.0, 60.0, 60.0, 60.0, 60.0};
//Car setting
const int OBSERVABLE_DIST_HUMAN_DRI = 100;
const int OBSERVABLE_BLOCKS_HUMAN_DRI = OBSERVABLE_DIST_HUMAN_DRI / BLOCK_LENGTH + 1;
    //Caution!!! OBSERVABLE_BLOCKS_HUMAN_DRI may exceeds the actual observable range!!!
const int NO_CAR_DIST = 50;
const int NO_CAR_BLOCK = NO_CAR_DIST / BLOCK_LENGTH;
const int TRIGGER_DIST = 150;
const int TRIGGER_BLOCK = TRIGGER_DIST / BLOCK_LENGTH;
const double COEF[4] = {0.0073, 0.0010, -0.0024, -0.0004};

class Car {
private:
    char type;      //self-driving 's' or human-driving 'h'
    double a;       //acceleration
    double v;       //velocity
    double s;       //position
    int lane;
    int blockPos;
public:

    Car(char type, double a, double v, double s, int lane, int blockPos) {
        this->type = type;
        this->a = a;
        this->v = v;
        this->a = a;
        this->lane = lane;
        this->blockPos = blockPos;
    }
    char getType() {
        return type;
    }
    double getA() {
        return a;
    }
    double getV() {
        return v;
    }
    double getS() {
        return s;
    }
    int getLane() {
        return lane;
    }
    void setA(double a) {
        this->a = a;
    }
    void setV(double v) {
        this->v = v;
    }
    void setS(double s) {
        //after running this method, the pointers road[][] need to be updated
        //TODO
        this->s = s;
    }
    void setLane(int lane) {
        this->lane = lane;
    }
    void updS() {
        //this method should be called only after new acceleration is set
        double vv = v;
        v =  std::max(std::min(v + a * DT, LANE_V_LIMIT[lane]), 0.0);
        s += (vv + v) / 2;
    }
    Car *frontCar(Car *road[NUM_LANE][NUM_BLOCKS_PER_LANE]) {
        for (int i = 1; (i < OBSERVABLE_DIST_HUMAN_DRI/BLOCK_LENGTH
             ) && blockPos + i <= NUM_BLOCKS_PER_LANE; i++) {
            if ((road[lane][blockPos+i] != NULL) &&
                (road[lane][blockPos+i]->getS()-s
                    <= OBSERVABLE_DIST_HUMAN_DRI)) {
                return road[lane][blockPos+i];
            }
        }
        return NULL;
    }
    Car *backCar(Car *road[NUM_LANE][NUM_BLOCKS_PER_LANE]) {
        for (int i = 1; (i < OBSERVABLE_DIST_HUMAN_DRI/BLOCK_LENGTH
                         ) && blockPos - i >= 0; i++) {
            if ((road[lane][blockPos-i] != NULL) &&
                (s-road[lane][blockPos-i]->getS()
                    <= OBSERVABLE_DIST_HUMAN_DRI)) {
                return road[lane][blockPos-i];
            }
        }
        return NULL;
    }
};

void move(Car *road[][NUM_BLOCKS_PER_LANE], int lane, int blockPos) {
    Car *thisCar = road[lane][blockPos];
    Car *frontCar = thisCar->frontCar(road);
    Car *backCar = thisCar->backCar(road);
    
    double x = 0;
    if (frontCar != NULL) {
        x += COEF[0] * (frontCar->getV() - thisCar->getV());
        x += COEF[1] * (frontCar->getS() - thisCar->getS());
    }
    if (backCar != NULL) {
        x += COEF[2] * (thisCar->getV() - backCar->getV());
        x += COEF[3] * (thisCar->getS() - backCar->getS());
    }
    double a = 6.0 / (1+exp(-4.0*pow(x, 7.0))) - 3.0;       //new acceleration
    thisCar->setA(a);
    thisCar->updS();
    if (thisCar->getS() > ROAD_LENGTH) {
        delete thisCar;
        road[lane][blockPos] = NULL;
    }
    else {
        int newBlockPos = ((int) thisCar->getS()) / BLOCK_LENGTH;
        road[lane][newBlockPos] = road[lane][blockPos];//assuming no lane change
        road[lane][blockPos] = NULL;
    }
}

void runDT(Car *road[][NUM_BLOCKS_PER_LANE]) {
    for (int blockPos = NUM_BLOCKS_PER_LANE - 1; blockPos >= 0; blockPos--) {
        for (int lane = 0; lane < NUM_LANE; lane++) {
            move(road, lane, blockPos);
        }
    }
    for (int lane = 0; lane < NUM_LANE; lane++) {
        int nearestCarBlock = TRIGGER_BLOCK;
        for (int blockPos = TRIGGER_BLOCK - 1; blockPos >= 0; blockPos--) {
            if (road[lane][blockPos] != NULL) {
                nearestCarBlock = blockPos;
            }
        }
        if (rand() <
            (RAND_MAX *
                ((double) (nearestCarBlock - NO_CAR_BLOCK))
                    /(TRIGGER_DIST - NO_CAR_BLOCK))) {
                Car *newCar = new Car('h', 1.5, 30.0, 0.0, lane, 0);
                road[lane][0] = newCar;
        }
    }
}

int main() {
    Car *road[NUM_LANE][NUM_BLOCKS_PER_LANE];
        for (double t = 0.0; t < END_TIME; t += DT) {
            runDT(road);
        }
    return 0;
}
