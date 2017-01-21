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
#include "assert.h"

//Time
const double DT = 0.1;
const double END_TIME = 100.0;
//Road
const int NUM_LANE = 4;
const int ROAD_LENGTH = 2000;
const int BLOCK_LENGTH = 4;
const int NUM_BLOCKS_PER_LANE = ROAD_LENGTH / BLOCK_LENGTH;
const double LANE_V_LIMIT[5] = { 31.29, 26.82, 26.82, 26.82, 26.82 };// In m/s. equivalent to 70 and 60 in mph. Caution!!!Remember to change initial speed
//Car setting
const int OBSERVABLE_DIST_FORWARD = 100;
const int OBSERVABLE_BLOCKS_FORWARD = OBSERVABLE_DIST_FORWARD / BLOCK_LENGTH + 1;
//Caution!!! OBSERVABLE_BLOCKS_FORWARD may exceeds the actual observable range!!!
const int OBSERVABLE_DIST_BACKWARD = 50;
const int OBSERVABLE_BLOCKS_BACKWARD = OBSERVABLE_DIST_BACKWARD / BLOCK_LENGTH + 1;
const int REACTION_TIME = 15; // 0.1s
const int NO_CAR_DIST = 20;
const int NO_CAR_BLOCK = NO_CAR_DIST / BLOCK_LENGTH;
const int TRIGGER_DIST = 100;
const int TRIGGER_BLOCK = TRIGGER_DIST / BLOCK_LENGTH;
const double COEF[5] = { 0.0159, 0.0010, -0.005736, -0.0004, 0.0224 };
//CAUTION!!! SerialNumber is a global variable
int SN = 0;

class Car {
private:
	char type;                         //self-driving 's' or human-driving 'h'
	double a;                          //acceleration
	double preA[REACTION_TIME];        //previous acceleration
	double v;                          //velocity
	double preV[REACTION_TIME];        //previous velocity
	double s;                          //position
	double preS[REACTION_TIME];        //previous position
	int lane;
	int preLane[REACTION_TIME];        //previous lane
	int blockPos;
	int preBlockPos[REACTION_TIME];    //previous block
	int serialNum;
public:

	Car(char type, double a, double v, double s, int lane, int blockPos, int serialNum) {
		this->type = type;
		this->a = a;
		this->v = v;
		this->s = s;
		this->lane = lane;
		this->blockPos = blockPos;
		this->serialNum = serialNum;
		for (int i = 0; i < REACTION_TIME; i++) {
			this->preA[i] = 0;
			this->preV[i] = v;
			this->preS[i] = s;
			this->preLane[i] = lane;
			this->preBlockPos[i] = 0;
		}
	}
	char getType() {
		return type;
	}
	double getA() {
		return a;
	}
	double getPreA(int preTime) {
		return preA[preTime];
	}
	double getV() {
		return v;
	}
	double getPreV(int preTime) {
		return preV[preTime];
	}
	double getS() {
		return s;
	}
	double getPreS(int preTime) {
		return preS[preTime];
	}
	int getLane() {
		return lane;
	}
	int getPreLane(int preTime) {
		return preLane[preTime];
	}
	int getBlockPos() {
		return blockPos;
	}
	int getPreBlockPos(int preTime) {
		return preBlockPos[preTime];
	}
	int getSerialNum() {
		return serialNum;
	}
	void setA(double a) {
		for (int preTime = REACTION_TIME - 1; preTime >= 1; preTime--) {
			this->preA[preTime] = this->preA[preTime - 1]; //NOTE!!!
		}
		this->preA[0] = this->a;
		this->a = a;
	}
	void setV(double v) {
		for (int preTime = REACTION_TIME - 1; preTime >= 1; preTime--) {
			this->preV[preTime] = this->preV[preTime - 1]; //NOTE!!!
		}
		this->preV[0] = this->v;
		this->v = v;
	}
	void setS(double s) {
		//after running this method, the pointers road[][] need to be updated
		for (int preTime = REACTION_TIME - 1; preTime >= 1; preTime--) {
			this->preS[preTime] = this->preS[preTime - 1]; //NOTE!!!
		}
		this->preS[0] = this->s;
		this->s = s;
	}
	void setLane(int lane) {
		for (int preTime = REACTION_TIME - 1; preTime >= 1; preTime--) {
			this->preLane[preTime] = this->preLane[preTime - 1]; //NOTE!!!
		}
		this->preLane[0] = this->lane;
		this->lane = lane;
	}
	void setBlockPos(Car *road[][NUM_BLOCKS_PER_LANE], int blockPos) {
		int lastBlockPos = preBlockPos[REACTION_TIME - 1];
		//        printf("blockPos:%d  lastBlockPos:%d\n",blockPos,lastBlockPos);
		//        printf("cond1:%d  cond2:%d  cond3:%d\n",(lastBlockPos != blockPos),(road[lane][lastBlockPos] != NULL),(road[lane][lastBlockPos]->getSerialNum() == serialNum));
		if ((lastBlockPos != blockPos) && (road[lane][lastBlockPos] != NULL) && (road[lane][lastBlockPos]->getSerialNum() == serialNum)) {
			road[lane][lastBlockPos] = NULL;
			//            printf("LINE 138: buffer[lane] = NULL excuted\n");
			//            printf("lane:%d  lastBlockPos:%d  set as NULL\n",lane,lastBlockPos);
		}
		for (int preTime = REACTION_TIME - 1; preTime >= 1; preTime--) {
			this->preBlockPos[preTime] = this->preBlockPos[preTime - 1]; //NOTE!!!
		}
		this->preBlockPos[0] = this->blockPos;
		this->blockPos = blockPos;
	}
	void updVSBlockPos(Car *road[][NUM_BLOCKS_PER_LANE]) {//TODO change pre attributes
		//this method should be called only after new acceleration is set
		setV(std::max(std::min(v + a * DT, LANE_V_LIMIT[lane]), 0.0));
		setS(s + ((v + preV[0]) / 2.0) * DT);
		setBlockPos(road, std::min(((int)s) / BLOCK_LENGTH, NUM_BLOCKS_PER_LANE - 1));

		    //   printf("a:%f  v:%f  s:%f  lane:%d  blockPos:%d  SN:%d\n", a, v, s, lane, blockPos, serialNum);
	}
	Car *frontCar(Car *road[NUM_LANE][NUM_BLOCKS_PER_LANE]) {
		if (type == 's') {   //self-driving car
			for (int i = 1; blockPos + i < NUM_BLOCKS_PER_LANE; i++) {
				if ((road[lane][blockPos + i] != NULL) &&
					(road[lane][blockPos + i]->getPreBlockPos(0) == blockPos + i)) {
					return road[lane][blockPos + i];
				}
			}
			return NULL;
		}
		else {    //human-driving car
			for (int i = 1; (i < OBSERVABLE_BLOCKS_FORWARD)
				&& (blockPos + i < NUM_BLOCKS_PER_LANE); i++) {
				if ((road[lane][blockPos + i] != NULL) &&
					(road[lane][blockPos + i]->getPreBlockPos(0) == blockPos + i) &&
					(road[lane][blockPos + i]->getS() - s
					<= OBSERVABLE_DIST_FORWARD)) {
					return road[lane][blockPos + i];
				}
			}
			return NULL;
		}
	}
	Car *backCar(Car *road[NUM_LANE][NUM_BLOCKS_PER_LANE]) {
		if (type == 's') {
			for (int i = 1; blockPos - i >= 0; i++) {
				if ((road[lane][blockPos - i] != NULL) &&
					(road[lane][blockPos - i]->getBlockPos() == blockPos - i)) {
					return road[lane][blockPos - i];
				}
			}
			return NULL;
		}
		for (int i = 1; (i < OBSERVABLE_BLOCKS_BACKWARD
			) && blockPos - i >= 0; i++) {
			if ((road[lane][blockPos - i] != NULL) &&
				(road[lane][blockPos - i]->getBlockPos() == blockPos - i) &&
				(s - road[lane][blockPos - i]->getS()
				<= OBSERVABLE_DIST_BACKWARD)) {
				return road[lane][blockPos - i];
			}
		}
		return NULL;
	}
};

void move(Car *road[][NUM_BLOCKS_PER_LANE], Car *buffer[NUM_BLOCKS_PER_LANE], int lane, int blockPos);
void moveSelfCar(Car *road[][NUM_BLOCKS_PER_LANE], Car *buffer[NUM_BLOCKS_PER_LANE], int lane, int blockPos);
void moveHumanCar(Car *road[][NUM_BLOCKS_PER_LANE], Car *buffer[NUM_BLOCKS_PER_LANE], int lane, int blockPos);
void runDT(Car *road[][NUM_BLOCKS_PER_LANE], Car *buffer[NUM_BLOCKS_PER_LANE]);
void printRoad(Car *road[][NUM_BLOCKS_PER_LANE], char parameter);

int main() {
	Car *road[NUM_LANE][NUM_BLOCKS_PER_LANE];
	for (int lane = 0; lane < NUM_LANE; lane++) {
		for (int blockPos = 0; blockPos < NUM_BLOCKS_PER_LANE; blockPos++) {
			road[lane][blockPos] = NULL;
		}
	}
	Car *buffer[NUM_LANE];
	for (int lane = 0; lane < NUM_LANE; lane++) {
		buffer[lane] = NULL;
	}
	//    printRoad(road, 'v');

	for (double t = 0.0; t < END_TIME; t += DT) {
		printf("t=%f\n", t);
		runDT(road, buffer);
	}
	printRoad(road, 's');
	return 0;
}


void move(Car *road[][NUM_BLOCKS_PER_LANE], Car *buffer[NUM_BLOCKS_PER_LANE], int lane, int blockPos) {
	if (road[lane][blockPos]->getType() == 's') {
		moveSelfCar(road, buffer, lane, blockPos);
	}
	else {
		moveHumanCar(road, buffer, lane, blockPos);
	}
}

void moveSelfCar(Car *road[][NUM_BLOCKS_PER_LANE], Car *buffer[NUM_BLOCKS_PER_LANE], int lane, int blockPos) {
	Car *thisCar = road[lane][blockPos];
	Car *frontCar = thisCar->frontCar(road);
	Car *backCar = thisCar->backCar(road);

	double x = -0.1;
	if (frontCar != NULL) {
		x += COEF[0] * (frontCar->getPreV(REACTION_TIME - 1) - thisCar->getV());
		x += COEF[1] * (frontCar->getPreS(REACTION_TIME - 1) - thisCar->getS());
	}
	if (backCar != NULL) {
		x += COEF[2] * (thisCar->getPreV(REACTION_TIME - 1) - backCar->getV());
		x += COEF[3] * (thisCar->getPreS(REACTION_TIME - 1) - backCar->getS());
	}
	x = x + COEF[4] * (LANE_V_LIMIT[thisCar->getLane()] - thisCar->getPreV(REACTION_TIME - 1));//deal with v3

	double a = 6.0 / (1 + exp(-4.0*pow(x, 1.0))) - 3.0;       //new acceleration

	if (frontCar != NULL) {
		a += frontCar->getA() - frontCar->getPreA(0);
	}

	thisCar->setA(a);
	thisCar->updVSBlockPos(road);
	road[lane][thisCar->getBlockPos()] = thisCar;
	if (thisCar->getS() > ROAD_LENGTH) {
		if (buffer[lane] != NULL) {
			for (int preTime = 0; preTime < REACTION_TIME; preTime++) {
				if ((road[lane][buffer[lane]->getPreBlockPos(preTime)] != NULL) &&
					(road[lane][buffer[lane]->getPreBlockPos(preTime)]->getSerialNum()
					== buffer[lane]->getSerialNum())) {
					road[lane][buffer[lane]->getPreBlockPos(preTime)] = NULL;
				}
			}
			delete buffer[lane];
			buffer[lane] = thisCar;
		}
	}
	road[lane][thisCar->getBlockPos()] = thisCar;
}
void moveHumanCar(Car *road[][NUM_BLOCKS_PER_LANE], Car *buffer[NUM_BLOCKS_PER_LANE], int lane, int blockPos) {
	Car *thisCar = road[lane][blockPos];
	Car *frontCar = thisCar->frontCar(road);
	Car *backCar = thisCar->backCar(road);

	double x = -0.1;
	if (frontCar != NULL) {
		x += COEF[0] * (frontCar->getPreV(REACTION_TIME - 1) - thisCar->getV());
		x += COEF[1] * (frontCar->getPreS(REACTION_TIME - 1) - thisCar->getS());
	}
	if (backCar != NULL) {
		x += COEF[2] * (thisCar->getPreV(REACTION_TIME - 1) - backCar->getV());
		x += COEF[3] * (thisCar->getPreS(REACTION_TIME - 1) - backCar->getS());
	}
	x = x + COEF[4] * (LANE_V_LIMIT[thisCar->getLane()] - thisCar->getPreV(REACTION_TIME - 1));//deal with v3
	double a = 6.0 / (1 + exp(-4.0*pow(x, 1.0))) - 3.0;       //new acceleration

	thisCar->setA(a);
	thisCar->updVSBlockPos(road);
	
	if (thisCar->getS() > ROAD_LENGTH) {
		if (buffer[lane] != NULL) {
			for (int preTime = 0; preTime < REACTION_TIME; preTime++) {

				if ((road[lane][buffer[lane]->getPreBlockPos(preTime)] != NULL) &&
					(road[lane][buffer[lane]->getPreBlockPos(preTime)]->getSerialNum()
					== buffer[lane]->getSerialNum())) {
					road[lane][buffer[lane]->getPreBlockPos(preTime)] = NULL;
				}
			}
			delete buffer[lane];
			buffer[lane] = thisCar;
		}
	}
	road[lane][thisCar->getBlockPos()] = thisCar;

	//if (thisCar->getLane()==0){
		printf("a=%lf,v=%lf, s=%lf, SN=%d,lane=%d\n", thisCar->getA(), thisCar->getV(), thisCar->getS(), thisCar->getSerialNum(),thisCar->getLane());
	//}
}

void runDT(Car *road[][NUM_BLOCKS_PER_LANE], Car *buffer[NUM_BLOCKS_PER_LANE]) {
	for (int blockPos = NUM_BLOCKS_PER_LANE - 1; blockPos >= 0; blockPos--) {
		for (int lane = 0; lane < NUM_LANE; lane++) {
			if ((road[lane][blockPos] != NULL) &&
				(road[lane][blockPos]->getBlockPos() == blockPos)) {
				move(road, buffer, lane, blockPos);
				//printf("a=%lf,v=%lf, s=%lf, SN=%d", thisCar->getA(), thisCar->getV(), thisCar->getS(), thisCar->getSerialNum());
			}
		}
	}
	for (int lane = 0; lane < NUM_LANE; lane++) {
		int nearestCarBlock = TRIGGER_BLOCK;
		for (int blockPos = TRIGGER_BLOCK - 1; blockPos >= 0; blockPos--) {
			if (road[lane][blockPos] != NULL) {
				nearestCarBlock = blockPos;
			}
		}
		double probStartACar = (double(nearestCarBlock - NO_CAR_BLOCK)) / (TRIGGER_DIST - NO_CAR_BLOCK);
		//        probStartACar = 0.5;
		if (rand() < RAND_MAX * probStartACar) {
			Car *newCar = new Car('h', 0, 15.0 + (rand() % 10), 0.1, lane, 0, SN++);
			road[lane][0] = newCar;
		}
	}
}

void printRoad(Car *road[][NUM_BLOCKS_PER_LANE], char parameter) {
	switch (parameter) {
	case 't': {
		for (int blockPos = 0; blockPos < NUM_BLOCKS_PER_LANE; blockPos++) {
			printf("%5d ", blockPos);
			for (int lane = 0; lane < NUM_LANE; lane++) {
				if ((road[lane][blockPos] == NULL) ||
					(road[lane][blockPos]->getBlockPos() != blockPos)) {
					printf("  ");
				}
				else {
					printf("%c ", road[lane][blockPos]->getType());
				}
			}
			printf("\n");
		}
	}
			  break;
	case 'a': {
		for (int blockPos = 0; blockPos < NUM_BLOCKS_PER_LANE; blockPos++) {
			printf("%5d ", blockPos);
			for (int lane = 0; lane < NUM_LANE; lane++) {
				if ((road[lane][blockPos] == NULL) ||
					(road[lane][blockPos]->getBlockPos() != blockPos)) {
					printf("     ");
				}
				else {
					printf("%5.1f ", road[lane][blockPos]->getA());
				}
			}
			printf("\n");
		}
	}
			  break;
	case 'v': {
		for (int blockPos = 0; blockPos < NUM_BLOCKS_PER_LANE; blockPos++) {
			printf("%5d ", blockPos);
			for (int lane = 0; lane < NUM_LANE; lane++) {
				if ((road[lane][blockPos] == NULL) ||
					(road[lane][blockPos]->getBlockPos() != blockPos)) {
					printf("     ");
				}
				else {
					printf("%5.1f ", road[lane][blockPos]->getV());
				}
			}
			printf("\n");
		}
	}
			  break;
	case 's': {
		for (int blockPos = 0; blockPos < NUM_BLOCKS_PER_LANE; blockPos++) {
			printf("%5d ", blockPos);
			for (int lane = 0; lane < NUM_LANE; lane++) {
				if ((road[lane][blockPos] == NULL) ||
					(road[lane][blockPos]->getBlockPos() != blockPos)) {
					printf("     ");
				}
				else {
					printf("%5.1f ", road[lane][blockPos]->getS());
				}
			}
			printf("\n");
		}
	}
			  break;

	}
}
