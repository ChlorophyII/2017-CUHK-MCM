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
#include <ctime>
#include <chrono>
#include <thread>
#include <cstdlib>
#include <queue>

//Time
const double DT = 0.1;
const double END_TIME = 1000.0;
//Road
const int NUM_LANE = 4;
const int NUM_LANE_SELF = 1;
const int ROAD_LENGTH = 1000;
const int BLOCK_LENGTH = 5;
const int NUM_BLOCKS_PER_LANE = ROAD_LENGTH / BLOCK_LENGTH;
const double LANE_V_LIMIT[5] = { 31.29, 26.82, 26.82, 26.82, 26.82 };
// In m/s. equivalent to 70 and 60 in mph.
// Caution!!!Remember to change initial speed
const double AVG_CAR_PER_SEC = 2.5;
//Car setting
const int OBSERVABLE_DIST_FORWARD = 100;
const int OBSERVABLE_BLOCKS_FORWARD = OBSERVABLE_DIST_FORWARD / BLOCK_LENGTH + 1;
//Caution!!! OBSERVABLE_BLOCKS_FORWARD may exceeds the actual observable range!!!
const int OBSERVABLE_DIST_BACKWARD = 50;
const int OBSERVABLE_BLOCKS_BACKWARD = OBSERVABLE_DIST_BACKWARD / BLOCK_LENGTH + 1;
const int SAFE_DIST = 30;
const int REACTION_TIME = 15; // 0.1s
const double A_LIMIT = 5;
const int NO_CAR_DIST = 15;
const int NO_CAR_BLOCK = NO_CAR_DIST / BLOCK_LENGTH;
const int TRIGGER_DIST = 30;
const int TRIGGER_BLOCK = TRIGGER_DIST / BLOCK_LENGTH;
const double COEF[5] = { 0.0159, 0.0010, -0.005736, -0.0004, 0.0224 };
const double A_COEF[4] = { 0.19, -0.15, -0.12, 70 };
const double SELF_RATIO = 0.59;

//Print function
const bool CAR_WATERFALL = false;
const char PRINT_TYPE = 'e'; //'e' emoji, 'a' acceleration, 'v' velocity, 's' position
const double REFRESH_FREQ = 0.1; // s
const int REFRESH_NANO_SEC = REFRESH_FREQ * 1000000000;
const int START_BLOCK = 0;
const int END_BLOCK = 200;
// Constructor:
// Car(char type, double a, double v, double s, int lane, int blockPos, int serialNum)
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
	void setLaneAll(int lane) {
		this->lane = lane;
		for (int preTime = 0; preTime < REACTION_TIME; preTime++) {
			this->preLane[preTime] = lane;
		}
	}
	void setBlockPos(Car *road[][NUM_BLOCKS_PER_LANE], int blockPos) {
		int lastBlockPos = preBlockPos[REACTION_TIME - 1];
		if ((lastBlockPos != blockPos) && (road[lane][lastBlockPos] != NULL) && (road[lane][lastBlockPos]->getSerialNum() == serialNum)) {
			road[lane][lastBlockPos] = NULL;
		}
		for (int preTime = REACTION_TIME - 1; preTime >= 1; preTime--) {
			this->preBlockPos[preTime] = this->preBlockPos[preTime - 1]; //NOTE!!!
		}
		this->preBlockPos[0] = this->blockPos;
		this->blockPos = blockPos;
	}
	void updVSBlockPos(Car *road[][NUM_BLOCKS_PER_LANE]) {
		//this method should be called only after new acceleration is set
		setV(std::max(std::min(v + a * DT, LANE_V_LIMIT[lane]), 0.0));
		setS(s + ((v + preV[0]) / 2.0) * DT);
		setBlockPos(road, std::min(((int)s) / BLOCK_LENGTH, NUM_BLOCKS_PER_LANE - 1));
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


//******************************   CAUTION!!!   ******************************
// The followings are global variables!!!
int SN = 0;                         //serial number
double t = 0.0;                     //time
std::queue<Car *> queueSelf;       //queue of self-driving car
std::queue<Car *> queueHuman;      //queue of human-driving car
double carNumber = 0.0;
//******************************   CAUTION!!!   ******************************

//TODO lane need to be re-assigned when putting the car on the road

void move(Car *road[][NUM_BLOCKS_PER_LANE], Car *buffer[NUM_BLOCKS_PER_LANE], int lane, int blockPos);
void moveSelfCar(Car *road[][NUM_BLOCKS_PER_LANE], Car *buffer[NUM_BLOCKS_PER_LANE], int lane, int blockPos);
void moveHumanCar(Car *road[][NUM_BLOCKS_PER_LANE], Car *buffer[NUM_BLOCKS_PER_LANE], int lane, int blockPos);
void addCarInQueue();
void runDT(Car *road[][NUM_BLOCKS_PER_LANE], Car *buffer[NUM_BLOCKS_PER_LANE]);
void printRoad(Car *road[][NUM_BLOCKS_PER_LANE], char parameter);



int main() {
	srand((unsigned)clock());
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

	std::clock_t start = clock();
	for (; t < END_TIME; t += DT) {
		// printf("t=%f\n", t);
		runDT(road, buffer);
	}
	printRoad(road, 't');
	std::cout << (clock() - start) / (double)CLOCKS_PER_SEC << "s consumed\n";
	printf("%d", SN);
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
	double a;
	if (frontCar == NULL) {
		a = A_COEF[0] * (LANE_V_LIMIT[lane] - thisCar->getV());
		a = std::max(std::min(a, A_LIMIT), -A_LIMIT);
	}
	else if (frontCar->getS() - thisCar->getS() < SAFE_DIST) {
		a = A_COEF[1] * (A_COEF[3] - (frontCar->getS() - thisCar->getS()));
		a = std::max(std::min(a, A_LIMIT), -A_LIMIT);
	}
	else if ((backCar != NULL) &&
		(thisCar->getS() - backCar->getS() < SAFE_DIST)) {
		a = A_COEF[2] * (A_COEF[3] - thisCar->getS() - backCar->getS());
		//a = std::max(std::min(a, A_LIMIT), -A_LIMIT);
	}
	else {
		double x = -0.1;
		x += COEF[0] * (frontCar->getPreV(0) - thisCar->getV());
		x += COEF[1] * (frontCar->getPreS(0) - thisCar->getS());
		x += COEF[4] * (LANE_V_LIMIT[4] - thisCar->getV());
		if (backCar != NULL) {
			x += COEF[2] * (thisCar->getV() - backCar->getPreV(0));
			x += COEF[3] * (thisCar->getS() - backCar->getPreS(0));
		}
		a = 6.0 / (1 + exp(-4.0*pow(x, 1.0))) - 3.0;
		//a = std::max(std::min(a, A_LIMIT), -A_LIMIT);
	}

	if (frontCar != NULL){
		a = a + frontCar->getA() - frontCar->getPreA(0);
	}// add delta a
	a = std::max(std::min(a, A_LIMIT), -A_LIMIT);
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
}
void moveHumanCar(Car *road[][NUM_BLOCKS_PER_LANE], Car *buffer[NUM_BLOCKS_PER_LANE], int lane, int blockPos) {
	Car *thisCar = road[lane][blockPos];
	Car *frontCar = thisCar->frontCar(road);
	Car *backCar = thisCar->backCar(road);
	double a;
	if (frontCar == NULL) {
		a = A_COEF[0] * (LANE_V_LIMIT[lane] - thisCar->getV());
		a = std::max(std::min(a, A_LIMIT), -A_LIMIT);
	}
	else if (frontCar->getS() - thisCar->getS() < SAFE_DIST) {
		a = A_COEF[1] * (A_COEF[3] - (frontCar->getS() - thisCar->getS()));
		a = std::max(std::min(a, A_LIMIT), -A_LIMIT);
	}
	else if ((backCar != NULL) &&
		(thisCar->getS() - backCar->getS() < SAFE_DIST)) {
		a = A_COEF[2] * (A_COEF[3] - thisCar->getS() - backCar->getS());
		a = std::max(std::min(a, A_LIMIT), -A_LIMIT);
	}
	else {
		double x = -0.1;
		x += COEF[0] * (frontCar->getPreV(0) - thisCar->getV());
		x += COEF[1] * (frontCar->getPreS(0) - thisCar->getS());
		x += COEF[4] * (LANE_V_LIMIT[4] - thisCar->getV());
		if (backCar != NULL) {
			x += COEF[2] * (thisCar->getV() - backCar->getPreV(0));
			x += COEF[3] * (thisCar->getS() - backCar->getPreS(0));
		}
		a = 6.0 / (1 + exp(-4.0*pow(x, 1.0))) - 3.0;
		a = std::max(std::min(a, A_LIMIT), -A_LIMIT);
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
}

void addCarInQueue() {
	carNumber += DT * AVG_CAR_PER_SEC;
	if (carNumber > 1.0) {
		carNumber -= 1.0;
		if (rand() < RAND_MAX * SELF_RATIO) {
			//create a self-driving car
			Car *thisCar = new Car('s', 0, 25 + rand() % 5, 0, 0, 0, SN++);
			queueSelf.push(thisCar);
		}
		else {
			//create a human-driving car
			Car *thisCar = new Car('h', 0, 20 + rand() % 4, 0, 0, 0, SN++);
			queueHuman.push(thisCar);
		}
	}
	int A = (int)queueHuman.size();
	int B = (int)queueSelf.size();
	//printf("human size:%3d self size:%3d\n", A, B);
}

void runDT(Car *road[][NUM_BLOCKS_PER_LANE], Car *buffer[NUM_BLOCKS_PER_LANE]) {
	addCarInQueue();
	for (int blockPos = NUM_BLOCKS_PER_LANE - 1; blockPos >= 0; blockPos--) {
		for (int lane = 0; lane < NUM_LANE; lane++) {
			if ((road[lane][blockPos] != NULL) &&
				(road[lane][blockPos]->getBlockPos() == blockPos)) {
				move(road, buffer, lane, blockPos);
			}
		}
	}
	//    for (int lane = 0; lane < NUM_LANE; lane++) {
	//        int nearestCarBlock = TRIGGER_BLOCK;
	//        for (int blockPos = TRIGGER_BLOCK - 1; blockPos >= 0; blockPos--) {
	//            if (road[lane][blockPos] != NULL) {
	//                nearestCarBlock = blockPos;
	//            }
	//        }
	//        double probStartACar = (double(nearestCarBlock - NO_CAR_BLOCK))/(TRIGGER_DIST - NO_CAR_BLOCK);
	////        probStartACar = 0.5;
	//        if (rand() < RAND_MAX * probStartACar) {
	//                Car *newCar = new Car('h', 0, 25.0 + (rand() % 10), 0.1, lane, 0, SN++);
	//                road[lane][0] = newCar;
	//            }
	//    }
	//Self-driving car only lanes
	for (int lane = 0; lane < NUM_LANE_SELF; lane++) {
		int nearestCarBlock = TRIGGER_BLOCK;
		for (int blockPos = TRIGGER_BLOCK - 1; blockPos >= 0; blockPos--) {
			if (road[lane][blockPos] != NULL) {
				nearestCarBlock = blockPos;
			}
		}
		double probStartACar = (double(nearestCarBlock - NO_CAR_BLOCK)) / (TRIGGER_BLOCK - NO_CAR_BLOCK);
		if (rand() < RAND_MAX * probStartACar) {
			if (!queueSelf.empty()) {
				Car *thisCar = queueSelf.front();
				queueSelf.pop();
				thisCar->setLaneAll(lane);
				road[lane][0] = thisCar;
			}
		}
	}
	//Mixing lanes
	for (int lane = NUM_LANE_SELF; lane < NUM_LANE; lane++) {
		int nearestCarBlock = TRIGGER_BLOCK;
		for (int blockPos = TRIGGER_BLOCK - 1; blockPos >= 0; blockPos--) {
			if (road[lane][blockPos] != NULL) {
				nearestCarBlock = blockPos;
			}
		}
		double probStartACar = (double(nearestCarBlock - NO_CAR_BLOCK)) / (TRIGGER_BLOCK - NO_CAR_BLOCK);
		if (rand() < RAND_MAX * probStartACar) {
			if (queueHuman.empty()) {
				if (!queueSelf.empty()) {
					Car *thisCar = queueSelf.front();
					queueSelf.pop();
					thisCar->setLaneAll(lane);
					road[lane][0] = thisCar;
				}
			}
			else {
				if (queueSelf.empty()) {
					Car *thisCar = queueHuman.front();
					queueHuman.pop();
					thisCar->setLaneAll(lane);
					road[lane][0] = thisCar;
				}
				else {
					Car *selfCar, *humanCar;
					selfCar = queueSelf.front();
					queueSelf.pop();
					humanCar = queueHuman.front();
					queueHuman.pop();
					if (selfCar->getSerialNum() < humanCar->getSerialNum()) {
						selfCar->setLaneAll(lane);
						road[lane][0] = selfCar;
					}
					else {
						humanCar->setLaneAll(lane);
						road[lane][0] = humanCar;
					}
				}
			}
		}
	}
	if (CAR_WATERFALL) {
		printRoad(road, PRINT_TYPE);
	}
}

void printRoad(Car *road[][NUM_BLOCKS_PER_LANE], char parameter) {
	using namespace std::this_thread;
	using namespace std::chrono;
	switch (parameter) {
	case 'e': {
		for (int blockPos = START_BLOCK; blockPos < END_BLOCK; blockPos++) {
			if (blockPos == 0) {
				std::cout << "🏁 😃 🏁  ";
			}
			else {
				printf("%6d ", blockPos);
			}
			for (int lane = 0; lane < NUM_LANE; lane++) {
				if ((road[lane][blockPos] == NULL) ||
					(road[lane][blockPos]->getBlockPos() != blockPos)) {
					printf("   ");
				}
				else {
					switch (road[lane][blockPos]->getSerialNum() % 10) {
					case 0: {
						printf("🚗 ");
					}
							break;
					case 1: {
						printf("🛁 ");
					}
							break;
					case 2: {
						printf("🛩 ");
					}
							break;
					case 3: {
						printf("🚲 ");
					}
							break;
					case 4: {
						printf("🛴 ");
					}
							break;
					case 5: {
						printf("🚔 ");
					}
							break;
					case 6: {
						printf("🏎 ");
					}
							break;
					case 7: {
						printf("🚜 ");
					}
							break;
					case 8: {
						printf("🚀 ");
					}
							break;
					case 9: {
						printf("🛁 ");
					}
							break;
					}
				}
			}
			printf("\n");
		}
		if (CAR_WATERFALL) {
			sleep_for(nanoseconds(REFRESH_NANO_SEC));
			std::cout << "\x1B[2J\x1B[H";
		}
	}
			  break;
	case 't': {
		for (int blockPos = START_BLOCK; blockPos < END_BLOCK; blockPos++) {
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
		if (CAR_WATERFALL) {
			sleep_for(nanoseconds(REFRESH_NANO_SEC));
			std::cout << "\x1B[2J\x1B[H";
		}
	}
			  break;
	case 'a': {
		for (int blockPos = START_BLOCK; blockPos < END_BLOCK; blockPos++) {
			printf("%5d ", blockPos);
			for (int lane = 0; lane < NUM_LANE; lane++) {
				if ((road[lane][blockPos] == NULL) ||
					(road[lane][blockPos]->getBlockPos() != blockPos)) {
					printf("     ");
				}
				else {
					printf("%4.1f ", road[lane][blockPos]->getA());
				}
			}
			printf("\n");
		}
		if (CAR_WATERFALL) {
			sleep_for(nanoseconds(REFRESH_NANO_SEC));
			std::cout << "\x1B[2J\x1B[H";
		}
	}
			  break;
	case 'v': {
		for (int blockPos = START_BLOCK; blockPos < END_BLOCK; blockPos++) {
			printf("%5d ", blockPos);
			for (int lane = 0; lane < NUM_LANE; lane++) {
				if ((road[lane][blockPos] == NULL) ||
					(road[lane][blockPos]->getBlockPos() != blockPos)) {
					printf("     ");
				}
				else {
					printf("%4.1f ", road[lane][blockPos]->getV());
				}
			}
			printf("\n");
		}
		if (CAR_WATERFALL) {
			sleep_for(nanoseconds(REFRESH_NANO_SEC));
			std::cout << "\x1B[2J\x1B[H";
		}
	}
			  break;
	case 's': {
		for (int blockPos = START_BLOCK; blockPos < END_BLOCK; blockPos++) {
			printf("%5d ", blockPos);
			for (int lane = 0; lane < NUM_LANE; lane++) {
				if ((road[lane][blockPos] == NULL) ||
					(road[lane][blockPos]->getBlockPos() != blockPos)) {
					printf("     ");
				}
				else {
					printf("%4.1f ", road[lane][blockPos]->getS());
				}
			}
			printf("\n");
		}
		if (CAR_WATERFALL) {
			sleep_for(nanoseconds(REFRESH_NANO_SEC));
			std::cout << "\x1B[2J\x1B[H";
		}
	}
			  break;

	}
}