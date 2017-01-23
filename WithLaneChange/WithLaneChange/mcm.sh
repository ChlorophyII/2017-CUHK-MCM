#!/bin/bash
make main
rm result.csv
export NL=4
export SVL=31.30
export HVL=26.82
export SIV=28
export HIV=24
#NUM_LANE, NUM_LANE_SELF, AVG_CAR_PER_SEC, SELF_RATIO, SELF_INIT_V, HUMN_INIT_V, queueSelf.size, queueHumn.size >> result.csv
for (( i = 0; i < 4; i++ )); do
    for (( j = 0; j < 100; j++ )); do
        for (( k = 10; k < 50; k++ )); do
            export SL=$i
            export SR=$j
            export CR=$k
            ./main
        done
    done
done
# ./mcm.sh
