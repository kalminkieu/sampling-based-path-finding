#!/bin/bash

NUM_MAPS=5
OUTPUT_DIR="/tmp/path_finder_logs"
mkdir -p $OUTPUT_DIR

for ((i=1; i<=NUM_MAPS; i++))
do
  echo "Generating map $i"
  ./build.sh
  source devel/setup.bash
  roslaunch path_finder test_planners.launch > $OUTPUT_DIR/test_planners_map$i.log 2>&1 &
  LAUNCH_PID=$!
  wait $LAUNCH_PID
  mv /tmp/brrt_multi_map_results.csv /tmp/brrt_multi_map_results_map$i.csv 2>/dev/null || true
done

echo "All tests completed. Terminal logs saved in $OUTPUT_DIR/test_planners_map*.log"
echo "Results saved in /tmp/brrt_multi_map_results_map*.csv"