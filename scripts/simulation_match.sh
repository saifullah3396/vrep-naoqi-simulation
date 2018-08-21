#!/bin/bash

trap killgroup SIGINT

killgroup() {
  echo killing...
  kill 0
}

$PATH_TO_VREP_DIR/vrep.sh ../scenes/MatchRobocup.ttt -s
