#!/bin/bash

echo "Starting the CARLA evaluation script."
echo "This may take some time. Sit tight!"

# For description of flags, see:
# https://leaderboard.carla.org/get_started/#22-understanding-the-leaderboard-components
python3 ${LEADERBOARD_ROOT}/leaderboard/leaderboard_evaluator.py \
--port=2014 \
--routes=${LEADERBOARD_ROOT}/data/routes_training.xml \
--routes-subset="10" \
--repetitions=1 \
--track=MAP \
--checkpoint=/navigator/log/checkpoints.json \
--debug-checkpoint=/navigator/log/checkpoints_debug.json \
--agent=/navigator/src/interface/carla_interface/carla_interface/carla_agent.py \
--debug=1 \
--record=/navigator/log/carla \
# --agent-config=${TEAM_CONFIG} \
# --resume=${RESUME}