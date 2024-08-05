#! /bin/bash
source venv/bin/activate
edm -x -eolc -one -m "P=mecaRobot,R=''" edl/mecaRobotDiff.edl &
python -i mecaRobot.py
deactivate