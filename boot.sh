#! /bin/bash
edm -x -eolc -noedit -m "P=mecaRobot,R=''" edl/mecaRobotDiff.edl &
python -i mecaRobot.py