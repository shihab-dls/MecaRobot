#! /bin/bash
edm -x -eolc -noedit -one -m "P=mecaRobot,R=''" edl/mecaRobotDiff.edl &
python -i mecaRobot.py