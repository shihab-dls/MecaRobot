#! /bin/bash
source venv/bin/activate
edm -x -eolc -one -m -noedit "P=mecaRobot,R=''" edl/mecaRobotDiff.edl &
python -i mecaRobot.py
deactivate