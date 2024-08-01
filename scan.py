import os
import shutil
from ophyd import Device, EpicsSignal, EpicsSignalRO
from ophyd import Component as Cpt
from ophyd.utils import set_and_wait
from bluesky.plan_stubs import mv
from bluesky import RunEngine
import numpy as np
import statistics as st

RE = RunEngine()

class Robot(Device):
    connect = Cpt(EpicsSignal, 'Connect')
    activate = Cpt(EpicsSignal, 'Activate')
    home = Cpt(EpicsSignal, 'Home.PROC')
    buffer_step = Cpt(EpicsSignal, 'BufferSize')
    crossover = Cpt(EpicsSignal, 'BufferCross')
    frequency = Cpt(EpicsSignal, 'Frequency')
    file = Cpt(EpicsSignal, 'IKfile')
    parse = Cpt(EpicsSignal, 'Parse.PROC')
    buffer = Cpt(EpicsSignal, 'BufferAll.PROC')
    reset = Cpt(EpicsSignal, 'Reset.PROC')
    
    checkpoint = Cpt(EpicsSignalRO, 'Checkpoint')
    var = 0

class Camera(Device):
    acquire = Cpt(EpicsSignal, 'CAM:Acquire')
    path = Cpt(EpicsSignal, 'HDF5:FilePath')
    mode = Cpt(EpicsSignal, 'HDF5:FileWriteMode')
    autosave = Cpt(EpicsSignal, 'HDF5:AutoSave')
    filename = Cpt(EpicsSignal, 'HDF5:FileName')
    capture_num = Cpt(EpicsSignal, 'HDF5:NumCapture')
    capture_trig = Cpt(EpicsSignal, 'HDF5:Capture')
    capture = 0

my_robot = Robot('mecaRobot:', name = 'my_robot', read_attrs = ["checkpoint"])
my_camera = Camera('LA84R-DI-DCAM-01:', name = 'my_camera')

def prepare():
    path = f'{os.getcwd()}/data/temp'
    shutil.rmtree(path)
    os.makedirs(path, exist_ok=True)
    os.chmod(path, 0o777)
    my_camera.path.put(path)
    yield from mv(my_camera.mode, 'Capture')
    yield from mv(my_camera.autosave, 'Yes')
    yield from mv(my_robot.connect, 1)
    yield from mv(my_robot.activate, 1)
    yield from mv(my_robot.home, 1)

def capture(**kwargs):
    if my_camera.capture:
        my_camera.acquire.put(1)

def scan(file,frequency = 10, buffer_step = 100, crossover = 1, capture = 0, capture_num = 200):
    my_camera.capture = capture
    yield from mv(my_camera.capture_num, capture_num)
    yield from mv(my_camera.capture_trig, 1)
    yield from mv(my_robot.reset, 1)
    yield from mv(my_robot.file, file)
    my_camera.filename.put(file.split(".")[0])
    yield from mv(my_robot.frequency, frequency)
    yield from mv(my_robot.buffer_step, buffer_step)
    yield from mv(my_robot.crossover, crossover)
    yield from mv(my_robot.parse, 1)
    yield from mv(my_robot.buffer, 1)

my_robot.checkpoint.subscribe(capture)
