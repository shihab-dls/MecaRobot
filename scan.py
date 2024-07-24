import os
import shutil
from ophyd import Device, EpicsSignal, EpicsSignalRO
from ophyd import Component as Cpt
from ophyd.utils import set_and_wait
from bluesky.plan_stubs import mv
from bluesky import RunEngine

RE = RunEngine()

class Robot(Device):
    connect = Cpt(EpicsSignal, 'Connect')
    activate = Cpt(EpicsSignal, 'Activate')
    home = Cpt(EpicsSignal, 'Home')
    buffer_step = Cpt(EpicsSignal, 'BufferSize')
    crossover = Cpt(EpicsSignal, 'BufferCross')
    frequency = Cpt(EpicsSignal, 'Frequency')
    file = Cpt(EpicsSignal, 'IKfile')
    parse = Cpt(EpicsSignal, 'Parse.PROC')
    buffer = Cpt(EpicsSignal, 'BufferAll.PROC')
    
    checkpoint = Cpt(EpicsSignalRO, 'Checkpoint')

class Camera(Device):
    acquire = Cpt(EpicsSignal, 'CAM:Acquire')
    path = Cpt(EpicsSignal, 'HDF5:FilePath')
    mode = Cpt(EpicsSignal, 'HDF5:FileWriteMode')
    autosave = Cpt(EpicsSignal, 'HDF5:AutoSave')
    filename = Cpt(EpicsSignal, 'HDF5:FileNumber')
    form = Cpt(EpicsSignal, 'HDF5:FileTemplate')
    capture = 1

my_robot = Robot('mecaRobot:', name = 'my_robot', read_attrs = ["checkpoint"])
my_camera = Camera('LA84R-DI-DCAM-01:', name = 'my_camera')

def prepare():
    path = f'{os.getcwd()}/data/temp'
    shutil.rmtree(path)
    os.makedirs(path, exist_ok=True)
    os.chmod(path, 0o777)
    my_camera.path.put(path)
    yield from mv(my_camera.mode, 'Single')
    yield from mv(my_camera.autosave, 'Yes')
    yield from mv(my_robot.connect, 1)
    yield from mv(my_robot.activate, 1)
    yield from mv(my_robot.home, 1)

def capture(**kwargs):
    if my_camera.capture:
        my_camera.filename.put(f'{int(my_robot.checkpoint.get())}')
        my_camera.acquire.put(1)

def scan(file,frequency = 10, buffer_step = 100, crossover = 1, capture = 0):
    my_camera.capture = capture
    yield from mv(my_robot.file, file)
    yield from mv(my_robot.frequency, frequency)
    yield from mv(my_robot.buffer_step, buffer_step)
    yield from mv(my_robot.crossover, crossover)
    yield from mv(my_robot.parse, 1)
    yield from mv(my_robot.buffer, 1)

my_robot.checkpoint.subscribe(capture)