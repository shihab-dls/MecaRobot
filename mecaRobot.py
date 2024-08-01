# Import the basic framework components.
from softioc import softioc, builder
from cothread.catools import caget, caput, camonitor
from softioc.builder import records
import cothread
import socket
import re
import time
import csv
import numpy as np

# Record Prefix
builder.SetDeviceName("mecaRobot")
 
# Records

## Initialisation
address = builder.stringOut('Address', initial_value="172.23.17.169")
port = builder.aOut('Port', initial_value=10000)
monport = builder.aOut('MonPort', initial_value=10001)
ikFile = builder.stringOut('IKfile')

## Status
homeStatus = builder.aOut('HomeStatus', initial_value=0)
activateStatus = builder.aOut('ActivateStatus', initial_value=0)
parseStatus = builder.aOut("ParseStatus", initial_value=0)
errorStatus = builder.aOut("Error", initial_value=0)

## Read Back Values
checkPoint = builder.aIn('Checkpoint', initial_value=0)
terminal = builder.stringOut("Terminal")

## Control
connectTrig = builder.aOut("Connect", initial_value=0)
activate = builder.aOut('Activate', initial_value=0)

home = builder.aOut('Home', initial_value=0)
abort = builder.aOut('Abort', initial_value=0)
reset = builder.aOut('Reset', initial_value=0)
pause = builder.aOut('Pause', initial_value=0)

vel = builder.aOut("Vel", EGU="%", initial_value=5)
acc = builder.aOut("Acc", EGU="%", initial_value=100)
blend = builder.aOut("Blend", EGU="%", initial_value=100)
frequency = builder.aOut("Frequency", EGU="Hz", initial_value=10)

parse = builder.aOut('Parse', initial_value=0)
bufferSize = builder.aOut("BufferSize", initial_value=2)
crossover = builder.aOut("BufferCross", initial_value=1)
bufferGroup = builder.aOut("BufferGroup", initial_value=1)
buffer = builder.aOut("Buffer")
bufferAll = builder.aOut("BufferAll")
getjoints = builder.aOut("GetJoints", initial_value=1)

# Boilerplate to get the IOC started
builder.LoadDatabase()
softioc.iocInit()
 
class meca500:
 
    def __init__(self):
        self.commands = ""
        self.buffer = [0,0]
        self.crossover = 0
        self.sock = None
        self.sockMon = None
        self.activation_status = 0
        self.homing_status = 0
        self.error_status = 0
        self.pause_status = 1
        self.vel = vel.get()
        self.acc = acc.get()
        self.blend = blend.get()
        self.current = ""
        self.freq = 0
 
    def isConnect(self,*args):
        if connectTrig.get():
            self.Connect()
            self.ConnectStatus()
        else:
            self.Disconnect()
    
    def isActivate(self,*args):
        if activate.get():
            self.Activate()
        else:
            self.Deactivate()
    
    def isPause(self,*args):
        if pause.get():
            self.Pause()
        else:
            self.Resume()

    def Connect(self):  # Connect to meca500 control port
        server_addr = (address.get(), int(port.get()))
        terminal.set(f">Connecting to {server_addr[0]}:{server_addr[1]}<")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setblocking(True)  # Sets socket to blocking mode
        self.sock.connect_ex(server_addr)
        self.sock.setblocking(False)  # Sets socket to blocking mode
        self.SetAcc(); self.SetVel(); self.SetBlen()


    def ConnectStatus(self):  # Connect to meca500 monitoring port
        server_addr = (address.get(), int(monport.get()))
        self.sockMon = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sockMon.setblocking(True)  # Sets socket to blocking mode
        self.sockMon.connect_ex(server_addr)
        self.sockMon.setblocking(False)  # Sets socket to blocking mode

    def Disconnect(self):  # Close Socket
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
            self.sockMon.shutdown(socket.SHUT_RDWR)
        except OSError as e:
            terminal.set(f">Socket shutdown error")
            print(e)

        try:
            self.sock.close()
            self.sockMon.close()
            terminal.set(">Socket closed<")
        except OSError as e:
            terminal.set(f">Socket close error<")
            print(e)
    
    def Send(self, command):
        try:
            self.sock.send(bytes(command + "\0", 'ascii'))  # Send cmd to meca
        except:
            connectTrig.set(0)  # If error, represent need for Connect()
            terminal.set(f">Cannot send through socket<")


    def SetVel(self,*args):  # Limit joint velocity
        if connectTrig.get():
            self.Send(f"SetJointVel({vel.get()})")
            self.vel = vel.get()
        else:
            vel.set(self.vel)
            self.SetError("Velocity","Disconnected")

    def SetAcc(self,*args):  # Limit joint acceleration
        if connectTrig.get():
            self.Send(f"SetJointAcc({acc.get()})")
            self.acc = acc.get()
        else:
            acc.set(self.acc)
            self.SetError("Acceleration","Disconnected")

    def SetBlen(self,*args):  # Set blending amount
        if connectTrig.get():
            self.Send(f"SetBlending({blend.get()})")
            self.blend = blend.get()
        else:
            blend.set(self.blend)
            self.SetError("Blending","Disconnected")

    def TimeBase(self, jfile, step):
        initial_vel = vel.get()
        j = [[] for i in range(6)]
        vels = [150,150,180,300,300,500]

        with open(jfile, 'r') as file:
            data = csv.reader(file)
            for row in data:
                for i in range(6):
                    j[i].append(float(row[i]))

        lims = []

        for i in range(len(j[0])-1):
            tmax = 0
            for ii in range(6):
                t = abs(j[ii][i]-j[ii][i+1])/vels[ii]
                if t > tmax:
                    tmax = t
            lim = tmax/(step-0.00002)
            lims.append(lim)
        lims.append(initial_vel)
        return lims

    def Parse(self,*args):  # Parse a trajectory file into motion commands
        try:
            fileName = f"Trajectories/{ikFile.get()}"
            file = open(fileName, mode='r', encoding='utf-8-sig')
        except IsADirectoryError:
            terminal.set(">Select file<")
            return
        except:
            terminal.set(">File not found<")
            return
        
        lines = file.readlines()
        angles = []
        commands = "MoveJoints(0,0,0,0,0,0)\n"
        point = 0
        file.close()
        lims = self.TimeBase(fileName,(1/frequency.get())/100)

        for line in lines:  # Tokenizes file rows
            line = [i.strip() for i in line]
            angles.append(line)
        
        for points in angles:  # Produces joint commands & via point RBVs
            point += 1
            commands += "MoveJoints("
            for angle in points:
                commands += angle
            commands += ")\n"
            commands = commands + f"SetCheckpoint({point})\n" + f"SetJointVel({lims[point-1]})\n"
            parseStatus.set(1)

        self.commands = commands + "MoveJoints(0,0,0,0,0,0)"  # No more composite commands appended

    def BufferStep(self,*args):  # Send commands in chunks, start to finish, with a step size
        if not self.commands:
            terminal.set(">Parse The File<")
            return
        size = int(bufferSize.get())
        cmd = self.commands
        self.buffer[1] += size  # Move end pointer for correct window size

        # Search for checkpoints at start and end pointers
        bufferStart = re.search(f"SetCheckpoint\({self.buffer[0]}\)",cmd)
        bufferEnd = re.search(f"SetCheckpoint\({self.buffer[1]}\)",cmd)

        # Handle first and last window edge cases
        bufferStart = 0 if not bufferStart else bufferStart.span()[1] + 1
        bufferEnd = len(self.commands[:-1]) if not bufferEnd else bufferEnd.span()[1]

        # Restart pointers if all commands buffered
        if bufferEnd != len(self.commands[:-1]):
            self.crossover = self.buffer[1] - crossover.get()
            self.buffer[0] = self.buffer[1]
            bufferGroup.set(bufferGroup.get()+1)
        else:
            self.crossover = 0
            self.buffer = [0,0]
            bufferGroup.set(1)

        # Produce command
        cmd = self.commands[bufferStart:bufferEnd]
        print(cmd.strip(" "))
        self.Send(cmd)
        terminal.set(">Moving Through Steps<")

    def BufferAll(self,*args): 
        if not self.commands:
            terminal.set(">Parse The File<")
            return
        self.Send(self.commands)  # Send entire list of parsed commands
        self.crossover = 0  # Set cross over to erroneous checkpoint, such that Bufferstep() not triggered
        terminal.set(">Moving Through All<")
     
    def Activate(self): 
        self.sockMon.setblocking(True)  # Blocking set true to allow for subsequent commands to queue (such as hitting activate then immedietly home)
        self.Send("ActivateRobot")  # Activate meca
        self.sockMon.setblocking(False)  # Blocking set false
    
    def Deactivate(self): 
        self.Send("ClearMotion")  # Abort any residual motion commands
        cothread.Sleep(1)  # Delay to ensure clear command is executed first
        self.Send("DeactivateRobot")  # Deactivate meca
 
    def Home(self,*args): self.Send("Home")  # Calibrate meca; must be done upon activation

    def Reset(self,*args): self.Send("ResetError")  # If in error, reset state
    
    def Resume(self,*args): self.Send("ResumeMotion")  # Resume motion
    
    def Pause(self,*args): self.Send("PauseMotion")  # Pause motion

    def SetError(self,param,err): terminal.set(f">Cannot Set {param}: {err}<")
    
    def Abort(self,*args): 
        self.Send("ClearMotion")  # Erase all commands from meca buffer
        # Resets buffer pointers for BufferStep() method
        self.buffer = [0,0]
        bufferGroup.set(1);pause.set(1)

    def AbortLocal(self,*args): # Erase all commands in class buffer 
        self.commands = ""
        parseStatus.set(0)

rb = meca500()  # Instantiate object of meca
rb.Connect()
rb.ConnectStatus()
rb.Send("SetMonitoringInterval(0.001)")  # Sets monitoring port frequency
rb.Send("SetRealTimeMonitoring(2210)")  # Sends constant flux of real joint positions and time (encoder-based)

def Listener():  # Handles checkpoint meca responses
    while True:
        try:
            response = rb.sock.recv(1024).decode('ascii')
            matchCheck = re.search(r'\[3030\]\[(\d+)\]', response)  # Checkpoint RBVs?
            if matchCheck:  # If truthy, update PV
                checkPoint.set(int(matchCheck.group(1)))
                if int(matchCheck.group(1)) == rb.crossover:
                    rb.BufferStep()  ## If point == end of buffer step, send next block in
                print(f'{rb.current},')
            else:  # Any other response, print to terminal
                terminal.set(response)
                print(response)
        except:
            pass
        cothread.Sleep(0.0001)  # Short sleep to prevent ring buffer error

def Status():  # Handles status meca responses
    while True:
        try:
            response = rb.sockMon.recv(1024).decode('ascii')
            matchStatus = re.search(r'\[2007\]\[(\d+(?:,\d+)*)\]', response)  # Status RBVs?
            matchJoints = re.search(r'\[2210\]\[(.*?)\]', response)  # Joint RBVs?
            if matchStatus:  # If truthy, update class attributes
                status = matchStatus.group(1).split(",")
                rb.activation_status = int(status[0])
                rb.homing_status = int(status[1])
                rb.error_status = int(status[3])
                rb.pause_status = int(status[4])
                activateStatus.set(rb.activation_status)
                activate.set(rb.activation_status)
                homeStatus.set(rb.homing_status)
                errorStatus.set(rb.error_status)
                pause.set(rb.pause_status)
            elif matchJoints:
                rb.current = matchJoints.group(1)
                #print(f'{rb.current},')
        except:
            pass
        cothread.Sleep(0.0001)  # Short sleep to prevent ring buffer error

cothread.Spawn(Listener)  # Spin Listener in a thread
cothread.Spawn(Status)  # Spin Status in a thread
 
camonitor("mecaRobot:Connect",rb.isConnect)
camonitor("mecaRobot:Parse.PROC",rb.Parse)
camonitor("mecaRobot:IKfile",rb.AbortLocal)
camonitor("mecaRobot:Frequency",rb.AbortLocal)
camonitor("mecaRobot:Vel",rb.SetVel)
camonitor("mecaRobot:Acc",rb.SetAcc)
camonitor("mecaRobot:Blend",rb.SetBlen)
camonitor("mecaRobot:BufferAll.PROC",rb.BufferAll)
camonitor("mecaRobot:Buffer.PROC",rb.BufferStep)
camonitor("mecaRobot:Activate",rb.isActivate)
camonitor("mecaRobot:Abort.PROC",rb.Abort)
camonitor("mecaRobot:Reset.PROC",rb.Reset)
camonitor("mecaRobot:Home.PROC",rb.Home)
camonitor("mecaRobot:Pause",rb.isPause)


softioc.interactive_ioc(globals())