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
import math

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
missed = builder.stringOut("Missed")
freqlim = builder.stringOut("FreqLimits")

## Control
connectTrig = builder.aOut("Connect", initial_value=0)
activate = builder.aOut('Activate', initial_value=0)

home = builder.aOut('Home', initial_value=0)
abort = builder.aOut('Abort', initial_value=0)
reset = builder.aOut('Reset', initial_value=0)
pause = builder.aOut('Pause', initial_value=0)

vel = builder.aOut("Vel", EGU="%", initial_value=10, DRVL=0.001, DRVH=100)
acc = builder.aOut("Acc", EGU="%", initial_value=100, DRVL=0.001, DRVH=100)
blend = builder.aOut("Blend", EGU="%", initial_value=100, DRVL=0, DRVH=100)
frequency = builder.aOut("Frequency", EGU="Hz", initial_value=0.01, DRVL=0.0021)

parse = builder.aOut('Parse', initial_value=0)
bufferSize = builder.aOut("BufferSize", initial_value=100, DRVL=2)
buffer = builder.aOut("Buffer")
bufferAll = builder.aOut("BufferAll")

# Boilerplate to get the IOC started
builder.LoadDatabase()
softioc.iocInit()
 
class meca500:
 
    def __init__(self):
        self.commands = []
        self.buffer = 0
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
        self.history = []
        self.missed = []
        self.group = -1
 
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
        j = [[] for i in range(6)]
        vels = [150,150,180,300,300,500]

        with open(jfile, 'r') as file:
            data = csv.reader(file)
            for row in data:
                for i in range(6):
                    j[i].append(float(row[i]))

        lims = []
        lower = [0,0]
        upper = [0,0]

        for i in range(len(j[0])-1):
            tmax = 0
            for ii in range(6):
                t = abs(j[ii][i]-j[ii][i+1])/vels[ii]
                if t > tmax:
                    tmax = t
            lim = round(tmax/(step-0.00002),3)
            lims.append(lim)
            if lim < 0.001:
                if 0.001-lim > lower[0]:
                    lower[0] = 0.001-lim
                    lower[1] = (((tmax / 0.001)+0.00002)*100)
            elif lim > 100:
                if lim - 100 > upper[0]:
                    upper[0] = lim - 100
                    upper[1] = (((tmax / 100)+0.00002)*100)
        
        print(lims)

        if lower[0]:
            if round(lower[1],3) == round(step*100,3):
                lims = self.TimeBase(jfile,(lower[1]-0.00001)/100)
            else:
                freqlim.set(f'Largest Period: {round(lower[1],3)}s')
                frequency.set(round(lower[1],3))
        elif upper[0]:
            if round(upper[1],3) == round(step*100,3):
                lims = self.TimeBase(jfile,(upper[1]+0.00001)/100)
            else:
                freqlim.set(f'Smallest Period: {round(upper[1],3)}s')
                frequency.set(round(upper[1],3))
        
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
        
        freqlim.set("")
        lines = file.readlines()
        angles = []
        commands = []
        self.commands.append("MoveJoints(0,0,0,0,0,0)")
        self.buffer = 0
        self.missed = []
        self.history = []
        point = -1
        file.close()
        lims = self.TimeBase(fileName,(frequency.get())/100)


        for line in lines:  # Tokenizes file rows
            line = [i.strip() for i in line]
            angles.append(line)
        
        for points in angles:  # Produces joint commands & via point RBVs
            point += 1
            command = ""
            command += "MoveJoints("
            for angle in points:
                command += angle
            command += ")\n"
            command = command + f"SetJointVel({lims[point-1]})\n"+ f"SetCheckpoint({(point%7999)+1})"
            self.commands.append(command)
            parseStatus.set(1)

        self.commands.append(f'SetJointVel({vel.get()})\nMoveJoints(0,0,0,0,0,0)')  # No more composite commands appended
    
    def BufferStep(self,*args):  # Send commands in chunks, start to finish, with a step size
        if not self.commands:
            terminal.set(">Parse The File<")
            return
        if self.buffer == 0:
            size = int(bufferSize.get())
            cmd = ""
            self.group = -1
            self.history = []
            for i in range(min(size,len(self.commands))):
                cmd += f'{self.commands[i]}\n'
            self.buffer = i+1
            self.Send(cmd)
        else:
            if self.buffer < len(self.commands):
                self.Send(f'{self.commands[self.buffer]}\n')
                self.buffer += 1
            else:
                self.buffer = 0

        terminal.set(">Moving Through Steps<")

    def BufferAll(self,*args): 
        if not self.commands:
            terminal.set(">Parse The File<")
            return
        self.history = []
        self.group = -1
        cmd = ""
        for command in self.commands:
            cmd += f'{command}\n'
        self.Send(cmd)  # Send entire list of parsed commands
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
        self.buffer = 0
        self.history = []
        self.SetVel()
        pause.set(1)

    def AbortLocal(self,*args): # Erase all commands in class buffer 
        self.commands = []
        parseStatus.set(0)
    
    def MissedPoints(self,*args):
        self.missed = []
        for i in range(len(self.history)):
            if ((i%7999)+1) != self.history[i]:
                last = self.history[i-1]+1 if i else 0
                for j in range(last,self.history[i]):
                    self.missed.append(j)
        missed.set(f'CheckPoints Lost = {self.missed}')

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
            matchEnd = re.search(r'\[3012\]\[(.*)\]', response)  # End of Trajectory?
            if matchCheck:  # If truthy, update PV
                print(f'{rb.current},')
                if int(matchCheck.group(1)) == 1:
                    rb.group += 1
                checkPoint.set(int(matchCheck.group(1))+(rb.group*8000))
                rb.history.append(int(matchCheck.group(1)))
                if rb.buffer:
                    rb.BufferStep()  ## If point == end of buffer step, send next block in
            elif matchEnd:
                rb.MissedPoints()
                print(response)
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