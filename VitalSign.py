# author: kylin, cql22@mails.tsinghua.edu.cn
import serial
import time
import numpy as np
import struct
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets
from sklearn.cluster import dbscan
import pandas
from mmVS.com import serialConfig, parseConfigFile
import sys

configFileName = 'profiles/xwr6843_profile_VitalSigns_20fps_Front.cfg'

CLIport = {}
Dataport = {}
byteBuffer = np.zeros(2 ** 15, dtype='uint8')
byteBufferLength = 0
numRangeBinProcessed = 33 - 11 + 1
Breathsignal = list(range(0, 250))
Heartbeatsignal = list(range(0, 250))
Chestdisplacement = [0]*250
Rangeprofile = [0]*250
Breathenerge = [0]*250
Heartenerge = [0]*250

def readAndParseData68xx(Dataport, configParameters):
    global byteBuffer, byteBufferLength, numRangeBinProcessed
    OBJ_STRUCT_SIZE_BYTES = 12
    BYTE_VEC_ACC_MAX_SIZE = 2 ** 15
    MMWDEMO_UART_MSG_DETECTED_POINTS = 1
    MMWDEMO_UART_MSG_RANGE_PROFILE = 2
    MMWDEMO_UART_MSG_VITALSIGN = 6
    maxBufferSize = 2 ** 15
    tlvHeaderLengthInBytes = 8
    pointLengthInBytes = 16
    magicWord = [2, 1, 4, 3, 6, 5, 8, 7]

    magicOK = 0
    dataOK = 0
    frameNumber = 0
    vitalsign = {}

    readBuffer = Dataport.read(Dataport.in_waiting)
    byteVec = np.frombuffer(readBuffer, dtype='uint8')
    byteCount = len(byteVec)
    #if len(readBuffer):
    #    print(readBuffer)
    #    print(byteVec)
    #    print(byteCount)
    #    print("💩\n")
    if (byteBufferLength + byteCount) < maxBufferSize:
        byteBuffer[byteBufferLength:(byteBufferLength + byteCount)] = byteVec[0:byteCount]
        byteBufferLength = byteBufferLength + byteCount

    if byteBufferLength > 16:

        possibleLocs = np.where(byteBuffer == magicWord[0])[0]
        startIdx = []
        for loc in possibleLocs:
            check = byteBuffer[loc:loc + 8]
            if np.all(check == magicWord):
                startIdx.append(loc)

        if startIdx:
            if 0 < startIdx[0] < byteBufferLength:
                byteBuffer[:byteBufferLength - startIdx[0]] = byteBuffer[startIdx[0]:byteBufferLength]
                byteBuffer[byteBufferLength - startIdx[0]:] = np.zeros(len(byteBuffer[byteBufferLength - startIdx[0]:]),
                                                                       dtype='uint8')
                byteBufferLength = byteBufferLength - startIdx[0]

            if byteBufferLength < 0:
                byteBufferLength = 0
            if byteBufferLength < 16:
                return dataOK, None, None
            totalPacketLen = int.from_bytes(byteBuffer[12:12 + 4], byteorder='little')
            if (byteBufferLength >= totalPacketLen) and (byteBufferLength != 0):
                magicOK = 1
    if magicOK:
        idX = 0
        magicNumber = byteBuffer[idX:idX + 8]
        idX += 8
        version = format(int.from_bytes(byteBuffer[idX:idX + 4], byteorder='little'), 'x')
        idX += 4
        totalPacketLen = int.from_bytes(byteBuffer[idX:idX + 4], byteorder='little')
        idX += 4
        platform = format(int.from_bytes(byteBuffer[idX:idX + 4], byteorder='little'), 'x')
        idX += 4
        frameNumber = int.from_bytes(byteBuffer[idX:idX + 4], byteorder='little')
        idX += 4
        timeCpuCycles = int.from_bytes(byteBuffer[idX:idX + 4], byteorder='little')
        idX += 4
        vitalsign["numDetectedObj"] = numDetectedObj = int.from_bytes(byteBuffer[idX:idX + 4], byteorder='little')
        idX += 4
        numTLVs = int.from_bytes(byteBuffer[idX:idX + 4], byteorder='little')
        idX += 4
        subFrameNumber = int.from_bytes(byteBuffer[idX:idX + 4], byteorder='little')
        idX += 4
        for tlvIdx in range(numTLVs):
            tlv_type = int.from_bytes(byteBuffer[idX:idX + 4], byteorder='little')
            idX += 4
            tlv_length = int.from_bytes(byteBuffer[idX:idX + 4], byteorder='little')
            idX += 4
            if tlv_type == MMWDEMO_UART_MSG_VITALSIGN:
                vitalsign["rangeBinIndexMax"] = int.from_bytes(byteBuffer[idX:idX + 2], byteorder='little')
                idX += 2
                vitalsign["rangeBinIndexPhase"] = int.from_bytes(byteBuffer[idX:idX + 2], byteorder='little')
                idX += 2
                vitalsign["maxVal"] = struct.unpack('<f', byteBuffer[idX:idX + 4])[0]
                idX += 4
                vitalsign["processingCyclesOut"] = int.from_bytes(byteBuffer[idX:idX + 4], byteorder='little')
                idX += 4
                vitalsign["rangeBinStartIndex"] = int.from_bytes(byteBuffer[idX:idX + 2], byteorder='little')
                idX += 2
                vitalsign["rangeBinEndIndex"] = int.from_bytes(byteBuffer[idX:idX + 2], byteorder='little')
                idX += 2
                vitalsign["unwrapPhasePeak_mm"] = byteBuffer[idX:idX + 4].view(dtype=np.float32)[0]
                idX += 4
                vitalsign["outputFilterBreathOut"] = struct.unpack('<f', byteBuffer[idX:idX + 4])[0]
                idX += 4
                vitalsign["outputFilterHeartOut"] = struct.unpack('<f', byteBuffer[idX:idX + 4])[0]
                idX += 4
                vitalsign["heartRateEst_FFT"] = struct.unpack('<f', byteBuffer[idX:idX + 4])[0]
                idX += 4
                vitalsign["heartRateEst_FFT_4Hz"] = struct.unpack('<f', byteBuffer[idX:idX + 4])[0] / 2
                idX += 4
                vitalsign["heartRateEst_xCorr"] = struct.unpack('<f', byteBuffer[idX:idX + 4])[0]
                idX += 4
                vitalsign["heartRateEst_peakCount"] = struct.unpack('<f', byteBuffer[idX:idX + 4])[0] # zero always
                idX += 4
                vitalsign["breathingRateEst_FFT"] = struct.unpack('<f', byteBuffer[idX:idX + 4])[0]
                idX += 4
                vitalsign["breathingRateEst_xCorr"] = struct.unpack('<f', byteBuffer[idX:idX + 4])[0]
                idX += 4
                vitalsign["breathingRateEst_peakCount"] = struct.unpack('<f', byteBuffer[idX:idX + 4])[0]
                idX += 4
                vitalsign["confidenceMetricBreathOut"] = struct.unpack('<f', byteBuffer[idX:idX + 4])[0]
                idX += 4
                vitalsign["confidenceMetricBreathOut_xCorr"] = struct.unpack('<f', byteBuffer[idX:idX + 4])[0]
                idX += 4
                vitalsign["confidenceMetricHeartOut"] = struct.unpack('<f', byteBuffer[idX:idX + 4])[0]
                idX += 4
                vitalsign["confidenceMetricHeartOut_4Hz"] = struct.unpack('<f', byteBuffer[idX:idX + 4])[0]
                idX += 4
                vitalsign["confidenceMetricHeartOut_xCorr"] = struct.unpack('<f', byteBuffer[idX:idX + 4])[0]
                idX += 4
                vitalsign["sumEnergyBreathWfm"] = struct.unpack('<f', byteBuffer[idX:idX + 4])[0]
                idX += 4
                vitalsign["sumEnergyHeartWfm"] = struct.unpack('<f', byteBuffer[idX:idX + 4])[0]
                idX += 4
                vitalsign["motionDetectedFlag"] = struct.unpack('<f', byteBuffer[idX:idX + 4])[0]
                idX += 4
                idX += 40
                vitalsign["RPlength"]=struct.unpack('<f', byteBuffer[idX:idX + 4])[0]
                dataOK = 1

            if tlv_type == MMWDEMO_UART_MSG_RANGE_PROFILE:
                if vitalsign.__contains__("rangeBinEndIndex"):
                    numRangeBinProcessed = vitalsign["rangeBinEndIndex"]-vitalsign["rangeBinStartIndex"]+1
                vitalsign["RangeProfile"] = []
                for i in range(numRangeBinProcessed):
                    RPrealpart = int.from_bytes(byteBuffer[idX:idX + 2], byteorder='big')
                    idX += 2
                    RPimagelpart = int.from_bytes(byteBuffer[idX:idX + 2], byteorder='big')
                    idX += 2
                    vitalsign["RangeProfile"].append(pow(RPrealpart*RPrealpart+RPimagelpart*RPimagelpart,0.5))
        if 0 < idX < byteBufferLength:
            shiftSize = totalPacketLen

            byteBuffer[:byteBufferLength - shiftSize] = byteBuffer[shiftSize:byteBufferLength]
            byteBuffer[byteBufferLength - shiftSize:] = np.zeros(len(byteBuffer[byteBufferLength - shiftSize:]),
                                                                 dtype='uint8')
            byteBufferLength = byteBufferLength - shiftSize
            if byteBufferLength < 0:
                byteBufferLength = 0

    return dataOK, frameNumber, vitalsign

def update():
    global vitalsign
    global s1, s2, s3, s4, s5, s6
    global Breathsignal
    global Heartbeatsignal
    global Chestdisplacement
    global Rangeprofile
    global Breathenerge
    global Heartenerge
    dataOk, frameNumber, vitalsign = readAndParseData68xx(Dataport, configParameters)
    if dataOk:
        #print(f"Got {frameNumber} Data!\n")
        Breathsignal.append(vitalsign["outputFilterBreathOut"])
        Heartbeatsignal.append(vitalsign["outputFilterHeartOut"])
        Chestdisplacement.append(float(vitalsign["unwrapPhasePeak_mm"]))
        Breathenerge.append(float(vitalsign["sumEnergyBreathWfm"])/1e6)
        Heartenerge.append(float(vitalsign["sumEnergyHeartWfm"]))

        if vitalsign.__contains__("RangeProfile"):
            Rangeprofile=vitalsign["RangeProfile"]
        if len(Breathsignal) > 250:
            Breathsignal.pop(0)
        if len(Heartbeatsignal) > 250:
            Heartbeatsignal.pop(0)
        if len(Chestdisplacement) > 250:
            Chestdisplacement.pop(0)
        if len(Breathenerge) > 250:
            Breathenerge.pop(0)
        if len(Heartenerge) > 250:
            Heartenerge.pop(0)
        s1.setData(np.array((list(range(0, 250)), Breathsignal)).T)
        s2.setData(np.array((list(range(0, 250)), Heartbeatsignal)).T)
        s3.setData(np.array((list(range(0, 250)), Chestdisplacement)).T)
        s4.setData(np.array((list(np.arange(0, numRangeBinProcessed*configParameters["rangeResolutionMeters"], configParameters["rangeResolutionMeters"])), Rangeprofile)).T)
        s5.setData(np.array((list(range(0, 250)), Breathenerge)).T)
        s6.setData(np.array((list(range(0, 250)), Heartenerge)).T)
        labelItem1.setText(text='Breath Rate:' + str(vitalsign["breathingRateEst_FFT"]), size='12pt', color='#000000')
        labelItem2.setText(text='Heart Rate:' + str(vitalsign["heartRateEst_FFT"]), size='12pt', color='#000000')
        QtWidgets.QApplication.processEvents()

    return dataOk

# Configurate the serial port
CLIport, Dataport = serialConfig(configFileName)

# Get the configuration parameters from the configuration file
configParameters = parseConfigFile(configFileName)

# START QtAPPfor the plot
app = QtWidgets.QApplication([])

# Set the plot
pg.setConfigOption('background', 'w')
win = pg.GraphicsLayoutWidget(show=True, title="Vital Sign")
#win = pg.GraphicsLayoutWidget(show=False, title="Vital Sign")
#win=QtWidgets.QWidget()
win.resize(1200, 700)
p1 = win.addPlot(row=1, col=0)
p1.setTitle("Breathing Waveform", color=(0, 128, 128),size='12pt')
p1.setXRange(0, 250)
p1.setYRange(-2, 2)
p1.setLabel('left', text='Position (mm)')
p1.setLabel('bottom', text='Time (pre 50ms)')
ticks = list(("10s","20s","30s","40s"))
ax = p1.getAxis("bottom")
#ax.setTicks(ticks)
PEN = pg.mkPen(width=2, color='r')
s1 = p1.plot([], [], pen=PEN)

labelItem1 = pg.LabelItem(text='Breath Rate:')
win.addItem(labelItem1, row=0, col=0)

p2 = win.addPlot(row=1, col=1)
p2.setTitle("Cardiac Waveform", color='#008080',size='12pt')
p2.setXRange(0, 250)
p2.setYRange(-2, 2)
p2.setLabel('left', text='Position (mm)')
p2.setLabel('bottom', text='Time (pre 50ms)')
PEN = pg.mkPen(width=2, color='r')
s2 = p2.plot([], [], pen=PEN)

labelItem2 = pg.LabelItem(text='Heartbeat Rate:')
win.addItem(labelItem2, row=0, col=1)

p3 = win.addPlot(row=2, col=0)
p3.setTitle("Chest Displacement", color='#008080',size='12pt')
p3.setXRange(0, 250)
# p3.setYRange(-100, 100)
p3.setLabel('left', text='Displacement (a.u.)')
p3.setLabel('bottom', text='Frame (pre index)')
PEN = pg.mkPen(width=2, color='r')
s3 = p3.plot([], [], pen=PEN)

p4 = win.addPlot(row=2, col=1)
p4.setTitle("Range Profile", color='#008080',size='12pt')
# p4.setXRange(0, numRangeBinProcessed*configParameters["rangeResolutionMeters"])
# p4.setYRange(0, 20000)
p4.setLabel('left', text='Magnitude (a.u.)')
p4.setLabel('bottom', text='Range (m)')
p4.setYRange(0, 100000, padding=0)
PEN = pg.mkPen(width=2, color='r')
s4 = p4.plot([], [], pen=PEN)

p5 = win.addPlot(row=3, col=0)
p5.setTitle("Breath Energy", color='#008080',size='12pt')
p5.setXRange(0, 250)
# p5.setYRange(0, 20000)
p5.setLabel('left', text='Wave Energy (a.u.10^6)')
p5.setLabel('bottom', text='Time (pre 50ms)')
PEN = pg.mkPen(width=2, color='r')
s5 = p5.plot([], [], pen=PEN)

p6 = win.addPlot(row=3, col=1)
p6.setTitle("Cardiac Energy", color='#008080',size='12pt')
p6.setXRange(0, 250)
# p5.setYRange(0, 20000)
p6.setLabel('left', text='Wave Energy (a.u.)')
p6.setLabel('bottom', text='Time (pre 50ms)')
PEN = pg.mkPen(width=2, color='r')
s6 = p6.plot([], [], pen=PEN)

labelItem3 = pg.LabelItem(text='Range Start:')
win.addItem(labelItem3, row=5, col=0)
labelItem3.setText(text='Range Start:' + str(configParameters["rangeStart"]) + " m", size='12pt', color='#000000')

labelItem4 = pg.LabelItem(text='Range End:')
win.addItem(labelItem4, row=5, col=1)
labelItem4.setText(text='Range End:' + str(configParameters["rangeEnd"]) + " m", size='12pt', color='#000000')

labelItem5 = pg.LabelItem(text='Max Range:')
win.addItem(labelItem5, row=6, col=0)
labelItem5.setText(text='Max Range:' + str(round(configParameters["maxRange"],2)) + " m", size='12pt', color='#000000')

labelItem6 = pg.LabelItem(text='Range Resolution Meters:')
win.addItem(labelItem6, row=6, col=1)
labelItem6.setText(text='Range Resolution:' + str(round(configParameters["rangeResolutionMeters"]*100,2)) + " cm", size='12pt', color='#000000')

detObj = {}
frameData = {}
currentIndex = 0

#pg.exec()
while True:
    try:
        # Update the data and check if the data is okay
        dataOk = update()

        # time.sleep(0.04)

    except KeyboardInterrupt:
        CLIport.write(('sensorStop\n').encode())
        print('sensorStop\n')
        CLIport.close()
        Dataport.close()
        win.close()
        break
