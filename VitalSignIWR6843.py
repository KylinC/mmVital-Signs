#
# File:         通过串口读取IWR6843平台的处理得到的心跳与呼吸的波形速率
#

import serial
import time
import numpy as np
import struct
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui
from sklearn.cluster import dbscan
import pandas

# The configuration file
configFileName = 'xwr1642_profile_VitalSigns_20fps_Back.cfg'

CLIport = {}
Dataport = {}
byteBuffer = np.zeros(2 ** 15, dtype='uint8')
byteBufferLength = 0
Breathsignal = list(range(0, 250))
Heartbeatsignal = list(range(0, 250))


# ------------------------------------------------------------------
# Function to configure the serial ports and send the data from
# the configuration file to the radar
def serialConfig(configFileName):
    global CLIport
    global Dataport
    # Open the serial ports for the configuration and the data ports
    CLIport = serial.Serial('COM8', 115200)
    Dataport = serial.Serial('COM7', 921600)

    # Read the configuration file and send it to the board
    config = [line.rstrip('\r\n') for line in open(configFileName)]
    for i in config:
        print(i)
        CLIport.write((i + '\n').encode())
        time.sleep(0.03)

    return CLIport, Dataport


# ------------------------------------------------------------------

# Function to parse the data inside2 the configuration file
def parseConfigFile(configFileName):
    configParameters = {}  # Initialize an empty dictionary to store the configuration parameters

    # Read the configuration file and send it to the board
    config = [line.rstrip('\r\n') for line in open(configFileName)]
    for i in config:

        # Split the line
        splitWords = i.split(" ")

        # Hard code the number of antennas, change if other configuration is used
        numRxAnt = 4
        numTxAnt = 2

        # Get the information about the profile configuration
        if "profileCfg" in splitWords[0]:
            startFreq = int(float(splitWords[2]))
            idleTime = int(splitWords[3])
            rampEndTime = float(splitWords[5])
            freqSlopeConst = float(splitWords[8])
            numAdcSamples = int(splitWords[10])
            numAdcSamplesRoundTo2 = 1

            while numAdcSamples > numAdcSamplesRoundTo2:
                numAdcSamplesRoundTo2 = numAdcSamplesRoundTo2 * 2

            digOutSampleRate = int(splitWords[11])

        # Get the information about the frame configuration
        elif "frameCfg" in splitWords[0]:
            chirpStartIdx = int(splitWords[1])
            chirpEndIdx = int(splitWords[2])
            numLoops = int(splitWords[3])
            numFrames = int(splitWords[4])
            framePeriodicity = float(splitWords[5])

    # Combine the read data to obtain the configuration parameters
    numChirpsPerFrame = (chirpEndIdx - chirpStartIdx + 1) * numLoops
    configParameters["numDopplerBins"] = numChirpsPerFrame / numTxAnt
    configParameters["numRangeBins"] = numAdcSamplesRoundTo2
    configParameters["rangeResolutionMeters"] = (3e8 * digOutSampleRate * 1e3) / (
            2 * freqSlopeConst * 1e12 * numAdcSamples)
    configParameters["rangeIdxToMeters"] = (3e8 * digOutSampleRate * 1e3) / (
            2 * freqSlopeConst * 1e12 * configParameters["numRangeBins"])
    configParameters["dopplerResolutionMps"] = 3e8 / (
            2 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * configParameters["numDopplerBins"] * numTxAnt)
    configParameters["maxRange"] = (300 * 0.9 * digOutSampleRate) / (2 * freqSlopeConst * 1e3)
    configParameters["maxVelocity"] = 3e8 / (4 * startFreq * 1e9 * (idleTime + rampEndTime) * 1e-6 * numTxAnt)

    return configParameters


# ------------------------------------------------------------------

# Funtion to read and parse the incoming data
def readAndParseData18xx(Dataport, configParameters):
    global byteBuffer, byteBufferLength

    # Constants
    OBJ_STRUCT_SIZE_BYTES = 12
    BYTE_VEC_ACC_MAX_SIZE = 2 ** 15
    MMWDEMO_UART_MSG_DETECTED_POINTS = 1
    MMWDEMO_UART_MSG_RANGE_PROFILE = 2
    MMWDEMO_UART_MSG_VITALSIGN = 6
    maxBufferSize = 2 ** 15
    tlvHeaderLengthInBytes = 8
    pointLengthInBytes = 16
    magicWord = [2, 1, 4, 3, 6, 5, 8, 7]

    # Initialize variables
    magicOK = 0  # Checks if magic number has been read
    dataOK = 0  # Checks if the data has been read correctly
    frameNumber = 0
    vitalsign = {}

    readBuffer = Dataport.read(Dataport.in_waiting)
    byteVec = np.frombuffer(readBuffer, dtype='uint8')
    byteCount = len(byteVec)

    # Check that the buffer is not full, and then add the data to the buffer
    if (byteBufferLength + byteCount) < maxBufferSize:
        byteBuffer[byteBufferLength:(byteBufferLength + byteCount)] = byteVec[0:byteCount]
        byteBufferLength = byteBufferLength + byteCount

    # Check that the buffer has some data
    if byteBufferLength > 16:

        # Check for all possible locations of the magic word
        possibleLocs = np.where(byteBuffer == magicWord[0])[0]

        # Confirm that is the beginning of the magic word and store the index in startIdx
        startIdx = []
        for loc in possibleLocs:
            check = byteBuffer[loc:loc + 8]
            if np.all(check == magicWord):
                startIdx.append(loc)

        # Check that startIdx is not empty
        if startIdx:

            # Remove the data before the first start index
            if 0 < startIdx[0] < byteBufferLength:
                byteBuffer[:byteBufferLength - startIdx[0]] = byteBuffer[startIdx[0]:byteBufferLength]
                byteBuffer[byteBufferLength - startIdx[0]:] = np.zeros(len(byteBuffer[byteBufferLength - startIdx[0]:]),
                                                                       dtype='uint8')
                byteBufferLength = byteBufferLength - startIdx[0]

            # Check that there have no errors with the byte buffer length
            if byteBufferLength < 0:
                byteBufferLength = 0

            # Read the total packet length
            totalPacketLen = int.from_bytes(byteBuffer[12:12 + 4], byteorder='little')

            # Check that all the packet has been read
            if (byteBufferLength >= totalPacketLen) and (byteBufferLength != 0):
                magicOK = 1

    # If magicOK is equal to 1 then process the message
    if magicOK:

        # Initialize the pointer index
        idX = 0

        # Read the header
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
        numDetectedObj = int.from_bytes(byteBuffer[idX:idX + 4], byteorder='little')
        idX += 4
        numTLVs = int.from_bytes(byteBuffer[idX:idX + 4], byteorder='little')
        idX += 4
        subFrameNumber = int.from_bytes(byteBuffer[idX:idX + 4], byteorder='little')
        idX += 4

        # Read the TLV messages
        for tlvIdx in range(numTLVs):

            # Check the header of the TLV message
            tlv_type = int.from_bytes(byteBuffer[idX:idX + 4], byteorder='little')
            idX += 4
            tlv_length = int.from_bytes(byteBuffer[idX:idX + 4], byteorder='little')
            idX += 4

            # Read the data depending on the TLV message
            if tlv_type == MMWDEMO_UART_MSG_VITALSIGN:
                # vitalsign["rangeBinIndexMax"] = int.from_bytes(byteBuffer[idX:idX + 2], byteorder='little')
                idX += 2
                # vitalsign["rangeBinIndexPhase"] = int.from_bytes(byteBuffer[idX:idX + 2], byteorder='little')
                idX += 2
                # vitalsign["maxVal"] = struct.unpack('<f', byteBuffer[idX:idX + 4])[0]
                idX += 4
                # vitalsign["processingCyclesOut"] = int.from_bytes(byteBuffer[idX:idX + 4], byteorder='little')
                idX += 4
                # vitalsign["rangeBinStartIndex"] = int.from_bytes(byteBuffer[idX:idX + 2], byteorder='little')
                idX += 2
                # vitalsign["rangeBinEndIndex"] = int.from_bytes(byteBuffer[idX:idX + 2], byteorder='little')
                idX += 2
                # vitalsign["unwrapPhasePeak_mm"] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                idX += 4
                vitalsign["outputFilterBreathOut"] = struct.unpack('<f', byteBuffer[idX:idX + 4])[0]
                idX += 4
                vitalsign["outputFilterHeartOut"] = struct.unpack('<f', byteBuffer[idX:idX + 4])[0]
                idX += 4
                vitalsign["heartRateEst_FFT"] = struct.unpack('<f', byteBuffer[idX:idX + 4])[0]
                idX += 4
                # vitalsign["heartRateEst_FFT_4Hz"] = struct.unpack('<f', byteBuffer[idX:idX + 4])[0] / 2
                idX += 4
                # vitalsign["heartRateEst_xCorr"] = struct.unpack('<f', byteBuffer[idX:idX + 4])[0]
                idX += 4
                # vitalsign["heartRateEst_peakCount"] = struct.unpack('<f', byteBuffer[idX:idX + 4])[0]
                idX += 4
                vitalsign["breathingRateEst_FFT"] = struct.unpack('<f', byteBuffer[idX:idX + 4])[0]
                idX += 4
                # vitalsign["breathingRateEst_xCorr"] = struct.unpack('<f', byteBuffer[idX:idX + 4])[0]
                idX += 4
                # vitalsign["breathingRateEst_peakCount"] = struct.unpack('<f', byteBuffer[idX:idX + 4])[0]
                idX += 4
                # vitalsign["confidenceMetricBreathOut"] = struct.unpack('<f', byteBuffer[idX:idX + 4])[0]
                idX += 4
                # vitalsign["confidenceMetricBreathOut_xCorr"] = struct.unpack('<f', byteBuffer[idX:idX + 4])[0]
                idX += 4
                # vitalsign["confidenceMetricHeartOut"] = struct.unpack('<f', byteBuffer[idX:idX + 4])[0]
                idX += 4
                # vitalsign["confidenceMetricHeartOut_4Hz"] = struct.unpack('<f', byteBuffer[idX:idX + 4])[0]
                idX += 4
                # vitalsign["confidenceMetricHeartOut_xCorr"] = struct.unpack('<f', byteBuffer[idX:idX + 4])[0]
                idX += 4
                # vitalsign["sumEnergyBreathWfm"] = struct.unpack('<f', byteBuffer[idX:idX + 4])[0]
                idX += 4
                # vitalsign["sumEnergyHeartWfm"] = struct.unpack('<f', byteBuffer[idX:idX + 4])[0]
                idX += 4
                vitalsign["motionDetectedFlag"] = struct.unpack('<f', byteBuffer[idX:idX + 4])[0]
                idX += 44
                dataOK = 1

        # Remove already processed data
        if 0 < idX < byteBufferLength:
            shiftSize = totalPacketLen

            byteBuffer[:byteBufferLength - shiftSize] = byteBuffer[shiftSize:byteBufferLength]
            byteBuffer[byteBufferLength - shiftSize:] = np.zeros(len(byteBuffer[byteBufferLength - shiftSize:]),
                                                                 dtype='uint8')
            byteBufferLength = byteBufferLength - shiftSize

            # Check that there are no errors with the buffer length
            if byteBufferLength < 0:
                byteBufferLength = 0

    return dataOK, frameNumber, vitalsign


# ------------------------------------------------------------------

# Funtion to update the data and display in the plot
def update():
    dataOk = 0
    global vitalsign
    global s
    global Breathsignal
    global Heartbeatsignal

    # Read and parse the received data
    dataOk, frameNumber, vitalsign = readAndParseData18xx(Dataport, configParameters)
    if dataOk:
        # if vitalsign["motionDetectedFlag"] == 0:
        Breathsignal.append(vitalsign["outputFilterBreathOut"])
        Heartbeatsignal.append(vitalsign["outputFilterHeartOut"])
        if len(Breathsignal) > 250:
            Breathsignal.pop(0)
        if len(Heartbeatsignal) > 250:
            Heartbeatsignal.pop(0)
        s1.setData(np.array((list(range(0, 250)), Breathsignal)).T)
        s2.setData(np.array((list(range(0, 250)), Heartbeatsignal)).T)
        # if vitalsign["motionDetectedFlag"] == 0:
        labelItem1.setText(text='Breath Rate:' + str(vitalsign["breathingRateEst_FFT"]), size='12pt', color='000000')
        labelItem2.setText(text='Heart Rate:' + str(vitalsign["heartRateEst_FFT"]), size='12pt', color='000000')
        # else:
        #     labelItem1.setText(text='motionDetected', size='12pt', color='56121b')
        #     labelItem2.setText(text='motionDetected', size='12pt', color='56121b')
        QtGui.QApplication.processEvents()

    return dataOk


# -------------------------    MAIN   -----------------------------------------

# Configurate the serial port
CLIport, Dataport = serialConfig(configFileName)

# Get the configuration parameters from the configuration file
configParameters = parseConfigFile(configFileName)

# START QtAPPfor the plot
app = QtGui.QApplication([])

# Set the plot
pg.setConfigOption('background', 'w')
win = pg.GraphicsWindow(title="Vital Sign")

win.resize(1200, 500)
p1 = win.addPlot()
p1.setXRange(0, 250)
p1.setYRange(-2, 2)
p1.setLabel('left', text='Y position (mm)')
p1.setLabel('bottom', text='time (pre 50ms)')
PEN = pg.mkPen(width=3, color='r')
s1 = p1.plot([], [], pen=PEN)

labelItem1 = pg.LabelItem(text='Breath Rate:')
win.addItem(labelItem1)

p2 = win.addPlot()
p2.setXRange(0, 250)
p2.setYRange(-2, 2)
p2.setLabel('left', text='Y position (mm)')
p2.setLabel('bottom', text='time (pre 50ms)')
PEN = pg.mkPen(width=3, color='r')
s2 = p2.plot([], [], pen=PEN)

labelItem2 = pg.LabelItem(text='Heartbeat Rate:')
win.addItem(labelItem2)

# Main loop
detObj = {}
frameData = {}
currentIndex = 0
while True:
    try:
        # Update the data and check if the data is okay
        dataOk = update()

        # time.sleep(0.04)

    # Stop the program and close everything if Ctrl + c is pressed
    except KeyboardInterrupt:
        CLIport.write(('sensorStop\n').encode())
        print('sensorStop\n')
        CLIport.close()
        Dataport.close()
        win.close()
        break
