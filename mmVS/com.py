import serial
import time
import numpy as np
import struct
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui
from sklearn.cluster import dbscan
import pandas

def serialConfig(configFileName):
    global CLIport
    global Dataport
    CLIport = serial.Serial('COM8', 115200)
    Dataport = serial.Serial('COM7', 921600)
    config = [line.rstrip('\r\n') for line in open(configFileName)]
    for i in config:
        print(i)
        CLIport.write((i + '\n').encode())
        time.sleep(0.03)

    return CLIport, Dataport


def parseConfigFile(configFileName):
    configParameters = {}
    config = [line.rstrip('\r\n') for line in open(configFileName)]
    for i in config:

        splitWords = i.split(" ")
        numRxAnt = 4
        numTxAnt = 2
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
        elif "frameCfg" in splitWords[0]:
            chirpStartIdx = int(splitWords[1])
            chirpEndIdx = int(splitWords[2])
            numLoops = int(splitWords[3])
            numFrames = int(splitWords[4])
            framePeriodicity = float(splitWords[5])

        elif "vitalSignsCfg" in splitWords[0]:
            rangeStart = float(splitWords[1])
            rangeEnd = float(splitWords[2])
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
    configParameters["rangeStart"] = rangeStart
    configParameters["rangeEnd"] = rangeEnd
    return configParameters
