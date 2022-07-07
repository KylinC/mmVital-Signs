import serial
import time
import numpy as np
import struct
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui
from sklearn.cluster import dbscan
import pandas

def readAndParseData14xx(Dataport, configParameters):
    global byteBuffer, byteBufferLength

    OBJ_STRUCT_SIZE_BYTES = 12
    BYTE_VEC_ACC_MAX_SIZE = 2 ** 15
    MMWDEMO_UART_MSG_DETECTED_POINTS = 1
    MMWDEMO_UART_MSG_RANGE_PROFILE = 2
    maxBufferSize = 2 ** 15
    tlvHeaderLengthInBytes = 8
    pointLengthInBytes = 16
    magicWord = [2, 1, 4, 3, 6, 5, 8, 7]

    magicOK = 0
    dataOK = 0
    frameNumber = 0
    detObj = {}

    readBuffer = Dataport.read(Dataport.in_waiting)
    byteVec = np.frombuffer(readBuffer, dtype='uint8')
    byteCount = len(byteVec)

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
        numDetectedObj = int.from_bytes(byteBuffer[idX:idX + 4], byteorder='little')
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

            if tlv_type == MMWDEMO_UART_MSG_DETECTED_POINTS:

                x = np.zeros(numDetectedObj, dtype=np.float32)
                y = np.zeros(numDetectedObj, dtype=np.float32)
                z = np.zeros(numDetectedObj, dtype=np.float32)
                velocity = np.zeros(numDetectedObj, dtype=np.float32)

                for objectNum in range(numDetectedObj):
                    x[objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                    idX += 4
                    y[objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                    idX += 4
                    z[objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                    idX += 4
                    velocity[objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                    idX += 4
                detObj = {"numObj": numDetectedObj, "x": x, "y": y, "z": z, "velocity": velocity}
                dataOK = 1

        if 0 < idX < byteBufferLength:
            shiftSize = totalPacketLen
            byteBuffer[:byteBufferLength - shiftSize] = byteBuffer[shiftSize:byteBufferLength]
            byteBuffer[byteBufferLength - shiftSize:] = np.zeros(len(byteBuffer[byteBufferLength - shiftSize:]),
                                                                 dtype='uint8')
            byteBufferLength = byteBufferLength - shiftSize
            if byteBufferLength < 0:
                byteBufferLength = 0
    return dataOK, frameNumber, detObj
