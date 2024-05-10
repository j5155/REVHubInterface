import binascii
import multiprocessing as mp
import time

import serial

from . import REVComPorts, REVmessages as REVMsg
from .REVModule import Module


class REVcomm:

    def __init__(self):
        self.serialReceive_Thread = False
        self.FunctionReturnTime = 0
        self.msgNum = 1
        self.totalTime = 0
        self.rxQueue = mp.Queue(256)
        self.txQueue = mp.Queue(256)
        self.roundTripAverage = 0
        self.numMsgs = 0
        self.enablePrinting = False
        self.msgSendTime = 0
        self.msgRcvTime = 0
        self.discoveryTimeout = 0.5
        self.averageMsgTime = 0
        self.REVProcessor = serial.Serial(baudrate=460800, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                                          stopbits=serial.STOPBITS_ONE)

    def list_ports(self):
        REVComPorts.populateSerialPorts()
        return REVComPorts.REVPorts

    # commenting this out
    # setActivePort doesn't exist, also I can find no use for this function at all??
    # might be useful for multi hub so not completely removing it
    # def set_active_port_by_sn(self, sn):
    #     REVComPorts.populateSerialPorts()
    #     for port in REVComPorts.serialPorts:
    #         if port.getSN() == sn:
    #             setActivePort(port)

    def open_active_port(self):
        num_serial_errors = 2
        while not self.REVProcessor.isOpen():
            self.REVProcessor.port = self.list_ports()[0].getName()
            try:
                self.REVProcessor.open()
            except serial.SerialException as e:
                print('Serial port error: ' + str(e) + ' retrying...')
                num_serial_errors -= 1
                if num_serial_errors == 0:
                    break
                time.sleep(1)

    def close_active_port(self):
        self.REVProcessor.close()

    def get_time_ms(self):
        return int(round(time.time() * 1000))

    def send_and_receive(self, packet_to_write, destination):
        self.WaitForFrameByte1 = 1
        self.WaitForFrameByte2 = 2
        self.WaitForPacketLengthByte1 = 3
        self.WaitForPacketLengthByte2 = 4
        self.WaitForDestByte = 5
        self.WaitForSourceByte = 6
        self.WaitForMsgNumByte = 7
        self.WaitForRefNumByte = 8
        self.WaitForPacketTypeByte = 9
        self.WaitForPayloadBytes = 10
        parse_state = 1
        self.parseState = self.WaitForFrameByte1
        incoming_packet = ''
        packet_length = 0
        msg_num = 0
        retry = True
        try:
            retry_attempt = 0
            while retry:
                packet_to_write.header.destination = destination
                if isinstance(packet_to_write, REVMsg.REVPacket):
                    max_retries = 3
                    packet_to_write.header.msgNum = msg_num
                    msg_num = (msg_num + 1) % 256
                    if msg_num == 0:
                        msg_num = 1
                    print_data = packet_to_write.header.packetType.data >> 8 | packet_to_write.header.packetType.data % 256 << 8
                    discovery_mode = False
                    if print_data == REVMsg.MsgNum.Discovery:
                        discovery_mode = True
                    if self.enablePrinting:
                        print('-->', REVMsg.printDict[print_data]['Name'], '::', packet_to_write.getPacketData())
                    self.REVProcessor.write(binascii.unhexlify(packet_to_write.getPacketData()))
                    wait_time_start = time.time()
                    timeout = False
                    while self.REVProcessor.inWaiting() == 0:
                        if time.time() - wait_time_start > 1:
                            timeout = True
                            retry_attempt += 1
                            if retry_attempt > max_retries:
                                retry = False
                            break
                    if timeout:
                        continue
                    if discovery_mode:
                        packet = []
                    # TODO: all of this should be a proper state machine
                    if self.REVProcessor.inWaiting() > 0:
                        while self.REVProcessor.inWaiting() > 0:
                            retry = False
                            # TODO: this is a little weird
                            # why not just put it all on one line?
                            new_byte = binascii.hexlify(self.REVProcessor.read(1)).upper()
                            new_byte = str(new_byte)
                            new_byte = new_byte[2:]
                            new_byte = new_byte[:-1]
                            # todo: why exactly is it doing this??
                            if parse_state == self.WaitForFrameByte1:
                                if new_byte == '44':
                                    parse_state = self.WaitForFrameByte2
                            elif parse_state == self.WaitForFrameByte2:
                                if new_byte == '44':
                                    pass  # TODO: this is EXTREMELY weird
                                elif new_byte == '4B':
                                    parse_state = self.WaitForPacketLengthByte1
                                else:
                                    parse_state = self.WaitForFrameByte1
                            elif parse_state == self.WaitForPacketLengthByte1:
                                incoming_packet = '444B' + new_byte
                                # TODO: does it even need length_bytes? can it just use new_byte directly??
                                length_bytes = new_byte
                                parse_state = self.WaitForPacketLengthByte2
                            elif parse_state == self.WaitForPacketLengthByte2:
                                incoming_packet += new_byte
                                length_bytes += new_byte
                                length_bytes = int(int(length_bytes, 16) >> 8 | int(length_bytes, 16) % 256 << 8)
                                if length_bytes <= REVMsg.PAYLOAD_MAX_SIZE:
                                    packet_length = length_bytes
                                    parse_state = self.WaitForPayloadBytes
                                elif new_byte == '44':
                                    parse_state = self.WaitForFrameByte2
                                else:
                                    parse_state = self.WaitForFrameByte1
                            elif parse_state == self.WaitForPayloadBytes:
                                incoming_packet += new_byte
                                if len(incoming_packet) / 2 == packet_length:
                                    msgRcvTime = time.time()
                                    receivedChkSum = int(incoming_packet[-2:], 16)
                                    chksumdata = self.checkPacket(incoming_packet, receivedChkSum)
                                    if chksumdata[0]:
                                        newPacket = self.processPacket(incoming_packet)
                                        if self.enablePrinting:
                                            print('<--', REVMsg.printDict[int(newPacket.header.packetType)]['Name'],
                                                  '::', newPacket.getPacketData())
                                        if discovery_mode:
                                            packet.append(newPacket)
                                            time.sleep(2)
                                            if self.REVProcessor.inWaiting() > 0:
                                                pass
                                            else:
                                                return packet
                                        else:
                                            return newPacket
                                    else:
                                        print('Invalid ChkSum: ', chksumdata[1], '==', chksumdata[2])
                                    rcvStarted = False
                                    parse_state = self.WaitForFrameByte1

                else:
                    exit('\n\n\n!!!Attempting to send something other than a REVPacket!!!\n\n\n')

        except serial.SerialException:
            self.REVProcessor.close()
            return False

        return True

    def checkResponse(self, receivedPacket, PacketToWrite):
        packetType = int(receivedPacket.header.packetType)
        data = PacketToWrite.header.packetType.data >> 8 | PacketToWrite.header.packetType.data % 256 << 8
        responseExpected = REVMsg.printDict[data]['Response']
        if packetType == responseExpected:
            if receivedPacket.header.refNum == PacketToWrite.header.msgNum:
                return True
            else:
                if packetType == REVMsg.RespNum.Discovery_RSP:
                    return True
                print('This response is for a different message. Sent: %d, Received: %d.' % (
                receivedPacket.header.refNum, PacketToWrite.header.msgNum))
                return False

        else:
            if packetType == REVMsg.MsgNum.NACK:
                printData = PacketToWrite.header.packetType.data >> 8 | PacketToWrite.header.packetType.data % 256 << 8
                print('NACK Code: ', receivedPacket.payload.nackCode)
                print("NACK'd Packet: ", REVMsg.printDict[printData]['Name'], '::', PacketToWrite.getPacketData())
                return False
            else:
                print('Incorrect Response Type. Response Expected: ', binascii.hexlify(str(data)),
                      ', Response Received: ', binascii.hexlify(str(packetType)))
                return False

    def checkPacket(self, incomingPacket, receivedChkSum):
        calcChkSum = 0
        for bytePointer in range(0, len(incomingPacket) - 2, 2):
            calcChkSum += int(incomingPacket[bytePointer:bytePointer + 2], 16)
            calcChkSum %= 256

        return (receivedChkSum == calcChkSum, receivedChkSum, calcChkSum)

    def processPacket(self, incomingPacket):
        packetFrameBytes = int(incomingPacket[REVMsg.REVPacket.FrameIndex_Start:REVMsg.REVPacket.FrameIndex_End], 16)
        packetLength = int(
            self.swapEndianess(incomingPacket[REVMsg.REVPacket.LengthIndex_Start:REVMsg.REVPacket.LengthIndex_End]), 16)
        packetDest = int(incomingPacket[REVMsg.REVPacket.DestinationIndex_Start:REVMsg.REVPacket.DestinationIndex_End],
                         16)
        packetSrc = int(incomingPacket[REVMsg.REVPacket.SourceIndex_Start:REVMsg.REVPacket.SourceIndex_End], 16)
        packetMsgNum = int(incomingPacket[REVMsg.REVPacket.MsgNumIndex_Start:REVMsg.REVPacket.MsgNumIndex_End], 16)
        packetRefNum = int(incomingPacket[REVMsg.REVPacket.RefNumIndex_Start:REVMsg.REVPacket.RefNumIndex_End], 16)
        packetCommandNum = int(self.swapEndianess(
            incomingPacket[REVMsg.REVPacket.PacketTypeIndex_Start:REVMsg.REVPacket.PacketTypeIndex_End]), 16)
        packetPayload = incomingPacket[REVMsg.REVPacket.HeaderIndex_End:-2]
        packetChkSum = int(incomingPacket[-2:], 16)
        newPacket = REVMsg.printDict[packetCommandNum]['Packet']()
        newPacket.assignRawBytes(incomingPacket)
        newPacket.header.length = packetLength
        newPacket.header.destination = packetDest
        newPacket.header.source = packetSrc
        newPacket.header.msgNum = packetMsgNum
        newPacket.header.refNum = packetRefNum
        newPacket.header.packetType = packetCommandNum
        bytePointer = 0
        for payloadMember in newPacket.payload.getOrderedMembers():
            valueToAdd = REVMsg.REVBytes(len(payloadMember))
            valueToAdd.data = int(self.swapEndianess(packetPayload[bytePointer:bytePointer + len(payloadMember) * 2]),
                                  16)
            newPacket.payload.payloadMember = valueToAdd
            bytePointer = bytePointer + len(payloadMember) * 2

        return newPacket

    def swapEndianess(self, bytes):
        swappedBytes = ''
        for bytePointer in range(0, len(bytes), 2):
            thisByte = bytes[bytePointer:bytePointer + 2]
            swappedBytes = thisByte + swappedBytes

        return swappedBytes

    def getModuleStatus(self, destination):
        getModuleStatusMsg = REVMsg.GetModuleStatus()
        getModuleStatusMsg.payload.clearStatus = 1
        packet = self.send_and_receive(getModuleStatusMsg, destination)
        return packet.payload.motorAlerts

    def keepAlive(self, destination):
        keepAliveMsg = REVMsg.KeepAlive()
        return self.send_and_receive(keepAliveMsg, destination)

    def failSafe(self, destination):
        failSafeMsg = REVMsg.FailSafe()
        self.send_and_receive(failSafeMsg, destination)

    def setNewModuleAddress(self, destination, moduleAddress):
        setNewModuleAddressMsg = REVMsg.SetNewModuleAddress()
        setNewModuleAddressMsg.payload.moduleAddress = moduleAddress
        self.send_and_receive(setNewModuleAddressMsg, destination)

    def queryInterface(self, destination, interfaceName):
        queryInterfaceMsg = REVMsg.QueryInterface()
        queryInterfaceMsg.payload.interfaceName = interfaceName
        packet = self.send_and_receive(queryInterfaceMsg, destination)
        return (
            packet.payload.packetID, packet.numValues)

    def setModuleLEDColor(self, destination, redPower, greenPower, bluePower):
        setModuleLEDColorMsg = REVMsg.SetModuleLEDColor()
        setModuleLEDColorMsg.payload.redPower = redPower
        setModuleLEDColorMsg.payload.greenPower = greenPower
        setModuleLEDColorMsg.payload.bluePower = bluePower
        self.send_and_receive(setModuleLEDColorMsg, destination)

    def getModuleLEDColor(self, destination):
        getModuleLEDColorMsg = REVMsg.GetModuleLEDColor()
        packet = self.send_and_receive(getModuleLEDColorMsg, destination)
        return (
            packet.payload.redPower, packet.payload.greenPower, packet.payload.bluePower)

    def setModuleLEDPattern(self, destination, stepArray):
        setModuleLEDPatternMsg = REVMsg.SetModuleLEDPattern()
        for i, step in enumerate(stepArray.patt):
            setattr(setModuleLEDPatternMsg.payload, ('rgbtStep{}').format(i), step)
        self.send_and_receive(setModuleLEDPatternMsg, destination)

    def getModuleLEDPattern(self, destination):
        getModuleLEDPatternMsg = REVMsg.GetModuleLEDPattern()
        packet = self.send_and_receive(getModuleLEDPatternMsg, destination)
        return packet

    def debugLogLevel(self, destination, groupNumber, verbosityLevel):
        debugLogLevelMsg = REVMsg.DebugLogLevel()
        debugLogLevelMsg.payload.groupNumber = groupNumber
        debugLogLevelMsg.payload.verbosityLevel = verbosityLevel
        self.send_and_receive(debugLogLevelMsg, destination)

    def discovery(self):
        self.discovered = REVMsg.Discovery()
        packets = self.send_and_receive(self.discovered, 255)
        REVModules = []
        for packet in packets:
            module = Module(self, packet.header.source, packet.payload.parent)
            module.init_periphs()
            REVModules.append(module)
        return REVModules

    def getBulkInputData(self, destination):
        getBulkInputDataMsg = REVMsg.GetBulkInputData()
        packet = self.send_and_receive(getBulkInputDataMsg, destination)
        return packet

    def phoneChargeControl(self, destination, enable):
        phoneChargeControlMsg = REVMsg.PhoneChargeControl()
        phoneChargeControlMsg.payload.enable = enable
        self.send_and_receive(phoneChargeControlMsg, destination)

    def phoneChargeQuery(self, destination):
        phoneChargeQueryMsg = REVMsg.PhoneChargeQuery()
        packet = self.send_and_receive(phoneChargeQueryMsg, destination)
        return packet.payload.enable

    def injectDataLogHint(self, destination, length, hintText):
        injectDataLogHintMsg = REVMsg.InjectDataLogHint()
        injectDataLogHintMsg.payload.length = length
        injectDataLogHintMsg.payload.hintText = hintText
        self.send_and_receive(injectDataLogHintMsg, destination)

    def readVersionString(self, destination):
        readVersionStringMsg = REVMsg.ReadVersionString()
        packet = self.send_and_receive(readVersionStringMsg, destination)
        return packet.payload.versionString

    def getBulkMotorData(self, destination):
        getBulkMotorDataMsg = REVMsg.GetBulkMotorData()
        packet = self.send_and_receive(getBulkMotorDataMsg, destination)
        return packet

    def getBulkADCData(self, destination):
        getBulkADCDataMsg = REVMsg.GetBulkADCData()
        packet = self.send_and_receive(getBulkADCDataMsg, destination)
        return packet

    def getBulkI2CData(self, destination):
        getBulkI2CDataMsg = REVMsg.GetBulkI2CData()
        packet = self.send_and_receive(getBulkI2CDataMsg, destination)
        return packet

    def getBulkServoData(self, destination):
        getBulkServoDataMsg = REVMsg.GetBulkServoData()
        packet = self.send_and_receive(getBulkServoDataMsg, destination)
        return packet
