import binascii
import multiprocessing as mp
import time

import serial

from . import REVComPorts, REVmessages as REVMsg
from .REVModule import Module


def list_ports():
    REVComPorts.populateSerialPorts()
    return REVComPorts.REVPorts


def get_time_ms():
    return int(round(time.time() * 1000))


def check_response(received_packet, packet_to_write):
    packet_type = int(received_packet.header.packetType)
    data = packet_to_write.header.packetType.data >> 8 | packet_to_write.header.packetType.data % 256 << 8
    response_expected = REVMsg.printDict[data]['Response']
    if packet_type == response_expected:
        if received_packet.header.refNum == packet_to_write.header.msgNum:
            return True
        else:
            if packet_type == REVMsg.RespNum.Discovery_RSP:
                return True
            print('This response is for a different message. Sent: %d, Received: %d.' % (
                received_packet.header.refNum, packet_to_write.header.msgNum))
            return False

    else:
        if packet_type == REVMsg.MsgNum.NACK:
            print_data = packet_to_write.header.packetType.data >> 8 | packet_to_write.header.packetType.data % 256 << 8
            print('NACK Code: ', received_packet.payload.nackCode)
            print("NACK'd Packet: ", REVMsg.printDict[print_data]['Name'], '::', packet_to_write.getPacketData())
            return False
        else:
            print('Incorrect Response Type. Response Expected: ', binascii.hexlify(data),
                  ', Response Received: ', str(packet_type))  # this used to be wrapped in a hexlify
            return False


def check_packet(incoming_packet, received_chk_sum):
    calc_chk_sum = 0
    for bytePointer in range(0, len(incoming_packet) - 2, 2):
        calc_chk_sum += int(incoming_packet[bytePointer:bytePointer + 2], 16)
        calc_chk_sum %= 256

    return received_chk_sum == calc_chk_sum, received_chk_sum, calc_chk_sum


def swap_endianess(in_bytes):
    swapped_bytes = ''
    for bytePointer in range(0, len(in_bytes), 2):
        this_byte = in_bytes[bytePointer:bytePointer + 2]
        swapped_bytes = this_byte + swapped_bytes

    return swapped_bytes


def process_packet(incoming_packet):
    packet_length = int(
        swap_endianess(incoming_packet[REVMsg.REVPacket.LengthIndex_Start:REVMsg.REVPacket.LengthIndex_End]), 16)
    packet_dest = int(incoming_packet[REVMsg.REVPacket.DestinationIndex_Start:REVMsg.REVPacket.DestinationIndex_End],
                      16)
    packet_src = int(incoming_packet[REVMsg.REVPacket.SourceIndex_Start:REVMsg.REVPacket.SourceIndex_End], 16)
    packet_msg_num = int(incoming_packet[REVMsg.REVPacket.MsgNumIndex_Start:REVMsg.REVPacket.MsgNumIndex_End], 16)
    packet_ref_num = int(incoming_packet[REVMsg.REVPacket.RefNumIndex_Start:REVMsg.REVPacket.RefNumIndex_End], 16)
    packet_command_num = int(swap_endianess(
        incoming_packet[REVMsg.REVPacket.PacketTypeIndex_Start:REVMsg.REVPacket.PacketTypeIndex_End]), 16)
    packet_payload = incoming_packet[REVMsg.REVPacket.HeaderIndex_End:-2]
    new_packet = REVMsg.printDict[packet_command_num]['Packet']()
    new_packet.assignRawBytes(incoming_packet)
    new_packet.header.length = packet_length
    new_packet.header.destination = packet_dest
    new_packet.header.source = packet_src
    new_packet.header.msgNum = packet_msg_num
    new_packet.header.refNum = packet_ref_num
    new_packet.header.packetType = packet_command_num
    byte_pointer = 0
    for payloadMember in new_packet.payload.getOrderedMembers():
        value_to_add = REVMsg.REVBytes(len(payloadMember))
        value_to_add.data = int(swap_endianess(packet_payload[byte_pointer:byte_pointer + len(payloadMember) * 2]),
                                16)
        new_packet.payload.payloadMember = value_to_add
        byte_pointer = byte_pointer + len(payloadMember) * 2

    return new_packet


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
        while not self.REVProcessor.is_open:
            self.REVProcessor.port = list_ports()[0].getName()
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

    def send_and_receive(self, packet_to_write, destination):
        # TODO: this should be an enum
        wait_for_frame_byte1 = 1
        wait_for_frame_byte2 = 2
        wait_for_packet_length_byte1 = 3
        wait_for_packet_length_byte2 = 4
        # TODO: these were unused, not sure why
        """
        wait_for_dest_byte = 5
        wait_for_source_byte = 6
        wait_for_msg_num_byte = 7
        wait_for_ref_num_byte = 8
        wait_for_packet_type_byte = 9
        """
        wait_for_payload_bytes = 10
        parse_state = 1
        incoming_packet = ''
        packet_length = 0
        msg_num = 0
        retry = True
        length_bytes = 0
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
                    packet_type_data = packet_to_write.header.packetType.data
                    print_data = packet_type_data >> 8 | packet_type_data % 256 << 8
                    discovery_mode = False
                    if print_data == REVMsg.MsgNum.Discovery:
                        discovery_mode = True
                    if self.enablePrinting:
                        print('-->', REVMsg.printDict[print_data]['Name'], '::', packet_to_write.getPacketData())
                    self.REVProcessor.write(binascii.unhexlify(packet_to_write.getPacketData()))
                    wait_time_start = time.time()
                    timeout = False
                    while self.REVProcessor.in_waiting == 0:
                        if time.time() - wait_time_start > 1:
                            timeout = True
                            retry_attempt += 1
                            if retry_attempt > max_retries:
                                retry = False
                            break
                    if timeout:
                        continue
                    packet = []
                    # TODO: all of this should be a proper state machine
                    if self.REVProcessor.in_waiting > 0:  # TODO: this if statement isnt needed at all???
                        while self.REVProcessor.in_waiting > 0:
                            retry = False
                            # TODO: this is a little weird
                            # why not just put it all on one line?
                            new_byte = binascii.hexlify(self.REVProcessor.read(1)).upper()
                            new_byte = str(new_byte)
                            new_byte = new_byte[2:]
                            new_byte = new_byte[:-1]
                            # todo: why exactly is it doing this??
                            if parse_state == wait_for_frame_byte1:
                                if new_byte == '44':
                                    parse_state = wait_for_frame_byte2
                            elif parse_state == wait_for_frame_byte2:
                                if new_byte == '44':
                                    pass  # TODO: this is EXTREMELY weird
                                elif new_byte == '4B':
                                    parse_state = wait_for_packet_length_byte1
                                else:
                                    parse_state = wait_for_frame_byte1
                            elif parse_state == wait_for_packet_length_byte1:
                                incoming_packet = '444B' + new_byte
                                # TODO: does it even need length_bytes? can it just use new_byte directly??
                                length_bytes = new_byte
                                parse_state = wait_for_packet_length_byte2
                            elif parse_state == wait_for_packet_length_byte2:
                                incoming_packet += new_byte
                                length_bytes += new_byte
                                length_bytes = int(int(length_bytes, 16) >> 8 | int(length_bytes, 16) % 256 << 8)
                                if length_bytes <= REVMsg.PAYLOAD_MAX_SIZE:
                                    packet_length = length_bytes
                                    parse_state = wait_for_payload_bytes
                                elif new_byte == '44':
                                    parse_state = wait_for_frame_byte2
                                else:
                                    parse_state = wait_for_frame_byte1
                            elif parse_state == wait_for_payload_bytes:
                                incoming_packet += new_byte
                                if len(incoming_packet) / 2 == packet_length:
                                    received_chk_sum = int(incoming_packet[-2:], 16)
                                    chksumdata = check_packet(incoming_packet, received_chk_sum)
                                    if chksumdata[0]:
                                        new_packet = process_packet(incoming_packet)
                                        if self.enablePrinting:
                                            print('<--', REVMsg.printDict[int(new_packet.header.packetType)]['Name'],
                                                  '::', new_packet.getPacketData())
                                        if discovery_mode:
                                            packet.append(new_packet)
                                            time.sleep(2)
                                            if self.REVProcessor.in_waiting > 0:
                                                pass
                                            else:
                                                return packet
                                        else:
                                            return new_packet
                                    else:
                                        print('Invalid ChkSum: ', chksumdata[1], '==', chksumdata[2])
                                    parse_state = wait_for_frame_byte1

                else:
                    exit('\n\n\n!!!Attempting to send something other than a REVPacket!!!\n\n\n')

        except serial.SerialException:
            self.REVProcessor.close()
            return False

        return True

    def get_module_status(self, destination):
        get_module_status_msg = REVMsg.GetModuleStatus()
        get_module_status_msg.payload.clearStatus = 1
        packet = self.send_and_receive(get_module_status_msg, destination)
        return packet.payload.motorAlerts

    def keep_alive(self, destination):
        keep_alive_msg = REVMsg.KeepAlive()
        return self.send_and_receive(keep_alive_msg, destination)

    def fail_safe(self, destination):
        fail_safe_msg = REVMsg.FailSafe()
        self.send_and_receive(fail_safe_msg, destination)

    def set_new_module_address(self, destination, module_address):
        set_new_module_address_msg = REVMsg.SetNewModuleAddress()
        set_new_module_address_msg.payload.moduleAddress = module_address
        self.send_and_receive(set_new_module_address_msg, destination)

    def query_interface(self, destination, interface_name):
        query_interface_msg = REVMsg.QueryInterface()
        query_interface_msg.payload.interfaceName = interface_name
        packet = self.send_and_receive(query_interface_msg, destination)
        return (
            packet.payload.packetID, packet.numValues)

    def set_module_led_color(self, destination, red_power, green_power, blue_power):
        set_module_led_color_msg = REVMsg.SetModuleLEDColor()
        set_module_led_color_msg.payload.redPower = red_power
        set_module_led_color_msg.payload.greenPower = green_power
        set_module_led_color_msg.payload.bluePower = blue_power
        self.send_and_receive(set_module_led_color_msg, destination)

    def get_module_led_color(self, destination):
        get_module_led_color_msg = REVMsg.GetModuleLEDColor()
        packet = self.send_and_receive(get_module_led_color_msg, destination)
        return (
            packet.payload.redPower, packet.payload.greenPower, packet.payload.bluePower)

    def set_module_led_pattern(self, destination, step_array):
        set_module_led_pattern_msg = REVMsg.SetModuleLEDPattern()
        for i, step in enumerate(step_array.patt):
            setattr(set_module_led_pattern_msg.payload, 'rgbtStep{}'.format(i), step)
        self.send_and_receive(set_module_led_pattern_msg, destination)

    def get_module_led_pattern(self, destination):
        get_module_led_pattern_msg = REVMsg.GetModuleLEDPattern()
        packet = self.send_and_receive(get_module_led_pattern_msg, destination)
        return packet

    def debug_log_level(self, destination, group_number, verbosity_level):
        debug_log_level_msg = REVMsg.DebugLogLevel()
        debug_log_level_msg.payload.groupNumber = group_number
        debug_log_level_msg.payload.verbosityLevel = verbosity_level
        self.send_and_receive(debug_log_level_msg, destination)

    def discovery(self):
        packets = self.send_and_receive(REVMsg.Discovery(), 255)
        rev_modules = []
        for packet in packets:
            module = Module(self, packet.header.source, packet.payload.parent)
            module.init_periphs()
            rev_modules.append(module)
        return rev_modules

    def get_bulk_input_data(self, destination):
        get_bulk_input_data_msg = REVMsg.GetBulkInputData()
        packet = self.send_and_receive(get_bulk_input_data_msg, destination)
        return packet

    def phone_charge_control(self, destination, enable):
        phone_charge_control_msg = REVMsg.PhoneChargeControl()
        phone_charge_control_msg.payload.enable = enable
        self.send_and_receive(phone_charge_control_msg, destination)

    def phone_charge_query(self, destination):
        phone_charge_query_msg = REVMsg.PhoneChargeQuery()
        packet = self.send_and_receive(phone_charge_query_msg, destination)
        return packet.payload.enable

    def inject_data_log_hint(self, destination, length, hint_text):
        inject_data_log_hint_msg = REVMsg.InjectDataLogHint()
        inject_data_log_hint_msg.payload.length = length
        inject_data_log_hint_msg.payload.hintText = hint_text
        self.send_and_receive(inject_data_log_hint_msg, destination)

    def read_version_string(self, destination):
        read_version_string_msg = REVMsg.ReadVersionString()
        packet = self.send_and_receive(read_version_string_msg, destination)
        return packet.payload.versionString

    def get_bulk_motor_data(self, destination):
        get_bulk_motor_data_msg = REVMsg.GetBulkMotorData()
        packet = self.send_and_receive(get_bulk_motor_data_msg, destination)
        return packet

    def get_bulk_adc_data(self, destination):
        get_bulk_adc_data_msg = REVMsg.GetBulkADCData()
        packet = self.send_and_receive(get_bulk_adc_data_msg, destination)
        return packet

    def get_bulk_i2_c_data(self, destination):
        get_bulk_i2_c_data_msg = REVMsg.GetBulkI2CData()
        packet = self.send_and_receive(get_bulk_i2_c_data_msg, destination)
        return packet

    def get_bulk_servo_data(self, destination):
        get_bulk_servo_data_msg = REVMsg.GetBulkServoData()
        packet = self.send_and_receive(get_bulk_servo_data_msg, destination)
        return packet
