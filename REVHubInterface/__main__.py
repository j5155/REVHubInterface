import os
import tkinter as tk
import tkinter.filedialog
import tkinter.messagebox
import tkinter.ttk
from functools import partial
from tkinter.constants import *

from REVHubInterface import REVcomm  # relative imports don't work here due to pyinstaller issue
from REVHubInterface.REV2mSensor import REV2mSensor
from REVHubInterface.REVColorSensorV3 import REVColorSensorV3
from REVHubInterface.REVcomm import *


# try:
#     import ft232
# except Exception as e:
#     print(platform.system)
#     tkinter.messagebox.showerror('Drivers Not Detected',
#                                  'Please verify the correct drivers are installed. Without the correct drivers, '
#                                  'firmware update functionality will be unavailable.'
#                                  '\n\n - Windows 10 and above should'
#                                  ' automatically install the correct drivers when the Expansion Hub is plugged in.'
#                                  '\n\n - Windows 7 requires a manual installation. Please see this link '
#                                  'for the correct driver (FTDI D2xx): '
#                                  'https://www.ftdichip.com/Drivers/CDM/CDM21228_Setup.zip\n\n '
#                                  '- On macOS, install libftdi via Homebrew: "brew install libftdi"\n\n '
#                                  '- On Linux, install libftdi.  On Debian/Ubuntu-based systems, install it '
#                                  'via "sudo apt install libftdi1"\n\nException Message:\n' + str(e))

def validate_float(action, value_if_allowed, text):
    if action == '1':
        if text in '0123456789':
            try:
                if 255 > int(value_if_allowed) > 0:
                    return True
                else:
                    return False
            except ValueError:
                return False
        else:
            return False
    else:
        return True


class DeviceInfo:
    def __init__(self, root, set_address):
        root.grid_columnconfigure(0, weight=1)
        root.grid_rowconfigure(0, weight=1)
        root.grid(sticky="NSEW")
        self.device_info_frame = tkinter.ttk.Frame(root)
        self.device_label = tkinter.ttk.Label(self.device_info_frame)
        self.Frame_1 = tkinter.ttk.Frame(self.device_info_frame)
        self.Button_1 = tkinter.ttk.Button(self.Frame_1)

        self.device_info_frame.config(height=100, width=100)
        self.device_info_frame.grid(column=0, row=0, sticky="NSEW")
        self.device_info_frame.grid_rowconfigure(0, weight=1)
        self.device_info_frame.grid_columnconfigure(0, weight=1)

        self.device_label.config(text='Module: ', width=10)
        self.device_label.grid(column=1, padx=5, pady=5, row=0, sticky=E)

        self.Frame_1.config(height=200, width=200)
        self.Frame_1.grid(column=0, row=0, sticky=E)

        self.Button_1.config(command=set_address, text='Set Address', width=10)
        self.Button_1.grid(column=2, row=0, sticky=E)

        vcmd = (
            root.register(validate_float), '%d', '%i', '%P', '%s', '%S', '%v', '%V', '%W')

        self.addr_entry = tkinter.ttk.Entry(self.device_info_frame, validate='key', validatecommand=vcmd)
        self.addr_entry.config(width=10)
        self.addr_entry.grid(column=3, padx=5, pady=5, row=0, sticky=E)


# class firmware_tab():
#     def __init__(self, root, chooseBin, flashNow):
#         self.INTERFACE_VERSION = '1.3.0'
#         root.grid_columnconfigure(0, weight=1)
#         root.grid_rowconfigure(0, weight=1)
#         root.grid(sticky="NSEW")
#         self.firmware_frame = tkinter.ttk.Frame(root)
#         self.firmware_label = tkinter.ttk.Label(self.firmware_frame)
#         self.warning_block = tk.Text(self.firmware_frame)
#         self.Frame_1 = tkinter.ttk.Frame(self.firmware_frame)
#         self.Button_1 = tkinter.ttk.Button(self.Frame_1)
#         self.Button_2 = tkinter.ttk.Button(self.Frame_1)
#         self.Device_info_frame1 = tkinter.ttk.Frame(self.firmware_frame)
#         
#        self.firmware_frame.config(height=200, width=200)
#        self.firmware_frame.grid(column=0, row=0, sticky="NSEW")
#        self.firmware_frame.grid_rowconfigure(0, weight=1)
#        self.firmware_frame.grid_columnconfigure(0, weight=1)
#
#        self.Device_info_frame1.config(height=100, width=100)
#        self.Device_info_frame1.grid(column=0, row=4, sticky="NSEW")
#        self.Device_info_frame1.grid_rowconfigure(0, weight=1)
#        self.Device_info_frame1.grid_columnconfigure(0, weight=1)
#
#        self.firmware_label.config(text='Interface Version: ' + self.INTERFACE_VERSION, width=10)
#        self.firmware_label.grid(column=0, columnspan=2, padx=5, pady=5, row=1, sticky="EW")
#
#        self.warning_block.config(height=11, width=50, wrap='word')
#        self.warning_block.grid(column=0, columnspan=2, row=0, sticky="NSEW")
#
#        self.Frame_1.config(height=200, width=200)
#        self.Frame_1.grid(column=0, row=2, sticky=W)
#
#        self.Button_1.config(command=chooseBin, text='Choose .bin file')
#        self.Button_1.grid(column=0, row=0, sticky=W)
#
#        self.Button_2.config(command=flashNow, text='Flash')
#        self.Button_2.grid(column=1, row=0, sticky=W)


class DigitalSingle:
    def __init__(self, root, set_input_callback, set_output_callback, digital_set, digital_poll):
        root.grid_columnconfigure(0, weight=1)
        root.grid_rowconfigure(0, weight=1)
        root.grid(sticky="NSEW")
        self.var2 = tk.IntVar()
        self.digital_panel = tkinter.ttk.Frame(root)
        self.digital_label_1 = tkinter.ttk.Label(self.digital_panel)
        self.Frame_1 = tkinter.ttk.Frame(self.digital_panel)
        self.poll_button = tkinter.ttk.Button(self.digital_panel)
        self.Separator_1 = tkinter.ttk.Separator(self.Frame_1)
        self.State_label = tkinter.ttk.Label(self.Frame_1)
        self.input_button = tkinter.ttk.Button(self.Frame_1)
        self.output_button = tkinter.ttk.Button(self.Frame_1)
        self.Checkbutton_1 = tkinter.ttk.Checkbutton(self.Frame_1)

        self.digital_panel.config(height=200, width=200, padding=(5, 5, 5, 5), relief='groove')
        self.digital_panel.grid(column=0, row=0, sticky="NSEW")
        self.digital_panel.grid_rowconfigure(0, weight=1)
        self.digital_panel.grid_columnconfigure(0, weight=1)

        self.digital_label_1.config(takefocus=True, text='Digital 0:', width=8)
        self.digital_label_1.grid(column=0, row=0, sticky=W, padx=0, pady=0, columnspan=1)

        self.Frame_1.config(height=200, width=200)
        self.Frame_1.grid(column=0, row=1, columnspan=1, sticky="NSEW")

        self.Separator_1.config(orient='vertical')
        self.Separator_1.grid(column=3, padx=5, row=1, sticky="NS")

        self.State_label.config(text='State read:')
        self.State_label.grid(column=4, row=1)

        self.input_button.config(command=set_input_callback, text='IN')
        self.input_button.grid(column=0, row=1, sticky="NSEW")

        self.output_button.config(command=set_output_callback, text='OUT')
        self.output_button.grid(column=1, row=1, sticky="NSEW")

        self.Checkbutton_1.config(command=digital_set, offvalue=0, onvalue=1, state='disabled', variable=self.var2,
                                  width=0)
        self.Checkbutton_1.grid(column=5, row=1)

        self.poll_button.config(command=digital_poll, text='POLL', width=5)
        self.poll_button.grid(column=2, row=0, sticky=E)


class AnalogSingle:
    def __init__(self, root):
        root.grid_columnconfigure(0, weight=1)
        root.grid_rowconfigure(0, weight=1)
        root.grid(sticky="NSEW")
        self.analog_panel = tkinter.ttk.Frame(root)
        self.analog_label_1 = tkinter.ttk.Label(self.analog_panel)
        self.Frame_1 = tkinter.ttk.Frame(self.analog_panel)
        self.voltage_label_1 = tkinter.ttk.Label(self.Frame_1)
        self.voltage_value_1 = tkinter.ttk.Label(self.Frame_1)
        self.analog_scale_1 = tkinter.ttk.Progressbar(self.Frame_1)
        self.java_label_1 = tkinter.ttk.Label(self.Frame_1)
        self.java_value_1 = tkinter.ttk.Label(self.Frame_1)

        self.analog_panel.config(height=200, padding=(5, 5, 5, 5), relief='ridge', width=200)
        self.analog_panel.grid(column=0, row=0, sticky="NSEW")
        self.analog_panel.grid_rowconfigure(0, weight=1)
        self.analog_panel.grid_rowconfigure(1, weight=1)
        self.analog_panel.grid_columnconfigure(0, weight=1)

        self.analog_label_1.config(text='Analog 0')
        self.analog_label_1.grid(column=0, padx=0, pady=0, row=0, sticky=W)

        self.Frame_1.config(height=200, width=200)
        self.Frame_1.grid(column=0, pady=5, row=1, sticky="NSEW")
        self.Frame_1.grid_rowconfigure(0, weight=1)
        self.Frame_1.grid_rowconfigure(1, weight=1)
        self.Frame_1.grid_columnconfigure(0, weight=1)
        self.Frame_1.grid_columnconfigure(1, weight=1)
        self.Frame_1.grid_columnconfigure(2, weight=1)

        self.voltage_label_1.config(borderwidth=0, text='Voltage: ')
        self.voltage_label_1.grid(column=0, row=1, sticky=W)

        self.voltage_value_1.config(borderwidth=0, text='value')
        self.voltage_value_1.grid(column=1, padx=5, row=1, sticky=W)

        self.analog_scale_1.config(length=100, maximum=3.3, orient='horizontal', value=0)
        self.analog_scale_1.grid(column=4, row=1, sticky="EW")

        self.java_label_1.config(borderwidth=0, text='Java: ')
        self.java_label_1.grid(column=2, row=1, sticky=W)

        self.java_value_1.config(borderwidth=0, text='value')
        self.java_value_1.grid(column=3, padx=5, row=1, sticky=W)


class IoBox:
    def __init__(self, root, analog_add):
        root.grid_columnconfigure(0, weight=1)
        root.grid_rowconfigure(0, weight=1)
        root.grid(sticky="NSEW")
        self.io_pack = tkinter.ttk.Frame(root)
        self.analog_pack = tkinter.ttk.Labelframe(self.io_pack)
        self.digital_pack = tkinter.ttk.Labelframe(self.io_pack)
        self.Button_1 = tkinter.ttk.Button(self.analog_pack)
        self.innerFrame = tkinter.ttk.Frame(self.analog_pack)
        self.innerFrame_1 = tkinter.ttk.Frame(self.digital_pack)

        self.io_pack.config(height=200, width=200)
        self.io_pack.grid(column=0, row=0, sticky="NSEW")
        self.io_pack.grid_rowconfigure(0, weight=1)
        self.io_pack.grid_rowconfigure(1, weight=1)
        self.io_pack.grid_columnconfigure(0, weight=1)

        self.analog_pack.config(height=200, text='Analog I/O', width=200)
        self.analog_pack.grid(column=0, ipadx=5, ipady=5, row=0, sticky="NSEW")
        self.analog_pack.grid_rowconfigure(0, weight=1)
        self.analog_pack.grid_rowconfigure(1, weight=0)
        self.analog_pack.grid_columnconfigure(0, weight=1)

        self.Button_1.config(command=lambda: analog_add())
        self.Button_1.config(text='POLL')
        self.Button_1.grid(column=0, row=1, sticky="EW")

        self.innerFrame.config(height=200, width=200)
        self.innerFrame.grid(column=0, row=0, sticky="NSEW")

        self.digital_pack.config(height=200, text='Digital I/O', width=200)
        self.digital_pack.grid(column=0, row=1, sticky="NSEW")
        self.digital_pack.grid_rowconfigure(0, weight=1)
        self.digital_pack.grid_columnconfigure(0, weight=1)

        self.innerFrame_1.config(height=200, width=200)
        self.innerFrame_1.grid(column=0, row=0, sticky="NSEW")


class ImuBox:
    def __init__(self, root, poll_imu_callback):
        root.grid_columnconfigure(0, weight=1)
        root.grid_rowconfigure(0, weight=1)
        root.grid(sticky="NSEW")
        self.i2c_pack = tkinter.ttk.Labelframe(root)
        self.Label_2 = tkinter.ttk.Label(self.i2c_pack)
        self.Euler_label = tkinter.ttk.Label(self.i2c_pack)
        self.Accel_label = tkinter.ttk.Label(self.i2c_pack)
        self.Euler_value = tkinter.ttk.Label(self.i2c_pack)
        self.Accel_value = tkinter.ttk.Label(self.i2c_pack)
        self.Poll_button = tkinter.ttk.Button(self.i2c_pack)

        self.i2c_pack.config(height=200, text='IMU', width=200)
        self.i2c_pack.grid(column=0, padx=5, pady=5, row=0, sticky="NSEW")

        self.Label_2.config(text='IMU Sensor')
        self.Label_2.grid(column=0, padx=5, pady=5, row=0, sticky="NSEW")

        self.Euler_label.config(text='Heading, Roll, Pitch:')
        self.Euler_label.grid(column=0, columnspan=2, padx=5, pady=5, row=1, sticky=W)

        self.Accel_label.config(text='Gravity (m/s^2):')
        self.Accel_label.grid(column=0, columnspan=2, padx=5, pady=5, row=2, sticky=W)

        self.Euler_value.config(text='n/a')
        self.Euler_value.grid(column=2, padx=5, pady=5, row=1, sticky=W)

        self.Accel_value.config(text='n/a')
        self.Accel_value.grid(column=2, padx=5, pady=5, row=2, sticky=W)

        self.Poll_button.config(command=poll_imu_callback, text='POLL', width=5)
        self.Poll_button.grid(column=1, padx=5, pady=5, row=0)


class I2cChan:
    def __init__(self, root, add_col_callback, poll_col_callback):
        root.grid_columnconfigure(0, weight=1)
        root.grid_rowconfigure(0, weight=1)
        self.Frame_1 = tkinter.ttk.Frame(root)
        self.i2c_pack = tkinter.ttk.Labelframe(self.Frame_1)
        self.Frame_2 = tkinter.ttk.Frame(self.i2c_pack)
        self.I2C_label = tkinter.ttk.Label(self.Frame_2)
        self.Val_label = tkinter.ttk.Label(self.Frame_2)
        self.I2C_value = tkinter.ttk.Label(self.Frame_2)
        self.Config_button = tkinter.ttk.Button(self.Frame_2)
        self.Poll_button = tkinter.ttk.Button(self.Frame_2)

        self.Frame_1.config(height=200, width=200)
        self.Frame_1.grid(column=0, row=0, sticky="NSEW")
        self.Frame_1.grid_rowconfigure(0, weight=1)
        self.Frame_1.grid_columnconfigure(0, weight=1)

        self.i2c_pack.config(height=200, padding=(1, 1, 1, 1), text='test', width=200)
        self.i2c_pack.grid(column=0, row=0, sticky="NSEW")
        self.i2c_pack.grid_rowconfigure(0, weight=1)
        self.i2c_pack.grid_columnconfigure(0, weight=1)

        self.Frame_2.config(borderwidth=2, height=200, width=100)
        self.Frame_2.grid(column=0, row=0, sticky="NSEW")
        self.Frame_2.grid_rowconfigure(0, weight=1)
        self.Frame_2.grid_rowconfigure(1, weight=1)
        self.Frame_2.grid_rowconfigure(2, weight=2)
        self.Frame_2.grid_rowconfigure(3, weight=1)
        self.Frame_2.grid_rowconfigure(4, weight=1)
        self.Frame_2.grid_rowconfigure(5, weight=1)
        self.Frame_2.grid_rowconfigure(6, weight=1)
        self.Frame_2.grid_columnconfigure(0, weight=1)
        self.Frame_2.grid_columnconfigure(1, weight=1)

        self.I2C_label.config(text='I2C Device (default: Color Sensor)')
        self.I2C_label.grid(column=0, padx=5, pady=5, row=0, sticky="NSEW")

        self.Val_label.config(text='Value (default: R,G,B,C,Prox)')
        self.Val_label.grid(column=0, padx=5, pady=5, row=3, sticky=W)

        self.I2C_value.config(text='n/a', width=12)
        self.I2C_value.grid(column=1, columnspan=2, padx=5, pady=5, row=3, sticky="EW")

        self.Config_button.config(command=add_col_callback, text='INIT', width=6)
        self.Config_button.grid(column=1, padx=5, pady=5, row=0, sticky="EW")

        self.Poll_button.config(command=poll_col_callback, text='POLL', state=tkinter.DISABLED, width=10)
        self.Poll_button.grid(column=2, columnspan=1, padx=5, pady=5, row=0, sticky="EW")


class ServoMotor:
    def __init__(self, root, slider_0_callback, java_0_callback, ms_0_callback, slider_1_callback, java_1_callback,
                 ms_1_callback):
        root.grid_columnconfigure(0, weight=1)
        root.grid_rowconfigure(0, weight=1)
        self.Frame_1 = tkinter.ttk.Frame(root)
        self.servo_pack = tkinter.ttk.Labelframe(self.Frame_1)
        self.servo_0 = tkinter.ttk.Frame(self.servo_pack)
        self.servo_1 = tkinter.ttk.Frame(self.servo_pack)
        self.Servo_num_0 = tkinter.ttk.Label(self.servo_0)
        self.Java_label_0 = tkinter.ttk.Label(self.servo_0)
        self.Speed_slider_0 = tkinter.ttk.Scale(self.servo_0)
        self.Java_entry_0 = tkinter.ttk.Entry(self.servo_0)
        self.Ms_entry_0 = tkinter.ttk.Entry(self.servo_0)
        self.Ms_label_0 = tkinter.ttk.Label(self.servo_0)
        self.Java_button_0 = tkinter.ttk.Button(self.servo_0)
        self.Ms_button_0 = tkinter.ttk.Button(self.servo_0)
        self.Servo_num_1 = tkinter.ttk.Label(self.servo_1)
        self.Java_label_1 = tkinter.ttk.Label(self.servo_1)
        self.Speed_slider_1 = tkinter.ttk.Scale(self.servo_1)
        self.Java_entry_1 = tkinter.ttk.Entry(self.servo_1)
        self.Ms_entry_1 = tkinter.ttk.Entry(self.servo_1)
        self.Ms_label_1 = tkinter.ttk.Label(self.servo_1)
        self.Java_button_1 = tkinter.ttk.Button(self.servo_1)
        self.Ms_button_1 = tkinter.ttk.Button(self.servo_1)

        self.Java_entry_0.bind('<Return>', self.update_java0)
        self.java_0_callback = java_0_callback

        self.Ms_entry_0.bind('<Return>', self.update_ms0)
        self.ms_0_callback = ms_0_callback

        self.Java_entry_1.bind('<Return>', self.update_java1)
        self.java_1_callback = java_1_callback

        self.Ms_entry_1.bind('<Return>', self.update_ms1)
        self.ms_1_callback = ms_1_callback

        self.Frame_1.config(borderwidth=5, height=200, width=200)
        self.Frame_1.grid(column=0, row=0, sticky="NSEW")
        self.Frame_1.grid_rowconfigure(0, weight=1)
        self.Frame_1.grid_columnconfigure(0, weight=1)

        self.servo_pack.config(height=200, padding=(1, 1, 1, 1), text='test', width=200)
        self.servo_pack.grid(column=0, row=0, sticky="NSEW")
        self.servo_pack.grid_rowconfigure(0, weight=1)
        self.servo_pack.grid_rowconfigure(1, weight=1)
        self.servo_pack.grid_columnconfigure(0, weight=1)
        self.servo_pack.grid_columnconfigure(1, weight=0)

        self.servo_0.config(borderwidth=2, padding=(1, 1, 1, 1))
        self.servo_0.grid(column=0, padx=1, pady=1, row=0, sticky="NSEW")
        self.servo_0.grid_rowconfigure(0, weight=1)
        self.servo_0.grid_rowconfigure(1, weight=1)
        self.servo_0.grid_columnconfigure(0, weight=1)
        self.servo_0.grid_columnconfigure(1, weight=1)
        self.servo_0.grid_columnconfigure(2, weight=1)
        self.servo_0.grid_columnconfigure(3, weight=1)

        self.Servo_num_0.config(justify='left', text='Servo 0', width=0)
        self.Servo_num_0.grid(column=0, padx=5, pady=5, row=0, sticky="EW")

        self.Java_label_0.config(text='Java (0,1)')
        self.Java_label_0.grid(column=2, padx=5, pady=5, row=1, sticky="NEW")

        self.Speed_slider_0.config(command=slider_0_callback, from_=500, orient='horizontal', to=2500, value=1500)
        self.Speed_slider_0.grid(column=1, padx=5, pady=5, row=0, sticky="EW")

        self.Java_entry_0.config(width=10)
        self.Java_entry_0.grid(column=2, padx=5, pady=5, row=0, sticky="NSEW")

        self.Ms_entry_0.config(width=10)
        self.Ms_entry_0.grid(column=3, padx=5, pady=5, row=0, sticky="NSEW")

        self.Ms_label_0.config(text='MS (500,2500)')
        self.Ms_label_0.grid(column=3, padx=5, pady=5, row=1, sticky="NEW")

        self.Java_button_0.config(command=java_0_callback, text='set', width=3)
        self.Java_button_0.grid(column=2, padx=5, pady=5, row=0, sticky="NSE")

        self.Ms_button_0.config(command=ms_0_callback, text='set', width=3)
        self.Ms_button_0.grid(column=3, padx=5, pady=5, row=0, sticky="NSE")

        self.servo_1.config(borderwidth=2, padding=(1, 1, 1, 1))
        self.servo_1.grid(column=0, padx=1, pady=1, row=1, sticky="NSEW")
        self.servo_1.grid_rowconfigure(0, weight=1)
        self.servo_1.grid_rowconfigure(1, weight=1)
        self.servo_1.grid_columnconfigure(0, weight=1)
        self.servo_1.grid_columnconfigure(1, weight=1)
        self.servo_1.grid_columnconfigure(2, weight=1)
        self.servo_1.grid_columnconfigure(3, weight=1)

        self.Servo_num_1.config(justify='left', text='Servo 1', width=0)
        self.Servo_num_1.grid(column=0, padx=5, pady=5, row=0, sticky="EW")

        self.Java_label_1.config(text='Java (0,1)')
        self.Java_label_1.grid(column=2, padx=5, pady=5, row=1, sticky="NEW")

        self.Speed_slider_1.config(command=slider_1_callback, from_=500, orient='horizontal', to=2500, value=1500)
        self.Speed_slider_1.grid(column=1, padx=5, pady=5, row=0, sticky="EW")

        self.Java_entry_1.config(width=10)
        self.Java_entry_1.grid(column=2, padx=5, pady=5, row=0, sticky="NSEW")

        self.Ms_entry_1.config(width=10)
        self.Ms_entry_1.grid(column=3, padx=5, pady=5, row=0, sticky="NSEW")

        self.Ms_label_1.config(text='MS (500,2500)')
        self.Ms_label_1.grid(column=3, padx=5, pady=5, row=1, sticky="NEW")

        self.Java_button_1.config(command=java_1_callback, text='set', width=3)
        self.Java_button_1.grid(column=2, padx=5, pady=5, row=0, sticky="NSE")

        self.Ms_button_1.config(command=ms_1_callback, text='set', width=3)
        self.Ms_button_1.grid(column=3, padx=5, pady=5, row=0, sticky="NSE")

    # event is defined as an arg but unused so other things can call it (very weird)
    def update_java0(self, event):
        self.java_0_callback()

    def update_java1(self, event):
        self.java_1_callback()

    def update_ms0(self, event):
        self.ms_0_callback()

    def update_ms1(self, event):
        self.ms_1_callback()


class DcMotor:
    def __init__(self, root, speed_slider_callback, speed_button_callback, java_button_callback):
        self.root = root
        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_rowconfigure(0, weight=1)
        self.Frame_5 = tkinter.ttk.Frame(root)
        self.Motor_pack = tkinter.ttk.Labelframe(self.Frame_5)
        self.ZeroButton = tkinter.ttk.Button(self.Motor_pack)
        self.Speed_slider = tkinter.ttk.Scale(self.Motor_pack)
        self.Motor_values = tkinter.ttk.Label(self.Motor_pack)
        self.Java_label = tkinter.ttk.Label(self.Motor_pack)
        self.Speed_button = tkinter.ttk.Button(self.Motor_pack)
        self.Java_entry = tkinter.ttk.Entry(self.Motor_pack)
        self.Java_button = tkinter.ttk.Button(self.Motor_pack)
        self.Controls_label = tkinter.ttk.Label(self.Motor_pack)
        self.Java_entry.bind('<Return>', self.update_java)
        self.java_button_callback = java_button_callback

        self.Frame_5.config(height=200, width=200)
        self.Frame_5.grid(column=0, row=0, sticky="NSEW")
        self.Frame_5.grid_rowconfigure(0, weight=1)
        self.Frame_5.grid_columnconfigure(0, weight=1)

        self.Motor_pack.config(height=200, padding=(5, 5, 5, 5), text='motornum', width=200)
        self.Motor_pack.grid(column=0, padx=5, pady=5, row=0, sticky="NSEW")
        self.Motor_pack.grid_rowconfigure(0, weight=1)
        self.Motor_pack.grid_rowconfigure(1, weight=1)
        self.Motor_pack.grid_rowconfigure(2, weight=1)
        self.Motor_pack.grid_columnconfigure(0, weight=1)
        self.Motor_pack.grid_columnconfigure(1, weight=1)
        self.Motor_pack.grid_columnconfigure(2, weight=2)
        self.Motor_pack.grid_columnconfigure(3, weight=2)

        self.Speed_slider.config(command=speed_slider_callback, from_=-32000, orient='horizontal', to=32000)
        self.Speed_slider.grid(column=1, padx=5, pady=5, row=0, sticky="EW")

        self.Motor_values.config(justify='left', text='Current (mA): %3d\n\nEncoder: %3d' % (0, 0))
        self.Motor_values.grid(column=0, padx=5, pady=5, row=1, sticky="EW")

        self.Java_label.config(text='Speed (-1,1)')
        self.Java_label.grid(column=3, padx=5, pady=5, row=1, sticky="EW")

        self.Speed_button.config(command=speed_button_callback, text='Zero')
        self.Speed_button.grid(column=2, padx=5, pady=5, row=0, sticky=E)

        self.Java_entry.config(width=10)
        self.Java_entry.grid(column=3, padx=5, pady=5, row=0, sticky="NSEW")

        self.Java_button.config(command=java_button_callback, text='set')
        self.Java_button.grid(column=3, padx=5, pady=5, row=0, sticky=E)

        self.Controls_label.config(text='Controls:')
        self.Controls_label.grid(column=0, padx=5, pady=5, row=0, sticky="EW")

    # event argument allows other things to call it (don't ask me why)
    def update_java(self, event):
        self.java_button_callback()


def is_valid_firmware(filename):
    name, ext = os.path.splitext(filename)
    if ext != '.bin':
        return False, "Invalid file name, firmware file extension is '.bin'"
    filesize = os.path.getsize(filename)
    if filesize > 1048576 or filesize < 1000:
        return False, 'Invalid binary size, valid firmware is < 1MB'
    return (
        True, '')


class Application:

    def __init__(self, root):
        self.I2C_packs = []
        self.IMUs = []
        self.IO_packs = []
        self.Digital_panels = []
        self.Analog_panels = []
        self.device_info = []
        self.Servo_packs = []
        self.Motor_packs = []
        self.moduleNames = []
        self.root = root
        self.REVModules = []
        self.commMod = REVcomm()
        self.repetitiveFunctions = []
        self.root.grid_rowconfigure(0, weight=1)
        self.root.grid_columnconfigure(0, weight=1)
        self.Main_window = tkinter.ttk.Frame(root)
        style = tkinter.ttk.Style()
        style.configure("Quit.TButton", foreground='red')
        self.Tab_frame = tkinter.ttk.Notebook(self.Main_window)
        self.Connected_Label = tkinter.ttk.Label(self.Main_window)
        try:
            dir_path = os.path.dirname(os.path.realpath(__file__))
            self.Top_Banner_Image = tk.PhotoImage(file=dir_path + '/resource/banner.gif')
            self.Top_Banner = tkinter.Label(self.Main_window, image=self.Top_Banner_Image)
        except tk.TclError:
            self.Top_Banner = tkinter.Label(self.Main_window)

        self.Connect_button = tkinter.ttk.Button(self.Main_window)
        self.Quit_button = tkinter.ttk.Button(self.Main_window)
        self.DC_Motor = tkinter.ttk.Frame(self.Tab_frame)
        self.Servo_Motor = tkinter.ttk.Frame(self.Tab_frame)
        self.I2C_Device = tkinter.ttk.Frame(self.Tab_frame)
        # self.Firmware_Update = tkinter.ttk.Frame(self.Tab_frame)
        self.IO = tkinter.ttk.Frame(self.Tab_frame)
        self.DC_Motor_frame = tkinter.ttk.Frame(self.DC_Motor)
        self.Servo_Motor_frame = tkinter.ttk.Frame(self.Servo_Motor)
        self.I2C_Device_frame = tkinter.ttk.Frame(self.I2C_Device)
        # self.Firmware_tab = tkinter.ttk.Frame(self.Firmware_Update)
        self.IO_tab = tkinter.ttk.Frame(self.IO)

        self.Main_window.config(height=800, width=900)
        self.Main_window.grid(column=0, row=0, sticky="NSEW")
        self.Main_window.grid_rowconfigure(0, minsize=0, weight=1)
        self.Main_window.grid_rowconfigure(1, minsize=700, weight=1)
        self.Main_window.grid_columnconfigure(0, minsize=450, weight=1)
        self.Main_window.grid_columnconfigure(1, minsize=80, weight=0)
        self.Main_window.grid_columnconfigure(2, minsize=20, weight=0)

        self.Tab_frame.config(height=240, padding=(0, 2, 0, 0), width=320)
        self.Tab_frame.grid(column=0, columnspan=3, padx=5, pady=5, row=1, sticky="NSEW")

        self.Connected_Label.grid(row=0, sticky=E)
        self.Connected_Label.config(text=' Disconnected ', background='red', foreground='white')

        self.Top_Banner.grid(row=0, sticky=W)

        self.DC_Motor_frame.config(height=250, width=200)
        self.DC_Motor_frame.grid(column=0, row=0, sticky="NSEW")
        self.DC_Motor_frame.grid_rowconfigure(0, weight=1)
        self.DC_Motor_frame.grid_columnconfigure(0, minsize=0, weight=1)

        self.Servo_Motor_frame.config(height=200, width=200)
        self.Servo_Motor_frame.grid(column=0, row=0, sticky="NSEW")
        self.Servo_Motor_frame.grid_rowconfigure(0, weight=1)
        self.Servo_Motor_frame.grid_columnconfigure(0, weight=1)

        self.I2C_Device_frame.config(height=200, width=200)
        self.I2C_Device_frame.grid(column=0, row=0, sticky="NSEW")
        self.I2C_Device_frame.grid_rowconfigure(0, weight=1)
        self.I2C_Device_frame.grid_columnconfigure(0, weight=1)

        # self.Firmware_tab = tkinter.ttk.Frame(self.Tab_frame)
        # self.Firmware_tab.config(height=200, padding=(2, 6, 2, 6), width=200)
        # self.Firmware_tab.grid(column=0, row=0, sticky="NSEW")
        # self.Firmware_tab.grid_rowconfigure(0, weight=1)
        # self.Firmware_tab.grid_columnconfigure(0, weight=1)

        self.IO_tab.config(height=200, padding=(2, 6, 2, 6), width=200)
        self.IO_tab.grid(column=0, row=0, sticky="NSEW")
        self.IO_tab.grid_rowconfigure(0, weight=1)
        self.IO_tab.grid_columnconfigure(0, weight=1)

        self.Connect_button.config(command=self.on_connect_button_callback, text='CONNECT', width=10)
        self.Connect_button.grid(column=1, row=0, ipadx=0, ipady=0, padx=5, pady=5, sticky="NSEW")

        self.Quit_button.config(command=self.on_quit_button_callback, text='E-STOP', width=7, style='Quit.TButton')
        self.Quit_button.grid(column=2, row=0, padx=5, pady=5, sticky="NSEW")

        self.Tab_frame.add(self.DC_Motor, text='DC Motors')

        self.DC_Motor.grid_columnconfigure(0, weight=1)
        self.DC_Motor.grid_rowconfigure(0, weight=1)

        self.Tab_frame.add(self.Servo_Motor, text='Servo Motors')

        self.Servo_Motor.grid_columnconfigure(0, weight=1)
        self.Servo_Motor.grid_rowconfigure(0, weight=1)

        self.Tab_frame.add(self.I2C_Device, text='I2C Devices')

        self.I2C_Device.grid_columnconfigure(0, weight=1)
        self.I2C_Device.grid_rowconfigure(0, weight=1)

        self.Tab_frame.add(self.IO, text='GPIO Control')

        self.IO.grid_columnconfigure(0, weight=1)
        self.IO.grid_rowconfigure(0, weight=1)

        # self.Tab_frame.add(self.Firmware_Update, text='Firmware')
        # self.Firmware_Update.grid_columnconfigure(0, weight=1)
        # self.Firmware_Update.grid_rowconfigure(0, weight=1)
        # self.buildFirmwareFrame()

    def send_all_ka(self):
        for module in self.REVModules:
            is_alive = module.sendKA()
            if not is_alive:
                self.on_quit_button_callback()
                self.Connected_Label.config(text=' Disconnected ', background='red', foreground='white')
            else:
                self.Connected_Label.config(text=' Connected ', background='green', foreground='white')
                module.getStatus()

    def speed_motor_slider(self, speed, module_number, motor_number):
        self.Motor_packs[module_number * 4 + motor_number].Java_entry.delete(0, END)
        self.Motor_packs[module_number * 4 + motor_number].Java_entry.insert(0, '%.2f' % (float(speed) / 32000))
        self.REVModules[module_number].motors[motor_number].setPower(float(speed))
        self.REVModules[module_number].motors[motor_number].enable()
        self.repetitiveFunctions = [
            (lambda: self.send_all_ka())]
        self.repetitiveFunctions.append((lambda: self.update_motor_labels(motor_number, module_number)))
        return True

    def speed_motor_entry(self, motor_number, module_number):
        speed = 0
        self.Motor_packs[module_number * 4 + motor_number].Speed_slider.set(speed)
        self.Motor_packs[module_number * 4 + motor_number].Java_entry.delete(0, END)
        self.Motor_packs[module_number * 4 + motor_number].Java_entry.insert(0, '%.2f' % float(speed / 32000))
        self.REVModules[module_number].motors[motor_number].setPower(float(speed))
        self.REVModules[module_number].motors[motor_number].enable()
        self.repetitiveFunctions = [
            (lambda: self.send_all_ka())]
        self.repetitiveFunctions.append((lambda: self.update_motor_labels(motor_number, module_number)))
        return True

    def java_motor_entry(self, motor_number, module_number):
        try:
            speed = float(self.Motor_packs[module_number * 4 + motor_number].Java_entry.get())
        except ValueError:
            print('Invalid speed entered: ' + self.Motor_packs[module_number * 4 + motor_number].Java_entry.get())
            return False

        self.Motor_packs[module_number * 4 + motor_number].Speed_slider.set(speed * 32000)
        self.REVModules[module_number].motors[motor_number].setPower(float(speed * 32000))
        self.REVModules[module_number].motors[motor_number].enable()
        self.repetitiveFunctions = [
            (lambda: self.send_all_ka())]
        self.repetitiveFunctions.append((lambda: self.update_motor_labels(motor_number, module_number)))
        return True

    def update_motor_labels(self, motor_number, module_number):
        current = self.REVModules[int(module_number)].motors[motor_number].getCurrent()
        position = self.REVModules[int(module_number)].motors[motor_number].getPosition()
        self.Motor_packs[module_number * 4 + motor_number].Motor_values.config(
            text='Current (mA): %3d\n\nEncoder: %3d' % (current, position))

    def servo_slider(self, pulse, module_number, servo_number):
        if servo_number % 2 == 0:
            pulse = float(pulse)
            self.Servo_packs[module_number * 3 + int(servo_number / 2)].Java_entry_0.delete(0, END)
            self.Servo_packs[module_number * 3 + int(servo_number / 2)].Java_entry_0.insert(0, '%.2f' % float(
                (pulse - 500) / 2000))
            self.Servo_packs[module_number * 3 + int(servo_number / 2)].Ms_entry_0.delete(0, END)
            self.Servo_packs[module_number * 3 + int(servo_number / 2)].Ms_entry_0.insert(0, '%.2f' % float(pulse))
        else:
            pulse = float(pulse)
            self.Servo_packs[module_number * 3 + int(servo_number / 2)].Java_entry_1.delete(0, END)
            self.Servo_packs[module_number * 3 + int(servo_number / 2)].Java_entry_1.insert(0, '%.2f' % float(
                (pulse - 500) / 2000))
            self.Servo_packs[module_number * 3 + int(servo_number / 2)].Ms_entry_1.delete(0, END)
            self.Servo_packs[module_number * 3 + int(servo_number / 2)].Ms_entry_1.insert(0, '%.2f' % float(pulse))
        self.REVModules[int(module_number)].servos[servo_number].setPulseWidth(pulse)
        self.REVModules[int(module_number)].servos[servo_number].enable()
        self.repetitiveFunctions = [
            (lambda: self.send_all_ka())]
        return True

    def servo_java(self, servo_number, module_number):
        if servo_number % 2 == 0:
            try:
                pulse = float(self.Servo_packs[module_number * 3 + int(servo_number / 2)].Java_entry_0.get())
            except ValueError:
                print('Invalid value entered: ' + self.Servo_packs[
                    module_number * 3 + int(servo_number / 2)].Java_entry_0.get())
                return False

            self.Servo_packs[module_number * 3 + int(servo_number / 2)].Ms_entry_0.delete(0, END)
            self.Servo_packs[module_number * 3 + int(servo_number / 2)].Ms_entry_0.insert(0, '%.2f' % float(
                pulse * 2000 + 500))
            self.Servo_packs[module_number * 3 + int(servo_number / 2)].Speed_slider_0.set(pulse * 2000 + 500)
        else:
            try:
                pulse = float(self.Servo_packs[module_number * 3 + int(servo_number / 2)].Java_entry_1.get())
            except ValueError:
                print('Invalid value entered: ' + self.Servo_packs[
                    module_number * 3 + int(servo_number / 2)].Java_entry_1.get())
                return False

            self.Servo_packs[module_number * 3 + int(servo_number / 2)].Ms_entry_1.delete(0, END)
            self.Servo_packs[module_number * 3 + int(servo_number / 2)].Ms_entry_1.insert(0, '%.2f' % float(
                pulse * 2000 + 500))
            self.Servo_packs[module_number * 3 + int(servo_number / 2)].Speed_slider_1.set(pulse * 2000 + 500)
        self.REVModules[int(module_number)].servos[servo_number].setPulseWidth(pulse * 2000 + 500)
        self.REVModules[int(module_number)].servos[servo_number].enable()
        self.repetitiveFunctions = [
            (lambda: self.send_all_ka())]
        return True

    def servo_ms(self, servo_number, module_number):
        if servo_number % 2 == 0:
            try:
                pulse = float(self.Servo_packs[module_number * 3 + int(servo_number / 2)].Ms_entry_0.get())
            except ValueError:
                print('Invalid value entered: ' + self.Servo_packs[
                    module_number * 3 + int(servo_number / 2)].Ms_entry_0.get())
                return False

            self.Servo_packs[module_number * 3 + int(servo_number / 2)].Java_entry_0.delete(0, END)
            self.Servo_packs[module_number * 3 + int(servo_number / 2)].Java_entry_0.insert(0, '%.2f' % float(
                (pulse - 500) / 2000))
            self.Servo_packs[module_number * 3 + int(servo_number / 2)].Speed_slider_0.set(pulse)
        else:
            try:
                pulse = float(self.Servo_packs[module_number * 3 + int(servo_number / 2)].Ms_entry_1.get())
            except ValueError:
                print('Invalid value entered: ' + self.Servo_packs[
                    module_number * 3 + int(servo_number / 2)].Ms_entry_1.get())
                return False

            self.Servo_packs[module_number * 3 + int(servo_number / 2)].Java_entry_1.delete(0, END)
            self.Servo_packs[module_number * 3 + int(servo_number / 2)].Java_entry_1.insert(0, '%.2f' % float(
                (pulse - 500) / 2000))
            self.Servo_packs[module_number * 3 + int(servo_number / 2)].Speed_slider_1.set(pulse)
        self.REVModules[int(module_number)].servos[servo_number].setPulseWidth(float(pulse))
        self.REVModules[int(module_number)].servos[servo_number].enable()
        self.repetitiveFunctions = [
            (lambda: self.send_all_ka())]
        return True

    def color_sense_add(self, module_number, bus_number):
        self.I2C_packs[module_number * 4 + bus_number].Config_button.config(text='Wait')
        self.root.update_idletasks()

        # TODO: what is even happening with this code. why is sensor even defined. why is there a blank except. ?????
        is_2m_sensor = False
        sensor = None
        try:
            sensor = REV2mSensor(self.commMod, bus_number, self.REVModules[module_number].getModuleAddress())
            is_2m_sensor = sensor.Is2mDistanceSensor()
        except:  # TODO: catch specific exception here!
            pass

        is_initialized = False
        if is_2m_sensor & (sensor is not None):
            self.I2C_packs[module_number * 4 + bus_number].I2C_label.config(
                text='2m Distance Sensor                     ')
            self.I2C_packs[module_number * 4 + bus_number].Val_label.config(text='Value (Distance mm)    ')
            self.REVModules[module_number].i2cChannels[bus_number].addI2CDevice(
                str(module_number) + 'COL' + str(bus_number), sensor)
            if self.REVModules[module_number].i2cChannels[bus_number].getDevices()[
                str(module_number) + 'COL' + str(bus_number)].initialize():
                self.I2C_packs[module_number * 4 + bus_number].I2C_value.config(text='REV 2m Distance Sensor Found')
                is_initialized = True
        else:
            cs = REVColorSensorV3(self.commMod, bus_number, self.REVModules[module_number].getModuleAddress())
            if cs.initSensor():
                self.I2C_packs[module_number * 4 + bus_number].I2C_label.config(
                    text='Color Sensor V3                        ')
                self.REVModules[module_number].i2cChannels[bus_number].addI2CDevice(
                    str(module_number) + 'COL' + str(bus_number), cs)
                is_initialized = True
                self.I2C_packs[module_number * 4 + bus_number].I2C_value.config(text='Color Sensor V3 Found')
            else:
                self.REVModules[module_number].i2cChannels[bus_number].addColorSensor(
                    str(module_number) + 'COL' + str(bus_number))
                if self.REVModules[module_number].i2cChannels[bus_number].getDevices()[
                    str(module_number) + 'COL' + str(bus_number)].initSensor():
                    self.I2C_packs[module_number * 4 + bus_number].I2C_value.config(text='Color Sensor V2 Found')
                    is_initialized = True
                self.I2C_packs[module_number * 4 + bus_number].I2C_label.config(
                    text='I2C Device (default: Color Sensor)')
            self.I2C_packs[module_number * 4 + bus_number].Val_label.config(text='Value (default: R,G,B,C,Prox)')
        self.I2C_packs[module_number * 4 + bus_number].Config_button.config(text='INIT')
        if is_initialized:
            self.I2C_packs[module_number * 4 + bus_number].Poll_button.config(state=tkinter.NORMAL)
        else:
            self.I2C_packs[module_number * 4 + bus_number].I2C_value.config(text='Device did not initialize')

    def color_sense_poll(self, module_number, bus_number):
        sensor_type = self.REVModules[module_number].i2cChannels[bus_number].getDevices()[
            str(module_number) + 'COL' + str(bus_number)].getType()
        self.repetitiveFunctions = [(lambda: self.send_all_ka())]
        if sensor_type == 'REV2mSensor':
            self.repetitiveFunctions.append((lambda: self.update_2m_sensor(module_number, bus_number)))
        elif sensor_type == 'REVColorSensorV3':
            self.repetitiveFunctions.append((lambda: self.update_color_device_v3(module_number, bus_number)))
        else:
            self.repetitiveFunctions.append((lambda: self.update_color_device(module_number, bus_number)))

    def update_2m_sensor(self, module_number, bus_number):
        distance_mm = self.REVModules[module_number].i2cChannels[bus_number].getDevices()[
            str(module_number) + 'COL' + str(bus_number)].readRangeContinuousMillimeters()
        color_string = str(distance_mm) + 'mm'
        self.I2C_packs[module_number * 4 + bus_number].I2C_value.config(text=color_string)
        self.I2C_packs[module_number * 4 + bus_number].Val_label.config(text='Value (Distance mm)')

    def update_color_device(self, module_number, bus_number):
        red = self.REVModules[module_number].i2cChannels[bus_number].getDevices()[
            str(module_number) + 'COL' + str(bus_number)].getRedValue()
        green = self.REVModules[module_number].i2cChannels[bus_number].getDevices()[
            str(module_number) + 'COL' + str(bus_number)].getGreenValue()
        blue = self.REVModules[module_number].i2cChannels[bus_number].getDevices()[
            str(module_number) + 'COL' + str(bus_number)].getBlueValue()
        clear = self.REVModules[module_number].i2cChannels[bus_number].getDevices()[
            str(module_number) + 'COL' + str(bus_number)].getClearValue()
        prox = self.REVModules[module_number].i2cChannels[bus_number].getDevices()[
            str(module_number) + 'COL' + str(bus_number)].getProxValue()
        color_string = str(red) + ', ' + str(green) + ', ' + str(blue) + ', ' + str(clear) + ', ' + str(prox)
        self.I2C_packs[module_number * 4 + bus_number].I2C_value.config(text=color_string)
        if len(color_string) > 28:
            self.I2C_packs[module_number * 4 + bus_number].I2C_value.config(font=('TkDefaultFont',
                                                                                  8))
        else:
            self.I2C_packs[module_number * 4 + bus_number].I2C_value.config(font=('TkDefaultFont',
                                                                                  10))

    def update_color_device_v3(self, module_number, bus_number):
        red, green, blue, ir, prox = self.REVModules[module_number].i2cChannels[bus_number].getDevices()[
            str(module_number) + 'COL' + str(bus_number)].getAll()
        clear = red + green + blue - 2 * ir
        color_string = str(red) + ', ' + str(green) + ', ' + str(blue) + ', ' + str(clear) + ', ' + str(prox)
        self.I2C_packs[module_number * 4 + bus_number].I2C_value.config(text=color_string)

    def imu_add(self, module_number):
        self.REVModules[module_number].i2cChannels[0].addIMU(str(module_number) + 'IMU')
        self.REVModules[module_number].i2cChannels[0].getDevices()[str(module_number) + 'IMU'].initSensor()
        self.repetitiveFunctions = [
            (lambda: self.send_all_ka())]
        self.repetitiveFunctions.append((lambda: self.update_imu_device(module_number, 0)))

    def update_imu_device(self, module_number, bus_number):
        heading, roll, pitch = self.REVModules[module_number].i2cChannels[bus_number].getDevices()[
            str(module_number) + 'IMU'].getAllEuler()
        gx, gy, gz = self.REVModules[module_number].i2cChannels[bus_number].getDevices()[
            str(module_number) + 'IMU'].getGravity()
        euler_string = '%2.3f, %2.3f, %2.3f' % (heading, roll, -pitch)
        linacc_string = 'X: %2.3f, Y: %2.3f, Z: %2.3f' % (gx, gy, gz)
        self.IMUs[module_number].Euler_value.config(text=euler_string)
        self.IMUs[module_number].Accel_value.config(text=linacc_string)

    def analog_add(self, module_number):
        self.repetitiveFunctions = [
            (lambda: self.send_all_ka())]
        self.repetitiveFunctions.append((lambda: self.analog_update(module_number)))

    def analog_update(self, module_number):
        for i in range(0, 4):
            adc_data = self.REVModules[module_number].adcPins[i].getADC(0)
            self.Analog_panels[module_number * 4 + i].voltage_value_1.config(
                text=str(float(adc_data) / 1000.0) + ' volts')
            self.Analog_panels[module_number * 4 + i].java_value_1.config(text=str(float(adc_data) / 1000.0))
            self.Analog_panels[module_number * 4 + i].analog_scale_1.config(value=float(adc_data) / 1000.0)

    def digital_set_as_output(self, module_number, dio_number):
        self.repetitiveFunctions = [(lambda: self.send_all_ka())]
        self.REVModules[module_number].dioPins[module_number * 2 + dio_number].setAsOutput()
        # TODO: use styles here, see https://docs.python.org/3/library/tkinter.ttk.html#ttk-widgets
        self.Digital_panels[module_number * 8 + dio_number].output_button.config(background='#aaccff')
        self.Digital_panels[module_number * 8 + dio_number].input_button.config(background='#ffffff')
        self.Digital_panels[module_number * 8 + dio_number].poll_button.config(state='disabled')
        self.Digital_panels[module_number * 8 + dio_number].Checkbutton_1.config(state='normal')
        self.Digital_panels[module_number * 8 + dio_number].var2.set(1)

    def digital_set_as_input(self, module_number, dio_number):
        self.REVModules[module_number].dioPins[module_number * 2 + dio_number].setAsInput()
        # TODO: use styles here, see https://docs.python.org/3/library/tkinter.ttk.html#ttk-widgets
        self.Digital_panels[module_number * 8 + dio_number].input_button.config(background='#aaccff')
        self.Digital_panels[module_number * 8 + dio_number].output_button.config(background='#ffffff')
        self.Digital_panels[module_number * 8 + dio_number].poll_button.config(state='normal')
        self.Digital_panels[module_number * 8 + dio_number].Checkbutton_1.config(state='disabled')

    def digital_set_callback(self, module_number, dio_number):
        self.REVModules[module_number].dioPins[module_number * 2 + dio_number].setOutput(
            int(self.Digital_panels[module_number * 8 + dio_number].var2.get()))

    def digital_add(self, module_number, dio_number):
        self.repetitiveFunctions = [
            (lambda: self.send_all_ka())]
        self.repetitiveFunctions.append((lambda: self.digital_update(module_number, dio_number)))

    def digital_update(self, module_number, dio_number):
        value = self.REVModules[module_number].dioPins[dio_number].getInput()
        if not int(value):
            self.Digital_panels[module_number * 8 + dio_number].var2.set(0)
        else:
            self.Digital_panels[module_number * 8 + dio_number].var2.set(1)

    def check_for_modules(self):
        self.REVModules = []
        self.REVModules = self.commMod.discovery()
        self.repetitiveFunctions.append((lambda: self.send_all_ka()))
        self.moduleNames = []
        for i in range(0, len(self.REVModules)):
            self.moduleNames.append('REV Expansion Hub ' + str(i))

        return self.moduleNames

    def set_address_callback(self, module_number):
        addr = int(self.device_info[module_number].addr_entry.get())
        if addr < 1 or addr > 255:
            return
        self.REVModules[module_number].setAddress(addr)
        for i in range(0, len(self.REVModules)):
            self.device_info[i].addr_entry.delete(0, END)
            self.device_info[i].addr_entry.insert(0, str(self.REVModules[i].getModuleAddress()))

    def on_connect_button_callback(self):
        self.commMod.open_active_port()
        module_tot = len(self.check_for_modules())
        self.Quit_button.config(state='enabled')
        for tab in self.Tab_frame.tabs():
            self.Tab_frame.tab(tab, state='normal')

        self.Motor_packs = []
        for module_number in range(0, module_tot):
            for motor_number in range(0, 4):
                self.DC_Motor_frame.grid_rowconfigure(motor_number, weight=1)
                self.DC_Motor_frame.grid_columnconfigure(module_number, weight=1)
                frame = tkinter.ttk.Frame(self.DC_Motor_frame, borderwidth=5)
                frame.grid(row=motor_number, column=module_number, sticky="NSEW")
                self.Motor_packs.append(
                    DcMotor(frame,
                            partial(self.speed_motor_slider, motor_number=motor_number, module_number=module_number),
                            partial(self.speed_motor_entry, motor_number=motor_number, module_number=module_number),
                            partial(self.java_motor_entry, motor_number=motor_number, module_number=module_number)))
                self.Motor_packs[-1].Motor_pack.config(
                    text='Module: ' + str(module_number) + ' Motors: ' + str(motor_number))

        self.Servo_packs = []
        for module_number in range(0, module_tot):
            for motor_number in range(0, 3):
                self.Servo_Motor_frame.grid_rowconfigure(motor_number, weight=1)
                self.Servo_Motor_frame.grid_columnconfigure(module_number, weight=1)
                frame = tkinter.ttk.Frame(self.Servo_Motor_frame, borderwidth=5)
                frame.grid(row=motor_number, column=module_number, sticky="NSEW")
                self.Servo_packs.append(ServoMotor(frame, partial(self.servo_slider, servo_number=2 * motor_number,
                                                                  module_number=module_number),
                                                   partial(self.servo_java, servo_number=motor_number * 2,
                                                           module_number=module_number),
                                                   partial(self.servo_ms, servo_number=motor_number * 2,
                                                           module_number=module_number),
                                                   partial(self.servo_slider, servo_number=motor_number * 2 + 1,
                                                           module_number=module_number),
                                                   partial(self.servo_java, servo_number=motor_number * 2 + 1,
                                                           module_number=module_number),
                                                   partial(self.servo_ms, servo_number=motor_number * 2 + 1,
                                                           module_number=module_number)))
                self.Servo_packs[-1].servo_pack.config(
                    text='Module: ' + str(module_number) + ' Motors: ' + str(motor_number * 2) + ' & ' + str(
                        motor_number * 2 + 1))

        self.I2C_packs = []
        self.IMUs = []
        for module_number in range(0, module_tot):
            self.I2C_Device_frame.grid_rowconfigure(0, weight=1)
            self.I2C_Device_frame.grid_columnconfigure(module_number, weight=1)
            frame = tkinter.ttk.Frame(self.I2C_Device_frame, borderwidth=5)
            frame.grid(row=0, column=module_number, sticky=W)
            self.IMUs.append(ImuBox(frame, partial(self.imu_add, module_number)))
            for i2cNumber in range(0, 4):
                self.I2C_Device_frame.grid_rowconfigure(i2cNumber, weight=1)
                self.I2C_Device_frame.grid_columnconfigure(module_number, weight=1)
                frame = tkinter.ttk.Frame(self.I2C_Device_frame, borderwidth=5)
                frame.grid(row=i2cNumber + 1, column=module_number, sticky="NSEW")
                self.I2C_packs.append(
                    I2cChan(frame, partial(self.color_sense_add, bus_number=i2cNumber, module_number=module_number),
                            partial(self.color_sense_poll, bus_number=i2cNumber, module_number=module_number)))
                self.I2C_packs[-1].i2c_pack.config(text='Module: ' + str(module_number) + ' I2C Bus: ' + str(i2cNumber))

        self.IO_packs = []
        self.Digital_panels = []
        self.Analog_panels = []
        for module_number in range(0, module_tot):
            self.IO_tab.grid_columnconfigure(module_number, weight=1)
            frame = tkinter.ttk.Frame(self.IO_tab, borderwidth=5)
            frame.grid(row=0, column=module_number, sticky="NSEW")
            self.IO_packs.append(IoBox(frame, partial(self.analog_add, module_number)))
            self.IO_packs[-1].analog_pack.config(text='Analog Inputs Module: ' + str(module_number))
            self.IO_packs[-1].digital_pack.config(text='Digital Input/Outputs Module: ' + str(module_number))
            self.IO_packs[-1].innerFrame.grid_columnconfigure(0, weight=1)
            for i in range(0, 4):
                frame = tkinter.ttk.Frame(self.IO_packs[-1].innerFrame, borderwidth=5)
                frame.grid(row=i, column=0, sticky="NSEW")
                self.IO_packs[-1].innerFrame.grid_rowconfigure(i, weight=1)
                self.Analog_panels.append(AnalogSingle(frame))
                self.Analog_panels[-1].analog_label_1.config(text=str('Analog ' + str(i)))

            for i in range(0, 4):
                for j in range(0, 2):
                    frame = tkinter.ttk.Frame(self.IO_packs[-1].innerFrame_1, borderwidth=5)
                    frame.grid(row=i, column=j, sticky="NSEW")
                    self.IO_packs[-1].innerFrame_1.grid_rowconfigure(i, weight=1)
                    self.IO_packs[-1].innerFrame_1.grid_columnconfigure(j, weight=1)
                    self.Digital_panels.append(
                        DigitalSingle(frame, partial(self.digital_set_as_input, module_number, i * 2 + j),
                                      partial(self.digital_set_as_output, module_number, i * 2 + j),
                                      partial(self.digital_set_callback, module_number, i * 2 + j),
                                      partial(self.digital_add, module_number, i * 2 + j)))
                    self.Digital_panels[-1].digital_label_1.config(text=str(i * 2 + j))

        self.device_info = []
        """
        for module_number in range(0, module_tot):
             frame = tkinter.ttk.Frame(self.firmware.Device_info_frame1, borderwidth=5)
             frame.grid(row=1, column=module_number, sticky="NSEW")
             self.device_info.append(device_info(frame, partial(self.set_address_callback, 
                                                                module_number=module_number)))
             self.device_info[-1].addr_entry.delete(0, END)
             self.device_info[-1].addr_entry.insert(0, str(self.REVModules[module_number].getModuleAddress()))
             self.device_info[-1].device_label.config(text='Module: ' + str(module_number))
        """
        # self.firmware.firmware_label.config(text='Interface Version: ' + self.firmware.INTERFACE_VERSION +
        #                                         '\nFirmware Version: ' + self.REVModules[0].getVersionString())
        self.root.after(500, self.every_second)

    """
     def buildFirmwareFrame(self):
         frame = tkinter.ttk.Frame(self.Firmware_tab, borderwidth=5)
         frame.grid(row=0, column=0, sticky="NSEW")
         self.firmware = firmware_tab(frame, partial(self.firmware_bin_select), partial(self.firmware_flash))
         self.firmware.warning_block.insert(END, 'Firmware update to be performed to the Expansion Hub '
                                                 'connected via USB only. '
                                                 '\n\t\t\nFirmware update is to be performed with only REV qualified '
                                                 '.bin files located in the default installation directory'
                                                 '\n\t\t\n\nWARNING: incorrect firmware can brick the device.'
                                                 '\n\nModified firmware files are not FTC legal.\n')
         self.firmware.warning_block.config(state='disabled')
     """

    def on_quit_button_callback(self):
        for module in self.REVModules:
            module.killSwitch()

        self.repetitiveFunctions = []
        self.commMod.close_active_port()
        self.Quit_button.config(state='disabled')
        self.Connected_Label.config(text=' Disconnected ', background='red', foreground='white')
        for i in range(0, len(self.Tab_frame.tabs())):
            if i < 4:
                self.Tab_frame.tab(i, state='disabled')

    def every_second(self):
        for func in self.repetitiveFunctions:
            func()
        self.root.after(250, self.every_second)

    def join_threads(self):
        self.repetitiveFunctions = []
        self.commMod.close_active_port()
        self.root.quit()

    """ commented out: firmware doesn't work yet
    def firmware_bin_select(self):
        tmp_filename = tkinter.filedialog.askopenfilename(initialdir='./', title='Select file',
                                                          filetypes=(('bin files', '*.bin'), ('all files', '*.*')))
        if tmp_filename is None or tmp_filename == '':
            return
        is_valid, err = is_valid_firmware(tmp_filename)
        if not is_valid:
            err_msg = 'Attempted to open invalid firmware file: ' + tmp_filename + '\r\n' + err
            tkinter.messagebox.showinfo('Invalid Firmware', err_msg)
            print(err_msg)
        self.filename = tmp_filename
        return

    def firmware_flash(self):
        try:
            if self.filename is None or self.filename == '':
                return
        except:  # TODO: uncertain why this is here. Remove or specify exception?
            return

        is_valid, err = is_valid_firmware(self.filename)
        if not is_valid:
            err_msg = ('Attempted to use an invalid firmware file: ' + self.filename + '\r\n' + err +
                       '\r\n\r\nNo action will be done')
            tkinter.messagebox.showinfo('Invalid Firmware', err_msg)
            self.firmware.warning_block.config(state='disabled')
            return
        else:
            self.on_quit_button_callback()
            ftserial = ''
            device_list = ft232.list_devices()
            if device_list:
                print(device_list)
                for FTDI_device in device_list:
                    for element in FTDI_device:
                        self.firmware.warning_block.config(state='normal')
                        if 'FT230X' in element:
                            print('element: ', element)
                            ftserial = FTDI_device[0]
                            print('serial: ', ftserial)
                        else:
                            self.firmware.warning_block.insert(END, 'looking for FT230X\n')
            else:
                self.firmware.warning_block.insert(END, 'no FTDI devices found\n')
                exit()
            if ftserial != '':
                ftdi_handle = ft232.Ft232(ftserial, baudrate=115200)
                ftdi_handle.cbus_setup(mask=3, init=3)
                ftdi_handle.cbus_write(0)
                time.sleep(0.1)
                ftdi_handle.cbus_write(1)
                time.sleep(0.1)
                ftdi_handle.cbus_write(3)
                self.firmware.warning_block.insert(END, 'board is in program mode, LED should not be flashing\n')
                ftdi_handle.close()
            else:
                self.firmware.warning_block.insert(END, 'did not find FT230X but found other FTDI parts\n')
                self.on_connect_button_callback()
                return
            port = ''
            if self.commMod.REVProcessor.port is not None:
                port = self.commMod.REVProcessor.port[3:]
            else:
                all_ports = self.commMod.listPorts()
                if len(all_ports) == 0:
                    err_msg = 'No available com ports, verify connection and try again.\n'
                    tkinter.messagebox.showinfo('Invalid Firmware', err_msg)
                    self.firmware.warning_block.insert(END, err_msg)
                else:
                    port = all_ports[0].getNumber()
            if port != '':
                if platform.system() == 'Linux':
                    print('Linux detected, using no extension for sflash executable\n')
                    os_extension = ''
                else:
                    os_extension = '.exe'
                cmd_line = [
                    'sflash' + os_extension, self.filename, '-c', port, '-b', '230400', '-s', '252']
                status_msg = '\n\nProgramming HUB: COM' + port + ' with file ' + self.filename + '\n\n'
                status_msg = status_msg + ' '.join(cmd_line) + '\n\nDO NOT REMOVE POWER WHILE PROGRAMMING...\n\n'
                self.firmware.warning_block.config(state='normal')
                self.firmware.warning_block.insert(END, status_msg)
                self.root.update_idletasks()
                self.firmware.warning_block.config(state='disabled')
                subprocess.call(cmd_line)
            else:
                err_msg = 'Com port failure, detected Com as: ' + port + '\r\nCheck connection and try again\n'
                tkinter.messagebox.showinfo('Invalid Firmware', err_msg)
                self.firmware.warning_block.insert(END, err_msg)
                self.on_connect_button_callback()
                self.firmware.warning_block.config(state='disabled')
                return
            self.firmware.warning_block.insert(END,
                                               'Programming Complete, status LED should be blinking,'
                                               '\nyou can now connect to the hub.')
            self.firmware.warning_block.config(state='disabled')

            self.root.update_idletasks()
            self.on_connect_button_callback()
            return
        """


def initwindow():
    mp.freeze_support()

    xroot = tk.Tk()

    # Attempt to import and load the Sun Valley theme
    try:
        import sv_ttk
        sv_ttk.set_theme("dark")
        print('Loaded Tk theme: Sun Valley')
    except Exception as e:
        # Print error, then fall back to default ttk theme
        print(e)
        pass

    xroot.title('REV Hub Interface - Community Edition - v1.3.1')
    """ commented out: no icon yet
    try:
        xroot.iconbitmap('resource\\\\favicon.ico')
    except:
        try:
            xroot.iconbitmap('favicon.ico')
        except:
            pass
    """

    app = Application(xroot)
    xroot.protocol('WM_DELETE_WINDOW', app.join_threads)
    print('Loading application...')
    xroot.mainloop()


if __name__ == "__main__":
    initwindow()
