"""

Author: Francisco Branco
Date: 04/07/2021
Description: Program to monitor temperature over time
             using a temperature probe.

"""


import serial
import time
import sys
import os
from matplotlib import pyplot as plt
from threading import Thread
from kivy.app import App
from kivy.uix.boxlayout import BoxLayout
from kivy.config import Config
from kivy.core.window import Window
from kivy.garden.matplotlib.backend_kivyagg import FigureCanvasKivyAgg


# Main widget for interface application
class RootWidget(BoxLayout):
    orientation = "horizontal"
    
    def __init__(self):
        super().__init__()
        self.add_widget(FigureCanvasKivyAgg(plt.gcf()))

    # Method that handles the press of the stop button
    def stop_button(self):
        if self.ids.start_button.text == "Start":
            self.ids.console.text = self.ids.console.text + "Program is not running! Click start button to start.\n"
            return
        else:
            self.ids.start_button.text = "Start"

    # Method that handles the press of the start button
    def start_button(self):
        # Check if program is running
        if self.ids.start_button.text != "Start":
            self.ids.console.text = self.ids.console.text + "Program is still running! Click stop button to stop.\n"
            return
        # Check if file is there, if not, create it
        try:
            test_file = open("./save_files/" + self.ids.name_input.text + ".csv", "wt")
            test_file.close()
        except TypeError as exc:
            self.ids.console.text = self.ids.console.text + "There was an error: " + str(exc) + ". Could not open specified filename.\n"
            return
        
        # Check if COMs are valid
        if "COM" not in self.ids.pcr_port.text or "COM" not in self.ids.control_port.text:
            self.ids.console.text = self.ids.console.text + "COMs not correctly specified. The right format is: COM0, where 0 can be any number.\n"
            return
        
        # Check the state of the program
        if self.ids.start_button.text == "Start":
            self.ids.start_button.text = "Running"
            
            PCR_port = self.ids.pcr_port.text
            control_port = self.ids.control_port.text

            self.comm_thread = Thread(target=self.communication, args=(PCR_port, control_port, 9600, 1, self.ids.name_input.text))
            self.comm_thread.start()
            
    # Communication setup method
    def communication(self, PCR_port, control_port, baudrate, timeout, trial_name="test"):
        # Setup serial connection to PCR thermocycler
        try:
            PCR_connection = serial.Serial(port=PCR_port, baudrate=baudrate, timeout=timeout)
            PCR_connection.flushInput()
        except serial.SerialException as err:
            self.ids.console.text = self.ids.console.text + "Error while creating PCR serial connection: " + str(err) + "\n"
            self.ids.start_button.text = "Start"
            return
        # Setup serial connection for monitoring temperature probe
        try:
            control_connection = serial.Serial(port=control_port, baudrate=baudrate, timeout=timeout)
            control_connection.flushInput()
        except serial.SerialException as err:
            self.ids.console.text = self.ids.console.text + "Error while creating control serial connection: " + str(err) + "\n"
            self.ids.start_button.text = "Start"
            return

        time.sleep(2)

        #Setup dictionaries for value lists
        data = {"time": [], "Temp": []}


        # Setup connection with PCR thermocycler
        while True:
            pcr_msg = PCR_connection.readline().decode().replace('\n', '').replace('\r', '')
            print(pcr_msg)
            if len(pcr_msg) > 0:
                if pcr_msg != "START PROGRAM":
                    if pcr_msg == "ERROR":
                        self.ids.console.text = self.ids.console.text + "Received error messaged: " + pcr_msg + "\n"
                        self.ids.start_button.text = "Start"
                        return
                    self.ids.console.text = self.ids.console.text + "Received wrong messaged: " + pcr_msg + "\n"
                else:
                    break
        # Answer to monitoring temperature probe
        control_connection.write(bytes("READY TO RECEIVE", 'utf-8'))

        # Connect to PCR thermocycler to start registering data
        self.register_loop(trial_name, data, control_connection)

        # Close serial connections
        PCR_connection.close()
        control_connection.close()

    # Registering loop method to get the data, save and plot it
    def register_loop(self, trial_name, data, control_connection):
        # Open and setup file
        f = open("./save_files/" + trial_name + ".csv", "wt")
        f.write("time,Temperature,\n")
        f.close()
        
        # Begin loop
        start = -1
        last_index = -1
        last_save = time.mktime(time.gmtime())
        plt.ion()
        while self.ids.start_button.text == "Running":
            # Read first message sent by PCR thermocycler
            new_line = control_connection.readline().decode().replace('\n', '').replace('\r', '')
            
            # Check if message is the write size
            if len(new_line) >= 22:
                start = -1
                
                # Check if message received reported an error
                if "ERROR" in new_line:
                    self.ids.console.text = self.ids.console.text + "Error reported by arduino\n"
                    break
                
                # Save data on dictionaries
                data["time"].append(int(new_line[12:22]))
                data["Temp"].append(float(new_line[3:8]))

                # From 10 to 10 mins, the results are saved
                if time.mktime(time.gmtime()) - last_save > 600:
                    # Register data on file
                    f = open("./save_files/" + trial_name + ".csv", "a")

                    for i in range(len(data["time"]) - last_index - 1):
                        f.write(str(data["time"][last_index + 1 + i]) + "," + str(data["Temp"][last_index + 1 + i]) + ",\n")
                    f.close()

                    last_index = len(data["time"]) - 1
                    last_save = time.mktime(time.gmtime())

                # Plot the data
                plt.plot(data["time"], data["Temp"])
                plt.title("Temperature vs Time")
                plt.xlabel("time [s]")
                plt.ylabel("Temperature [°C]")
                plt.grid()
                plt.show()
                plt.pause(0.01)
                plt.cla()
            else:
                # Check if timeout
                if start < 0:
                    start = time.mktime(time.gmtime())

                if time.mktime(time.gmtime()) - start > 10:
                    print("Timeout: too long to receive message from arduino")
                    break
            # Write back to PCR thermocycler
            control_connection.write(bytes("READY TO RECEIVE", 'utf-8'))

        # Register final data on file
        f = open("./save_files/" + trial_name + ".csv", "a")
        
        for i in range(len(data["time"]) - last_index - 1):
            f.write(str(data["time"][last_index + 1 + i]) + "," + str(data["Temp"][last_index + 1 + i]) + ",\n")
        f.close()


# Main app for interface
class MainApp(App):
    def __init__(self):
        super().__init__()
        Config.set('graphics','resizable',0)
        Window.size = (750, 500)

    def build(self):
        self.root_widget = RootWidget()
        return self.root_widget


# Main thread
if __name__ == '__main__':
    if os.path.isdir("./save_files") == False:
        os.mkdir("./save_files")
    
    MainApp().run()
    sys.exit()