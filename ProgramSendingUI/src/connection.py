"""

Author: Francisco Branco
Date: 29/07/2021
Description: This module handles the connection between
             arduino (thermocycler) and computer.

"""


import serial
import time


class Connection:
    def __init__(self, port=None, baudrate=9600, timeout=1):
        # Create serial connection
        self.serial = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
        self.serial.flushInput()
        
        time.sleep(2)

    # Method to send program to PCR machine via serial connection
    def send_program(self, n_lines, array2d, console, prog_name="Program"):
        # Send firstly the number of lines and then each line of the program
        for i in range(-1, n_lines):
            # Formulate number of lines first
            if i == -1:
                aux_str = str(n_lines)
                for _ in range(4 - len(str(n_lines))):
                    aux_str = " " + aux_str
                out_str = "NEW PROGRAM " + aux_str + "     "
            # Formulate each line
            else:
                out_str = ""
                for j in range(len(array2d[i])):
                    
                    if j == 1:
                        aux_str = str(array2d[i][j])
                    else:
                        aux_str = str(array2d[i][j])
                        for k in range(4 - len(str(array2d[i][j]))):
                            aux_str = " " + aux_str
                    if j != len(array2d[i]) - 1:
                        out_str = out_str + aux_str + " "
                    else:
                        out_str = out_str + aux_str

            # Send through serial connection the formulated message
            console.text = console.text + "Number of characters: " + str(len(out_str)) + "\n"
            console.text = console.text + "Line: " + out_str + "\n"
            self.serial.write(bytes(out_str, 'utf-8'))

            # Set initial time for potential timeout
            start = time.mktime(time.gmtime())
            
            # Try to read correct line from arduino or timeout
            some_string = self.serial.readline().decode().replace('\n', '').replace('\r', '')
            print(some_string)
            while some_string != "READY TO RECEIVE":
                
                if time.mktime(time.gmtime()) - start > 10:
                    console.text = console.text + "Timeout: too long to receive message from arduino\n"
                    return
                some_string = self.serial.readline().decode().replace('\n', '').replace('\r', '')
                print(some_string)
        # Formulate program name lastly
        aux_str = prog_name
        for k in range(16 - len(prog_name)):
            aux_str = aux_str + " "
        out_str = "NAME " + aux_str
        console.text = console.text + "Number of characters: " + str(len(out_str)) + "\n"
        console.text = console.text + "Line: " + out_str + "\n"

        # Send program name
        self.serial.write(str.encode(out_str))

        console.text = console.text + "\nDone! Program has been sent\n"
        
    # Method to end serial connection
    def close(self):
        self.serial.close()
