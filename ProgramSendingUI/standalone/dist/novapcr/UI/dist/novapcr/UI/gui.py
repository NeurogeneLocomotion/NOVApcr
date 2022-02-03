"""

Author: Francisco Branco
Date: 04/07/2021
Description: This module handles the GUI of the program
             and makes the bridge between the back-end 
             and the front-end.

"""


from kivy.app import App
from kivy.uix.boxlayout import BoxLayout
from kivy.config import Config
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.popup import Popup
from kivy.properties import ObjectProperty
from readfile import load
import connection as ct
import serial
import os


# Floating window for program file selection
class LoadFile(FloatLayout):
    load = ObjectProperty(None)
    cancel = ObjectProperty(None)


# Main Widget methods
class RootWidget(BoxLayout):
    def __init__(self):
        super().__init__()
        self.current_program = []

    def press_button(self):
        load()

    # Method to open the program file selection window
    def load_button(self):
        content = LoadFile(load=self.load, cancel=self.dismiss_popup)
        self._popup = Popup(title="Load file", content=content, size_hint=(0.9, 0.9))
        self._popup.open()
    
    # Method for handling the selection of the program file
    def load(self, filenames):
        # Check number of files chosen
        if len(filenames) > 1 or len(filenames) <= 0:
            self.ids.console.text = self.ids.console.text + "Error when opening the file: more than 1 file selected.\n"
            self._popup.dismiss()
            return
        # Check file name size
        if len(os.path.basename(filenames[0]).split(".")[0]) > 16 or os.path.basename(filenames[0]).split(".")[1] != "txt":
            self.ids.console.text = self.ids.console.text + "Error when opening the file: program name maximum size is 16.\n"
            self._popup.dismiss()
            return
        self.ids.program_name.text = os.path.basename(filenames[0]).split(".")[0]
        self.current_file = filenames[0]
        # Load program
        result = load(filenames[0], self.current_program)

        self.ids.console.text = self.ids.console.text + "Load result: " + result
        if ("Error" or "error") in self.ids.console.text:
            self.current_program = []
            self.ids.program_name.text = ""
        
        self.dismiss_popup()

    # Method to close program file selection window
    def dismiss_popup(self):
        self._popup.dismiss()
    
    # Method to send pcr program to thermocycler
    def send_button(self):
        # If program is empty, we have to load one first
        if self.current_program == []:
            self.ids.console.text = self.ids.console.text + "Error while sending the program: Nothing has been loaded yet.\n"
            return

        n_lines = len(self.current_program)
        
        # Check COM parameter
        if "COM" not in self.ids.pcr_port.text or "COM" not in self.ids.pcr_port.text:
            self.ids.console.text = self.ids.console.text + "COMs not correctly specified. The right format is: COM0, where 0 can be any number.\n"
            return

        # Start bluetooth communication
        try:
            connect = ct.Connection(self.ids.pcr_port.text, 9600, 1)
            connect.send_program(n_lines, self.current_program, self.ids.console, self.ids.program_name.text)
            connect.close()
        except serial.SerialException as err:
            self.ids.console.text = self.ids.console.text + "Error while sending PCR Program to thermocycler: " + str(err) + "\n"
        


# Main application to run GUI
class MainApp(App):
    def __init__(self):
        super().__init__()
        Config.set('graphics', 'width', '800')
        Config.set('graphics', 'height', '560')

    def build(self):
        self.main_widget = RootWidget()
        return self.main_widget