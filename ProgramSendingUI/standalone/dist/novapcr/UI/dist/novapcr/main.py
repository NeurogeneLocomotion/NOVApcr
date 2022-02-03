"""

Author: Francisco Branco
Date: 29/07/2021
Description: This module deals with the main function
             of the program.

"""


from gui import MainApp
import sys
import os



if __name__ == '__main__':
    # Make program directory
    if os.path.isdir("./programs") == False:
        os.mkdir("./programs")
    
    MainApp().run()
    sys.exit()