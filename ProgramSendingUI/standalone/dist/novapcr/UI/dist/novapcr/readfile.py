"""

Author: Francisco Branco
Date: 29/07/2021
Description: This module handles the loading of PCR
             programs in the /programs folder - .txt
             files with the program description.

"""


def load(path, program):
    # Open file and setup for pcr program reading
    f = open(path,"r")
    new_line = f.readline().split()

    counter = 1
    last_goto = 0

    # Main loop for reading the file
    while new_line != []:
        # Setup line
        line = []
        
        # Check if index of line
        try:
            index = float(new_line[0])
        except ValueError as err:
            program = []
            return ("Error while loading; index of new line is not a number: " + err + "\n")
        if index % 1 != 0:
            program = []
            return ("Error while loading; index of new line is not an integer.\n")
        if index != counter:
            program = []
            return ("Error while loading; index of new line does not match.\n")
        if index > 50:
            program = []
            return ("Error while loading; index is bigger than 50 (max lines).\n")
        line.append(int(index))

        # Check if action is valid and which action is selected
        if new_line[1] == "STEP":
            line.append(0)

            # Check if temperature is valid
            try:
                temperature = float(new_line[2])
            except ValueError as err:
                program = []
                return ("Error while loading; temperature of STEP is not a number: " + err + "\n")
            
            if temperature < 5 or temperature > 100:
                    program = []
                    return ("Error while loading; temperature of STEP is not between 5 and 100 degrees celsius\n")
            temperature = round(temperature, 1) * 10
            line.append(int(temperature))

            # Check if temperature units are correct
            if new_line[3] != "C":
                program = []
                return ("Error while loading; temperature of STEP has incorrect celsius unit indicator.\n")
            
            # Check if duration is valid
            try:
                duration = int(new_line[4])
            except ValueError as err:
                program = []
                return ("Error while loading; duration of STEP is not a number: " + err + "\n")
            if duration <= 0:
                program = []
                return ("Error while loading; duration of STEP is equal or less than 0\n")
            line.append(duration)

            # Check if duration units are correct
            if new_line[5] != "s":
                program = []
                return ("Error while loading; duration of STEP has incorrect seconds unit indicator.\n")

            # Check if there is increment
            if len(new_line) > 6:
                # Check if increment is valid
                try:
                    increment = float(new_line[6])
                except ValueError as err:
                    program = []
                    return ("Error while loading; increment of STEP is not a number: " + err + "\n")

                if increment < 5 or increment > 100:
                    program = []
                    return ("Error while loading; increment of STEP is not between 5 and 100 degrees celsius\n")
                increment = round(increment, 1) * 10
                line.append(int(increment))

                # Check if increment units are correct
                if new_line[7] != "C":
                    program = []
                    return ("Error while loading; increment of STEP has incorrect celsius unit indicator.\n")
            else:
                line.append(0)

        elif new_line[1] == "GOTO":            
            line.append(1)

            # Check if goto line is valid
            try:
                goto = int(new_line[2])
            except ValueError as err:
                program = []
                return ("Error while loading; goto line of GOTO is not a number: " + err + "\n")
            if goto >= index or goto <= 0 or goto <= last_goto:
                program = []
                return ("Error while loading; goto line of GOTO is invalid.\n")
            line.append(goto)

            # Check if goto units are correct
            if new_line[3] != "line":
                program = []
                return ("Error while loading; line of GOTO has incorrect line unit indicator.\n")
            
            # Check if cycles is valid
            try:
                cycles = int(new_line[4])
            except ValueError as err:
                program = []
                return ("Error while loading; cycles of GOTO is not a number: " + err + "\n")
            if cycles <= 0:
                program = []
                return ("Error while loading; cycles of GOTO is invalid.\n")
            line.append(cycles)

            # Check if cycles units are correct
            if new_line[5] != "cycles":
                program = []
                return ("Error while loading; cycles of GOTO has incorrect cycles unit indicator.\n")

            line.append(0)

            last_goto = index

        elif new_line[1] == "END":
            line.append(2)

            # Check if temperature is valid
            try:
                temperature = float(new_line[2])
            except ValueError as err:
                program = []
                return ("Error while loading; temperature of END is not a number: " + err + "\n")
            
            if temperature < 5 or temperature > 100:
                program = []
                return ("Error while loading; temperature of END is not between 5 and 100 degrees celsius\n")
            temperature = round(temperature, 1) * 10
            line.append(int(temperature))

            # Check if temperature units are correct
            if new_line[3] != "C":
                program = []
                return ("Error while loading; temperature of END has incorrect celsius unit indicator.\n")
            
            line.append(0)
            line.append(0)

            program.append(line)
            break

        else:
            program = []
            return ("Error while loading; action not recognized: " + new_line[1] + "\n")

        program.append(line)

        new_line = f.readline().split()
        counter = counter + 1
    
    f.close()

    return ("Load complete!\n")