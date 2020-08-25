#! /usr/bin/env python

import re
import time
import fileinput

# filename = '../urdf/mallard_main.xacro'
filename = 'mallard_main.xacro'


initial_value = 8.0
final_value = 20.0
step_value = 0.4

initial = True
found = False
previous_mass = '0.0' #initialize
string_1 = '<mass value="'
string_2 = '"/>'
print(repr(string_1) + repr(string_2))

while initial_value <= final_value:
    current_mass =str(initial_value)
    print("current mass: ", current_mass)

    file = fileinput.FileInput(filename, inplace=True, backup='.bak')
    for line in file:
        line = line.rstrip()  
        # find '<mass value=' and extract digits; executes once only:
        if (string_1 in line) and (string_2 in line) and (initial == True):
            # [int(s) for s in line.split() if s.isdigit()]
            # my_digits = s
            my_digits = re.findall(r"[-+]?[.]?[\d]+(?:,\d\d\d)*[\.]?\d*(?:[eE][-+]?\d+)?",line)
            previous_mass = my_digits[0]

            found = True
            initial = False


        previous_str = '<mass value="' + previous_mass + '"/>'
        current_str = '<mass value="' + current_mass + '"/>'
        print(line.replace(previous_str,current_str)) 
 
    file.close()


    if found:
        print("Found match!")
        print(repr(my_digits[0]))
        found = False
    # update 
    previous_mass = current_mass
    initial_value += step_value

    # run start_gazebo_mallard.py...

# When finished reinitialize to standard value:
print("Writing defaults to URDF file")
time.sleep(5)
previous_str = '<mass value="' + previous_mass + '"/>'
default_str = '<mass value="10.5"/>'
file = fileinput.FileInput(filename, inplace=True, backup='.bak')
for line in file:
    line = line.rstrip('\r\n')  
    print(line.replace(previous_str,default_str))
file.close()

print("Default mass: ", default_str)