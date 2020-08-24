#! /usr/bin/env python

import time
import fileinput

# filename = '../urdf/mallard_main.xacro'
filename = 'mallard_main.xacro'

# file = fileinput.FileInput(filename) #for testing -does not change input file
# file = fileinput.FileInput(filename, inplace=True, backup='.bak')

previous_mass = '10.0'
for value in list(range(11,15)):

    current_mass = str(float(value))
    print("previous mass: ", previous_mass)
    print("curretn_mass: ", current_mass)

    previous_str = '<mass value="' + previous_mass + '"/>'
    current_str = '<mass value="' + current_mass + '"/>'
    # override mass value inside URDF file:
    # file = fileinput.FileInput(filename) #for testing -does not change input file
    file = fileinput.FileInput(filename, inplace=True, backup='.bak')
    for line in file:
        line = line.rstrip('\r\n')  
        print(line.replace(previous_str,current_str))
    file.close()

    previous_mass = current_mass
    time.sleep(2)
    