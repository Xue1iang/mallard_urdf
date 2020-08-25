#! /usr/bin/env python

import re
import time
import fileinput

# filename = '../urdf/mallard_main.xacro'
filename = 'mallard_main.xacro'

# file = fileinput.FileInput(filename) #for testing -does not change input file
# file = fileinput.FileInput(filename, inplace=True, backup='.bak')

initial_value = 8.0
final_value = 20.0
step_value = 0.4

initial = True
found = False
previous_mass = '0.0' #initialize
string_1 = '<mass value="'
string_2 = '"/>'
# string_2 = 
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

        # print(line.replace(previous_str,current_str))    
    file.close()


    if found:
        print("Found match!")
        print(repr(my_digits[0]))
        found = False
    # update 
    previous_mass = current_mass
    initial_value += step_value
    # time.sleep(5)

# When finished reinitialize to standard value:
time.sleep(5)
previous_str = '<mass value="' + previous_mass + '"/>'
default_str = '<mass value="10.5"/>'
file = fileinput.FileInput(filename, inplace=True, backup='.bak')
for line in file:
    line = line.rstrip('\r\n')  
    print(line.replace(previous_str,default_str))
file.close()

print("Default mass: ", default_str)



# previous_mass = '10.0'
# for value in list(range(11,15)):

#     current_mass = str(float(value))
#     print("previous mass: ", previous_mass)
#     print("curretn_mass: ", current_mass)

#     previous_str = '<mass value="' + previous_mass + '"/>'
#     current_str = '<mass value="' + current_mass + '"/>'
#     # override mass value inside URDF file:
#     # file = fileinput.FileInput(filename) #for testing -does not change input file
#     file = fileinput.FileInput(filename, inplace=True, backup='.bak')
#     for line in file:
#         line = line.rstrip('\r\n')  
#         print(line.replace(previous_str,current_str))
#     file.close()

#     previous_mass = current_mass
    # time.sleep(2)
    