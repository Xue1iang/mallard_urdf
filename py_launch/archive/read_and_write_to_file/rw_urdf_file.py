#! /usr/bin/env python

# import os.path, glob
import fileinput

filename = 'rw_test_file.txt'

text_replacement = {
    'line_4' : 'replacement for line 4'
}
# file = fileinput.FileInput(filename) #for testing -does not change input file
file = fileinput.FileInput(filename, inplace=True, backup='.bak')

for line in file:
    line = line.rstrip('\r\n')
    # print(text_replacement.get(line,line))
    print(line.replace('line_3','replacement'))
    # print(line)
file.close()