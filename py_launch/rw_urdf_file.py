#! /usr/bin/env python

# import os.path, glob
import fileinput

filename = 'rw_test_file.txt'

f = fileinput.FileInput(filename, inplace=True, backup='.bak')

for file in f:
    for line in file:
        print(line.replace('other', 'replacement_for_other'))
f.close()

# with fileinput.FileInput(filename, inplace=True, backup='.bak') as file:
#     for line in file:
#         print(line.replace('other', 'replacement_for_other'))   


# for file in glob.glob("*.bag"):
#     bag_name = os.path.splitext(file)[0]

#     # ----- read the csv file and put data of interest into a list -----
#     # filename = '_slash_teensy_test_topic.csv'
#     filename = bag_name + "/_slash_mallard_slash_thruster_command.csv"
#     with open(filename) as my_file:
#         csv_reader = csv.reader(my_file)
#         next(csv_reader) #jump to next row; next time reader is called it will read numerical data
        
#         data = []
#         initial_time_stamp = 0
#         for row in csv_reader:
#             # convert string to float then round values:
#             data_row = ast.literal_eval(row[10])
#             # remove thruster 3 and 4
#             del data_row[2:4]

#             for i,value in enumerate(data_row):
#                 data_row[i] = round(value,4)