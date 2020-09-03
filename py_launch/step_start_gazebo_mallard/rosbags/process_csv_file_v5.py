#! /usr/bin/env python3
import csv
import ast
import math
import os.path, glob
import numpy as np
import matplotlib.pyplot as plt

# ------------------- definitions ----------------------
# find nearest value in the array
def find_nearest(array, value):
    array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    # returns index for the closest value
    # return array[idx]
    return idx
#Save values into csv file:
def write_to_csv(file_name,data):
    # search for directory 'processed' if it doesn't exist create one
    if not os.path.exists('processed'):
        os.makedirs('processed')
        os.makedirs('processed/figures')

    # create csv file if it doesn't exist (+w) and store values:
    with open(file_name, 'w+') as csvfile:
        csvfile.truncate() #clear previous values
        filewriter = csv.writer(csvfile, delimiter=',',quotechar='|', quoting=csv.QUOTE_MINIMAL)
        filewriter.writerow([str('TIME'),str('INPUT_thr1'),str('INPUT_thr2'),str('OUTPUT_pos'),str('OUTPUT_vel'),str('GOAL_pos'),str('GOAL_vel')])

        for i in data:
            filewriter.writerow([i[0],i[1],i[2],i[3],i[4],i[5],i[6]])
# ------------------- end of definitions ---------------------

for file in glob.glob("*.bag"):
    bag_name = os.path.splitext(file)[0]

    # ----- read the csv file and put data of interest into a list of  goals-----
    filename = bag_name + "/_slash_mallard_slash_goals.csv"
    with open(filename) as my_file:
        csv_reader = csv.reader(my_file)
        next(csv_reader) #jump to next row; next time reader is called it will read numerical data
        # print("\nrosbag: " + str(filename) + "\n") 
        data,goals  = [],[]
        init_t_stamp,init_pos,init_vel = 0,0,0
        for row in csv_reader:
            # Get values from each row
            t_stamp = float(row[0])
            # read from list of lists therfore parsing to float() doesnt work
            pos_vel_acc = ast.literal_eval(row[4]) # all values from this cell are processed
            #  Get initial values to substruct later
            if(init_t_stamp == 0):
                init_t_stamp = t_stamp
                init_pos     = pos_vel_acc[1] 
                init_vel     = pos_vel_acc[4]
    
            data = [round((t_stamp-init_t_stamp)*10**-9,6),\
                    round(pos_vel_acc[1] - init_pos,6),\
                    round(pos_vel_acc[4] - init_vel,6) ]
            goals.append(data)
        # test
        # for t in goals[150:160]:
        #     print("goals:",t)

     # ----- read the csv file and put data of interest into a list of poses -----
    # filename = bag_name + "/_slash_slam_out_pose.csv" # hector slam less reliable than Gazebo
    filename = bag_name + "/_slash_gazebo_slash_model_states.csv"
    with open(filename) as my_file:
        csv_reader = csv.reader(my_file)
        next(csv_reader) #jump to next row; next time reader is called it will read numerical data
        # print("\nrosbag: " + str(filename) + "\n")
        data,pose  = [],[]
        init_t_stamp,init_pos,init_vel = 0,0,0
        for row in csv_reader:
            # Get values from each row
            time_stamp = float(row[0])
            pos = float(row[13])
            vel = float(row[31])
            #  Get initial values to substruct later
            if(init_t_stamp == 0):
                init_t_stamp = time_stamp
                init_pos     = pos
                init_vel     = vel
    
            data = [round((time_stamp-init_t_stamp)*10**-9,6),\
                    round(pos - init_pos,6),\
                    round(vel - init_vel,6)]
            pose.append(data)

            #  _slash_mallard_slash_thruster_command
    
    filename = bag_name + "/_slash_mallard_slash_thruster_command.csv"
    with open(filename) as my_file:
        csv_reader = csv.reader(my_file)
        next(csv_reader) #jump to next row; next time reader is called it will read numerical data
        
        thrust = []
        initial_time_stamp = 0
        for row in csv_reader:
            # convert string to float then round values:
            data_row = ast.literal_eval(row[10])
            # remove thruster values 
            del data_row[2:]

            for i,value in enumerate(data_row):
                data_row[i] = round(value,4)
   
            # convert (str to float) for time stamp then save first time and position:
            time_stamp = ast.literal_eval(row[0])
            if(initial_time_stamp == 0):
                initial_time_stamp = time_stamp

            # append-left time to data, substract first time stamp to get time in seconds:
            data_row.insert(0,round((time_stamp-initial_time_stamp)*10**-9,2))
            thrust.append(data_row)

    
    # ------------------------------------------------------    
    # --- Match Gazebo goals, pose data to thruster data ---
    # ------------------------------------------------------

    # separate time from pose into single list
    pose_time = []
    for p in pose:
        pose_time.append(p[0])
    #same for thrust
    goals_time = []
    for g in goals:
        goals_time.append(g[0])

    for t in thrust:
        pose_index = find_nearest(pose_time,t[0]) # g[0] -is time in goals
        t.insert(3,pose[pose_index][1]) #position
        t.insert(4,pose[pose_index][2]) #velocity

        goals_index = find_nearest(goals_time,t[0])
        t.insert(5,goals[goals_index][1]) #goal position
        t.insert(6,goals[goals_index][2]) #goal velocity


    for t in thrust[100:104]:
        print(t)


    # save to file with given bag nameinside processed folder
    file_name = "processed/" + bag_name + ".csv"
    write_to_csv(file_name,thrust)

    # ------------------------------------------------------------
    # --------------------------- Plot ---------------------------
    # ------------------------------------------------------------
    time,thrust_sum,position,velocity,goal_pos,goal_vel = [],[],[],[],[],[]
    # for t in thrust:
    #     thrust_sum = t[1]+t[2]
        
    for t in thrust:
        time.append(t[0])
        thrust_sum.append(t[1]+t[2])
        position.append(t[3])
        velocity.append(t[4])
        goal_pos.append(t[5])
        goal_vel.append(t[6])
        # print("time: " + str(time) + " goal: " + str(goal) + " pose: " + str(pose))

    plt.figure()
    plt.suptitle("Position & Velocity Tracking, " + bag_name)
    plt.subplot(311)
    plt.plot(time,goal_pos,'tab:red',time,position,'tab:blue')
    plt.ylabel("distance[m]")

    plt.subplot(312)
    plt.plot(time,goal_vel,'tab:red',time,velocity,'tab:blue')
    plt.xlabel("time[s]")
    plt.ylabel("velocity[m/s]")

    plt.subplot(313)
    plt.plot(time,thrust_sum,'tab:blue')
    plt.xlabel("time[s]")
    plt.ylabel("force[N]")
    # save to file:
    plt.savefig("processed/figures/" + bag_name + ".png")
    # ot plot using GUI:
    # plt.show()