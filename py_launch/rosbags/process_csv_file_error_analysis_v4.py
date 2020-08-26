
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
        # filewriter.writerow([str('time_s'),str('prbs'),str('ctrl_pwm1'),str('sum_prbs_ctrl_pwm1'),str('thruster_1_pwm'),str('ctrl_pwm2'),str('sum_prbs_ctrl_pwm2'),str('thruster_2_pwm'),str('pos_vel_acc_m')])
        filewriter.writerow([str('time'),str('goal_position'),str('position'),str('goal_velocity'),str('velocity')])

        for i in data:
            filewriter.writerow([i[0],i[1],i[2],i[3],i[4]])
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
        # for t in goals[:10]:
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


    
    # --- Match Gazebo pose data to goals ---

    # separate time from pose into single list
    pose_time = []
    for p in pose:
        pose_time.append(p[0])
    # search for pose time instances that match goals time
    for g in goals:
        index = find_nearest(pose_time,g[0]) # g[0] -is time in goals
        g.insert(2,pose[index][1])
        g.insert(4,pose[index][2])
        # goals[time,goal_pos,pos,goal_vel,vel]
    # test    
    # for goal in goals[20:25]:
    #     print(goal)

    # save to file with given bag nameinside processed folder
    file_name = "processed/" + bag_name + ".csv"
    write_to_csv(file_name,goals)

    # ------------------------------------------------------------
    # --------------------------- Plot ---------------------------
    # ------------------------------------------------------------
    time,goal_pos,goal_vel,position,velocity = [],[],[],[],[]
    for g in goals:
        time.append(g[0])
        goal_pos.append(g[1])
        position.append(g[2])
        goal_vel.append(g[3])
        velocity.append(g[4])
        # print("time: " + str(time) + " goal: " + str(goal) + " pose: " + str(pose))

    plt.figure()
    plt.suptitle("Position & Velocity Tracking, " + bag_name)
    plt.subplot(211)
    plt.plot(time,goal_pos,'tab:red',time,position,'tab:blue')
    plt.ylabel("distance[m]")

    plt.subplot(212)
    plt.plot(time,goal_vel,'tab:red',time,velocity,'tab:blue')
    plt.xlabel("time[s]")
    plt.ylabel("velocity[m/s]")
    # save to file:
    plt.savefig("processed/figures/" + bag_name + ".png")
    # ot plot using GUI:
    # plt.show()