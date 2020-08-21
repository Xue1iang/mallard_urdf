#!/usr/bin/env python
import os
import signal
import socket
from subprocess import Popen
import subprocess
import time

# rosbag_start = False


def run(cmd, stdout, stderr):
    """Run a given `cmd` in a subprocess, write logs to stdout / stderr.

    Parameters
    ----------
    cmd : list of str
        Command to run.
    stdout : str or subprocess.PIPE object
        Destination of stdout output.
    stderr : str or subprocess.PIPE object
        Destination of stderr output.

    Returns
    -------
    A subprocess.Popen instance.
    """
    return Popen(cmd, stdout=stdout, stderr=stderr, shell=False,
                 preexec_fn=os.setsid)


def get_stdout_stderr(typ, datetime, dir):
    """Create stdout / stderr file paths."""
    out = '%s_%s_stdout.log' % (datetime, typ)
    err = '%s_%s_stderr.log' % (datetime, typ)
    print(out)
    return os.path.join(dir, out), os.path.join(dir, err)


def check_files_exist(files):
    """Check if given list of files exists.

    Parameters
    ----------
    files : list of str
        Files to check for existence.

    Returns
    -------
    None if all files exist. Else raises a ValueError.
    """
    errors = []
    for f in files:
        if not os.path.exists(f):
            errors.append(f)
    if errors:
        raise ValueError('File does not exist: %s' % errors)


def start_process(cmd, typ, start_time, dpath_logs):
    """Start a subprocess with the given command `cmd`.

    Parameters
    ----------
    cmd : list of str
        Command to run.
    typ : str
        Type of subprocess. This will be included in the logs' file names.
    start_time : str
        Datetime string, will be included in the logs' file names as well as
        the resulting bag's name.
    dpath_logs :
        Path to log direcotry.

    Returns
    -------
    A subprocess.Popen instance.
    """
    print('Starting', typ.upper())
    stdout, stderr = get_stdout_stderr(typ, start_time, dpath_logs)
    with open(stdout, 'wb') as out, open(stderr, 'wb') as err:
        return run(cmd, stdout=out, stderr=err)

def start_socket(host, port,script_rosbag,name_rosbag,start_time,dpath_logs):
    # You might put mallard launch here to avoid race conditions
    # where mallard launches without socket connecting to goal selector
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((host, port))
    s.listen(1)
    clientsocket, address = s.accept()
    print('Connected by', address)
    while True:
        data = clientsocket.recv(1024)

        if data == '':
            raise RuntimeError("socket connection broken") 
        if data != 'killall':
            print("DATA RECEIVED")
            print(data)
            # start after 2 seconds settling time
            if(data == 'counter: 20'):
                print("Starting rosbag record: " + name_rosbag)
                session = start_process(['/bin/bash',script_rosbag,name_rosbag],
                                        'rosbag_record',start_time,dpath_logs) 
        else:
            # Send the signal to all the process groups
            print('\nReceived killall signal.')
            break

    return session

def main(args):
    dpath_logs = args.dpath_logs
    script_gazebo = args.script_gazebo
    script_mallard = args.script_mallard
    script_rosbag = args.script_rosbag
    
    check_files_exist([script_gazebo, script_mallard, script_rosbag, dpath_logs])
    # print('gazebo: ',script_gazebo,' mallard: ',script_mallard)

    start_time = time.strftime('%Y%m%d_%H%M%S')

    session_gazebo = start_process(['/bin/bash', script_gazebo],
                               'gazebo_launchfile', start_time,dpath_logs)   
    # Wait for gazebo, otherwise HECTOR SLAM error
    time.sleep(5)
    session_mallard = start_process(['/bin/bash', script_mallard],
                               'mallard_launchfile', start_time,dpath_logs)  

    # session_rosbag = start_process(['/bin/bash',script_rosbag],
    #                                 'rosbag_record',start_time,dpath_logs)    
                      
    # print pids in case something goes wrong
    print('PGID GAZEBO LAUNCH: ', os.getpgid(session_gazebo.pid))
    print('PGID MALLARD LAUNCH: ', os.getpgid(session_mallard.pid))
    # print('PGID ROSBAG RECORD: ', os.getpgid(session_rosbag.pid))

    # Needs this to know when Mallard reaches final goal
    # so everthing can be shut down. Create socket and listen on the port.
    HOST = socket.gethostbyname("localhost")
    PORT = 65432
    name_rosbag = 'rosbag_test_4'
    session_rosbag = start_socket(HOST, PORT,script_rosbag,name_rosbag,start_time,dpath_logs)
    print('Socket connection terminated')

    print('\nKilling rosbag record')
    # make sure signal is SIGINT (not SIGTERM) which is equivalent to ctrl + c
    os.killpg(os.getpgid(session_rosbag.pid), signal.SIGINT)

    time.sleep(3)
    print('Killing controller and Gazebo simulator.')
    os.killpg(os.getpgid(session_mallard.pid), signal.SIGTERM)
    time.sleep(3)
    os.killpg(os.getpgid(session_gazebo.pid), signal.SIGTERM)


if __name__ == '__main__':
    """Start ROS and the talker node each as a subprocess.

    Examples
    --------

    python  start_ros.py --script_node /notebooks/workspace/talker.sh \
    -l /notebooks/workspace/src/scripts
    """
    import argparse

    parser = argparse.ArgumentParser()
    # GAZEBO & MALLARD launch sessions
    parser.add_argument('--script_gazebo', type=str,
                        default='/home/konrad/ROS/workspaces/ros_gazebo_ws/src'
                                 '/mallard_urdf/py_launch/bash_launch/gazebo.sh')
    parser.add_argument('--script_mallard', type=str,
                        default='/home/konrad/ROS/workspaces/ros_gazebo_ws/src'
                                 '/mallard_urdf/py_launch/bash_launch/mallard.sh')
    parser.add_argument('--script_rosbag', type=str,
                        default='/home/konrad/ROS/workspaces/ros_gazebo_ws/src'
                                 '/mallard_urdf/py_launch/bash_launch/rosbag.sh')

    # LOG FILES                        
    parser.add_argument('--dpath_logs', '-l', type=str,
                        default='/home/konrad/ROS/workspaces/ros_gazebo_ws/src'
                                 '/mallard_urdf/py_launch/log_files')
    args = parser.parse_args()
    main(args)
    