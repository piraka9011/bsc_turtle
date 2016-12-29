#!/usr/bin/env python

# PyLib
from multiprocessing import Process
import os
import time

if __name__ == "__main__":

    drift = raw_input("Start drift module? (y/n)")
    if drift == "y":
        driftLaunch = Process(target=os.system, args=["rosrun bsc_turtle driftBSC.py"])
        driftLaunch.start()
        time.sleep(1)

    startSend = raw_input("Send goal position? (y/n)")
    if startSend == "y":
        sendStart = Process(target=os.system, args=["rosrun bsc_turtle sendStart.py"])
        sendStart.start()
        time.sleep(1)

    readyExp = raw_input("Start experiment? (y/n)")
    if readyExp == "y":
        print("Please have subject ready for teleoperation...")
        print("Starting Experiment...")
        expLaunch = Process(target=os.system, args=["rosrun bsc_turtle expVars.py"])
        expLaunch.start()
        time.sleep(1)
