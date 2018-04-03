# -*- coding: utf-8 -*-
"""
example on how to plot decoded sensor data from crazyflie
@author: Xintong Du
@email : xintong.du@mail.utoronto.ca
"""
import CF_functions as cff
import sys, getopt, os
import numpy as np

LogGroups = {"control_inputs_pos"  : ["cpsx", "cpsy", "cpsz"],
	     "control_inputs_vel"  : ["cvlx", "cvly", "cvlz"],
	     "control_inputs_att"  : ["crol", "cpit", "cyaw"],
	     "estimated_states_pos"  : ["spsx", "spsy", "spsz"],
	     "estimated_states_vel"  : ["svlx", "svly", "svlz"],
	     "estimated_states_att"  : ["srol", "spit", "syaw"],
             "sensor_acc"      : ["accx", "accy", "accz"],
             "sensor_gyro"     : ["gyrx", "gyry", "gyrz"],
             "sensor_mag"      : ["magx", "magy", "magz"],
             "sensor_baro"     : ["asll", "pres", "temp"]}

def process_data(filename, foldername):
    logData = cff.decode(filename)

    cwd = os.getcwd()
    outputdir = cwd + '/' + foldername
    if not os.path.exists(outputdir):
        os.makedirs(outputdir)

    for group, vars in LogGroups.items():
        filename = outputdir + '/' + group +'.csv'
        names = "tick"
        values = [logData["tick"]]
        for var in vars:
            if var in logData:

                names += "," + var
                values.append(logData[var])

        if len(values) > 1:
            values = np.stack(values).T
            np.savetxt(filename, values, delimiter=',', header=names)


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("usage: python3 usdlog2csv.py -i <input_file> (-o <output+dir>)")

    else:
        inputfile = ''
        outputdir = ''

        opts, args = getopt.getopt(sys.argv[1:], "hi:o:", ["ifile=", "odir="])

        for opt, arg in opts:
            if opt == '-h':
                print("usage: python3 usdlog2csv.py -i <input_file> (-o <output_folder>)")
                sys.exit()

            elif opt in ("-i", "--ifile"):
                inputfile = arg
            elif opt in ("-o", "--ofolder"):
                outputdir = arg

        if outputdir == '':
            outputdir = 'log'

        print(inputfile + ' will be processed. Results will be saved under current directory in folder ' + outputdir + '\n')

        process_data(inputfile, outputdir)
