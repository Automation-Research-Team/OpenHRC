#!/usr/bin/env python

import os
import argparse
import glob
from time import sleep

parser = argparse.ArgumentParser()
parser.add_argument('-e', "--experiment", help="# of experiment")
parser.add_argument('-sub', "--subject", help="# of subject")
parser.add_argument('-s',"--set", help="# of set")
parser.add_argument('-t', "--trial", help="# of trial")
parser.add_argument('-n', "--note", help="note added to file name prefix")
parser.add_argument('-d', "--dir", help="data save dir under $HOME", default="")
args = parser.parse_args()

prefix = ""

if args.experiment:
    prefix += "exp-" + args.experiment +"_"

if args.subject:
    prefix += "subject-" + args.subject +"_"

if args.set:
    prefix += "set-" + args.set +"_"

if args.trial:
    prefix += "trial-" + args.trial +"_"

if args.note:
    prefix += "note-" + args.note +"_"


save_dir = os.path.expanduser("~") + "/" + args.dir


prefix = prefix[0:-1] # remove last "_"
print("rosbag prefix: " + prefix)
print("save dir: " + save_dir)

raw_input("Press Enter to start recording rosbag...")

print("Recording...")
prefix2 = prefix
if prefix != "":
    prefix2 = "-o " + prefix

prefix3 = prefix
if prefix != "":
    prefix3 = prefix + "_"

prev_nfile = len(glob.glob(os.getcwd() + '/' + prefix3 + '*.bag'))


os.system('rosbag record --tcp -q -a -x "(.*)/camera/(.*)" '+ prefix2)

print("Saving...")


count = 0
while True:
    count += 1
    bagfile = glob.glob(os.getcwd() + '/' + prefix3 + '*.bag')
    bagfile.sort(key=os.path.getmtime)
    
    if len(bagfile) - prev_nfile > 0:
        break
    else:
        if count > 100:
            print("Failed to record bag")
            exit()
        else:
            sleep(0.1)

# print("\n".join(bagfile))

if not os.path.exists(save_dir):
    os.makedirs(save_dir)

if save_dir[-1] != "/":
    save_dir += "/" 

os.rename(bagfile[-1], save_dir + os.path.basename(bagfile[-1]))
print("Saved as " + save_dir + os.path.basename(bagfile[-1]))