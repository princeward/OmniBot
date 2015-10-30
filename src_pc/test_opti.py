import sys
# add the upper directory
sys.path.append("..")
# include optitrack module
from src_optitrack.optitrack import *

from time import sleep

if __name__ == '__main__':
    opti = OptiTrackInterface()
    while(True):
	opti.PrintTrackInfo(5)
	sleep(0.1)
