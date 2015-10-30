import sys
# add the upper directory
sys.path.append("..")
# include optitrack module
from src_optitrack.optitrack import *

if __name__ == '__main__':

	opti = OptiTrackInterface()
	opti.PrintTrackInfo(1)