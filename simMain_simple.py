#!/usr/bin/python

from G16Modules.Globals import Specific
from G16Modules.Vision import VisionModule
from G16Modules.Navigation import NavigationModule, STATE

import cv2

# MAIN SCRIPT
if __name__ == '__main__':

	Specific.start()
	
	# Possibly override color ranges for simulator
	if hasattr(Specific, 'color_ranges'):
		VisionModule.color_ranges = Specific.color_ranges

	try:
		NavigationModule.init(STATE.AISLE_DOWN)		
		
		while True:
			NavigationModule.update()
			Specific.update()
			if cv2.waitKey(1) & 0xFF == ord('q'):
				break
			

	finally:
		# attempt to stop simulator
		Specific.end()