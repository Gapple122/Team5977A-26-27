# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       prane                                                        #
# 	Created:      9/27/2026, 11:05:24 AM                                       #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *

# Brain should be defined by default
brain=Brain()

brain.screen.print("Hello V5")

motor1 = Motor(Ports.PORT1) 

motor1.spin_for(5, SECONDS) 


        
