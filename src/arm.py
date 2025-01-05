#!/usr/bin/env python3

import os
import numpy as np 
from servo_control import ServoControl


class Arm:
    def __init__(self):
        Servo_controller = ServoControl()
        
