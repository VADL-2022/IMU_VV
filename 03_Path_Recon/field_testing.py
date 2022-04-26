# -*- coding: utf-8 -*-
"""
Created on Sat Apr 23 08:01:56 2022

@author: kdmen
"""

import numpy as np
import pandas as pd
from model_funcs import *

imu_data_file_and_path = '../../Full_FS3_SIFT1.csv'
#all_outputs_p05 = calc_displacement2(imu_data_file_and_path, 149, [13, 1.5], GPS_coords=[34.8946331, -86.6166411], my_thresh=45, my_post_drogue_delay=0.5, my_signal_length=3, ld_launch_angle=5*pi/180, ld_ssm=2.44, ld_dry_base=15.735, ld_m_motor=1.728187, ld_t_burn=2.05, ld_T_avg=1771)
all_outputs_p11 = calc_displacement2(imu_data_file_and_path, 149, [13, -2], GPS_coords=[34.8946331, -86.6166411], my_thresh=45, my_post_drogue_delay=0.5, my_signal_length=3, ld_launch_angle=5*pi/180, ld_ssm=2.44, ld_dry_base=15.735, ld_m_motor=1.728187, ld_t_burn=2.05, ld_T_avg=1771)



print()
print("TEST OUTPUT")
print()
#print(all_outputs_p05)
print(all_outputs_p11)