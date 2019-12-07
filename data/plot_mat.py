#!/usr/bin/python3
#coding:utf-8
import matplotlib.pyplot as plt
import pandas as pd
import math

my_cols = ['px_est', 'py_est', 'vx_est', 'vy_est', 'vx_filter', 'vy_filter' ,'yaw', 'px_meas', 'py_meas', 'px_gt', 'py_gt', 'vx_gt', 'vy_gt']
with open('/home/guangtong/project/ukf-code/data/output1.txt') as f:
    table_ekf_output = pd.read_table(f, sep=' ', header=None, names=my_cols, lineterminator='\n')

    # table_ekf_output
plt.subplot(2,2,1)
len=[]
plt.plot(table_ekf_output['px_est'],table_ekf_output['py_est'])
len.append("xy_estmation")
plt.plot(table_ekf_output['px_meas'],table_ekf_output['py_meas'])
len.append("xy_measurement")
plt.plot(table_ekf_output['px_gt'],table_ekf_output['py_gt'])
len.append("xy_groundtruth")
plt.legend(len)
plt.xlabel("axis x/m")
plt.ylabel("axis y/m")
plt.subplot(2,2,2)
len=[]
plt.plot(table_ekf_output['px_gt'],table_ekf_output['vx_est'])
len.append("vx_estmation")
plt.plot(table_ekf_output['px_gt'],table_ekf_output['vx_filter'])
len.append("vx_mean_filter_4")
plt.plot(table_ekf_output['px_gt'],table_ekf_output['vx_gt'])
len.append("vx_groundtruth")
plt.legend(len)
plt.xlabel("axis x/m")
plt.ylabel("axis vx/m")
plt.subplot(2,2,3)
len=[]
plt.plot(table_ekf_output['px_gt'],table_ekf_output['vy_est'])
len.append("vy_estmation")
plt.plot(table_ekf_output['px_gt'],table_ekf_output['vy_filter'])
len.append("vy_mean_filter_4")
plt.plot(table_ekf_output['px_gt'],table_ekf_output['vy_gt'])
len.append("vy_groundtruth")
plt.legend(len)
plt.xlabel("axis x/m")
plt.ylabel("axis vy/m")
plt.subplot(2,2,4)
len=[]
plt.plot(table_ekf_output['px_gt'],table_ekf_output['yaw'])
len.append("yaw_estmation")
plt.xlabel("axis x/m")
plt.ylabel("axis yaw/m")
# plt.plot(table_ekf_output['vx_gt'],table_ekf_output['px_gt'])
# len.append("vx_groundtruth")
plt.legend(len)
plt.show()