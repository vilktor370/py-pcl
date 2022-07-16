import os

# if 'pcl.so' in os.listdir('/home/haochen/Projects/pybinding/cmake-build-release'):
#     SRC = '/home/haochen/Projects/pybinding/cmake-build-release/pcl.so '
#     DST = '/home/haochen/Projects/pybinding/output/pcl.so'
#     os.system(f"mv{SRC} {DST}")


import lib.pcl as pcl
import numpy as np
pts = pcl.PointCloudXYZ(10, 10)
# for i in pts:
p = pcl.PointCloudXYZ(pts)
p.resize(50)
print(pcl)