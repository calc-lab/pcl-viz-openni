import numpy as np

loam_odom_filename = "/media/pku-m/OrangePassport4T/loam-201/xiaoyuan/odometry.txt"
gps_odom_filename = "/media/pku-m/OrangePassport4T/loam-201/xiaoyuan/odom_gps.txt"

def remove_duplicate(raw_data):
    new_data = []
    for i in range(1,len(raw_data)):
        time_now = raw_data[i,0]
        time_pre = raw_data[i-1,0]
        if time_now == time_pre:
            continue
        else:
            new_data.append(raw_data[i-1])

    return new_data

loam_raw_data = np.genfromtxt(loam_odom_filename)
loam_data = remove_duplicate(loam_raw_data)[0]

loam_data[:,0] = np.int64(loam_data[:,0])

np.savetxt("loam.txt", loam_data)

