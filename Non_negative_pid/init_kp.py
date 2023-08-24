import numpy as np

from pid_core import pid,pid_stability

def pid_parameters_opt_kp(exp_val,pid_itr,pid_para,res_kp,volume,init_speed,standard):
    break_flag=0
    print(standard)
    for j in range(1, 50):
        sensor_data=pid_implement(exp_val,pid_itr,pid_para, volume, init_speed)
        if np.max(sensor_data)>=exp_val:
            print(np.max(sensor_data))
            break
        if np.max(sensor_data)<exp_val*(standard-0.3):
            pid_para[0]+=res_kp*4
            print(np.max(sensor_data))
        if np.max(sensor_data)<exp_val*(standard-0.2):
            pid_para[0]+=res_kp*3
            print(np.max(sensor_data))
        if np.max(sensor_data) < exp_val * (standard-0.1):
            pid_para[0]+=res_kp*2
            print(np.max(sensor_data))
        else:
            pid_para[0]+=res_kp
            print(np.max(sensor_data))
    print('kp got')
    print(pid_para[0])
    return pid_para[0]