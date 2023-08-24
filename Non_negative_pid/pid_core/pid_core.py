

def pid(exp_val, sensor_data, kp, ki, kd,pid_now_err,pid_sum_err):
    pid_last_err = pid_now_err

    pid_now_err = exp_val - sensor_data

    pid_sum_err += pid_now_err

    pid_output_value = kp * (pid_now_err) + ki * pid_sum_err + kd* (pid_now_err - pid_last_err)
    #print(pid_output_value)
    #pid_output_value=pid_stability(exp_val, pid_output_value)
    return [pid_output_value,pid_now_err,pid_sum_err]

def pid_stability(pid_output_value):

    if pid_output_value<0:

        pid_output_value=1

    return pid_output_value