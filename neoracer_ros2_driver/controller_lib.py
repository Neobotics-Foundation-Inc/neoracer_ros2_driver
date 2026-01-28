"""
controller_lib.py

Helper functions for controller.py
"""

def parse_serial_data(line:str):
    """
    Parse the line data coming from the serial port in accordance to the following encoding rules:
    - If the line starts with the character [i], then it represents IMU data, in order:
        - qx qy qz qw ax ay az gx gy gz
    - If the line starts with the character [o], then it represents odom data, in order:
        - px py pz vx vy vz w

    Return two dictionaries that represent imu_data and odom_data.
    """

    data = {}

    parts = line.split()
    if not parts:
        return None, None

    cmd_type = parts[0]

    if cmd_type == 'i' and len(parts) == 11:
        # i qx qy qz qw ax ay az gx gy gz
        q_x, q_y, q_z, q_w = map(float, parts[1:5]) # quaternion
        a_x, a_y, a_z = map(float, parts[5:8])      # accelerometer
        g_x, g_y, g_z = map(float, parts[8:11])     # gyroscope

        # map to dict
        data['q_x'] = q_x
        data['q_y'] = q_y
        data['q_z'] = q_z
        data['q_w'] = q_w
        data['a_x'] = a_x
        data['a_y'] = a_y
        data['a_z'] = a_z
        data['g_x'] = g_x
        data['g_y'] = g_y
        data['g_z'] = g_z

        return data, 'i'

    if cmd_type == 'o' and len(parts) == 8:
        # o px py pz vx vy vz w
        p_x, p_y, p_z = map(float, parts[1:4])
        v_x, v_y, v_z = map(float, parts[4:7])
        yaw = float(parts[7])

        # map to dict
        data['p_x'] = p_x
        data['p_y'] = p_y
        data['p_z'] = p_z
        data['v_x'] = v_x
        data['v_y'] = v_y
        data['v_z'] = v_z
        data['yaw'] = yaw

        return data, 'o'

    return None, None

def quaternion_from_euler(roll, pitch, yaw):
    """Convers attitude value to a quaternion estimate"""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = sr * cp * cy - cr * sp * sy # x
    q[1] = cr * sp * cy + sr * cp * sy # y
    q[2] = cr * cp * sy - sr * sp * cy # z
    q[3] = cr * cp * cy + sr * sp * sy # w
    return q