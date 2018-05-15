
import numpy as np

def carla_position_to_position(carla_location):
    # Negating one component because of LHC
    carla_to_metric = 1.0 / 100.0
    return np.array([carla_location.x, -carla_location.y, carla_location.z]) * carla_to_metric

def carla_rotator_to_quaternion(carla_rotator):
    # Excerpt from unreal math
    # FMath::SinCos(&SP, &CP, Pitch*DIVIDE_BY_2);
    # FMath::SinCos(&SY, &CY, Yaw*DIVIDE_BY_2);
    # FMath::SinCos(&SR, &CR, Roll*DIVIDE_BY_2);
    # FQuat RotationQuat;
    # RotationQuat.X =  CR*SP*SY - SR*CP*CY;
    # RotationQuat.Y = -CR*SP*CY - SR*CP*SY;
    # RotationQuat.Z =  CR*CP*SY - SR*SP*CY;
    # RotationQuat.W =  CR*CP*CY + SR*SP*SY;
    # Extracting components, dividing by 2, converting to radians, and half negating due to LHC
    pitch = (carla_rotator.pitch / 2) * np.pi/180
    roll = -1 * (carla_rotator.roll / 2) * np.pi/180
    yaw = -1 * (carla_rotator.yaw / 2) * np.pi/180
    # Doing the sin and cos
    cr = np.cos(roll)
    sr = np.sin(roll)
    cp = np.cos(pitch)
    sp = np.sin(pitch)
    cy = np.cos(yaw)
    sy = np.sin(yaw)
    # Making the quaternion
    qx = cr*sp*sy - sr*cp*cy
    qy = -cr*sp*cy - sr*cp*sy
    qz = cr*cp*sy - sr*sp*cy
    qw = cr*cp*cy + sr*sp*sy
    # Creating the vector
    return np.array([qx, qy, qz, qw])