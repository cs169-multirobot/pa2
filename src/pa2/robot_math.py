import math
import tf
import sys

def rectify_angle_2pi(angle):
    """
    Helper function for rectify_angle_pi.
    Adjusts angle to be in the range [0, 2pi]

    Args:
        angle: Angle that needs to be adjusted.
    Returns:
        The angle adjusted in the range [0, 2pi]
    """
    while angle < 0:
        angle += 2 * math.pi
    while angle > 2 * math.pi:
        angle -= 2 * math.pi
    return angle


def rectify_angle_pi(angle):
    """
    Adjusts the angle to be in the range [-pi, pi]

    Args:
        angle: Angle that needs to be adjusted.
    Returns:
        The angle adjusted in the range [-pi, pi]
    """
    # adjust angle to be in the range [0, 2pi]
    angle = rectify_angle_2pi(angle)

    # angles over pi should wrap around in the range [-pi, 0]
    if angle > math.pi:
        angle -= 2 * math.pi

    return angle


def quat_to_euler(odom):
    """
    Convert quaternion to euler angles.

    Args:
        odom: Orientation parameterized using quaternions.
    Returns:
        Orientation represented as yaw.
    """
    quaternion = (
        odom.x,
        odom.y,
        odom.z,
        odom.w
    )
    euler = tf.transformations.euler_from_quaternion(quaternion)
    yaw = euler[2]

    return yaw
