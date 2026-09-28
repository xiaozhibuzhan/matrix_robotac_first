"""Pure geometry; metres, radians, ROS XYZW quaternions."""
import math
import numpy as np


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def normalized_quaternion(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all():
        raise ValueError('Invalid quaternion')
    norm = float(np.linalg.norm(q))
    if norm < 1e-9:
        raise ValueError('Zero quaternion')
    return q / norm


def yaw_of(q):
    x,y,z,w = normalized_quaternion(q)
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def rotation_matrix(q):
    x,y,z,w = normalized_quaternion(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def rpy_matrix(rpy):
    r,p,y = map(float,rpy)
    if not all(math.isfinite(v) for v in (r,p,y)):
        raise ValueError('Invalid sensor rotation')
    cr,sr,cp,sp,cy,sy = math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                     [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])


def align_pose(initial, odom):
    """Planar map <- odom transform; does not move the simulator."""
    if not all(math.isfinite(v) for v in (*initial,*odom)):
        raise ValueError('Invalid initial pose')
    theta=wrap(initial[2]-odom[2]); c,s=math.cos(theta),math.sin(theta)
    return (initial[0]-c*odom[0]+s*odom[1],initial[1]-s*odom[0]-c*odom[1],theta)


def transform_pose(transform, pose):
    x,y,a=transform; c,s=math.cos(a),math.sin(a)
    return (x+c*pose[0]-s*pose[1],y+s*pose[0]+c*pose[1],wrap(a+pose[2]))
