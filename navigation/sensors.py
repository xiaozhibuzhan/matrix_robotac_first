"""Decode simulator PointCloud2 without depending on task-one Python versions."""
import numpy as np


def lidar_points(message):
    width,height,step,row_step=map(int,(message.width,message.height,message.point_step,message.row_step))
    if width<=0 or height<=0 or step<=0 or row_step<width*step:
        raise ValueError('Invalid/empty PointCloud2 dimensions')
    data=memoryview(message.data)
    if data.nbytes!=height*row_step: raise ValueError('PointCloud2 payload length mismatch')
    endian='>' if message.is_bigendian else '<'
    fields={}
    for field in message.fields:
        if field.name not in ('x','y','z','intensity'): continue
        if field.name in fields or field.count!=1: raise ValueError('Duplicate/nonscalar cloud field')
        types={2:'u1',3:'i2',4:'u2',5:'i4',6:'u4',7:'f4',8:'f8'}
        if field.datatype not in types: raise ValueError('Unsupported cloud field type')
        if field.name!='intensity' and field.datatype not in (7,8): raise ValueError('XYZ must be floating point')
        dtype=np.dtype(endian+types[field.datatype])
        if field.offset<0 or field.offset+dtype.itemsize>step: raise ValueError('Cloud field exceeds point_step')
        fields[field.name]=np.ndarray((height,width),dtype=dtype,buffer=data,offset=field.offset,strides=(row_step,step)).reshape(-1)
    if not all(name in fields for name in ('x','y','z')): raise ValueError('Cloud is missing XYZ')
    xyz=np.column_stack([fields[name] for name in ('x','y','z')])
    if 'intensity' in fields:
        # Simulator metadata encodes absolute poses with intensity 111.
        # Remove these records; never interpret them as navigation pose or obstacles.
        xyz=xyz[~np.isclose(fields['intensity'],111.,rtol=0,atol=1e-5)]
    return xyz
