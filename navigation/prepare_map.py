"""Create a navigation-only, locally supported ground raster; never overwrite task 1."""
from pathlib import Path
import argparse
import hashlib
import json
import math
import numpy as np
from scipy.spatial import Delaunay, QhullError
from scipy.ndimage import label
import yaml
from mapping.clean_pcd import read_pcd
from .grid import GridMap,write_pgm
from .configuration import ROOT


def ground_support(points,grid,ground_z=0.,band=.06,max_edge=.30,max_span=.04,max_slope=.18):
    if not all(math.isfinite(x) for x in (ground_z,band,max_edge,max_span,max_slope)):
        raise ValueError('Nonfinite ground parameters')
    if min(band,max_edge,max_span,max_slope)<=0: raise ValueError('Nonpositive ground parameters')
    xyz=np.asarray(points,dtype=float)
    if xyz.ndim!=2 or xyz.shape[1]!=3: raise ValueError('Expected Nx3 points')
    ground=xyz[np.isfinite(xyz).all(axis=1)&(np.abs(xyz[:,2]-ground_z)<=band)]
    if len(ground)<3: raise ValueError('Insufficient measured ground support')
    # Keep actual measured representatives, not synthetic cell-centre vertices.
    _,indices=np.unique(np.floor(ground[:,:2]/(grid.resolution*.5)).astype(np.int64),axis=0,return_index=True)
    ground=ground[indices]
    try: triangulation=Delaunay(ground[:,:2])
    except QhullError as exc: raise ValueError('Ground samples have no 2D surface support') from exc
    vertices=ground[triangulation.simplices]
    edge1=vertices[:,1]-vertices[:,0]; edge2=vertices[:,2]-vertices[:,0]
    normals=np.cross(edge1,edge2); norm=np.linalg.norm(normals,axis=1)
    longest=np.max(np.stack([np.linalg.norm(vertices[:,i,:2]-vertices[:,j,:2],axis=1) for i,j in ((0,1),(1,2),(2,0))]),axis=0)
    valid=(longest<=max_edge)&(np.ptp(vertices[:,:,2],axis=1)<=max_span)&(norm>1e-6)
    valid &= np.abs(normals[:,2])>=math.cos(max_slope)*norm
    yy,xx=np.indices(grid.occupancy.shape); local=np.column_stack(((xx.ravel()+.5)*grid.resolution,(yy.ravel()+.5)*grid.resolution))
    ox,oy,yaw=grid.origin; c,s=math.cos(yaw),math.sin(yaw)
    query=local@np.array([[c,s],[-s,c]])+np.array([ox,oy])
    simplex=triangulation.find_simplex(query)
    support=(simplex>=0)&valid[np.maximum(simplex,0)]
    return support.reshape(grid.occupancy.shape),{'ground_representatives':len(ground),'triangles':len(valid),'accepted_triangles':int(valid.sum()),'max_edge_m':max_edge,'ground_band_m':band,'max_height_span_m':max_span,'max_slope_radians':max_slope}


def prepare(source_yaml,pcd,output,**kwargs):
    source_yaml=Path(source_yaml).resolve(); pcd=Path(pcd).resolve(); output=Path(output).resolve()
    allowed=(ROOT/'navigation/maps').resolve()
    if allowed not in output.parents: raise ValueError('Output must be a NEW subdirectory of navigation/maps')
    if output.exists(): raise FileExistsError(f'Output already exists; select a new directory: {output}')
    source=GridMap.load(source_yaml)
    xyz=read_pcd(pcd)
    support,details=ground_support(xyz,source,**kwargs)
    occupancy=source.occupancy.copy()
    inferred=support&(occupancy==-1)
    occupancy[inferred]=0  # Existing obstacles always win.
    result=GridMap(occupancy,source.resolution,source.origin,source.radius,source.margin)
    if not result.passable.any(): raise ValueError('No robot-sized supported space; acquire more data or inspect ground parameters')
    components,_=label(result.passable)
    counts=np.bincount(components.ravel()); counts[0]=0
    largest=np.argwhere(components==counts.argmax())
    nominal=np.array(result.cell((0.,0.)))[::-1]
    start_cell=largest[np.linalg.norm(largest-nominal,axis=1).argmin()][::-1]
    far_cell=largest[np.linalg.norm(largest-start_cell[::-1],axis=1).argmax()][::-1]
    start=result.world(start_cell); goal=result.world(far_cell)
    path=result.plan(start,goal)
    meta=yaml.safe_load(source_yaml.read_text(encoding='utf-8'))
    original_image=(source_yaml.parent/meta['image']).resolve()
    report={'method':'bounded measured-ground triangles; no extrapolation; task-one obstacles preserved',
            'source_yaml':source_yaml.relative_to(ROOT).as_posix() if ROOT in source_yaml.parents else str(source_yaml),
            'source_pcd':pcd.relative_to(ROOT).as_posix() if ROOT in pcd.parents else str(pcd),
            'source_sha256':{str(p.name):hashlib.sha256(p.read_bytes()).hexdigest() for p in (source_yaml,original_image,pcd)},
            'support':details,'inferred_free_cells':int(inferred.sum()),'before':source.summary(),'after':result.summary(),
            'footprint_inflation':'EDT minus resolution*sqrt(2): obstacle extent and any centre within a candidate cell',
            'offline_example_only':{'start':start,'goal':goal,'path':path,'length_m':sum(math.dist(a,b) for a,b in zip(path,path[1:]))},
            'limits':['Derived free space is an inference, not a new sensor measurement.','Verify map, gait footprint, localization and floor support on target Ubuntu.','Example coordinates are not the current robot pose or prescribed goals.']}
    output.mkdir(parents=True,exist_ok=False)
    image=np.full(occupancy.shape,205,dtype=np.uint8); image[occupancy==0]=254; image[occupancy==100]=0
    write_pgm(output/'field_map.pgm',np.flipud(image))
    write_pgm(output/'inferred_support.pgm',np.flipud(np.where(inferred,254,0).astype(np.uint8)))
    write_pgm(output/'robot_clearance.pgm',np.flipud(np.where(result.passable,254,0).astype(np.uint8)))
    # Normalize to canonical trinary encoding regardless of source image encoding.
    out_meta={'image':'field_map.pgm','resolution':result.resolution,'origin':list(result.origin),'negate':0,'occupied_thresh':.65,'free_thresh':.196}
    (output/'field_map.yaml').write_text(yaml.safe_dump(out_meta,sort_keys=False),encoding='utf-8')
    (output/'report.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    baseline=ROOT/'maps/run_20260911_223510_ujmvehqs'
    parser.add_argument('--source-map',type=Path,default=baseline/'field_map.yaml')
    parser.add_argument('--pcd',type=Path,default=baseline/'field_map.pcd')
    parser.add_argument('--output',type=Path,default=ROOT/'navigation/maps/task1_20260911')
    parser.add_argument('--max-edge',type=float,default=.30)
    parser.add_argument('--ground-z',type=float,default=0.)
    parser.add_argument('--band',type=float,default=.06)
    args=parser.parse_args()
    try:
        report=prepare(args.source_map,args.pcd,args.output,max_edge=args.max_edge,ground_z=args.ground_z,band=args.band)
        print(json.dumps({'output':str(args.output),'after':report['after'],'inferred_free_cells':report['inferred_free_cells'],'example':report['offline_example_only']},indent=2))
    except (ValueError,OSError) as exc: parser.exit(1,f'Cannot prepare navigation map: {exc}\n')


if __name__=='__main__': main()
