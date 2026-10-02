"""PGM/YAML loading, conservative footprint inflation, and A* planning."""
from dataclasses import dataclass
from pathlib import Path
import heapq
import math
import numpy as np
from scipy.ndimage import distance_transform_edt, label
import yaml


def read_pgm(path):
    data=Path(path).read_bytes(); pos=0
    def token():
        nonlocal pos
        while pos<len(data):
            if data[pos] in b' \t\r\n': pos+=1
            elif data[pos]==35:
                end=data.find(b'\n',pos)
                pos=len(data) if end<0 else end+1
            else: break
        start=pos
        while pos<len(data) and data[pos] not in b' \t\r\n#': pos+=1
        if start==pos: raise ValueError('Truncated PGM header')
        return data[start:pos]
    magic=token(); width,height,maximum=int(token()),int(token()),int(token())
    if width<=0 or height<=0 or maximum!=255 or magic not in (b'P5',b'P2'):
        raise ValueError('Expected nonempty 8-bit P5/P2 PGM')
    if magic==b'P5':
        if pos>=len(data) or data[pos] not in b' \t\r\n': raise ValueError('Missing PGM separator')
        pos+=2 if data[pos:pos+2]==b'\r\n' else 1
        pixels=np.frombuffer(data[pos:],dtype=np.uint8)
        if pixels.size!=width*height: raise ValueError('PGM payload size mismatch')
    else:
        values=[int(token()) for _ in range(width*height)]
        if min(values)<0 or max(values)>255: raise ValueError('Invalid PGM pixel')
        pixels=np.array(values,dtype=np.uint8)
    return pixels.reshape(height,width).copy()


def write_pgm(path,image):
    image=np.asarray(image,dtype=np.uint8)
    if image.ndim!=2 or not image.size: raise ValueError('Invalid image')
    header=f'P5\n{image.shape[1]} {image.shape[0]}\n255\n'.encode('ascii')
    Path(path).write_bytes(header+image.tobytes())


@dataclass
class GridMap:
    occupancy: np.ndarray  # Bottom row first: -1 unknown, 0 free, 100 occupied.
    resolution: float
    origin: tuple
    radius: float=0.35
    margin: float=0.05

    def __post_init__(self):
        self.occupancy=np.asarray(self.occupancy,dtype=np.int8)
        if self.occupancy.ndim!=2 or not self.occupancy.size: raise ValueError('Map is empty')
        if not np.isin(self.occupancy,[-1,0,100]).all(): raise ValueError('Invalid occupancy')
        if len(self.origin)!=3 or not all(math.isfinite(v) for v in (*self.origin,self.resolution,self.radius,self.margin)):
            raise ValueError('Invalid map geometry')
        if self.resolution<=0 or self.radius<0 or self.margin<0: raise ValueError('Invalid resolution/footprint')
        self.height,self.width=self.occupancy.shape
        free=np.pad(self.occupancy==0,1,constant_values=False)
        # Bound obstacle-square extent and any position in the candidate square.
        self.clearance=distance_transform_edt(free)[1:-1,1:-1]*self.resolution-self.resolution*math.sqrt(2)
        self.passable=(self.occupancy==0)&(self.clearance>=self.radius+self.margin)

    @classmethod
    def load(cls,path,radius=0.35,margin=0.05):
        path=Path(path).resolve(); meta=yaml.safe_load(path.read_text(encoding='utf-8'))
        if not isinstance(meta,dict): raise ValueError('Map YAML must be an object')
        image=read_pgm(path.parent/meta['image']); negate=int(meta.get('negate',0))
        if negate not in (0,1): raise ValueError('Invalid negate')
        occupied=float(meta.get('occupied_thresh',.65)); free=float(meta.get('free_thresh',.196))
        if not 0<=free<occupied<=1: raise ValueError('Invalid map thresholds')
        probability=(image.astype(float) if negate else 255-image.astype(float))/255.0
        values=np.full(image.shape,-1,dtype=np.int8)
        values[probability<free]=0; values[probability>occupied]=100
        return cls(np.flipud(values),float(meta['resolution']),tuple(map(float,meta['origin'])),radius,margin)

    def continuous_cell(self,point):
        x,y=point; ox,oy,a=self.origin; c,s=math.cos(a),math.sin(a)
        return ((c*(x-ox)+s*(y-oy))/self.resolution,(-s*(x-ox)+c*(y-oy))/self.resolution)

    def cell(self,point):
        if len(point)!=2 or not all(math.isfinite(v) for v in point): raise ValueError('Nonfinite point')
        x,y=self.continuous_cell(point)
        return (math.floor(x),math.floor(y))

    def world(self,cell):
        x,y=(np.asarray(cell,dtype=float)+.5)*self.resolution
        ox,oy,a=self.origin; c,s=math.cos(a),math.sin(a)
        return (ox+c*x-s*y,oy+s*x+c*y)

    def is_free(self,cell):
        x,y=cell
        return 0<=x<self.width and 0<=y<self.height and bool(self.passable[y,x])

    def _segment_cells(self,start,end):
        """Collision-checked supercover, including side cells at corner crossings."""
        ax,ay=self.continuous_cell(start); bx,by=self.continuous_cell(end)
        if not all(math.isfinite(v) for v in (ax,ay,bx,by)): return None
        x,y=math.floor(ax),math.floor(ay); ex,ey=math.floor(bx),math.floor(by)
        if not self.is_free((x,y)) or not self.is_free((ex,ey)): return None
        cells=[(x,y),(ex,ey)]
        dx,dy=bx-ax,by-ay; sx=1 if dx>0 else -1; sy=1 if dy>0 else -1
        tx=((x+1-ax) if dx>0 else (ax-x))/abs(dx) if dx else math.inf
        ty=((y+1-ay) if dy>0 else (ay-y))/abs(dy) if dy else math.inf
        stepx=1/abs(dx) if dx else math.inf; stepy=1/abs(dy) if dy else math.inf
        for _ in range(abs(ex-x)+abs(ey-y)+3):
            if (x,y)==(ex,ey): return cells
            # The endpoint was checked above. Do not walk beyond it when
            # travelling negatively to an endpoint exactly on a grid line.
            if min(tx,ty)>=1.-1e-12: return cells
            if abs(tx-ty)<1e-10:
                side=((x+sx,y),(x,y+sy))
                if not all(self.is_free(c) for c in side): return None
                cells.extend(side)
                x+=sx; y+=sy; tx+=stepx; ty+=stepy
            elif tx<ty: x+=sx; tx+=stepx
            else: y+=sy; ty+=stepy
            if not self.is_free((x,y)): return None
            cells.append((x,y))
        return None

    def segment_free(self,start,end):
        return self._segment_cells(start,end) is not None

    def _segment_clearance(self,start,end):
        cells=self._segment_cells(start,end)
        if cells is None: return -math.inf
        return min(self.clearance[y,x] for x,y in cells)

    def plan(self,start,goal,max_expansions=300000):
        source,target=self.cell(start),self.cell(goal)
        if not self.is_free(source): raise ValueError('Start lacks known-free footprint clearance')
        if not self.is_free(target): raise ValueError('Goal lacks known-free clearance; goal was NOT moved')
        # This is a preference, not extra footprint inflation: narrow legal
        # passages and clicked endpoints remain available. With adequate room,
        # reserve 10 cm beyond the hard radius+margin for tracking/turn drift.
        preferred=self.radius+self.margin+.10
        if self._segment_clearance(start,goal)>=preferred:
            return [tuple(start),tuple(goal)]
        penalties=1.+8.*np.maximum(0.,(preferred-self.clearance)/.10)
        def heuristic(cell):
            dx,dy=abs(cell[0]-target[0]),abs(cell[1]-target[1])
            return max(dx,dy)+(math.sqrt(2)-1.)*min(dx,dy)
        queue=[(heuristic(source),0.0,source)]; cost={source:0.0}; parent={}; expanded=0
        while queue:
            _,g,current=heapq.heappop(queue)
            if g!=cost[current]: continue
            if current==target: break
            expanded+=1
            if expanded>max_expansions: raise ValueError('Planning expansion budget exceeded')
            x,y=current
            for dx,dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
                nxt=(x+dx,y+dy)
                if not self.is_free(nxt): continue
                if dx and dy and (not self.is_free((x+dx,y)) or not self.is_free((x,y+dy))): continue
                weight=max(penalties[y,x],penalties[y+dy,x+dx])
                if dx and dy:
                    # The supercover also touches both side cells. Penalize
                    # their clearance so diagonal shortcuts cannot hug corners.
                    weight=max(weight,penalties[y,x+dx],penalties[y+dy,x])
                ng=g+math.hypot(dx,dy)*float(weight)
                if ng<cost.get(nxt,math.inf):
                    cost[nxt]=ng; parent[nxt]=current
                    heapq.heappush(queue,(ng+heuristic(nxt),ng,nxt))
        else: raise ValueError('No path in known free space after footprint inflation')
        cells=[target]
        while cells[-1]!=source: cells.append(parent[cells[-1]])
        raw=[tuple(start)]+[self.world(c) for c in reversed(cells)]+[tuple(goal)]
        # Simplification must not undo the planner's clearance preference.
        # Compare every shortcut with the minimum clearance of the grid path
        # that it replaces, capped at the desired reserve. Endpoint approaches
        # and unavoidable narrow passages keep their original legal clearance.
        clearances=[self._segment_clearance(a,b) for a,b in zip(raw,raw[1:])]
        path=[raw[0]]; i=0
        while i<len(raw)-1:
            minimum=np.minimum.accumulate(clearances[i:])
            j=len(raw)-1
            while j>i+1:
                required=min(preferred,float(minimum[j-i-1]))
                if self._segment_clearance(raw[i],raw[j])>=required-1e-12: break
                j-=1
            if not self.segment_free(raw[i],raw[j]): raise ValueError('Path segment failed collision validation')
            path.append(raw[j]); i=j
        return path

    def summary(self):
        components,count=label(self.passable)
        sizes=np.bincount(components.ravel())[1:]
        return {'width':self.width,'height':self.height,'resolution':self.resolution,
                'radius':self.radius,'margin':self.margin,'free_cells':int((self.occupancy==0).sum()),
                'passable_cells':int(self.passable.sum()),'components':count,
                'largest_component':int(sizes.max()) if len(sizes) else 0}
