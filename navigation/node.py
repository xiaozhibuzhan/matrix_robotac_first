"""ROS2 Humble adapter for RViz clicked-point navigation. Preview is the default."""
import argparse
import json
import math
from pathlib import Path
import time
import numpy as np
from .configuration import load_config
from .core import Navigator
from .geometry import yaw_of
from .grid import GridMap
from .sdk_bridge import SDKBridge
from .sensors import lidar_points


def _run_ros_node(rclpy, node_factory, ros_args, external_shutdown_exception,
                  rcl_error_exception):
    """Close application resources before idempotent ROS context cleanup."""
    rclpy.init(args=ros_args); node=None
    try:
        node=node_factory(); rclpy.spin(node)
    except (KeyboardInterrupt, external_shutdown_exception):
        pass
    finally:
        try:
            if node is not None:
                try:
                    node.close()
                finally:
                    node.destroy_node()
        finally:
            # Humble's C signal handler can race even with try_shutdown()'s
            # internal ok() check. Ignore only that error after confirming the
            # context is already inactive; unrelated cleanup failures matter.
            try:
                rclpy.try_shutdown()
            except rcl_error_exception as exc:
                message=str(exc).partition(', at ')[0]
                already_stopped=('failed to shutdown: rcl_shutdown already '
                                 'called on the given context')
                if message!=already_stopped or rclpy.ok():
                    raise


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',required=True); parser.add_argument('--run-dir',required=True)
    parser.add_argument('--drive',action='store_true'); parser.add_argument('--stand-up',action='store_true')
    parser.add_argument('--use-sim-time',action='store_true')
    args,ros_args=parser.parse_known_args()
    if args.stand_up and not args.drive: parser.error('--stand-up requires --drive')
    try:
        import rclpy
        from rclpy.executors import ExternalShutdownException
        from rclpy.impl.implementation_singleton import rclpy_implementation
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.clock import Clock,ClockType
        from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
        from geometry_msgs.msg import PointStamped,PoseWithCovarianceStamped,PoseStamped,Point,Twist
        from nav_msgs.msg import Odometry,OccupancyGrid,Path as RosPath
        from sensor_msgs.msg import PointCloud2
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        from visualization_msgs.msg import Marker,MarkerArray
    except ImportError as exc: parser.exit(1,f'Load the target ROS2 Humble system Python environment: {exc}'+chr(10))
    cfg=load_config(args.config); grid=GridMap.load(cfg['map'],cfg['robot_radius'],cfg['safety_margin'])
    if not grid.passable.any(): parser.exit(1,'Map contains no robot-sized free space; see prepare_map'+chr(10))
    run_dir=Path(args.run_dir); run_dir.mkdir(parents=True,exist_ok=True)

    class PointNavigationNode(Node):
        def __init__(self):
            super().__init__('task2_point_navigation',parameter_overrides=[Parameter('use_sim_time',value=args.use_sim_time)])
            self.nav=Navigator(grid,cfg); self.bridge=None; self.bridge_failed=False
            self.events=open(run_dir/'events.jsonl','a',encoding='utf-8',buffering=1)
            self.last_state=None; self.last_path=None; self.last_report=0.; self.errors={}
            sensor=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
            durable=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.map_pub=self.create_publisher(OccupancyGrid,'/task2/map',durable)
            self.clearance_pub=self.create_publisher(OccupancyGrid,'/task2/clearance',durable)
            self.path_pub=self.create_publisher(RosPath,'/task2/path',durable)
            self.pose_pub=self.create_publisher(PoseStamped,'/task2/pose',10)
            self.marker_pub=self.create_publisher(MarkerArray,'/task2/markers',durable)
            self.status_pub=self.create_publisher(String,'/task2/status',durable)
            self.command_pub=self.create_publisher(Twist,'/task2/cmd_vel' if args.drive else '/task2/command_preview',10)
            self.create_subscription(Odometry,cfg['odom_topic'],self.on_odom,sensor)
            self.create_subscription(PointCloud2,cfg['cloud_topic'],self.on_cloud,sensor)
            self.create_subscription(PoseWithCovarianceStamped,cfg['initialpose_topic'],self.on_initial,10)
            self.create_subscription(PointStamped,cfg['clicked_point_topic'],self.on_goal,10)
            self.create_service(Trigger,'/task2/cancel',self.on_cancel)
            self.publish_map(grid.occupancy,self.map_pub)
            self.publish_map(np.where(grid.passable,0,100).astype(np.int8),self.clearance_pub)
            if args.drive:
                self.bridge=SDKBridge(args.config,run_dir/'sdk.log',args.stand_up)
            self.record('startup',{'drive':args.drive,'map':grid.summary(),'config':cfg})
            # A paused /clock must not pause the stop/command freshness timers.
            self.steady_clock=Clock(clock_type=ClockType.STEADY_TIME)
            self.timer=self.create_timer(1/cfg['control_hz'],self.on_tick,clock=self.steady_clock)
            self.get_logger().info('DRIVE enabled' if args.drive else 'PREVIEW: SDK not loaded; no robot commands sent')

        def record(self,event,details):
            self.events.write(json.dumps({'event':event,'wall_time':time.time(),'monotonic':time.monotonic(),'details':details},ensure_ascii=False,allow_nan=False)+chr(10))

        def publish_map(self,values,publisher):
            msg=OccupancyGrid(); msg.header.frame_id=cfg['frame']; msg.header.stamp=self.get_clock().now().to_msg()
            msg.info.map_load_time=msg.header.stamp; msg.info.resolution=float(grid.resolution)
            msg.info.width=grid.width; msg.info.height=grid.height
            msg.info.origin.position.x=float(grid.origin[0]); msg.info.origin.position.y=float(grid.origin[1])
            msg.info.origin.orientation.z=math.sin(grid.origin[2]/2); msg.info.origin.orientation.w=math.cos(grid.origin[2]/2)
            msg.data=values.ravel().tolist(); publisher.publish(msg)

        def sensor_stamp(self,header):
            stamp=header.stamp.sec+header.stamp.nanosec*1e-9
            current=self.get_clock().now().nanoseconds*1e-9
            if stamp<=0 or abs(current-stamp)>cfg['max_header_age']:
                raise ValueError('Sensor header is stale/future or wrong clock; check ROS /clock setting')
            return stamp

        def rejected(self,kind,exc,lose=False):
            self.nav.fail(f'{kind}: {exc}',lose)
            self.send_zero()
            key=(kind,str(exc)); now=time.monotonic()
            if now-self.errors.get(key,-math.inf)>2:
                self.errors[key]=now; self.get_logger().warning(self.nav.reason)
                self.record('rejected',{'kind':kind,'reason':str(exc)})

        def on_odom(self,msg):
            try:
                p=msg.pose.pose.position; q=msg.pose.pose.orientation
                self.nav.update_odometry((p.x,p.y,p.z),(q.x,q.y,q.z,q.w),self.sensor_stamp(msg.header),time.monotonic(),(msg.header.frame_id,msg.child_frame_id))
            except (ValueError,TypeError,OverflowError) as exc: self.rejected('odometry',exc,True)

        def on_cloud(self,msg):
            try:
                stamp=self.sensor_stamp(msg.header)
                if msg.header.frame_id!=cfg['cloud_frame']: raise ValueError('Unexpected LiDAR frame; verify extrinsics')
                points=lidar_points(msg)
                self.nav.update_cloud(points,stamp,time.monotonic())
            except (ValueError,TypeError,OverflowError) as exc:
                self.nav.cloud_received=None; self.rejected('LiDAR',exc)

        def input_frame(self,msg):
            if msg.header.frame_id!=cfg['frame']:
                raise ValueError(f'Expected {cfg["frame"]} coordinates, received {msg.header.frame_id!r}; target not transformed silently')

        def on_initial(self,msg):
            self.nav.cancel('Initial pose changed'); self.send_zero()
            try:
                self.input_frame(msg); p=msg.pose.pose.position; q=msg.pose.pose.orientation
                if not all(math.isfinite(v) for v in (p.x,p.y,p.z,*msg.pose.covariance)): raise ValueError('Nonfinite initial pose')
                accepted=self.nav.initialize((p.x,p.y,yaw_of((q.x,q.y,q.z,q.w))),time.monotonic())
                self.record('initial_pose',{'accepted':accepted,'pose':[p.x,p.y,self.nav.pose[2] if self.nav.pose else None],'reason':self.nav.reason})
            except (ValueError,TypeError) as exc: self.rejected('initial pose',exc,True)

        def on_goal(self,msg):
            self.nav.cancel('New goal requested'); self.send_zero()
            try:
                if self.bridge_failed: raise ValueError('SDK worker failed; restart navigation before driving')
                self.input_frame(msg); p=msg.point
                if not all(math.isfinite(v) for v in (p.x,p.y,p.z)) or abs(p.z-cfg['ground_z'])>.2:
                    raise ValueError('Click the 2D map plane, not a 3D obstacle/robot marker')
                accepted=self.nav.set_goal((p.x,p.y),time.monotonic())
                self.record('clicked_goal',{'accepted':accepted,'point':[p.x,p.y,p.z],'frame':msg.header.frame_id,'reason':self.nav.reason})
            except (ValueError,TypeError) as exc: self.rejected('goal',exc)

        def on_cancel(self,request,response):
            self.nav.cancel('Cancelled by /task2/cancel'); self.send_zero()
            self.record('cancel',{}); response.success=True; response.message='Goal cleared; zero velocity requested; check measured standstill'
            return response

        def send_zero(self):
            self.nav.command=(0.,0.)
            if self.bridge is not None and not self.bridge_failed:
                try: self.bridge.send(0.,0.)
                except (OSError,RuntimeError) as exc:
                    self.bridge_failed=True; self.nav.fail('SDK failed: '+str(exc))

        def on_tick(self):
            try:
                now=time.monotonic(); v,w=self.nav.tick(now)
                if self.bridge is not None and not self.bridge_failed:
                    try: self.bridge.send(v,w)
                    except (OSError,RuntimeError) as exc:
                        self.bridge_failed=True; self.nav.fail('SDK failed: '+str(exc)); v=w=0.
                if self.bridge_failed: self.nav.fail('SDK worker failed; restart navigation'); v=w=0.
                command=Twist(); command.linear.x=float(v); command.angular.z=float(w); self.command_pub.publish(command)
                state=self.nav.snapshot(); key=(state['state'],state['reason'])
                if key!=self.last_state or now-self.last_report>=.5:
                    state['mode']='drive' if args.drive else 'preview'
                    text=String(); text.data=json.dumps(state,allow_nan=False); self.status_pub.publish(text)
                    self.record('state' if key!=self.last_state else 'trajectory',state)
                    if key!=self.last_state: self.get_logger().info(text.data)
                    self.last_state=key; self.last_report=now
                    self.publish_visuals()
            except Exception:
                self.nav.fail('Unexpected navigation exception'); self.send_zero()
                raise

        def publish_visuals(self):
            stamp=self.get_clock().now().to_msg()
            path_key=tuple(self.nav.path)
            if path_key!=self.last_path:
                path=RosPath(); path.header.frame_id=cfg['frame']; path.header.stamp=stamp
                for x,y in self.nav.path:
                    p=PoseStamped(); p.header=path.header; p.pose.position.x=float(x); p.pose.position.y=float(y); p.pose.position.z=.03; p.pose.orientation.w=1.
                    path.poses.append(p)
                self.path_pub.publish(path); self.last_path=path_key
            markers=MarkerArray(); clear=Marker(); clear.action=Marker.DELETEALL; markers.markers.append(clear)
            if self.nav.pose is not None:
                x,y,yaw=self.nav.pose
                pose=PoseStamped(); pose.header.frame_id=cfg['frame']; pose.header.stamp=stamp
                pose.pose.position.x=x; pose.pose.position.y=y; pose.pose.position.z=.05
                pose.pose.orientation.z=math.sin(yaw/2); pose.pose.orientation.w=math.cos(yaw/2); self.pose_pub.publish(pose)
                ring=Marker(); ring.header=pose.header; ring.ns='task2'; ring.id=0; ring.type=Marker.CYLINDER; ring.action=Marker.ADD; ring.pose=pose.pose
                ring.scale.x=ring.scale.y=2*(cfg['robot_radius']+cfg['safety_margin']); ring.scale.z=.03
                ring.color.g=1.; ring.color.a=.35; markers.markers.append(ring)
                obstacles=Marker(); obstacles.header=pose.header; obstacles.ns='task2'; obstacles.id=2; obstacles.type=Marker.POINTS; obstacles.action=Marker.ADD; obstacles.pose.orientation.w=1.
                obstacles.scale.x=obstacles.scale.y=.05; obstacles.color.r=1.; obstacles.color.a=.8
                if len(self.nav.obstacles):
                    a=self.nav.alignment; c,s=math.cos(a[2]),math.sin(a[2]); xy=self.nav.obstacles@np.array([[c,s],[-s,c]])+np.array(a[:2])
                    _,indices=np.unique(np.floor(xy/.10).astype(np.int64),axis=0,return_index=True)
                    for ox,oy in xy[indices][::max(1,len(indices)//2500)]:
                        point=Point(); point.x=float(ox); point.y=float(oy); point.z=.08; obstacles.points.append(point)
                markers.markers.append(obstacles)
            if self.nav.goal is not None:
                goal=Marker(); goal.header.frame_id=cfg['frame']; goal.header.stamp=stamp; goal.ns='task2'; goal.id=1; goal.type=Marker.SPHERE; goal.action=Marker.ADD
                goal.pose.position.x=self.nav.goal[0]; goal.pose.position.y=self.nav.goal[1]; goal.pose.position.z=.12; goal.pose.orientation.w=1.
                goal.scale.x=goal.scale.y=goal.scale.z=.18; goal.color.r=1.; goal.color.g=.7; goal.color.a=1.; markers.markers.append(goal)
            self.marker_pub.publish(markers)

        def close(self):
            self.nav.cancel('Node shutting down'); self.send_zero()
            if self.bridge is not None: self.bridge.close()
            if not self.events.closed:
                self.record('shutdown',self.nav.snapshot()); self.events.close()

    _run_ros_node(rclpy, PointNavigationNode, ros_args, ExternalShutdownException,
                  rclpy_implementation.RCLError)


if __name__=='__main__': main()
