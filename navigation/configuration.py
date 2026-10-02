"""Validate settings before creating publishers or connecting to the SDK."""
from pathlib import Path
import math
import yaml
from .sdk_limits import MIN_FORWARD_SPEED,MIN_YAW_RATE

ROOT=Path(__file__).resolve().parents[1]
DEFAULT=Path(__file__).resolve().parent/'config/default.yaml'


def load_config(path=DEFAULT,map_override=None):
    defaults=yaml.safe_load(DEFAULT.read_text(encoding='utf-8'))
    data=yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    if not isinstance(data,dict) or set(data)-set(defaults):
        raise ValueError('Configuration is not an object or contains unknown keys')
    cfg={**defaults,**data}
    numeric=[key for key,value in defaults.items() if isinstance(value,(int,float))]
    for key in numeric:
        value=cfg[key]
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value):
            raise ValueError(f'{key} must be finite numeric')
        if key!='ground_z' and value<=0:
            raise ValueError(f'{key} must be positive')
    for key in ('sensor_translation','sensor_rpy'):
        if not isinstance(cfg[key],list) or len(cfg[key])!=3 or not all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) for v in cfg[key]):
            raise ValueError(f'Invalid {key}')
    for key in ('frame','odom_topic','cloud_topic','initialpose_topic','clicked_point_topic','cloud_frame','client_ip','robot_ip'):
        if not isinstance(cfg[key],str) or not cfg[key].strip(): raise ValueError(f'Invalid {key}')
    if not isinstance(cfg['client_port'],int) or not 1024<=cfg['client_port']<=65535:
        raise ValueError('Invalid client_port')
    if not isinstance(cfg['max_tracking_replans'],int) or not 1<=cfg['max_tracking_replans']<=5:
        raise ValueError('max_tracking_replans must be an integer from 1 to 5')
    if cfg['max_speed']>.5 or cfg['max_yaw_rate']>1 or cfg['control_hz']<10:
        raise ValueError('First-version limits: speed <=0.5, yaw <=1.0, control_hz >=10')
    if cfg['max_speed']<MIN_FORWARD_SPEED or cfg['max_yaw_rate']<MIN_YAW_RATE:
        raise ValueError('SDK requires max_speed >=0.05 and max_yaw_rate >=0.02')
    if cfg['command_timeout']<2/cfg['control_hz'] or cfg['command_timeout']>1:
        raise ValueError('Command watchdog must span >=2 control periods and <=1 second')
    if cfg['obstacle_min_height']>=cfg['obstacle_max_height']:
        raise ValueError('Invalid obstacle height interval')
    if cfg['goal_tolerance']>cfg['local_lookahead'] or cfg['stop_speed']>=cfg['max_speed']:
        raise ValueError('Invalid tracking/settling thresholds')
    source=Path(map_override or cfg['map']).expanduser()
    cfg['map']=str(source.resolve() if source.is_absolute() else (ROOT/source).resolve())
    return cfg
