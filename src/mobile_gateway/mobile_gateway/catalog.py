"""Allowlisted runbook commands; never interprets client input as shell code."""
from dataclasses import dataclass, field, replace
from pathlib import Path
import math
import re

MAPPERS = ('slam_mapping', 'slam_localization', 'rtab_mapping', 'rtab_localization', 'amcl')

@dataclass(frozen=True)
class Operation:
    id: str
    title: str
    kind: str = 'service'
    movement_capable: bool = False
    requires: tuple = ()
    requires_any: tuple = ()
    conflicts: tuple = ()
    description: str = ''
    timeout_seconds: int = 30
    parameters: list = field(default_factory=list)

def param(name, type='string', required=False, **kw):
    return dict(name=name, type=type, required=required, **kw)

NAME = param('name', required=True)
MAP = param('map', required=True)
POSE = [param('x', 'number', default=0, min=-10000, max=10000), param('y', 'number', default=0, min=-10000, max=10000), param('heading', 'number', default=0, min=-math.pi, max=math.pi)]
_ops = [
    Operation('motors', 'Motor driver', movement_capable=True, description='Owns the RoboClaw serial ports; accepts velocity commands.'),
    Operation('sensors', 'Sensors and odometry', requires=('motors',), description='Keep rover still for first five seconds.', parameters=[param('camera','boolean',default=True),param('lidar','boolean',default=True)]),
    Operation('joystick', 'Joystick teleoperation', movement_capable=True, requires=('motors',), conflicts=('navigation',), description='Physical gamepad shoulder button remains the deadman.'),
]
for id, title in [('slam_mapping','SLAM mapping'),('slam_localization','SLAM localization'),('rtab_mapping','RTAB mapping'),('rtab_localization','RTAB localization'),('amcl','AMCL localization')]:
    parameters = []
    if id == 'slam_localization': parameters = [MAP] + POSE
    if id == 'rtab_mapping': parameters = [NAME]
    if id == 'rtab_localization': parameters = [param('database',required=True)]
    if id == 'amcl': parameters = [MAP, param('family',default='slam_toolbox',options=['slam_toolbox','rtabmap']), param('recovery','boolean',default=False)]
    _ops.append(Operation(id,title,movement_capable=id=='amcl',requires=('sensors',),conflicts=tuple(x for x in MAPPERS if x!=id),parameters=parameters,description='Exclusive map transform authority. AMCL recovery is disabled unless explicitly enabled.'))
_ops += [
    Operation('navigation','Navigation servers',movement_capable=True,requires=('sensors',),requires_any=MAPPERS,conflicts=('joystick',),description='Starts autonomy servers. No phone goal submission is implemented.'),
    Operation('save_slam','Save SLAM map and graph','action',requires=('slam_mapping',),parameters=[NAME],timeout_seconds=120),
    Operation('save_rtab_map','Save RTAB occupancy map','action',requires_any=('rtab_mapping','rtab_localization'),parameters=[NAME],timeout_seconds=90),
    Operation('export_rtab_cloud','Export stopped RTAB cloud','action',conflicts=('rtab_mapping','rtab_localization'),parameters=[NAME,param('database',required=True)],timeout_seconds=300),
    Operation('recovery_abort','Abort AMCL recovery','action',requires=('amcl',)),
    Operation('recovery_relocalize','Force AMCL recovery','action',movement_capable=True,requires=('amcl','navigation'),conflicts=('joystick',)),
    Operation('global_localization','Scatter AMCL particles','action',movement_capable=True,requires=('amcl',),description='May trigger enabled autonomous recovery.'),
    Operation('diagnostics_nodes','List ROS nodes','action'),
    Operation('diagnostics_topics','List ROS topics and types','action'),
    *[Operation('diagnostics_'+name,'Inspect '+name+' lifecycle','action') for name in ('controller','planner','navigator')],
]
# Serialize all map writers, including launches which open an RTAB database.
_writers = ('save_slam', 'save_rtab_map', 'export_rtab_cloud')
_ops = [replace(op, conflicts=tuple(dict.fromkeys(op.conflicts + (
    tuple(x for x in _writers if x != op.id) if op.id in _writers else
    ('export_rtab_cloud',) if op.id in ('rtab_mapping', 'rtab_localization') else ()
)))) for op in _ops]
OPERATIONS = {op.id:op for op in _ops}

def validate_parameters(operation_id, parameters):
    if operation_id not in OPERATIONS: raise ValueError('Unknown operation')
    if not isinstance(parameters,dict): raise ValueError('parameters must be an object')
    definitions = OPERATIONS[operation_id].parameters
    if set(parameters)-{d['name'] for d in definitions}: raise ValueError('Unknown parameter')
    result = {}
    for d in definitions:
        key = d['name']
        if key not in parameters and 'default' not in d:
            if d['required']: raise ValueError('Missing parameter: '+key)
            continue
        v = parameters.get(key,d.get('default'))
        if d['type']=='boolean':
            if type(v) is not bool: raise ValueError(key+' must be boolean')
        elif d['type']=='number':
            if type(v) not in (int,float) or not d['min']<=v<=d['max'] or not math.isfinite(v): raise ValueError(key+' out of range')
        else:
            if not isinstance(v,str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,79}',v) or '..' in v: raise ValueError(key+' must be a simple filename/name (80 characters max)')
        if 'options' in d and v not in d['options']: raise ValueError('Invalid '+key)
        result[key] = v
    return result

def build_argv(operation_id, parameters, workspace, simulated=False):
    p = validate_parameters(operation_id,parameters)
    workspace = Path(workspace).resolve()
    package = workspace/'src'/'mapping_localization_pkg'
    def path(family, name, existing=True):
        root = package/family/'maps'
        target = root/name
        # Reject directory symlinks and output symlinks, including broken links.
        if not simulated:
            if root.resolve()!=root or target.is_symlink() or target.resolve().parent != root: raise ValueError('Map path escapes configured source directory')
            if existing and not target.is_file(): raise ValueError('Map file does not exist: '+name)
            if not existing and target.exists(): raise ValueError('Refusing to overwrite: '+name)
        return str(target)
    def launch(pkg, file, *args): return ['ros2','launch',pkg,file,*args]
    if operation_id=='motors': return launch('low_level_control_pkg','roboclaw_driver.launch.py')
    if operation_id=='sensors': return launch('sensor_fusion','bringup.launch.py',*[f'{k}:={str(p[k]).lower()}' for k in ('camera','lidar')],'rviz:=false')
    if operation_id=='joystick': return launch('low_level_control_pkg','joy_teleop.launch.py')
    if operation_id.startswith('slam_'):
        args = ['rviz:=false']
        if operation_id=='slam_localization':
            prefix=p['map']
            if prefix.endswith('.posegraph'): prefix=prefix[:-10]
            graph=path('slam_toolbox',prefix+'.posegraph'); path('slam_toolbox',prefix+'.data')
            args += ['localization:=true','map_file_name:='+graph[:-10],f"map_start_pose:=[{p['x']},{p['y']},{p['heading']}]"]
        return launch('mapping_localization_pkg','slam_toolbox.launch.py',*args)
    if operation_id.startswith('rtab_'):
        local=operation_id=='rtab_localization'
        filename=p['database'] if local else 'rtabmap_'+p['name']+'.db'
        if not filename.endswith('.db'): raise ValueError('database must end in .db')
        db=path('rtabmap',filename,existing=local)
        return launch('mapping_localization_pkg','rtabmap.launch.py','rviz:=false','rtabmap_viz:=false','publish_tf_map:=true','map_topic:=/map','localization:='+str(local).lower(),'database_path:='+db)
    if operation_id=='amcl':
        if not p['map'].endswith('.yaml'): raise ValueError('map must end in .yaml')
        return launch('mapping_localization_pkg','amcl_localization.launch.py','rviz:=false','map:='+path(p['family'],p['map']),'recovery:='+str(p['recovery']).lower())
    if operation_id=='navigation': return launch('navigation_pkg','navigation.launch.py','rviz:=false')
    if operation_id=='save_slam':
        for ext in ('pgm','yaml','posegraph','data'): path('slam_toolbox','slam_toolbox_'+p['name']+'.'+ext,False)
        return ['bash',str(package/'slam_toolbox/scripts/save_slam.sh'),p['name']]
    if operation_id=='save_rtab_map':
        for ext in ('pgm','yaml'): path('rtabmap','rtabmap_'+p['name']+'.'+ext,False)
        return ['ros2','run','nav2_map_server','map_saver_cli','-f',str(package/'rtabmap/maps'/('rtabmap_'+p['name'])),'-t','/map','--occ','0.65','--free','0.196','--fmt','pgm']
    if operation_id=='export_rtab_cloud':
        if not p['database'].endswith('.db'): raise ValueError('database must end in .db')
        path('rtabmap','rtabmap_'+p['name']+'_cloud.ply',False)
        return ['bash',str(package/'rtabmap/scripts/save_rtabmap.sh'),p['name'],'--cloud-only','--db',path('rtabmap',p['database'])]
    services={'recovery_abort':('/amcl_recovery/abort','Trigger'),'recovery_relocalize':('/amcl_recovery/relocalize','Trigger'),'global_localization':('/reinitialize_global_localization','Empty')}
    if operation_id in services:
        endpoint,type_name=services[operation_id]
        return ['ros2','service','call',endpoint,'std_srvs/srv/'+type_name,'{}']
    if operation_id=='diagnostics_nodes': return ['ros2','node','list']
    if operation_id=='diagnostics_topics': return ['ros2','topic','list','-t']
    node={'controller':'controller_server','planner':'planner_server','navigator':'bt_navigator'}[operation_id.removeprefix('diagnostics_')]
    return ['ros2','lifecycle','get','/'+node]
