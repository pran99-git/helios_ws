"""No ROS or hardware: catalog filesystem checks and simulated ownership."""
import json
import tempfile
import time
import unittest
from pathlib import Path

from mobile_gateway.catalog import OPERATIONS, build_argv, validate_parameters
from mobile_gateway.operations import OperationError, OperationManager
from mobile_gateway.processes import SimulatedProcessBackend


class CatalogTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.workspace = Path(self.tmp.name).resolve()
        self.package = self.workspace / 'src/mapping_localization_pkg'
        for family in ('slam_toolbox', 'rtabmap'):
            (self.package / family / 'maps').mkdir(parents=True)

    def build(self, operation, parameters):
        return build_argv(operation, parameters, self.workspace)

    def test_reject_unknown_operation_parameters_and_bad_types(self):
        for op, params in [('unknown', {}), ('motors', {'shell': 'ls'}), ('motors', []),
                           ('sensors', {'camera': 1}), ('slam_localization', {'map': 'test', 'x': True}),
                           ('slam_localization', {'map': 'test', 'x': float('nan')}),
                           ('slam_localization', {'map': 'test', 'x': float('inf')}),
                           ('slam_localization', {'map': 'test', 'x': 10**1000}),
                           ('slam_localization', {'map': 'test', 'heading': 4}),
                           ('amcl', {'map': 'test.yaml', 'family': 'elsewhere'}),
                           ('save_slam', {})]:
            with self.subTest(op=op, params=params), self.assertRaises(ValueError):
                validate_parameters(op, params)

    def test_names_reject_traversal_shell_and_unbounded_text(self):
        for name in ('../test', '/tmp/test', '-option', 'a/../b', 'x;id', '$(id)', 'a\\b', 'a\n', 'a'*81, 'a..b'):
            with self.subTest(name=name), self.assertRaises(ValueError):
                self.build('save_slam', {'name': name})

    def test_slam_requires_matching_graph_and_data(self):
        maps = self.package / 'slam_toolbox/maps'
        (maps / 'lab.posegraph').touch()
        with self.assertRaises(ValueError): self.build('slam_localization', {'map': 'lab'})
        (maps / 'lab.data').touch()
        argv = self.build('slam_localization', {'map': 'lab.posegraph', 'x': 1.5})
        self.assertIn('map_file_name:='+str(maps/'lab'), argv)
        self.assertIn('map_start_pose:=[1.5,0,0]', argv)

    def test_no_overwrite_of_every_slam_format(self):
        for extension in ('pgm', 'yaml', 'posegraph', 'data'):
            file = self.package / ('slam_toolbox/maps/slam_toolbox_lab.'+extension)
            file.touch()
            with self.subTest(extension=extension), self.assertRaises(ValueError):
                self.build('save_slam', {'name': 'lab'})
            file.unlink()
        argv = self.build('save_slam', {'name': 'lab'})
        self.assertEqual(argv, ['bash', str(self.package/'slam_toolbox/scripts/save_slam.sh'), 'lab'])

    def test_rtab_database_and_exports_refuse_overwrite(self):
        maps = self.package/'rtabmap/maps'
        (maps/'rtabmap_lab.db').touch()
        with self.assertRaises(ValueError): self.build('rtab_mapping', {'name':'lab'})
        argv = self.build('rtab_localization', {'database':'rtabmap_lab.db'})
        self.assertIn('map_topic:=/map',argv)
        self.assertIn('publish_tf_map:=true',argv)
        for suffix in ('.pgm','.yaml'):
            file = maps/('rtabmap_lab'+suffix)
            file.touch()
            with self.assertRaises(ValueError): self.build('save_rtab_map',{'name':'lab'})
            file.unlink()
        (maps/'rtabmap_lab_cloud.ply').touch()
        with self.assertRaises(ValueError): self.build('export_rtab_cloud',{'name':'lab','database':'rtabmap_lab.db'})

    def test_symlink_files_and_directory_refused(self):
        maps = self.package/'rtabmap/maps'
        (maps/'broken.db').symlink_to(maps/'missing')
        with self.assertRaises(ValueError): self.build('rtab_localization',{'database':'broken.db'})
        (maps/'rtabmap_lab.db').symlink_to(maps/'missing')
        with self.assertRaises(ValueError): self.build('rtab_mapping',{'name':'lab'})
        maps.rename(maps.with_name('real_maps'))
        maps.symlink_to(maps.with_name('real_maps'), target_is_directory=True)
        with self.assertRaises(ValueError): self.build('rtab_mapping',{'name':'new'})

    def test_simulation_skips_files_but_not_validation(self):
        argv = build_argv('amcl',{'map':'missing.yaml'},self.workspace,simulated=True)
        self.assertIn('recovery:=false',argv)
        with self.assertRaises(ValueError): build_argv('amcl',{'map':'../missing.yaml'},self.workspace,simulated=True)
        with self.assertRaises(ValueError): build_argv('amcl',{'map':'wrong.db'},self.workspace,simulated=True)

    def test_save_map_matches_rtab_topic_and_serialization(self):
        argv = self.build('save_rtab_map',{'name':'lab'})
        self.assertEqual(argv[argv.index('-t')+1],'/map')
        for id in ('save_slam','save_rtab_map'):
            self.assertIn('export_rtab_cloud',OPERATIONS[id].conflicts)
            self.assertIn(id,OPERATIONS['export_rtab_cloud'].conflicts)
        self.assertIn('export_rtab_cloud',OPERATIONS['rtab_mapping'].conflicts)


class TrackingBackend(SimulatedProcessBackend):
    def __init__(self):
        super().__init__()
        self.stops=[]
    def stop(self, id):
        self.stops.append(id)
        return super().stop(id)


class ManagerTests(unittest.TestCase):
    def setUp(self):
        self.now = [10.]
        self.backend = TrackingBackend()
        self.manager = OperationManager(self.backend,'/fixture',enabled=True,simulated=True,clock=lambda:self.now[0])
        self.addCleanup(self.manager.shutdown)
        self.client='client-0001'
        self.counter=0
        self.manager.heartbeat(self.client)

    def start(self, operation, **parameters):
        self.counter+=1
        return self.manager.start(operation,{'client_id':self.client,'request_id':f'request-{self.counter:04}','confirm':True,'parameters':parameters})['job']

    def assert_code(self,code,fn):
        with self.assertRaises(OperationError) as caught: fn()
        self.assertEqual(caught.exception.code,code)

    def wait(self,fn):
        end=time.monotonic()+2
        while not fn() and time.monotonic()<end: time.sleep(.01)
        self.assertTrue(fn())

    def test_commands_disabled(self):
        self.manager.enabled=False
        self.assert_code('commands_disabled',lambda:self.manager.heartbeat(self.client))
        self.assert_code('commands_disabled',lambda:self.start('motors'))
        self.assertFalse(self.manager.catalog()['commands_enabled'])

    def test_exclusive_lease_and_renewal(self):
        self.assert_code('lease_owned',lambda:self.manager.heartbeat('client-0002'))
        self.now[0]+=5
        self.manager.heartbeat(self.client)
        self.assertEqual(self.manager.catalog()['lease']['remaining_seconds'],10)
        self.assert_code('lease_required',lambda:self.manager.start('motors',{'client_id':'client-0002'}))

    def test_confirmation_exact_boolean(self):
        for confirm in (False,1,'true',None):
            self.assert_code('confirmation_required',lambda:self.manager.start('motors',{'client_id':self.client,'request_id':'request-0001','confirm':confirm}))
        self.assertEqual(self.start('motors')['state'],'running')

    def test_dependencies_mapper_and_joystick_conflicts(self):
        self.assert_code('dependencies',lambda:self.start('sensors'))
        self.start('motors'); self.start('sensors'); self.start('slam_mapping')
        self.assert_code('conflict',lambda:self.start('rtab_mapping',name='lab'))
        joystick=self.start('joystick')
        self.assert_code('conflict',lambda:self.start('navigation'))
        self.manager.stop(joystick['id'],self.client)
        self.start('navigation')
        self.assert_code('conflict',lambda:self.start('joystick'))

    def test_idempotent_retry_and_changed_intent_rejected(self):
        body={'client_id':self.client,'request_id':'request-idem','confirm':True}
        first=self.manager.start('motors',body)
        self.assertEqual(first,self.manager.start('motors',body))
        self.assertEqual(len(self.manager.jobs),1)
        self.assert_code('request_id_reused',lambda:self.manager.start('sensors',body))
        self.assert_code('already_running',lambda:self.start('motors'))

    def test_stop_dependents_and_reverse_stop_all_order(self):
        motors=self.start('motors'); sensors=self.start('sensors'); mapper=self.start('slam_mapping'); nav=self.start('navigation')
        self.assert_code('dependents',lambda:self.manager.stop(motors['id'],self.client))
        self.manager.stop_all(self.client)
        self.wait(lambda:not self.manager.stopping_all)
        self.assertEqual(self.backend.stops,[nav['id'],mapper['id'],sensors['id'],motors['id']])
        self.assertTrue(all(j['state']=='stopped' for j in self.backend.statuses().values()))

    def test_expired_lease_stops_owned_jobs_and_allows_new_owner(self):
        motors=self.start('motors')
        self.now[0]+=11
        self.assert_code('lease_required',lambda:self.start('sensors'))
        self.wait(lambda:self.backend.status(motors['id'])['state']=='stopped')
        self.wait(lambda:not self.manager.stopping_all)
        self.manager.heartbeat('client-0002')
        self.assertEqual(self.manager.owner,'client-0002')

    def test_prerequisite_failure_stops_dependents(self):
        motors=self.start('motors'); sensors=self.start('sensors')
        self.backend.stop(motors['id'])
        self.wait(lambda:self.backend.status(sensors['id'])['state']=='stopped')

    def test_stop_failed_prerequisite_stops_dependents(self):
        motors=self.start('motors'); sensors=self.start('sensors')
        with self.backend._lock:
            self.backend._entries[motors['id']]['state']='stop_failed'
        self.wait(lambda:self.backend.status(sensors['id'])['state']=='stopped')

    def test_unrelated_stuck_action_does_not_prevent_motor_stop(self):
        motors=self.start('motors'); action=self.start('diagnostics_nodes')
        with self.backend._lock:
            self.backend._entries[action['id']]['state']='stop_failed'
        self.manager.stop_all(self.client)
        self.wait(lambda:self.backend.status(motors['id'])['state']=='stopped')
        self.assertTrue(self.manager.stopping_all)
        self.assertEqual(self.backend.status(action['id'])['state'],'stop_failed')

    def test_one_shot_job_finishes_and_is_marked_simulated(self):
        job=self.start('diagnostics_nodes')
        self.assertTrue(job['simulated'])
        self.wait(lambda:self.manager.job(job['id'])['job']['state']=='succeeded')

    def test_full_history_catalog_stays_below_client_payload_limit(self):
        for index in range(128):
            job=self.start('diagnostics_nodes')
            with self.backend._lock:
                self.backend._entries[job['id']].update(
                    state='failed', output='\x00'*1024, error='\x01'*4096)
        catalog=self.manager.catalog()
        encoded=json.dumps(catalog,allow_nan=False,separators=(',',':')).encode()
        self.assertLess(len(encoded),256*1024)
        self.assertEqual(len(catalog['jobs']),128)
        self.assertTrue(all(len(j['output'])<=128 and len(j['error'])<=128 for j in catalog['jobs']))
        self.assert_code('history_full',lambda:self.start('diagnostics_topics'))
        stop_payload=self.manager.stop_all(self.client)
        self.assertLess(len(json.dumps(stop_payload).encode()),256*1024)

    def test_lease_rechecked_after_slow_external_guard(self):
        def guard(active, operation):
            self.now[0]+=11
        self.manager.external_guard=guard
        self.assert_code('lease_required',lambda:self.start('motors'))
        self.assertFalse(self.backend.statuses())
        self.assertFalse(self.manager.jobs)

    def test_invalid_identifiers_and_request_fields(self):
        self.assert_code('invalid_id',lambda:self.manager.heartbeat('../bad'))
        self.assert_code('invalid_parameters',lambda:self.manager.start('motors',{'client_id':self.client,'request_id':'request-test','confirm':True,'shell':'echo unsafe'}))
        self.assert_code('operation_not_found',lambda:self.start('arbitrary'))
        self.assert_code('invalid_parameters',lambda:self.start('motors',bad='parameter'))


class OutputVerificationTests(unittest.TestCase):
    def test_cli_zero_requires_all_nonempty_save_outputs(self):
        cases=[('save_slam','slam_toolbox',('.pgm','.yaml','.posegraph','.data'),'slam_mapping'),
               ('save_rtab_map','rtabmap',('.pgm','.yaml'),'rtab_mapping'),
               ('export_rtab_cloud','rtabmap',('_cloud.ply',),None)]
        for operation,family,extensions,mapper in cases:
            with self.subTest(operation=operation), tempfile.TemporaryDirectory() as directory:
                workspace=Path(directory).resolve()
                for folder in ('slam_toolbox','rtabmap'):
                    (workspace/'src/mapping_localization_pkg'/folder/'maps').mkdir(parents=True)
                backend=SimulatedProcessBackend()  # Never creates subprocesses.
                manager=OperationManager(backend,workspace,enabled=True,simulated=False)
                count=0
                def start(op,parameters=None):
                    nonlocal count
                    count+=1
                    return manager.start(op,{'client_id':'file-client','request_id':f'file-request-{count:04}',
                                             'confirm':True,'parameters':parameters or {}})['job']
                try:
                    manager.heartbeat('file-client')
                    if mapper:
                        start('motors'); start('sensors')
                        start(mapper,{'name':'session'} if mapper=='rtab_mapping' else {})
                    params={'name':'saved'}
                    if operation=='export_rtab_cloud':
                        (workspace/'src/mapping_localization_pkg/rtabmap/maps/session.db').write_bytes(b'database')
                        params['database']='session.db'
                    job=start(operation,params)
                    backend._finish(job['id'])  # Simulate CLI exit zero without producing files.
                    self.assertEqual(backend.status(job['id'])['state'],'succeeded')
                    self.assertEqual(manager.job(job['id'])['job']['state'],'failed')
                    prefix=workspace/'src/mapping_localization_pkg'/family/'maps'/(family+'_saved')
                    for extension in extensions:
                        Path(str(prefix)+extension).touch()
                    self.assertEqual(manager.job(job['id'])['job']['state'],'failed')
                    for extension in extensions[:-1]:
                        Path(str(prefix)+extension).write_bytes(b'output')
                    self.assertEqual(manager.job(job['id'])['job']['state'],'failed')
                    Path(str(prefix)+extensions[-1]).write_bytes(b'output')
                    self.assertEqual(manager.job(job['id'])['job']['state'],'succeeded')
                    self.assertFalse(manager.job(job['id'])['job']['simulated'])
                finally:
                    manager.shutdown()


if __name__=='__main__': unittest.main()
