"""Actual loopback sockets with simulated operations only; never ROS/hardware."""
import http.client
import json
import threading
import unittest

from mobile_gateway.operations import OperationManager
from mobile_gateway.processes import SimulatedProcessBackend
from mobile_gateway.server import TelemetryServer
from mobile_gateway.state import TelemetryState

TOKEN = 'operation-socket-test-' + 'x'*32
CLIENT = 'client-http-0001'


class OperationHTTPTests(unittest.TestCase):
    def setUp(self):
        self.manager = OperationManager(SimulatedProcessBackend(), '/fixture', enabled=True, simulated=True)
        self.server = TelemetryServer(('127.0.0.1',0), TelemetryState(source='fixture'), TOKEN, operations=self.manager, socket_timeout=.5)
        self.worker = threading.Thread(target=self.server.serve_forever,kwargs={'poll_interval':.01},daemon=True)
        self.worker.start()

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.worker.join(2)
        self.manager.shutdown()

    def request(self, method, path, body=None, *, auth='Bearer '+TOKEN, raw=None, headers=None):
        conn = http.client.HTTPConnection(*self.server.server_address, timeout=2)
        try:
            payload = raw if raw is not None else json.dumps(body).encode() if body is not None else b''
            supplied = {'Content-Type':'application/json'}
            if auth is not None: supplied['Authorization']=auth
            if headers: supplied.update(headers)
            conn.request(method,path,body=payload,headers=supplied)
            response=conn.getresponse()
            data=response.read()
            return response.status,dict(response.getheaders()),json.loads(data) if data else None
        finally: conn.close()

    def post(self,path,body): return self.request('POST',path,body)

    def heartbeat(self):
        status,_,body=self.post('/v1/control/heartbeat',{'client_id':CLIENT})
        self.assertEqual(status,200)
        self.assertEqual(body['lease']['client_id'],CLIENT)

    def test_catalog_authenticated_fixture_and_enabled(self):
        status,headers,body=self.request('GET','/v1/operations')
        self.assertEqual(status,200)
        self.assertEqual(headers['Cache-Control'],'no-store')
        self.assertEqual(body['source'],'fixture')
        self.assertTrue(body['commands_enabled'])
        self.assertTrue(next(op for op in body['operations'] if op['id']=='motors')['requires_confirmation'])

    def test_heartbeat_confirmation_start_idempotency_job_stop(self):
        self.heartbeat()
        request={'client_id':CLIENT,'request_id':'request-http-0001','parameters':{}}
        status,_,body=self.post('/v1/operations/motors/start',request)
        self.assertEqual((status,body['error']),(403,'confirmation_required'))
        request['confirm']=True
        status,_,first=self.post('/v1/operations/motors/start',request)
        self.assertEqual(status,202)
        self.assertEqual(first['job']['state'],'running')
        self.assertTrue(first['job']['simulated'])
        self.assertEqual(self.post('/v1/operations/motors/start',request)[2],first)
        id=first['job']['id']
        self.assertEqual(self.request('GET','/v1/jobs/'+id)[2],first)
        status,_,stopped=self.post('/v1/jobs/'+id+'/stop',{'client_id':CLIENT})
        self.assertEqual(status,202)
        self.assertEqual(stopped['job']['state'],'stopped')
        self.assertEqual(self.post('/v1/operations/stop-all',{'client_id':CLIENT})[0],202)

    def test_auth_required_for_read_and_mutation(self):
        for auth in (None,'Bearer incorrect',TOKEN):
            for method,path in [('GET','/v1/operations'),('POST','/v1/control/heartbeat'),('POST','/v1/operations/motors/start')]:
                with self.subTest(auth=auth,method=method,path=path):
                    status,_,body=self.request(method,path,{},auth=auth)
                    self.assertEqual(status,401)
                    self.assertNotIn(TOKEN,json.dumps(body))
        self.assertFalse(self.manager.jobs)
        self.assertIsNone(self.manager.owner)

    def test_unknown_job_and_bad_method(self):
        self.assertEqual(self.request('GET','/v1/jobs/missing')[0],404)
        for method,path,allow in [('POST','/v1/operations','GET'),('GET','/v1/control/heartbeat','POST'),('DELETE','/v1/jobs/missing','GET'),('CUSTOM','/v1/operations/motors/start','POST')]:
            status,headers,_=self.request(method,path,{})
            self.assertEqual(status,405)
            self.assertEqual(headers['Allow'],allow)

    def test_invalid_json_duplicate_nonfinite_and_nonobject(self):
        for payload in (b'{',b'{"client_id":"first","client_id":"second"}',b'{"client_id":NaN}',b'{"client_id":Infinity}',b'[]',b'null',b'"text"',b'{"nested":{"x":1,"x":2}}',b'\xff'):
            with self.subTest(payload=payload):
                status,_,body=self.request('POST','/v1/control/heartbeat',raw=payload)
                self.assertEqual(status,400)
                self.assertEqual(body['error'],'invalid_request')
        self.assertIsNone(self.manager.owner)

    def test_oversize_chunked_and_wrong_content_type(self):
        cases=[(b'x'*16385,{}),(b'{}',{'Transfer-Encoding':'chunked'}),(b'{}',{'Content-Type':'text/plain'}),(b'',{})]
        for raw,headers in cases:
            with self.subTest(headers=headers,length=len(raw)):
                self.assertEqual(self.request('POST','/v1/control/heartbeat',raw=raw,headers=headers)[0],400)

    def test_duplicate_content_length_and_authorization(self):
        for duplicate,expected in [('Content-Length',400),('Authorization',401)]:
            conn=http.client.HTTPConnection(*self.server.server_address,timeout=2)
            try:
                conn.putrequest('POST','/v1/control/heartbeat')
                conn.putheader('Authorization','Bearer '+TOKEN)
                conn.putheader('Content-Length','2')
                conn.putheader('Content-Type','application/json')
                conn.putheader(duplicate,'2' if duplicate=='Content-Length' else 'Bearer '+TOKEN)
                conn.endheaders(b'{}')
                response=conn.getresponse()
                self.assertEqual(response.status,expected)
                response.read()
            finally: conn.close()

    def test_disabled_manager_rejects_mutation(self):
        self.manager.enabled=False
        self.assertFalse(self.request('GET','/v1/operations')[2]['commands_enabled'])
        status,_,body=self.post('/v1/control/heartbeat',{'client_id':CLIENT})
        self.assertEqual((status,body['error']),(403,'commands_disabled'))


if __name__=='__main__': unittest.main()
