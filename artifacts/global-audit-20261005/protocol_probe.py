"""Offline framing and queue overload probes using current production classes."""
import ast
import binascii
from collections import deque
import json
import pathlib
import threading

ROOT = pathlib.Path(__file__).resolve().parents[2]
source = (ROOT/'third_party/sl_linka/sdk/python/sl_link/frame/frame.py').read_text(encoding='utf-8')
namespace = {'crc16_ccitt': lambda data: binascii.crc_hqx(data, 0xffff)}
exec(compile(source.replace('from .crc16 import crc16_ccitt', ''), 'production_frame.py', 'exec'), namespace)
Frame, Parser = namespace['SlFrame'], namespace['SlFrameParser']
frame = Frame(version=99, flags=2, seq=1, ack_seq=0, src_id=1, dst_id=16, comp_id=1, msg_id=256, payload=b'x')
accepted = Parser().parse(frame.pack())
good = Frame(version=1, flags=0, seq=2, ack_seq=0, src_id=1, dst_id=16, comp_id=1, msg_id=256, payload=b'x').pack()
broken = bytearray(good[:15])
broken[13:15] = b'\xff\xff'
parser = Parser()
following = parser.parse(bytes(broken) + good)
tree = ast.parse((ROOT/'catkin_ws/src/grinder_scheduler/src/grinder_scheduler/sl_linka_adapter.py').read_text(encoding='utf-8'))
cls = next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name == '_BoundedLatestWorker')
ns = dict(deque=deque, threading=threading, rospy=None)
exec(compile(ast.Module(body=[cls],type_ignores=[]),'production_queue.py','exec'),ns)
worker = ns['_BoundedLatestWorker'](2,'audit_offline')
entered, release, finished = threading.Event(), threading.Event(), threading.Event()
executed = []
def barrier():
    entered.set()
    release.wait(3)
worker.submit(barrier)
assert entered.wait(1)
worker.submit(executed.append, 'TaskConfig')
worker.submit(executed.append, 'START')
dropped = worker.submit(lambda: (executed.append('settings'), finished.set()))
release.set()
assert finished.wait(1)
worker.shutdown(wait=True)
print(json.dumps(dict(unsupported_version_and_encrypted_flag_accepted=bool(accepted), valid_frames_after_incomplete_65535_header=len(following), parser_retained_payload_bytes=len(parser.payload_buffer), dropped_request=dropped[1][0], executed_requests=executed),indent=2))
