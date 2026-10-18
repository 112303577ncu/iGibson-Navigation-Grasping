"""Small Python 2/3 TCP actuator; the resident service alone owns Serial.

No background heartbeat: callers must revalidate sensors before each command.
An optional health callback runs before every nonzero output.
"""
from __future__ import division
import json
import math
import socket


class ResidentMotion(object):
    resident_transport = True

    def __init__(self, host='127.0.0.1', port=7000, guard=None):
        self.host, self.port, self.guard = host, port, guard
        self.sock = None

    def connect(self):
        if self.sock is None:
            self.sock = socket.create_connection((self.host, self.port), timeout=1)
            self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        return self

    def set_car_motion(self, vx, vy, wz):
        values = [float(v) for v in (vx, vy, wz)]
        if any(math.isnan(v) or math.isinf(v) for v in values):
            self.stop()
            raise ValueError('nonfinite velocity')
        vx, vy, wz = values
        if vy != 0 or abs(vx) > .15 or abs(wz) > 1:
            self.stop()
            raise ValueError('resident velocity outside supported limits')
        if not vx and not wz:
            self.stop()
            return
        try:
            if self.guard is None:
                raise RuntimeError('live output requires a sensor/owner guard')
            self.guard()
            self.connect()
            self.sock.sendall((json.dumps({'action':'velocity','vx':vx,'wz':wz})+'\n').encode('ascii'))
        except Exception:
            self.stop()
            raise

    def stop(self):
        # Never open a new connection just to stop: that could queue behind the
        # active controller. Its own connection/watchdog must perform its stop.
        if self.sock is not None:
            try:
                self.sock.sendall(b'{"action":"stop"}\n')
            except socket.error:
                pass

    def close(self):
        self.stop()
        if self.sock is not None:
            self.sock.close()
            self.sock = None

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
