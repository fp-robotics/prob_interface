from threading import Thread
import time
import httplib
# import httplib
import random
import string
import socket
import json


def random_str(length):
    letters = string.ascii_lowercase + string.digits
    return ''.join(random.choice(letters) for c in range(length))


# SockJS Client class


class SockJSClient(Thread):
    TRANSPORT = "xhr_streaming"

    _wait_thread = 0
    _prefix = ""
    _host = ""
    _port = 80

    def __init__(self, prefix, host="localhost", port=80):
        self._prefix = prefix
        self._host = host
        self._port = port
        self.status = {}
        self.MessageCounter = 0
        self.running = True
        self._conn_id = ''
        self._r1 = 0
        Thread.__init__(self)

    def connect(self):

        self.get_socket_info()
        self.start()

    def disconnect(self):
        self.running = False

    def run(self):

        while self.running:
            #print(">>> ")
            conn = httplib.HTTPConnection(self._host, self._port)
            self._r1 = str(random.randint(0, 1000))
            self._conn_id = random_str(8)
            url = '/'.join([self._prefix, self._r1, self._conn_id, 'xhr_streaming'])
            #print("Connecting to URL: ", url)
            conn.request('POST', url)
            response = conn.getresponse()
            #print("connected: ", response.status)
            sock = socket.fromfd(response.fileno(), socket.AF_INET, socket.SOCK_STREAM)
            data = 1

            while data:
                data = sock.recv(1)
                if data == b'o':
                    self.MessageCounter = 0
                    #print("connected to API")
                if data == b'0':
                    if self.MessageCounter:
                        #print("Disconnected, Messages received:", self.MessageCounter)
                        self.MessageCounter = 0
                        break
                if data == b'c':
                    pass
                    # print("webinterface connection lost")
                if data == b'h':
                    pass
                if data in (b'm', b'a'):
                    buf = b''
                    msg = b''
                    while buf != b'\r':
                        buf = sock.recv(1)
                        msg += buf
                    if len(msg) > 3:
                        msg = json.loads(msg.decode())
                        self.MessageCounter += 1
                        # print("Message: ", self.MessageCounter, msg)
                        for obj in msg:
                            msg = json.loads(obj)
                            self.status.update(msg)
                        #print("Status: ", self.status)

                if not self.running:
                    return 0

            #time.sleep(1)
            #print("reconnecting")

    def get_status(self, argument):
        if argument in self.status:
            return self.status[argument]
        return 0

    def remove_status(self, argument):
        if argument in self.status:
            del self.status[argument]

    def get_socket_info(self):
        conn = 0
        try:
            conn = httplib.HTTPConnection(self._host, self._port)
            print "Getting info from ", self._host, ":", self._port
            conn.request('GET', '/socket/info')
            response = conn.getresponse()
            #print("INFO", response.status, response.reason, response.read())
        finally:
            if not conn:
                conn.close()

    def send(self, message):
        conn = httplib.HTTPConnection(self._host, self._port)
        url = '/'.join([self._prefix, self._r1, self._conn_id, 'xhr_send'])
        # print("Send to: ", url)
        conn.request('POST', url, "[" + message + "]", {'Content-Type': 'text/plain'})
        #print "Sent: ", conn.getresponse().status, conn.getresponse().reason
        return conn.getresponse().status in (200, 204)
