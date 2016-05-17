from threading import Thread
import websocket
import string
import json


# SockJS Client class
class SockJSClient(Thread):
    _prefix = ""
    _host = ""
    _port = 80

    def __init__(self, prefix, host="localhost", port=80):
        self._prefix = prefix
        self._host = host
        self._port = port
        self.status = {}
        Thread.__init__(self)
        
    def on_open(self, ws):
        print("### opened ###") 
        
    def on_message(self, ws, message):
        #print(message)
        message = json.loads(message)
        for obj in message:
            self.status.update(message)

    def on_error(self, ws, error):
        print(error)
    
    def on_close(self, ws):
        print("### closed ###")      
        
    def run(self):
        self.ws.run_forever()
        print("Thread terminating...")

    def connect(self):
        #websocket.enableTrace(True)
        host = "ws://"+self._host+":"+str(self._port)+self._prefix+"/websocket"
        self.ws = websocket.WebSocketApp(host,
                                on_message = self.on_message,
                                on_error = self.on_error,
                                on_close = self.on_close)
        self.ws.on_open = self.on_open
        self.start()

    def disconnect(self):
        self.ws.close()
        self.running = False 
        
    def send(self, message):
        self.ws.send(message)

    def get_status(self, argument):
        if argument in self.status:
            return self.status[argument]
        return 0

    def remove_status(self, argument):
        if argument in self.status:
            del self.status[argument]
