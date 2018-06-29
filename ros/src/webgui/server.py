#!/usr/bin/python
import SimpleHTTPServer
import SocketServer
import socket
import signal
import sys
import os

PORT = 12345

class MyTCPServer(SocketServer.TCPServer):
    def server_bind(self):
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(self.server_address)

    def signal_handler(self, signal, frame):
        httpd.server_close()
        sys.exit(0)

if __name__ == '__main__':
    Handler = SimpleHTTPServer.SimpleHTTPRequestHandler
    httpd = MyTCPServer(("0.0.0.0", PORT), Handler)
    os.chdir(os.path.dirname(__file__))
    signal.signal(signal.SIGINT, httpd.signal_handler)
    httpd.serve_forever()
        
        
