#!/usr/bin/python
import SimpleHTTPServer
import SocketServer
import signal
import sys
import os

PORT = 12345
Handler = SimpleHTTPServer.SimpleHTTPRequestHandler
httpd = SocketServer.TCPServer(("0.0.0.0", PORT), Handler)

def signal_handler(signal, frame):
    httpd.server_close()
    sys.exit(0)
    
if __name__ == '__main__':
    os.chdir(os.path.dirname(__file__))
    signal.signal(signal.SIGINT, signal_handler)
    httpd.serve_forever()
        
        
