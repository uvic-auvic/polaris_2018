#!/usr/bin/python
import SimpleHTTPServer
import SocketServer
import os

if __name__ == "__main__":
    PORT = 12345
    Handler = SimpleHTTPServer.SimpleHTTPRequestHandler
    httpd = SocketServer.TCPServer(("", PORT), Handler)
    os.chdir(os.path.dirname(__file__))
    try:
        httpd.serve_forever()
    finally:
        httpd.server_close()
