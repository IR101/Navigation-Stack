#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from http.server import HTTPServer, BaseHTTPRequestHandler
import threading
import time
import os

HOST = '172.17.69.104'  # Use '0.0.0.0' to allow connections from other devices
PORT = 8000
REFRESH = 0.1  # 10 FPS

HTML_PAGE = b"""<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <title>Live MJPEG Stream</title>
</head>
<body>
  <h2>Live Screen Stream</h2>
  <img id="mjpeg" src="/stream.mjpg" width="1280" height="720" onerror="this.src='/stream.mjpg';" />
  <script>
    window.onload = function() {
      document.getElementById("mjpeg").src = "/stream.mjpg";
    };
  </script>
</body>
</html>
"""

class MJPEGHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            self.wfile.write(HTML_PAGE)

        elif self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=--jpgboundary')
            self.end_headers()

            try:
                while True:
                    os.system("scrot /tmp/screen.jpg")  # Capture screen
                    with open("/tmp/screen.jpg", "rb") as f:
                        jpg = f.read()
                    self.wfile.write(b"--jpgboundary\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write("Content-Length: {}\r\n\r\n".format(len(jpg)).encode())
                    self.wfile.write(jpg)
                    self.wfile.write(b"\r\n")
                    time.sleep(REFRESH)
            except Exception as e:
                print("Stream ended: ", str(e))
        else:
            self.send_error(404)

def run_server():
    # Preload first image so stream starts immediately
    os.system("scrot /tmp/screen.jpg")

    server = HTTPServer((HOST, PORT), MJPEGHandler)
    print("MJPEG stream available at http://{}:{}/".format(HOST if HOST != '0.0.0.0' else 'localhost', PORT))
    server.serve_forever()

if __name__ == '__main__':
    run_server()
