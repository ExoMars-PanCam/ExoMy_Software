import socket
from picamera2 import Picamera2, Preview
import time
from libcamera import Transform

picam2 = Picamera2()
camera_config = picam2.create_preview_configuration(
    transform=Transform(hflip=True, vflip=True, transpose=False)
)
picam2.configure(camera_config)
picam2.start_preview(Preview.NULL)
picam2.start()
time.sleep(2)
picam2.capture_file("output/test.jpg")


HOST = "0.0.0.0"  # Bind to all interfaces so Docker can connect
PORT = 8015  # Port to listen on (avoiding all conflicts)

print(f"Starting echo server on {HOST}:{PORT}")
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((HOST, PORT))
    s.listen(1)
    print(f"Echo server listening on port {PORT}")
    
    while True:  # Keep accepting new connections
        conn, addr = s.accept()
        with conn:
            print(f"Connected by {addr}")
            while True:
                data = conn.recv(1024)
                if not data:
                    break
                print(f"Request: {data.decode('utf-8')}")
                picam2.capture_file("output/" +data.decode('utf-8') + ".jpg")
                # Echo the data back to the client
                conn.sendall(data)
        print(f"Client {addr} disconnected")