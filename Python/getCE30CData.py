# coding:utf-8
from undistortion import _init_fisheye_map, _remap
import socket
import signal
import numpy as np
from dataExchange import _recvhex, _sendmsg, _recvdata

WIDTH = 320
HEIGHT = 24
TOTALDATA = WIDTH * HEIGHT * 2
cmdStart = "getDistanceAndAmplitudeSorted"
cmdStop = "join"
cmdDisconnect = "disconnect"

# camera intrinsic matrix
cameraMatrix = np.zeros((3, 3))
cameraMatrix[0, 0] = 149.905
cameraMatrix[0, 1] = 0.0
cameraMatrix[0, 2] = 159.5
cameraMatrix[1, 1] = 150.24
cameraMatrix[1, 2] = 11.5
cameraMatrix[2, 2] = 1.0
# distortion coefficients
distCoeffs = np.array([-0.059868055, -0.001303471, 0.010260736, -0.006102915])
# init undistortion map
mapX, mapY = _init_fisheye_map(cameraMatrix, distCoeffs, 24, 660)

recorder = open("data.txt", 'a')

# set TCP/IP connnection
tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
tcp.setsockopt(socket.SOL_SOCKET, socket.TCP_NODELAY, 1)
tcp.connect(('192.168.1.80', 50660))
tcp.settimeout(2)

_sendmsg(tcp, cmdStart)
frame_n = 0

# ctrl+C to stop LiDAR and quit
def quit(signum, frame):
    _sendmsg(tcp, cmdStop)
    _sendmsg(tcp, cmdDisconnect)
signal.signal(signal.SIGINT, quit)
signal.signal(signal.SIGTERM, quit)


while True:
    try:
        distData = _recvdata(tcp, HEIGHT, WIDTH)
        ampData = _recvdata(tcp, HEIGHT, WIDTH)
        nearPt = _recvhex(tcp, 3)
        if distData is None or ampData is None or nearPt is None:
            break

        frame_n += 1
        recorder.write("Frame " + str(frame_n) + ':\n')
        recorder.write("Distance:\n")
        np.savetxt(recorder, distData, fmt='%d', newline='\n', delimiter=' ')
        recorder.write("Amplitude:\n")
        np.savetxt(recorder, ampData, fmt='%d', newline='\n', delimiter=' ')
        # undistortion
        undistortData = _remap(distData, mapX, mapY, 24, 660)
        recorder.write("Undistorted distance:\n")
        np.savetxt(recorder, undistortData, fmt='%d', newline='\n', delimiter=' ')
    except Exception:
        break

_sendmsg(tcp, cmdStop)
_sendmsg(tcp, cmdDisconnect)
