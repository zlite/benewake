import socket
import numpy as np

def _recvdata(tcp, _row, _colum):
    string = ''
    bytes_read = 0
    bytes_n = _row * _colum * 2
    while bytes_read < bytes_n:
        try:
            chunk = tcp.recv(bytes_n - bytes_read)
        except socket.timeout:
            print("Receiver timeout")
            return None
        bytes_read += len(chunk)
        string += chunk
    array = np.fromstring(string, dtype=np.uint16)
    array.shape = (_row, _colum)
    return array

def _sendmsg(tcp, _msg):
    msg_len = len(_msg)
    total_sent = 0
    _msg += "\0" * (50 - msg_len)
    while total_sent < 50:
        chunk_len = tcp.send(_msg[total_sent:])
        total_sent += chunk_len

def _recvhex(tcp, _len):
    string = ''
    bytes_read = 0
    while bytes_read < _len:
        try:
            chunk = tcp.recv(_len - bytes_read)
        except socket.timeout:
            print("Receiver timeout")
            return None
        bytes_read += len(chunk)
        string += chunk
    array = np.fromstring(string, dtype=np.ubyte)
    return array