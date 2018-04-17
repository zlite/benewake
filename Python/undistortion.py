from __future__ import division
import math
import numpy as np

def _bilinear_interpolation(_targetX, _targetY, _p11, _p12, _p21, _p22):
    alpha_x = _targetX % 1
    alpha_y = _targetY % 1
    interpl = int(_p11) * int(_p12) * int(_p21) * int(_p22)

    if interpl != 0:
        return (1 - alpha_x) * (1 - alpha_y) * _p11 + \
               alpha_x * (1 - alpha_y) * _p12 + \
               (1 - alpha_x) * alpha_y * _p21 + \
               alpha_x * alpha_y * _p22
    else:
        if alpha_x < 0.5 and alpha_y < 0.5:
            return _p11
        elif alpha_x >= 0.5 > alpha_y:
            return _p12
        elif alpha_x < 0.5 <= alpha_y:
            return _p21
        else:
            return _p22

def _remap(_src, _mapX, _mapY, _h, _w):
    dst = np.empty((24, 660))

    for i in range(_h):
        for j in range(_w):
            targetX = _mapX[i, j]
            targetY = _mapY[i, j]
            x = int(targetX)
            y = int(targetY)
            if 0 <= y <= 22:
                p11 = _src[y, x]
                p12 = _src[y, x + 1]
                p21 = _src[y + 1, x]
                p22 = _src[y + 1, x + 1]
                dst[i, j] = _bilinear_interpolation(targetX, targetY, p11, p12, p21, p22)
            else:
                dst[i, j] = 0
    return dst

def _inverse_matrix(_A, _n):
    C = np.empty((_n, _n))
    _B = np.empty((_n, _n))

    for i in range(_n):
        for j in range(_n):
            C[i, j] = _A[i, j]
            _B[i, j] = 1.0 if i == j else 0.0

    for i in range(_n):
        temp = C[i, i]
        for j in range(_n):
            C[i, j] = C[i, j] / temp
            _B[i, j] = _B[i, j] / temp

        for j in range(_n):
            if j != i:
                it = C[j, i]
                for k in range(_n):
                    C[j, k] = C[j, k] - C[i, k] * it
                    _B[j, k] = _B[j, k] - _B[i, k] * it
    return _B

def _init_fisheye_map(_cameraMatrix, _coeffs, _height, _width):
    _mapx = np.zeros((_height, _width), dtype=np.double)
    _mapy = np.zeros((_height, _width), dtype=np.double)

    newCameraMatrix = np.copy(_cameraMatrix)
    newCameraMatrix[0, 2] = (_width - 1) * 0.5
    newCameraMatrix[1, 2] = (_height - 1) * 0.5
    ir = _inverse_matrix(newCameraMatrix, 3)
    u0 = _cameraMatrix[0, 2]
    v0 = _cameraMatrix[1, 2]
    fx = _cameraMatrix[0, 0]
    fy = _cameraMatrix[1, 1]

    for i in range(_height):
        _x = i * ir[0, 1] + ir[0, 2]
        _y = i * ir[1, 1] + ir[1, 2]
        _w = i * ir[2, 1] + ir[2, 2]
        for j in range(_width):
            x = _x / _w
            y = _y / _w
            r = np.sqrt(x * x + y * y)
            theta = math.atan(r)
            theta_d = theta * (1 + _coeffs[0] * theta ** 2 + \
                               _coeffs[1] * theta ** 4 + \
                               _coeffs[2] * theta ** 6 + \
                               _coeffs[3] * theta ** 8)
            scale = 1.0 if r == 0 else theta_d / r
            _mapx[i, j] = fx * x * scale + u0
            _mapy[i, j] = fy * y * scale + v0

            _x += ir[0, 0]
            _y += ir[1, 0]
            _w += ir[2, 0]
    return [_mapx, _mapy]