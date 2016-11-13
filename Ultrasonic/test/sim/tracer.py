from math import *

import pyopencl as cl
import pyopencl.array as array
import numpy as np
import cv2

from model.util import *


class TracerStrategy:
    def __init__(self, image):
        self.image = image

    def trace(self, pos, dirs):
        raise NotImplementedError()

class PythonTracer(TracerStrategy):
    def __init__(self, image):
        TracerStrategy.__init__(self, image)

    def trace(self, pos, dirs):
        lens = []
        for d in dirs:
            lens.append(self._traceDir(pos, d))
        return np.array(lens, dtype=np.float32)

    def _traceDir(self, pos, dr):
        dir_ = np.array([dr[0] + 0.0001, dr[1] + 0.0001], np.float32)
        pos_ = np.array(pos, np.float32)
        
        xy = np.array(pos_, np.float32)
        x1y1 = np.array(pos_, np.float32)

        dd = np.array([
            np.sqrt(1 + (dir_[1] * dir_[1]) / (dir_[0] * dir_[0])),
            np.sqrt(1 + (dir_[0] * dir_[0]) / (dir_[1] * dir_[1]))
        ], np.float32)

        step = np.array([1,1], np.int32)
        sd = np.array(dd, np.float32)

        if dir_[0] < 0:
            step[0] = -1
            sd[0] = 0

        if dir_[1] < 0:
            step[1] = -1
            sd[1] = 0

        while xy[0] > 0 and xy[1] > 0 and xy[0] < self.image.shape[0]-1 and xy[1] < self.image.shape[1]-1 and self.image[xy[1], xy[0]] != 0x00:
            if sd[0] < sd[1]:
                sd[0] += dd[0]
                xy[0] += step[0]
            else:
                sd[1] += dd[1]
                xy[1] += step[1]

        return np.linalg.norm(xy - x1y1)

class OpenCLTracer(TracerStrategy):
    def __init__(self, image):
        TracerStrategy.__init__(self, image)

        self.context = cl.create_some_context()
        self.queue = cl.CommandQueue(self.context)
        f = open('sim/opencl/tracer.cl')
        prog_text = f.read()
        f.close()

        image = image.flatten()

        mf = cl.mem_flags
        self.image_buf = cl.Buffer(self.context, mf.READ_ONLY | mf.COPY_HOST_PTR, hostbuf=image)
        self.program = cl.Program(self.context, prog_text).build()

    def trace(self, pos, dirs):
        mf = cl.mem_flags
        src = np.array(dirs, dtype=array.vec.float2)
        dst = np.zeros(shape=(len(dirs)), dtype=np.float32)

        src_buf = cl.Buffer(self.context, mf.READ_ONLY | mf.COPY_HOST_PTR, hostbuf=src)
        dst_buf = cl.Buffer(self.context, mf.WRITE_ONLY, dst.nbytes)
        self.program.trace(self.queue, dst.shape, None, self.image_buf,
                                         np.int32(self.image.shape[0]), np.int32(self.image.shape[1]), np.int32(pos[0]), np.int32(pos[1]),
                                         src_buf, dst_buf)

        cl.enqueue_copy(self.queue, dst, dst_buf)

        return dst

def test():
    import time

    image = cv2.imread('sim/data/world_small.png')
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    print('init..')
    tr_py = PythonTracer(image)
    tr_cl = OpenCLTracer(image)
    dirs = [
        (cos(0 * pi/ 180.0), sin(0 * pi / 180.0)),
        (cos(5 * pi/ 180.0), sin(5 * pi / 180.0)),
        (cos(10 * pi/ 180.0), sin(10 * pi / 180.0)),
        (cos(15 * pi/ 180.0), sin(15 * pi / 180.0)),
        (cos(20 * pi/ 180.0), sin(20 * pi / 180.0)),
        (cos(30 * pi/ 180.0), sin(30 * pi / 180.0)),
        (cos(40 * pi/ 180.0), sin(40 * pi / 180.0)),
        (cos(50 * pi/ 180.0), sin(50 * pi / 180.0)),
        (cos(60 * pi/ 180.0), sin(60 * pi / 180.0)),
        (cos(70 * pi/ 180.0), sin(70 * pi / 180.0)),
        (cos(80 * pi/ 180.0), sin(80 * pi / 180.0)),
        (cos(90 * pi/ 180.0), sin(90 * pi / 180.0)),
        (cos(100 * pi/ 180.0), sin(100 * pi / 180.0)),
        (cos(180 * pi/ 180.0), sin(180 * pi / 180.0)),
        (0.0, 1.0)
    ]
    print('Python tracer: ')
    ticks = time.clock()
    print(tr_py.trace([356,256], dirs))
    print('Time: {0}'.format(time.clock() - ticks))

    print('OpenCL tracer: ')
    ticks = time.clock()
    print(tr_cl.trace([356,256], dirs))
    print('Time: {0}'.format(time.clock() - ticks))

if __name__ == "__main__":
    test()
