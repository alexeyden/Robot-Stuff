import math
import numpy as np
cimport numpy as np
import ctypes

from libcpp.string cimport string
from libcpp.vector cimport vector
from libc.stdint cimport uint8_t

np.import_array()

# Интерфейс для C++ класса обнаружения
cdef extern from "Detector.hpp" namespace "FFLD":
    cdef cppclass Detection:
        float score # оценка положения
        int x #
        int y # положение в пирамиде признаков
        int z #
        int left()
        int right()
        int top()
        int bottom()

# Интерфейс для C++ класса детектора
cdef extern from "Detector.hpp" namespace "FFLD":
    cdef cppclass FFLDDetector:
        int interval
        int padding
        double threshold
        double overlap
        
        FFLDDetector(const string&)
        const vector[Detection] run(int width, int height,
                                    int depth, const uint8_t* data)

# Python обертка для класса детектора
cdef class PyFFLDDetector:
    # Оборачиваемый класс C++
    cdef FFLDDetector* thisptr;
    
    def __cinit__(self, s):
        '''
        Конструктор. Создает детектор с моделью,
        загруженной из JSON-строки s
        '''
        self.thisptr = new FFLDDetector(s.encode('utf-8'));
    
    def __dealloc__(self):
        '''
        Деструктор
        '''
        del self.thisptr
    
    # Установка параметров детектора
    @property
    def interval(self):
        return self.thisptr.interval
    
    @interval.setter
    def interval(self, v : ctypes.double.int):
        self.thisptr.interval = v
        
    @property
    def padding(self):
        return self.thisptr.padding
    
    @padding.setter
    def padding(self, v : ctypes.int):
        self.thisptr.padding = v
        
    @property
    def threshold(self):
        return self.thisptr.threshold
    
    @threshold.setter
    def threshold(self, v : ctypes.double):
        self.thisptr.threshold = v
        
    @property
    def overlap(self):
        return self.thisptr.overlap
    
    @overlap.setter
    def overlap(self, v : ctypes.double):
        self.thisptr.overlap = v
    
    def detect(self, np.ndarray[np.uint8_t, ndim=2, mode="c"] image):
        """
        Метод выполняющий распознавание на входном изображении image
        """
        
        cdef np.npy_intp width
        cdef np.npy_intp height
        cdef np.npy_intp depth
        cdef vector[Detection] detections
        
        # Вызов C++ класса для детектирования
        detections = self.thisptr.run(<np.npy_intp> image.shape[1],
                                      <np.npy_intp> image.shape[0], 1,
#                                      <np.npy_intp> image.shape[2],
                                      <const uint8_t*> image.data)
        
        # Построение списка обнаружений в виде списка кортежей
        return [(d.score, d.left(), d.top(), d.right(), d.bottom()) for d in detections]
