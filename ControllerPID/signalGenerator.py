import numpy as np

class signalGenerator:
    '''
        This class inherits the Signal class. It is used to organize
        1 or more signals of different types: square_wave,
        sawtooth_wave, triangle_wave, random_wave.
    '''
    def __init__(self, amplitude=1, frequency=1, t_offset = False , y_offset=0):
        '''
            amplitude - signal amplitude.  Standard deviation for random.
            frequency - signal frequency
            y_offset  - signal y-offset
        '''
        self.amplitude = amplitude
        self.frequency = frequency
        self.t_offset = -t_offset
        self.y_offset = y_offset
        if t_offset:
            self.t_offset = 1/(self.frequency*4)

    def square(self, t):
        if (t+self.t_offset) % (1.0/self.frequency) <= 0.5/self.frequency:
            out = self.amplitude + self.y_offset
        else:
            out = - self.amplitude + self.y_offset
        return [out]
        #  returns a list of length 1

    def sawtooth(self, t):
        tmp = (t+self.t_offset) % (0.5/self.frequency)
        out = 4*self.amplitude*self.frequency*tmp - self.amplitude + self.y_offset
        return [out]
        #  returns a list of length 1

    def random(self, t):
        out = np.sqrt(self.amplitude)*np.random.rand() + self.y_offset
        return [out]
        #  returns a list of length 1

    def sin(self, t):
        out = self.amplitude*np.sin(2*np.pi*self.frequency*t+self.t_offset) + self.y_offset
        return [out]
        #  returns a list of length 1

    def cos(self, t):
        out = self.amplitude*np.cos(2*np.pi*self.frequency*t+self.t_offset) + self.y_offset
        return [out]
        #  returns a list of length 1

    def nsin(self, t):
        out = -self.amplitude*np.sin(2*np.pi*self.frequency*t+self.t_offset) + self.y_offset
        return [out]
        #  returns a list of length 1

    def ncos(self, t):
        out = -self.amplitude*np.cos(2*np.pi*self.frequency*t+self.t_offset) + self.y_offset
        return [out]
        #  returns a list of length 1
