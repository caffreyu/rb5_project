class pid:

    def __init__(
        self, 
        p = 0, 
        i = 0, 
        d = 0, 
        i_max = 0, 
        output_max = 0,
    ):
        self._kp, self._ki, self._kd, self._i_max, self._output_max =\
            p, i, d, i_max, output_max
        self.last_error = 0

    def calculate_pid(self, error):
        I_out = 0
        P_out = error * self._kp
        I_out += error * self._ki
        D_out = (error - self.last_error) * self._kd
        if I_out > self._i_max:
            I_out = self._i_max
        elif I_out < -self._i_max:
            I_out = -self._i_max
        output = P_out + I_out + D_out
        if output > self._output_max:
            output = self._output_max
        elif output < -self._output_max:
            output = -self._output_max
        self.last_error = error
        return output