
class Encoder:
    def __init__(self, resolution, start_count=0):
        self._resolution = resolution
        self._current_count = start_count
        # self._coefficient = 1

    def count_to_rpm(self, count, dt):
        self._previous_count = self._current_count
        self._current_count = count
        dcount = self._current_count - self._previous_count  # calculate delta count
        rotations = dcount / self._resolution
        if dt > 0:
            rps = rotations/dt
            return rps*60

        # if this is the first count update, rpm should be zero
        return 0
