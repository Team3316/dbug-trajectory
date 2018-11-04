from time import time

class FpsCounter:
    def __init__(self):
        self.time_elapsed = 0
        self.frame_count = 0
        self.start_time = time()

        self.sum = 0
        self.num_of_counts = 0

    def nexttick(self):
        self.time_elapsed = time() - self.start_time

        if 1 <= self.time_elapsed <= 1.1:
            self.num_of_counts += 1
            self.sum += self.frame_count
            print('%d fps, %3f avg. fps' % (self.frame_count, self.sum / self.num_of_counts))

            self.time_elapsed = 0
            self.frame_count = 0
            self.start_time = time()
        else:
            self.frame_count += 1
