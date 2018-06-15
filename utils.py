import numpy as np

class Utils(object):
    @classmethod
    def linspace(cls, lb: float, ub: float, samples: int) -> np.ndarray:
        sp = np.linspace(lb, ub, num=samples)
        m = np.linspace(0, 0, num=0)
        x, _ = np.meshgrid(sp, m, sparse=True)
        return x
