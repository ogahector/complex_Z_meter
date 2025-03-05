import numpy as np


def phasor_1sig(sig, f0, fs):
    I = Q = 0
    for i in range(len(sig)):
        I += sig[i] * np.cos(2*np.pi*i *f0/fs )
        Q += sig[i] * np.sin(2*np.pi*i *f0/fs)

    I /= len(sig)
    Q /= len(sig)

    mag = np.sqrt(I*I + Q*Q)
    phaserad = np.atan2(Q, I)

    return mag * (np.cos(phaserad) + 1j*np.sin(phaserad))

def phasor_2sig(sig, ref, f0, fs):
    return phasor_1sig(sig, f0, fs) / phasor_1sig(ref, f0, fs)

def phasors2Zx(v1, v2, Rref):
    return Rref * v2 / (v1 - v2)