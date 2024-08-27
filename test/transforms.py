from ramp_apriltag_detector.server import ScTf, compose

import numpy as np
from scipy.spatial.transform import Rotation as R
import copy
def test_ident():
    '''test inversion'''
    rot = R.from_euler('xyz', np.random.uniform(-90., 90., size=(3,)), degrees=True).as_quat()
    lin = np.random.randn(3)

    tf = ScTf(lin, rot)
    
    assert is_identity(tf.inv().apply(tf))
    assert is_identity(tf.apply(tf.inv()))

    assert is_identity(compose(tf, tf.inv()))
    assert is_identity(compose(tf.inv(), tf))


def test_square():
    '''test square transform'''
    rot = R.from_euler('xyz', [0, 0, 90] ,degrees=True).as_quat()
    lin = np.array([1.0,0,0.])
    T_1 = ScTf(lin, rot)

    transforms = [copy.deepcopy(T_1) for k  in range(4)]
    
    result = compose(*transforms)

    assert is_identity(result)


def is_identity(query):
    '''assert that '''

    quat = query.rot
    quat[-1] = np.abs(quat[-1])

    identity_quat = np.array([0., 0., 0., 1.])
    
    return np.allclose(query.lin, np.zeros((3,))) and np.allclose(quat, identity_quat)

if __name__ == '__main__':
    test_ident()
    test_square()