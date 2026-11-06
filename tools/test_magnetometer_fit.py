import unittest
import numpy as np
from fit_magnetometer import fit,header

class MagnetometerCalibrationTests(unittest.TestCase):
    def test_measured_ellipsoid_recovered(self):
        rng=np.random.default_rng(142)
        directions=rng.normal(size=(1500,3));directions/=np.linalg.norm(directions,axis=1)[:,None]
        m=np.array([[1.15,.03,-.02],[.03,.87,.04],[-.02,.04,1.07]])
        bias=np.array([12.,-8.,5.]);field=47.
        raw=np.einsum('nj,ij->ni',directions*field,np.linalg.inv(m))+bias+rng.normal(scale=.05,size=(1500,3))
        report=fit(raw,field)
        np.testing.assert_allclose(report['hard_iron_uT'],bias,atol=.02)
        np.testing.assert_allclose(report['soft_iron_matrix'],m,atol=.002)
        self.assertLess(report['held_out_radial_rms_uT'],.08)
        self.assertIn('namespace magnetic_calibration',header(report))
        self.assertIn('soft_iron[3][3]',header(report))
    def test_coplanar_input_rejected(self):
        t=np.linspace(0,2*np.pi,600)
        with self.assertRaises(ValueError):fit(np.column_stack((40*np.cos(t),40*np.sin(t),np.ones(600))),40.)
    def test_nonfinite_rejected(self):
        with self.assertRaises(ValueError):fit(np.full((600,3),np.nan),40.)

if __name__=='__main__':unittest.main()
