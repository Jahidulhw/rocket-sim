"""Flight-computer sensor models: barometric altimeter and vertical accelerometer.

Both are sampled at `rate_hz` (100 Hz) by the SIL runner. Every random draw
comes from the numpy Generator passed in, so a run is reproducible from its
seed. Each sample draws exactly one baro and one accel normal deviate, in that
order, whether or not the frame is later dropped by fault injection; that
keeps the noise sequence of a faulted run aligned with its nominal twin.

Barometric altimeter
--------------------
    baro = quantize(z_true + bias + N(0, sigma^2), q)

* The FC sees altitude, not pressure. Real altimeters convert pressure to
  altitude onboard; we model the error in the altitude domain directly. At
  the ~300 m apogees here, the ISA pressure-altitude curve is linear to
  better than 1 %, so this costs nothing.
* z_true is height above the pad (the simulator's z).

Parameters (defaults are assumptions for an MS5611-class MEMS barometer on a
small hobby rocket):

    baro_noise_sigma_m  0.5 m   datasheet RMS noise is ~0.1 m at the highest
                                oversampling; in flight, airflow over the
                                static vent ports and pressure fluctuations
                                in the airframe add more. 0.5 m is a
                                deliberately pessimistic in-flight figure.
    baro_bias_m         0.5 m   constant offset (reference pressure error /
                                thermal drift). A constant bias does not move
                                apogee timing (only altitude *changes* matter)
                                but it shifts reported altitudes; it is here
                                so the FC has to cope with it.
    baro_quant_m        0.1 m   MS5611 resolution at OSR 4096 is 0.012 mbar,
                                about 0.1 m of altitude near sea level.

Accelerometer (vertical specific force)
---------------------------------------
    accel = clip(f_z + bias + N(0, sigma^2), -range, +range)
    f_z   = a_z + g            (specific force, positive UP)

SIGN CONVENTION. An accelerometer measures specific force, i.e. acceleration
minus gravitational acceleration: f = a - g_vec, with g_vec = (0, 0, -g). So:

    on the pad, at rest            a_z = 0       ->  f_z = +g   (+9.81)
    boost                          a_z > 0       ->  f_z > +g
    coast (drag only, climbing)    a_z < -g      ->  f_z < 0    (drag pushes down)
    free fall in vacuum            a_z = -g      ->  f_z = 0
    under chute at terminal speed  a_z = 0       ->  f_z = +g

So an FC wanting kinematic vertical acceleration computes a_z = f_z - g.

* We model the *vertical* component directly, as if the IMU's attitude were
  known. The rocket is a point mass here (no attitude), and its flights are
  within a few degrees of vertical, where axial and vertical differ by under
  1 % (cos 5 deg = 0.996).

Parameters (assumptions for a Bosch BMI088-class +-24 g accelerometer):

    accel_noise_sigma_mps2  0.5 m/s^2   datasheet noise density gives ~0.01
                                        m/s^2 RMS at 100 Hz; motor and airframe
                                        vibration dominate in flight, so we use
                                        a pessimistic 0.5 m/s^2.
    accel_bias_mps2         0.2 m/s^2   BMI088 zero-g offset spec is +-20 mg.
    accel_range_mps2        235.4 m/s^2 +-24 g full scale. A C6 peaks near 22 g
                                        on this rocket, so a hot motor (+3 %
                                        impulse dispersion) can briefly clip.
                                        Real sensors saturate, so ours does.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .physics import G0


@dataclass
class SensorConfig:
    rate_hz: float = 100.0
    baro_noise_sigma_m: float = 0.5
    baro_bias_m: float = 0.5
    baro_quant_m: float = 0.1
    accel_noise_sigma_mps2: float = 0.5
    accel_bias_mps2: float = 0.2
    accel_range_mps2: float = 24.0 * G0

    @property
    def period_s(self) -> float:
        return 1.0 / self.rate_hz

    def validate(self) -> None:
        if self.rate_hz <= 0:
            raise ValueError("sensor rate must be positive")
        if min(self.baro_noise_sigma_m, self.accel_noise_sigma_mps2, self.baro_quant_m) < 0:
            raise ValueError("noise sigmas and quantization must be >= 0")
        if self.accel_range_mps2 <= 0:
            raise ValueError("accelerometer range must be positive")

    @classmethod
    def ideal(cls, rate_hz: float = 100.0) -> "SensorConfig":
        """Perfect sensors: no noise, bias, quantization or saturation."""
        return cls(rate_hz=rate_hz, baro_noise_sigma_m=0.0, baro_bias_m=0.0, baro_quant_m=0.0,
                   accel_noise_sigma_mps2=0.0, accel_bias_mps2=0.0, accel_range_mps2=np.inf)


def specific_force_z(accel_z_mps2: float) -> float:
    """Vertical specific force an ideal accelerometer reads, +up (see module doc)."""
    return accel_z_mps2 + G0


class SensorSuite:
    """Barometer + accelerometer sharing one seeded random stream."""

    def __init__(self, cfg: SensorConfig, rng: np.random.Generator):
        cfg.validate()
        self.cfg = cfg
        self.rng = rng

    def baro(self, z_true_m: float, noise: float) -> float:
        c = self.cfg
        v = z_true_m + c.baro_bias_m + c.baro_noise_sigma_m * noise
        if c.baro_quant_m > 0.0:
            v = c.baro_quant_m * round(v / c.baro_quant_m)
        return float(v)

    def accel(self, accel_z_mps2: float, noise: float) -> float:
        c = self.cfg
        v = specific_force_z(accel_z_mps2) + c.accel_bias_mps2 + c.accel_noise_sigma_mps2 * noise
        return float(min(max(v, -c.accel_range_mps2), c.accel_range_mps2))

    def sample(self, z_true_m: float, accel_z_mps2: float) -> tuple[float, float]:
        """One tick: (baro_alt_m, accel_mps2). Always consumes exactly two draws."""
        n_baro, n_accel = self.rng.standard_normal(2)
        return self.baro(z_true_m, n_baro), self.accel(accel_z_mps2, n_accel)
