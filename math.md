# Motor Control Equations Reference Guide

## Basic Motor Parameters

### Electrical Parameters
```
V = IR + L(dI/dt) + e
where:
- V: Applied voltage
- I: Phase current
- R: Phase resistance
- L: Phase inductance
- e: Back EMF
```

### Mechanical Parameters
```
Mechanical Angular velocity: ωm = 2π × rpm/60
Electrical Angular velocity: ωe = ωm × P/2
where:
- P: Number of poles
- rpm: Rotations per minute
```

## Back EMF

### BLDC Back EMF
```
Back-EMF constant: Ke = V/(ωm × √3)
Back-EMF: e = Ke × ωm
E = √3 × e
```

### PMSM Back EMF
```
ed = -ωe × Lq × iq
eq = ωe × (Ld × id + λm)
where:
- Ld, Lq: d-q axis inductances
- λm: Permanent magnet flux linkage
```

## Torque Equations

### BLDC Torque
```
Te = (ea × ia + eb × ib + ec × ic)/ωm
where:
- ea, eb, ec: Phase back EMFs
- ia, ib, ic: Phase currents
- Basically Power = Torque * Angular velocity
```

### PMSM Torque
```
Te = (3/2) × (P/2) × [λm × iq + (Ld - Lq) × id × iq]
Tr = (3/2) × (P/2) × (Ld - Lq) × id × iq
Tm = (3/2) × (P/2) × λm × iq
```

### Load Torque Balance
```
Te = TL + J(dωm/dt) + Bωm
where:
- TL: Load torque
- J: Moment of inertia
- B: Viscous friction coefficient
```

## BLDC Motor Equations

### Three-Phase Currents
```
ia = Im × cos(θe)
ib = Im × cos(θe - 2π/3)
ic = Im × cos(θe + 2π/3)
where:
- Im: Maximum phase current
- θe: Electrical angle
```

### Commutation Sequence
Six-step commutation angles: 0°, 60°, 120°, 180°, 240°, 300°

## PMSM Motor Equations

### Clarke Transformation
```
[iα] = [1   -1/2   -1/2  ] [ia]
[iβ] = [0   √3/2   -√3/2] [ib]
                          [ic]
```

### Park Transformation
```
[id] = [ cos(θe)   sin(θe)] [iα]
[iq] = [-sin(θe)   cos(θe)] [iβ]
```

### Voltage Equations
```
Vd = Rd × id - ωe × Lq × iq + Ld × (did/dt)
Vq = Rq × iq + ωe × Ld × id + Lq × (diq/dt) + ωe × λm
```

## Speed and Position

### Position Calculation
```
θe = ∫ωe dt
θm = θe × (2/P)
where:
- θe: Electrical angle
- θm: Mechanical angle
- P: Motor pole pair count
```

### Speed Estimation
```
ωm = e/(Ke × √3)
ωm = dθm/dt
```

## Power and Efficiency

### Power Calculations
```
Pin = Va × ia + Vb × ib + Vc × ic
Pout = Te × ωm
η = Pout/Pin × 100%
```

### Losses
```
Copper Losses: Pcu = R × (ia² + ib² + ic²)
Iron Losses: Pfe = kh × B² × f + ke × B² × f²
where:
- kh: Hysteresis loss coefficient
- ke: Eddy current loss coefficient
- B: Magnetic flux density
- f: Electrical frequency
- Copper losses are basically P = I²R
```

## Control System Parameters

### Current Controller
```
PI Controller: u(t) = Kp × e(t) + Ki × ∫e(t)dt
where:
- Kp: Proportional gain
- Ki: Integral gain
- e(t): Error signal
```

### Speed Controller
```
Speed PI: ωref = Kp_ω × (ωref - ωm) + Ki_ω × ∫(ωref - ωm)dt
```

### Position Controller
```
Position PID: θref = Kp_θ × eθ + Ki_θ × ∫eθdt + Kd_θ × (deθ/dt)
where:
- eθ: Position error
```

## Additional Notes

1. The equations above assume ideal conditions and may need modification for:
   - Temperature effects
   - Magnetic saturation
   - Cogging torque
   - Dead time effects
   - Switching losses

2. For practical implementation:
   - Include current limiting
   - Consider voltage headroom
   - Account for sensor offsets
   - Implement protection features
   - Add anti-windup for controllers