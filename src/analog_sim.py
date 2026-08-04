# analog_sim.py -- patchable analog-computer simulator
#
# micropython code, assume the use of a STM32 MCU. However, micropython
# implementation has enough hardware abstraction that almost any board will work.
#
# For a setpoint (potmeter) an analog input (PA0, PA1, PA2) is used.
# Two DACs are used for an X and Y channel (PA4, PA5).
#


import math
from pyb import DAC, ADC, Pin

# ---------------------------------------------------------------- blocks
class Block:
    out = 0.0
    def step(self, dt): pass

class Const(Block):
    def __init__(self, v): self.out = v

class Pot(Block):
    def __init__(self, pin, lo=0.0, hi=1.0):
        self.adc, self.lo, self.hi = ADC(Pin(pin)), lo, hi
    def step(self, dt):
        self.out = self.lo + (self.hi - self.lo) * (self.adc.read() / 4095)

class Sum(Block):
    def __init__(self, *pairs): self.pairs = pairs
    def step(self, dt):
        s = 0.0
        for b, g in self.pairs: s += g * b.out
        self.out = s

class Mul(Block):
    def __init__(self, a, b, k=1.0): self.a, self.b, self.k = a, b, k
    def step(self, dt): self.out = self.k * self.a.out * self.b.out

class Sqrt(Block):
    """|v| = sqrt(a^2 + b^2). Real machines used diode function gens."""
    def __init__(self, a, b): self.a, self.b = a, b
    def step(self, dt):
        self.out = math.sqrt(self.a.out * self.a.out + self.b.out * self.b.out)

class Integrator(Block):
    def __init__(self, src, k=1.0, ic=0.0, sat=1.0):
        self.src, self.k, self.out, self.sat = src, k, ic, sat
    def step(self, dt):
        self.out += self.k * self.src.out * dt
        if self.out >  self.sat: self.out =  self.sat
        if self.out < -self.sat: self.out = -self.sat
    def preset(self, v): self.out = v          # mode-control reset

class Comparator(Block):
    """out = 1 if a > b else 0. Fires on_cross(direction) once per crossing."""
    def __init__(self, a, b, on_cross=None):
        self.a, self.b, self.on_cross = a, b, on_cross
        self.prev = 0
    def step(self, dt):
        now = 1 if self.a.out > self.b.out else 0
        if now != self.prev and self.on_cross:
            self.on_cross(1 if now == 1 else -1)   # +1 rising, -1 falling
        self.prev = now
        self.out = now

class Osc(Block):
    def __init__(self, freq, shape='sine'):
        self.freq, self.shape, self.ph = freq, shape, 0.0
    def step(self, dt):
        self.ph = (self.ph + self.freq * dt) % 1.0
        p = self.ph
        if   self.shape == 'sine':  self.out = math.sin(2 * math.pi * p)
        elif self.shape == 'saw':   self.out = 2 * p - 1
        elif self.shape == 'tri':   self.out = 4 * abs(p - 0.5) - 1
        else:                       self.out = 1.0 if p < 0.5 else -1.0

# ---------------------------------------------------------------- engine + I/O
class Engine:
    def __init__(self, blocks, dt): self.blocks, self.dt = blocks, dt
    def step(self):
        for b in self.blocks: b.step(self.dt)

dac_x = DAC(1, bits=12)
dac_y = DAC(2, bits=12)

def mu_to_dac(v):
    n = int((v + 1.0) * 2047.5)
    return 0 if n < 0 else (4095 if n > 4095 else n)

# ---------------------------------------------------------------- patches

def lissajous_patch(fx=3.0, fy=2.0):
    a, b = Osc(fx, 'sine'), Osc(fy, 'sine')
    return [a, b], a, b

def lorenz_patch():
    X, Y, Z = Integrator(None, ic=0.1), Integrator(None), Integrator(None)
    XZ, XY = Mul(X, Z), Mul(X, Y)
    X.src = Sum((Y, 10.0), (X, -10.0))
    Y.src = Sum((X, 28.0), (XZ, -50.0), (Y, -1.0))
    Z.src = Sum((XY, 8.0), (Z, -8.0/3.0))
    return [XZ, XY, X.src, Y.src, Z.src, X, Y, Z], X, Z

def cannon_patch():
    """Quadratic-drag ballistics with wind.
       Pots: A0 = launch angle, A1 = muzzle velocity, A2 = wind."""
    ang  = Pot('A0', lo=0.35, hi=1.4)     # ~20 .. 80 degrees
    v0   = Pot('A1', lo=0.4,  hi=1.0)     # muzzle velocity
    wind = Pot('A2', lo=-0.3, hi=0.3)     # signed wind

    g, k = 0.3, 0.15
    x  = Integrator(None, ic=-0.9, sat=1.1)
    y  = Integrator(None, ic=-0.9, sat=1.1)
    vx = Integrator(None, ic=0.5,  sat=2.0)
    vy = Integrator(None, ic=0.5,  sat=2.0)

    dvx_wind = Sum((vx, 1.0), (wind, -1.0))          # vx - wind
    speed    = Sqrt(dvx_wind, vy)                     # |v_rel|
    drag_x   = Mul(speed, dvx_wind, k=-k)             # -k|v|(vx-w)
    drag_y   = Mul(speed, vy,       k=-k)             # -k|v|vy
    grav     = Const(-g)

    x.src  = vx
    y.src  = vy
    vx.src = drag_x
    vy.src = Sum((drag_y, 1.0), (grav, 1.0))

    def fire(direction):
        if direction != -1: return                    # only on downward cross
        x.preset(-0.9); y.preset(-0.9)
        vx.preset(v0.out * math.cos(ang.out))
        vy.preset(v0.out * math.sin(ang.out))

    ground = Const(-0.9)
    hit    = Comparator(y, ground, on_cross=fire)

    return [ang, v0, wind,
            dvx_wind, speed, drag_x, drag_y, grav,
            x.src, y.src, vx.src, vy.src,
            x, y, vx, vy, hit], x, y

def bouncing_ball_patch():
    """2D ball in a box. Pot A0 = coefficient of restitution."""
    e = Pot('A0', lo=0.5, hi=0.98)
    g = 0.4
    x  = Integrator(None, ic=-0.5, sat=1.1)
    y  = Integrator(None, ic= 0.7, sat=1.1)
    vx = Integrator(None, ic= 0.6, sat=2.0)
    vy = Integrator(None, ic= 0.0, sat=2.0)
    grav = Const(-g)
    x.src, y.src = vx, vy
    vx.src = Const(0.0)
    vy.src = grav

    hi, lo = Const(0.95), Const(-0.95)

    def wall(d):
        vx.preset(-e.out * vx.out)
    def floor(d):
        if d == -1:
            vy.preset(e.out * abs(vy.out))

    cx_hi = Comparator(x, hi, on_cross=lambda d: wall(d)  if d ==  1 else None)
    cx_lo = Comparator(x, lo, on_cross=lambda d: wall(d)  if d == -1 else None)
    cy_lo = Comparator(y, lo, on_cross=floor)

    return [e, x.src, y.src, vx.src, vy.src, x, y, vx, vy,
            cx_hi, cx_lo, cy_lo], x, y

def van_der_pol_patch(mu=1.5):
    """xdot = y ; ydot = mu(1-x^2)y - x  -- limit cycle."""
    x, y = Integrator(None, ic=0.1), Integrator(None, ic=0.0)
    xsq   = Mul(x, x)
    one_m = Sum((Const(1.0), 1.0), (xsq, -1.0))
    term  = Mul(one_m, y, k=mu)
    x.src = y
    y.src = Sum((term, 1.0), (x, -1.0))
    return [xsq, one_m, term, x.src, y.src, x, y], x, y

def rossler_patch(a=0.2, b=0.2, c=5.7):
    """Scaled Rossler attractor."""
    S = 15.0
    X, Y, Z = Integrator(None, ic=0.05), Integrator(None), Integrator(None)
    XZ = Mul(X, Z, k=S)
    X.src = Sum((Y, -1.0), (Z, -1.0))
    Y.src = Sum((X,  1.0), (Y,  a))
    Z.src = Sum((Const(b/S), 1.0), (XZ, 1.0), (Z, -c))
    return [XZ, X.src, Y.src, Z.src, X, Y, Z], X, Y

def harmonograph_patch():
    """Sum of damped sines per axis. Pots A0..A2 tune frequencies."""
    f1 = Pot('A0', lo=0.8, hi=1.2)
    f2 = Pot('A1', lo=1.9, hi=2.1)
    f3 = Pot('A2', lo=2.7, hi=3.3)
    t  = Integrator(Const(1.0))
    class Damped(Block):
        def __init__(self, freq, phase, decay, amp):
            self.freq, self.phase, self.decay, self.amp = freq, phase, decay, amp
        def step(self, dt):
            T = t.out
            self.out = self.amp * math.exp(-self.decay*T) * \
                       math.sin(2*math.pi*self.freq.out*T + self.phase)
    x1 = Damped(f1, 0.0, 0.02, 0.5)
    x2 = Damped(f3, 1.1, 0.03, 0.5)
    y1 = Damped(f2, 0.5, 0.02, 0.5)
    y2 = Damped(f1, 2.2, 0.04, 0.5)
    X  = Sum((x1, 1.0), (x2, 1.0))
    Y  = Sum((y1, 1.0), (y2, 1.0))
    return [f1, f2, f3, t, x1, x2, y1, y2, X, Y], X, Y

def lunar_lander_patch():
    """1D descent. Pot A0 = thrust (0..T_max). Comparator = touchdown.
       h = altitude, v = velocity (down = -), m = mass.
       hdot = v ; vdot = -g + T/m ; mdot = -T/(g0*Isp)"""
    thrust = Pot('A0', lo=0.0, hi=1.0)         # normalized thrust command

    g       = 1.62 / 10.0                       # lunar g, scaled to machine units
    g0_Isp  = 3.0                               # inverse specific impulse (tuned)
    T_max   = 0.35                              # peak thrust in machine units

    h = Integrator(None, ic=1.0,  sat=1.2)      # start 1 machine-unit up
    v = Integrator(None, ic=-0.05, sat=1.0)     # small initial descent rate
    m = Integrator(None, ic=0.9,  sat=1.0)      # start near-full fuel

    # Effective thrust in machine units: T = thrust * T_max
    class Scale(Block):
        def __init__(self, src, k): self.src, self.k = src, k
        def step(self, dt): self.out = self.k * self.src.out
    T = Scale(thrust, T_max)

    # Divide T/m; clamp denominator to avoid blow-up if fuel runs out
    class Div(Block):
        def __init__(self, a, b, eps=0.05):
            self.a, self.b, self.eps = a, b, eps
        def step(self, dt):
            d = self.b.out if abs(self.b.out) > self.eps else self.eps
            self.out = self.a.out / d
    T_over_m = Div(T, m)

    grav   = Const(-g)
    burn   = Scale(T, -1.0 / g0_Isp)

    h.src = v
    v.src = Sum((T_over_m, 1.0), (grav, 1.0))
    m.src = burn

    def touchdown(direction):
        if direction != -1: return
        v.preset(0.0)                            # crash or land: stop motion

    ground = Const(0.0)
    hit    = Comparator(h, ground, on_cross=touchdown)

    # Scope: X = fuel remaining (m), Y = altitude (h)
    return [thrust, T, T_over_m, grav, burn,
            h.src, v.src, m.src, h, v, m, hit], m, h

# ---------------------------------------------------------------- main
PATCH = lissajous_patch    # <- swap this line to change simulation

def run():
    blocks, ox, oy = PATCH()
    eng = Engine(blocks, dt=0.005)
    while True:
        eng.step()
        dac_x.write(mu_to_dac(ox.out))
        dac_y.write(mu_to_dac(oy.out))

if __name__ == '__main__':
    run()
