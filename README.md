# Water Rocket Simulator

**Design a water rocket, launch it, and see the whole flight — in your browser.**

🔗 **Live:** https://doublef35.github.io/waterrocketsim/

A full 6-degree flight simulator for compressed-air water rockets. Set the bottle
volume, fill ratio and pressure, pick a nozzle, size the parachute — then run it and
get altitude, velocity, acceleration, thrust and energy plotted against time, plus an
animated trajectory and a CSV of every timestep.

No install, no build step, no account. One HTML file and Chart.js.

---

## What it actually models

The flight is integrated with **RK4 at a 0.5 ms timestep** (logged every 10 ms), in two
dimensions, through three phases:

| Phase | What's happening |
|---|---|
| `thrust` | Water is forced out of the nozzle by the compressed air above it |
| `coast` | Water is gone — the rocket is ballistic, body drag only |
| `descent` | Parachute is out — drag switches to the canopy |

**Gas expansion.** As water leaves, the air volume grows and the pressure drops. You
choose the law:

```
adiabatic (γ = 1.4)   P = P₀ · (V_air0 / V_air)^1.4     ← more accurate
isothermal            P = P₀ · (V_air0 / V_air)
```

Adiabatic is the honest one for a launch that lasts ~0.2 s — the air has no time to
exchange heat with its surroundings, so it cools as it expands and pressure falls
faster than the isothermal curve suggests. Isothermal is there to show you the
difference.

**Thrust.** Exhaust velocity comes from Torricelli's law through a nozzle with a
discharge coefficient:

```
v_exhaust = Cd_nozzle · √(2·P_gauge / ρ_water)
F_thrust  = ρ_water · A_nozzle · v_exhaust²  =  2 · P_gauge · A_nozzle · Cd²
```

**Drag.** Standard quadratic drag, `F_d = ½·ρ_air·v²·Cd·A`, applied against the
velocity vector — with the reference area and Cd swapping to the parachute's the
moment it deploys.

### What it deliberately doesn't model

Worth knowing before you trust a number:

- **No air-only blowdown phase.** Thrust cuts off the instant the water runs out. A
  real rocket keeps pushing for a few more milliseconds as the residual compressed air
  escapes — a small but real extra impulse, typically a couple of percent of altitude.
- **No nozzle two-phase flow** at the water/air transition.
- **Drag coefficient is constant** — no Reynolds or Mach dependence (fine here; water
  rockets stay well below Mach 0.3).
- **No wind, no weathercocking, no fin aerodynamics.** Launch angle tilts the thrust
  vector, but the rocket does not rotate — it has no angle of attack.
- **Parachute opens instantly** — no inflation transient.

---

## Controls

| Group | Parameters |
|---|---|
| **Bottle** | Volume (L), fill ratio (%), initial pressure (bar), expansion law |
| **Nozzle** | Throat diameter (mm), discharge coefficient `Cd` (0.60–1.00) |
| **Airframe** | Dry mass (g), body diameter (mm), body `Cd` (0.15–1.00) |
| **Recovery** | Parachute diameter (cm), canopy `Cd`, deployment mode |
| **Launch / Env** | Launch angle (°), air density ρ, gravity g |

**Deployment modes:** at apogee (servo/barometric) · 2 s after apogee · 5 s after
apogee · altitude trigger at 30 m descending · none (ballistic return).

**Presets:** `beginner` · `intermediate` · `competition` · `altitude` · `heavy` ·
`angled` — six starting points that are already in sensible parameter ranges.

## Outputs

Six headline numbers — **max altitude, peak velocity, peak thrust, max acceleration,
flight time, landing speed** — plus tabbed charts for **altitude, velocity,
acceleration, thrust, energy and phases**, an animated 2D trajectory, and
**CSV export** of the full timestep log if you want to do your own analysis.

---

## Running it locally

It's a single static file with one CDN dependency, so:

```bash
git clone https://github.com/DoubleF35/waterrocketsim.git
cd waterrocketsim
python3 -m http.server 8000
# open http://localhost:8000
```

Opening `index.html` directly with `file://` works too.

---

## Also in this repo

- [`ESP/`](ESP/) — 100×100 colour video streaming from an OV7670 camera on an
  ESP32-C3 Super Mini to a Python viewer (Arduino firmware + viewer).

---

## Contributing

Issues and pull requests are welcome — particularly on the physics. If you have
launch data from a real rocket that disagrees with the simulator, that's the most
useful thing you could open an issue about.
