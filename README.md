# Simple Non-Linear 6DOF Aircraft Simulator

This non-linear 6DOF simulator is has been made as a sort of
minimum viable simulator without all the complexity and capability
of other simulation tools you may have found and tried to use.
It uses the flat Earth equations of motion and no propulsion model to limit complexity.

Hopefully this code is set out in a way that is readable, decently
well commented, and can help you understand the main steps that
go into an aircraft simulation.

It is set up with the same aircraft as my aircraft stability tool found here: [Aircraft Stability Tool](https://simmeon.github.io/aircraft-stability/). This should allow you to compare the non-linear simulation here to the linear simulation on the stability tool.

---

## Getting Started

This Python code has very limited dependencies and should be pretty easy to run. You only need numpy, scipy, and matplotlib.

- Clone/download repository
- Make sure dependencies are installed (in requirements.txt)
- Run `main.py`

## Using a different aircraft

This simulator is hopefully written in a way that makes it simple to change the aircraft properties or create a completely different aircraft.

I would recommend looking at `cessnalike_aircraft.py` to see what properties and coefficients are generally needed to get it going. In general you will need:

- Mass
- Inertias (Ixx, Iyy, Izz, Ixz)
- Reference lengths/areas (S: wing area, b: wingspan, c: mean chord length)
- Stability and control derivatives


You may not need all the stability and control derivatives that are defined (some may be zero for your aircraft), but you probably need most of them to be able to calculate accurate forces and moments.

## Changing the force/moment calculations

The `calculate_forces_and_moments.py` file contains the maths for calculating the forces and moments in the body frame using linear taylor expansions - i.e. the stability and control derivatives. If you have more coefficients available you could easily add them to the AircraftCoeffs dataclass in `aircraft.py` and then include them in the force/moment calculations.

You could also completely change these calculations so that they use non-linear functions (for example, use the drag-polar equation to calculate the drag force) or even calculate things based on lookup tables.

All that matters is that you return the forces and moments in the *body frame* in the order the simulator is expecting: `return (FX, FY, FZ, L, M, N)`.

## Simulating perturbations

The code is currently set up with a short elevator pulse to demonstrate a way of adding perturbations. Within the `run_sim()` simulation loop in `main.py` you can and should freely update the control deflections and thrust to be whatever you want. You could even write a controller if you wanted.

## Trimming

The code includes a basic trimmer of mine that has hopefully been incorporated correctly. The trimmer assumes `v = p = q = r = phi = psi = 0`, and expects to be given an altitude, airspeed, and flight path angle to find a trimmed state at. This is a somewhat arbitrary and personal choice, and you could modify it if you wanted to instead specify, say, thrust, and find a flight path angle for that thrust instead. I've left this fairly uncommented since I think it is useful for you to work through deriving the equations for trim yourself and do it mostly by hand initially. 

## Changing the equations of motion

The equations of motion are defined in the `dxdt()` function of `solver.py`. I've used the flat earth equations of motion written mostly in the body frame. If you want to, you could change these. You could modify them to account for a globe earth, a rotating earth, etc. Next to calculating the forces/moments, this is probably the easiest part to mess up and can be a pain to debug. It's also probably the most mathematically difficult part of the simulation and you really have to understand the equations if you want to change them.

## Improving the simulator

As I said, this was written to have basically the minimum functionality needed to do a 6DOF sim. To give some examples of what you could add or change to improve it yourself:

- Add a propulsion system model
    - Instead of just directly giving the thrust, make the control input a 'throttle setting' and have a model of what thrust force you get for that throttle setting, based on your current aircraft state. This could depend on things like altitude, speed, angle of attack, etc.
- Write your own better trim function
- Modify the simulation to be able to include wind
    - Depending on how you do this you might need to modify the position equations of motion
- And much more...