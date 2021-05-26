from scipy.integrate import ode
from numpy import *
import eom

t_final = 10.0
dt = 0.25

y0 = [0,0,20,0,0,1]
t0 = 0.0

y_result = []
t_output = []

# Initialisation:

backend = "dopri5"

solver = ode(eom.eom)
solver.set_integrator(backend)  # nsteps=1
solver.set_initial_value(y0, t0)

y_result.append(y0)
t_output.append(t0)

while solver.successful() and solver.t < t_final:
    solver.integrate(solver.t + dt, step=1)
    
    y_result.append(solver.y)
    t_output.append(solver.t)
    
y_result = array(y_result)
t_output = array(t_output)

print y_result