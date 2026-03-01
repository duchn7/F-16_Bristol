using Plots, DifferentialEquations, Interpolations
include("aero_8.jl"); include("eom_8.jl")

# Select a pair of control input (CI) and initial condition (IC)
# CI 1, IC 1: fully-developed spin
# CI 2, IC 2: deep stall
# CI 1, IC 2: spin entry from deep stall
# CI 2, IC 1: spin self-recovery (back to deep stall)

# CI 1
da = -20 # aileron in deg (+-21.5). Negative = right wing down
de = -5 # stabilator deg (+-25)
dr = 0 # rudder in deg (+-30) 
CG = 37.5 # CG position in %MAC (25.0 to 37.5)
T  = 10000 # thrust in N

# IC 1
alpha0 = 1.163325534316547 # rad
beta0  = -0.0142233141487139 # rad
V0     = 92.2774155855499 # m/s
p0     = -0.249806083516482 # rad/s
q0     = 0.239517453942822 # rad/s
r0     = -0.582082779189844 # rad/s
phi0   = -0.113403721456400 # rad
theta0 = -0.414023981066343 # rad
psi0   = 0 # rad

# # CI 2
# da = 0 # aileron in deg (+-21.5). Negative = right wing down
# de = 0 # stabilator deg (+-25)
# dr = 0 # rudder in deg (+-30) 
# CG = 37.5 # CG position in %MAC (25.0 to 37.5)
# T  = 10000 # thrust in N

# # IC 2
# alpha0 = 1.01392670620385 # rad
# beta0  = 0 # rad
# V0     = 79.6036079039396 # m/s
# p0     = 0 # rad/s
# q0     = 0 # rad/s
# r0     = 0 # rad/s
# phi0   = 0 # rad
# theta0 = 0.162105481242851 # rad
# psi0   = 0 # rad

# Solve ODE
t_sim = 500 # Simulation time in seconds
prob = ODEProblem(eom_8!, [alpha0 beta0 V0 p0 q0 r0 phi0 theta0], (0.0, t_sim), [da, de, dr, CG, T])
sol  = init(prob, Tsit5(), reltol=1e-4)

time_taken = @elapsed begin
    sol = solve(prob, Tsit5())
end

println("Solver run time: ", time_taken, " sec")

# Plot
plot_layout = @layout [a b c; d e f; g h i]

p1 = plot(sol.t, sol.u.|> x -> x[1]*180/π, color=:black, linewidth=1, ylabel="α (deg)", legend=false)
p2 = plot(sol.t, sol.u.|> x -> x[4]*180/π, color=:black, linewidth=1, ylabel="p (deg/s)", legend=false)
p3 = plot(sol.t, sol.u.|> x -> x[7]*180/π, color=:black, linewidth=1, ylabel="ϕ (deg)", legend=false)
p4 = plot(sol.t, sol.u.|> x -> x[2]*180/π, color=:black, linewidth=1, ylabel="β (deg)", legend=false)
p5 = plot(sol.t, sol.u.|> x -> x[5]*180/π, color=:black, linewidth=1, ylabel="q (deg/s)", legend=false)
p6 = plot(sol.t, sol.u.|> x -> x[8]*180/π, color=:black, linewidth=1, ylabel="θ (deg)", legend=false, xlabel="Time (s)")
p7 = plot(sol.t, sol.u.|> x -> x[3],       color=:black, linewidth=1, ylabel="V (m/s)", legend=false)
p8 = plot(sol.t, sol.u.|> x -> x[6]*180/π, color=:black, linewidth=1, ylabel="r (deg/s)", legend=false)

plot(p1, p2, p3, p4, p5, p6, p7, p8, layout=plot_layout, size=(800,500))