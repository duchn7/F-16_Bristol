using Plots, DifferentialEquations, Interpolations
include("aero_4.jl"); include("eom_4.jl")  

# Control inputs
de = -5    # stabilator in deg (+- 25)
CG = 25   # centre of gravity position in %MAC (25.0 to 37.5)
T  = 10000 # thrust in N

# Initial conditions
alpha0 = 1.08578827850333  # rad
V0     = 85.9137949570426  # m/s
q0     = 0                 # rad/s
theta0 = 0.184809486923954 # rad

# Solve ODE
prob = ODEProblem(eom_4!, [alpha0, V0, q0, theta0], (0.0, 500), [de, CG, T])
sol  = init(prob, Tsit5(), reltol=1e-4)

time_taken = @elapsed begin
    sol = solve(prob, Tsit5())
end

println("Solver run time: ", time_taken, " sec")

# Plot
plot_layout = @layout [a; b; c; d]

p1 = plot(sol.t, sol.u.|> x -> x[1]*180/π, color=:black, linewidth=1, ylabel="α (deg)", legend=false)
p2 = plot(sol.t, sol.u.|> x -> x[2],       color=:black, linewidth=1, ylabel="V (m/s)", legend=false)
p3 = plot(sol.t, sol.u.|> x -> x[3]*180/π, color=:black, linewidth=1, ylabel="q (deg/s)", legend=false)
p4 = plot(sol.t, sol.u.|> x -> x[4]*180/π, color=:black, linewidth=1, ylabel="θ (deg)", legend=false, xlabel="Time (s)")

plot(p1, p2, p3, p4, layout=plot_layout, size=(400,600))