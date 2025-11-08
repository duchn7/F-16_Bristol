# Run parallel simulations for parameter sweeps

using Plots, Distributed
addprocs(23) # Number of parallel workers. No more than number of cores - 1

# Load libraries and define variables on all workers
@everywhere begin
    using DifferentialEquations, Interpolations
    include("aero_4.jl"); include("eom_4.jl")    

    alpha0 = 1.08578827850333  # rad
    V0     = 85.9137949570426  # m/s
    q0     = 0                 # rad/s
    theta0 = 0.184809486923954 # rad

    # Function to solve the ODE
    function run_sim(p)
        de, CG, T = p  # Extract parameters from the tuple
        prob = ODEProblem(eom_4!, [alpha0, V0, q0, theta0], (0.0, 500), [de, CG, T])
        solve(prob, Tsit5())
    end
end

# Define parameter sweep (on master process)
de_range = -25:0.1:5
CG_range = 25:0.2:30
T        = 8000  # fixed

param_sweep = [(de, CG, T) for de in de_range, CG in CG_range]
param_sweep = vec(param_sweep)  # flatten to 1D vector of tuples

# Run simulations in parallel
time_taken = @elapsed begin
    results = pmap(run_sim, param_sweep)
end

println("Solver run time: ", time_taken, " sec")

# 2D plot
# dstab_values = [p[1] for p in param_sweep]
# final_matrix = hcat([r.u[end] for r in results]...)'
# plot(dstab_values,final_matrix[:,1])

alphas = [r.u[end][1]*180/π for r in results]  # final alpha from each sim

dstabs = [p[1] for p in param_sweep]
cgs    = [p[2] for p in param_sweep]

plotlyjs()  

scatter3d(cgs, dstabs, alphas;
          xlabel="CG (%MAC)", ylabel="Stabilator (deg)", zlabel="α (deg)",
          markerstrokecolor=:black, markersize=1,
          legend=false,
          size=(700,700))