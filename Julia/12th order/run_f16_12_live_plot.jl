using GLMakie, DifferentialEquations, Interpolations
include("aero_12.jl"); include("eom_12.jl"); include("get_vertices.jl")

t_sim         = 100 # Simulation time in seconds
t_refresh     = 2   # Refresh the figure every __ second of simulated time. Set to t_sim to refresh only once after the simulation finishes running
draw_aircraft = 1   # For each time the figure is refreshed, draw an aircraft on the top-right subplot? (1: yes, 0: no)
keep_aircraft = 1   # Retain the previously drawn aircraft in the top-right subplot after each refresh event? (1: yes, 0: no)
t_wait        = 0.1 # Wait __ seconds in real time each time the figure is refreshed (0 for fastest response)

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
X0     = 0 # m
Y0     = 0 # m
Z0     = 0 # m

# # CI 2
# da = 0 # aileron in deg (+-21.5). Negative = right wing down
# de = 0 # stabilator deg (+-25)
# dr = 0 # rudder in deg (+-30) 
# CG = 37.5 # CG position in %MAC (25.0 to 37.5)
# T  = 10000 # thrust in N

# # IC 2
# alpha0 = 1.08578827850333 # rad
# beta0  = 0 # rad
# V0     = 85.9137949570426 # m/s
# p0     = 0 # rad/s
# q0     = 0 # rad/s
# r0     = 0 # rad/s
# phi0   = 0 # rad
# theta0 = 0.184809486923954 # rad
# psi0   = 0 # rad
# X0     = 0 # m
# Y0     = 0 # m
# Z0     = 0 # m

# Solve ODE
prob = ODEProblem(eom_12!, [alpha0 beta0 V0 p0 q0 r0 phi0 theta0 psi0 X0 Y0 Z0], (0.0, t_sim), [da, de, dr, CG, T])
sol  = init(prob, Tsit5(), reltol=1e-4)

t = Float64[]
u = Vector{Float64}[] 

# Create observable data holders for live plotting
obs = Dict(:t => Observable(Float64[]),
           :α => Observable(Float64[]), :β => Observable(Float64[]), :V => Observable(Float64[]),
           :p => Observable(Float64[]), :q => Observable(Float64[]), :r => Observable(Float64[]), 
           :ϕ => Observable(Float64[]), :θ => Observable(Float64[]), :ψ => Observable(Float64[]), 
           :X => Observable(Float64[]), :Y => Observable(Float64[]), :Z => Observable(Float64[]))

# Set up live figures and lines
fig = Figure(size=(1800,650))
display(fig)  # Open the interactive window

# 2D plots
p1 = Axis(fig[1, 1], ylabel = "α (deg)"); lines!(p1, obs[:t], obs[:α], color = :black)
p2 = Axis(fig[1, 2], ylabel = "β (deg)"); lines!(p2, obs[:t], obs[:β], color = :black)
p3 = Axis(fig[1, 3], ylabel = "V (m/s)"); lines!(p3, obs[:t], obs[:V], color = :black)

p4 = Axis(fig[2, 1], ylabel = "p (deg/s)"); lines!(p4, obs[:t], obs[:p], color = :black)
p5 = Axis(fig[2, 2], ylabel = "q (deg/s)"); lines!(p5, obs[:t], obs[:q], color = :black)
p6 = Axis(fig[2, 3], ylabel = "r (deg/s)"); lines!(p6, obs[:t], obs[:r], color = :black)

p7 = Axis(fig[3, 1], ylabel = "ϕ (deg)"); lines!(p7, obs[:t], obs[:ϕ], color = :black)
p8 = Axis(fig[3, 2], ylabel = "θ (deg)"); lines!(p8, obs[:t], obs[:θ], color = :black)
p9 = Axis(fig[3, 3], ylabel = "ψ (deg)"); lines!(p9, obs[:t], obs[:ψ], color = :black)

p10 = Axis(fig[4, 1], ylabel = "X (m)"); lines!(p10, obs[:t], obs[:X], color = :black); p10.xlabel = "t (s)"
p11 = Axis(fig[4, 2], ylabel = "Y (m)"); lines!(p11, obs[:t], obs[:Y], color = :black); p11.xlabel = "t (s)"; 
p12 = Axis(fig[4, 3], ylabel = "Z (m)"); lines!(p12, obs[:t], obs[:Z], color = :black); p12.xlabel = "t (s)"; p12.yreversed = true

# 3D plots
p13 = Axis3(fig[1:2, 4], xlabel = "X (m)", ylabel = "Y (m)", zlabel = "Z (m)")
p13.yreversed = true; p13.zreversed = true
lines!(p13, obs[:X], obs[:Y], obs[:Z], color = :black)

p14 = Axis3(fig[3:4, 4], xlabel = "X (m)", ylabel = "Y (m)", zlabel = "Z (m)")
p14.yreversed = true; p14.zreversed = true
lines!(p14, obs[:X], obs[:Y], obs[:Z], color = :black)

next_plot_time = 0  # Initialize target refresh time

# Dynamic min/max trackers
mins_y = fill( Inf, 12)
maxs_y = fill(-Inf, 12)

while sol.t < t_sim
    step!(sol)
    push!(t, sol.t)
    push!(u, sol.u[:])
    push!(obs[:t][], sol.t)
    push!(obs[:α][], sol.u[1]*180/pi)
    push!(obs[:β][], sol.u[2]*180/pi)
    push!(obs[:V][], sol.u[3])
    push!(obs[:p][], sol.u[4]*180/pi)
    push!(obs[:q][], sol.u[5]*180/pi)
    push!(obs[:r][], sol.u[6]*180/pi)
    push!(obs[:ϕ][], sol.u[7]*180/pi)
    push!(obs[:θ][], sol.u[8]*180/pi)
    push!(obs[:ψ][], sol.u[9]*180/pi)
    push!(obs[:X][], sol.u[10])
    push!(obs[:Y][], sol.u[11])
    push!(obs[:Z][], sol.u[12])

    # Update dynamic extrema
    for i in 1:12
        mins_y[i] = min(mins_y[i], sol.u[i])
        maxs_y[i] = max(maxs_y[i], sol.u[i])
    end

    if sol.t ≥ next_plot_time 
        # Function to prevent error message when the line is flat (max value = min value): make the y-axis limit = flat value +-1
        function padded(lo, hi; pad=1)
            return lo == hi ? (lo - pad, hi + pad) : (lo, hi)
        end

        # 2D plots: dynamic axis limits
        p1.limits  = ([0, t_sim], padded(mins_y[1],  maxs_y[1]).*(180/pi))
        p2.limits  = ([0, t_sim], padded(mins_y[2],  maxs_y[2]).*(180/pi))
        p3.limits  = ([0, t_sim], padded(mins_y[3],  maxs_y[3]))
        p4.limits  = ([0, t_sim], padded(mins_y[4],  maxs_y[4]).*(180/pi))
        p5.limits  = ([0, t_sim], padded(mins_y[5],  maxs_y[5]).*(180/pi))
        p6.limits  = ([0, t_sim], padded(mins_y[6],  maxs_y[6]).*(180/pi))
        p7.limits  = ([0, t_sim], padded(mins_y[7],  maxs_y[7]).*(180/pi))
        p8.limits  = ([0, t_sim], padded(mins_y[8],  maxs_y[8]).*(180/pi))
        p9.limits  = ([0, t_sim], padded(mins_y[9],  maxs_y[9]).*(180/pi))
        p10.limits = ([0, t_sim], padded(mins_y[10], maxs_y[10]))
        p11.limits = ([0, t_sim], padded(mins_y[11], maxs_y[11]))
        p12.limits = ([0, t_sim], padded(mins_y[12], maxs_y[12]))

        # Draw aircraft
        if draw_aircraft == 1
            (new_vertices, faces) = get_vertices(sol.u[10], sol.u[11], sol.u[12],
                                                 sol.u[7] , sol.u[8] , sol.u[9])
            aircraft_drawn_p13 = mesh!(p13, new_vertices, faces, color = :lightblue, shading = FastShading)
        end    

        pad_3d = 8
        p13.limits = (padded(mins_y[10], maxs_y[10]) .+ (-pad_3d, pad_3d),
                      padded(mins_y[11], maxs_y[11]) .+ (-pad_3d, pad_3d),
                      padded(mins_y[12], maxs_y[12]) .+ (-pad_3d, pad_3d))

        aircraft_drawn_p14 = mesh!(p14, new_vertices, faces, color = :lightblue, shading = FastShading)
        p14.limits = (padded(sol.u[10] - pad_3d, sol.u[10] + pad_3d),
                      padded(sol.u[11] - pad_3d, sol.u[11] + pad_3d),
                      padded(sol.u[12] - pad_3d, sol.u[12] + pad_3d))
        p14.aspect = :data

        notify.(values(obs))
        sleep(t_wait)

        if sol.t < t_sim
            if keep_aircraft == 0
                delete!(p13.scene, aircraft_drawn_p13)
            end
            delete!(p14.scene, aircraft_drawn_p14)
        end

        global next_plot_time += t_refresh
    end
end

# Final update in case last step missed refresh
notify.(values(obs)) # Refresh Makie
yield() # prevent figure freezing
sleep(0)