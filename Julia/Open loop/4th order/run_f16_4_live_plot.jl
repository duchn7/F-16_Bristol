using GLMakie, DifferentialEquations, Interpolations
include("aero_4.jl"); include("eom_4.jl")  

# Live plot options
t_interval = 0 # time step size on the live plot (0 to plot everything)
t_wait     = 0 # wait __ sec of real time before adding the next data point (0 for fastest response)

# Control inputs
de = -5    # stabilator (deg) [-25 +25]
cg = 25   # centre of gravity position (%MAC) - try between 25 (stable) and 37.5 (unstable)
T  = 10000 # thrust (N) (enough for level flight at around Mach 0.6 at 30,000 ft)

# Initial conditions
alpha0 = 1.08578827850333  # rad
V0     = 85.9137949570426  # m/s
q0     = 0                 # rad/s
theta0 = 0.184809486923954 # rad

tspan = (0.0, 500)

# Solve ODE
prob = ODEProblem(eom_4!, [alpha0, V0, q0, theta0], tspan, [de, cg, T])
sol  = init(prob, Tsit5())
t = Float64[]
u = Vector{Float64}[] 

# Create observable data holders for live plotting
obs = Dict(:t  => Observable(Float64[]),
           :α  => Observable(Float64[]),
           :V  => Observable(Float64[]),
           :q  => Observable(Float64[]),
           :θ  => Observable(Float64[]))

# Set up live figures and lines
fig = Figure(size=(500,800))
display(fig)  # Open the interactive window

p1 = Axis(fig[1, 1], ylabel = "α (deg)")
p2 = Axis(fig[2, 1], ylabel = "V (m/s)")
p3 = Axis(fig[3, 1], ylabel = "q (deg/s)")
p4 = Axis(fig[4, 1], ylabel = "θ (deg)", xlabel = "t (s)")

lines!(p1, obs[:t], obs[:α], color = :black)
lines!(p2, obs[:t], obs[:V], color = :black)
lines!(p3, obs[:t], obs[:q], color = :black)
lines!(p4, obs[:t], obs[:θ], color = :black)

next_plot_time = 0  # Initialize target refresh time
while sol.t < tspan[2]
    step!(sol)
    push!(t, sol.t)
    push!(u, sol.u[:])

    if sol.t ≥ next_plot_time
        push!(obs[:t][], sol.t)
        push!(obs[:α][], sol.u[1] * 180 / pi)
        push!(obs[:V][], sol.u[2])
        push!(obs[:q][], sol.u[3] * 180 / pi)
        push!(obs[:θ][], sol.u[4] * 180 / pi)

        if length(u) > 1 # ensure there are at least 2 data points before changing the axis limits
            p1.limits = ([0, tspan[2]], extrema(getindex.(u, 1)) .* (180 / pi))
            p2.limits = ([0, tspan[2]], extrema(getindex.(u, 2)))
            p3.limits = ([0, tspan[2]], extrema(getindex.(u, 3)) .* (180 / pi))
            p4.limits = ([0, tspan[2]], extrema(getindex.(u, 4)) .* (180 / pi))
        end

        notify(obs[:t]) # Referesh Makeie
        yield() # prevent figure freezing
        sleep(t_wait)
        global next_plot_time += t_interval  # ← advance by fixed amount
    end
end

# Final update in case last step missed refresh
notify(obs[:t]) # Referesh Makeie
sleep(0)