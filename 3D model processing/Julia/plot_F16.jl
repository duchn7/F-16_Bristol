# Convert a 3D model in MATLAB .mat format into Julia .ser and plot the latter

using MAT, GLMakie, Serialization

# Load .mat file
matfile = matopen("F16.mat")
V = read(matfile, "V")  # Nx3 Float64
F = read(matfile, "F")  # Mx3 Float64
close(matfile)

# Fix 0-based indexing (common in MATLAB exports)
if minimum(F) == 0
    F .+= 1
end

# Convert faces to Int matrix (N×3, each row = triangle)
faces = convert(Matrix{Int}, F)

# Convert vertices to Point3f
vertices = Makie.Point3f.(eachrow(V))
# vertices = Makie.Point3f.((v[2], v[1], v[3]) for v in eachrow(V))

serialize("F16.ser", (vertices, faces))

(vertices, faces) = deserialize("F16.ser")

# Plot
f = Figure()
ax = LScene(f[1, 1])
mesh!(ax, vertices, faces, color=:lightblue, shading=FastShading)
f