using GLMakie

# Step 1: Define vertices (Nx3 array)
vertices = [
    0 0 0;  # 1
    1 0 0;  # 2
    1 1 0;  # 3
    0 1 0;  # 4
    0 0 1;  # 5
    1 0 1;  # 6
    1 1 1;  # 7
    0 1 1;  # 8
]

# Step 2: Define quads and convert to triangles (each triangle is a row)
quads = [
    (1, 2, 3, 4),
    (5, 6, 7, 8),
    (1, 2, 6, 5),
    (2, 3, 7, 6),
    (3, 4, 8, 7),
    (4, 1, 5, 8),
]

# Convert to triangles as an array of Ints (Nx3 matrix)
triangles = vcat([
    [a b c; a c d] for (a, b, c, d) in quads
]...)

# Step 3: Transpose vertices to 3×N (for mesh!)
vertices_t = permutedims(vertices)  # 3×N

# Step 4: Plot
fig = Figure()
ax = Axis3(fig[1, 1])
mesh!(ax, vertices_t, triangles, color = :grey, shading = true)
fig
