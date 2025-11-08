# Get the new vertices of a 3D shape based on its XYZ location and Euler angles

using GLMakie, LinearAlgebra, Serialization
using GeometryBasics: Vec
using Makie: boundingbox

function get_vertices(X, Y, Z, phi, theta, psi)
    (vertices, faces) = deserialize("CAD_F16.ser")
    vertices = vertices/100
    vertices = [Point{3, Float32}(-v[1], v[2], -v[3]) for v in vertices]
    
    Rx = [1 0 0;
          0 cos(phi) -sin(phi);
          0 sin(phi) cos(phi)]
    
    Ry = [cos(theta) 0 sin(theta);
          0 1 0;
         -sin(theta) 0 cos(theta)]
    
    Rz = [cos(psi) -sin(psi) 0;
          sin(psi) cos(psi) 0;
          0 0 1]
    
    R = Rz * Ry * Rx
    
    pos_vec = Vec(X,Y,Z)
    new_vertices = [Point(R * Vec(v) + pos_vec) for v in vertices]
    
    return new_vertices, faces
end