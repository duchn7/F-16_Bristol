% Load the original 3D model from the flight visualisation code by Rodney Rodriguez
% uk.mathworks.com/matlabcentral/fileexchange/86453-aircraft-3d-animation
load("f16_3d_model.mat"); 

% Initialize combined vertex and face arrays
Va = [];
Fa = [];

% Initialize vertex offset
offset = 0;

% Combine aircraft parts
for i = 1:2
    V = Model3D.Aircraft(i).stl_data.vertices;
    F = Model3D.Aircraft(i).stl_data.faces;

    Fa = [Fa; F + offset]; 
    Va = [Va; V];          

    offset = offset + size(V, 1);
end

% Combine control surfaces
for i = 1:7
    V = Model3D.Control(i).stl_data.vertices;
    F = Model3D.Control(i).stl_data.faces;

    Fa = [Fa; F + offset]; 
    Va = [Va; V];          

    offset = offset + size(V, 1);
end

% Plot the original model 
figure
subplot(211)
ax1 = gca;
patch('Faces', Fa, 'Vertices', Va, 'FaceColor', 'w', 'EdgeColor', 'k')
axis equal
lighting flat
title('Full resolution')
xlabel('X'); ylabel('Y'); zlabel('Z');

% Downsample: retain only 2% of the original vertices
b = reducepatch(Fa,Va,0.02,'verbose'); 

subplot(212)
ax2 = gca;
patch(b,'FaceColor','w','EdgeColor','k')
axis equal
lighting flat
title('Downsampled')
xlabel('X'); ylabel('Y'); zlabel('Z');

h = linkprop([ax1, ax2], {'CameraPosition', 'CameraTarget', 'CameraUpVector', 'CameraViewAngle'});

% To export the final model, save the following two variables as a .mat file
F = b.faces; 
V = b.vertices; 