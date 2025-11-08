function [M]=trajectory3z(x,y,z,pitch,roll,yaw,scale_factor,step)

if (length(x)~=length(y))|(length(x)~=length(z))|(length(y)~=length(z))
    disp('  Error:');
    disp('      Uncorrect Dimension of the center trajectory Vectors. Please Check the size');
    M=0;
    return;
end
if ((length(pitch)~=length(roll))||(length(pitch)~=length(yaw))||(length(roll)~=length(yaw)))
    disp('  Error:');
    disp('      Uncorrect Dimension of the euler''s angle Vectors. Please Check the size');
M=0;
    return;
end
if length(pitch)~=length(x)
    disp('  Error:');
    disp('      Size mismatch between euler''s angle vectors and center trajectory vectors');
    M=0;
    return
end
if step>=length(x)
    disp('  Error:');
    disp('      Attitude samplig factor out of range. Reduce step');
M=0;
    return
end
if step<1
    step=1;

end


if nargin>10
    disp('Too many inputs arguments');
    M=0;
    return
end


mov=nargout;

cur_dir=pwd;

load CAD_F16;
V=[-V(:,1) V(:,2) -V(:,3)];
V(:,1)=V(:,1)-round(sum(V(:,1))/size(V,1));
V(:,2)=V(:,2)-round(sum(V(:,2))/size(V,1));
V(:,3)=V(:,3)-round(sum(V(:,3))/size(V,1));

correction=max(abs(V(:,1)));
V=V./(scale_factor*correction);
ii=length(x);
resto=mod(ii,step);
%%%%%%%%%%%%%%%needed for the transformation%%%%%%%%%%%%%%%
  
y=y;
    z=z;
    pitch=pitch;
    roll=roll;
    yaw=-yaw;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
frame = 0;
for i=1:step:(ii-resto)
if mov | (i == 1)
      clf;
      plot3(x,y,z,'k');
      grid on;
      hold on;
      light;
    end
theta=pitch(i);
phi=-roll(i);
psi=yaw(i);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Tbe=[cos(psi)*cos(theta), -sin(psi)*cos(theta), sin(theta);
	 cos(psi)*sin(theta)*sin(phi)+sin(psi)*cos(phi) ...
	 -sin(psi)*sin(theta)*sin(phi)+cos(psi)*cos(phi) ...
	 -cos(theta)*sin(phi);
	 -cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi) ...
	 sin(psi)*sin(theta)*cos(phi)+cos(psi)*sin(phi) ...
	 cos(theta)*cos(phi)];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Vnew=V*Tbe;
rif=[x(i) y(i) z(i)];
X0=repmat(rif,size(Vnew,1),1);
Vnew=Vnew+X0;
p=patch('faces', F, 'vertices' ,Vnew);
set(p, 'facec', 'w'); % set the colour for the aircraft      
set(p, 'EdgeColor','k'); 
if mov | (i == 1)

      axis equal;
end
if mov
if i == 1
	ax = axis;
else
	axis(ax);
      end
      lighting none
      frame = frame + 1;
      M(frame) = getframe;
    end

end
hold on;

lighting none;
grid on;


daspect([1 1 1]) ;


xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');

cd (cur_dir);