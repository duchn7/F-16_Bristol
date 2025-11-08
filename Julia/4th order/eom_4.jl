function eom_4!(dx, x, u, t)

# States
alpha = x[1] # rad
V     = x[2] # m/s
q     = x[3] # rad/s
theta = x[4] # rad/s

# Control inputs
de = u[1] # stabilator in deg
CG = u[2] # centre of gravitiy position in %MAC
T  = u[3] # thrust in N

# Aircraft's physical parameters
S   = 27.87             # wing area in m^2 (300 ft)
c   = 3.4503            # mean aerodynamic chord in m (11.32 ft)
m   = 9300              # mass in kg (20,500 lbs)
Iy  = 75674             # kg/m2
g   = 9.81              # m/s^2
rho = 0.458312441644953 # air density in kg/m^3 at 9,144 m (30,000 ft)

## Aerodynamic coefficients
QU  = c*q/2/V;  

# 1D tables
CXQ = CXQ_intp1(alpha*180/pi)
CZQ = CZQ_intp1(alpha*180/pi)
DCM = DCM_intp1(alpha*180/pi)
CMQ = CMQ_intp1(alpha*180/pi)

# CX
CXBAS = CXBAS_intp2(alpha*180/pi,de)
CX    = CXBAS + QU*CXQ;
                    
# CZ
CZBAS = CZBAS_intp2(alpha*180/pi,de)
CZ = CZBAS + QU*CZQ
   
# CM
CMBAS = CMBAS_intp2(alpha*180/pi,de)
CM = CMBAS + DCM + QU*CMQ + CZ*(0.35-CG/100)

## Equations of motion
alphadot = (0.5*rho*V^2*S*(CZ*cos(alpha)-CX*sin(alpha)) - T*sin(alpha) + m*g*cos(theta-alpha)) / (m*V)  +  q
Vdot     = (0.5*rho*V^2*S*(CX*cos(alpha)+CZ*sin(alpha)) + T*cos(alpha) - m*g*sin(theta-alpha)) / m
qdot     =  0.5*rho*V^2*S*c*CM/Iy
thetadot =  q

dx[1] = alphadot
dx[2] = Vdot
dx[3] = qdot
dx[4] = thetadot

end