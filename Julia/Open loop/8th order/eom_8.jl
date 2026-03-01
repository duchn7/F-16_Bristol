function eom_8!(dx, x, u, t)

# States
alpha = x[1] # rad
beta  = x[2] # rad
V     = x[3] # m/s
p     = x[4] # rad/s
q     = x[5] # rad/s
r     = x[6] # rad/s
phi   = x[7] # rad
theta = x[8] # rad

# Control inputs
da = u[1] # degrees - aileron
de = u[2] # degrees - stabilator
dr = u[3] # degrees - rudder
CG = u[4] # % MAC
T  = u[5] # % N

# Aircraft's physical parameters
S   = 27.87             # wing area in m2 (300 ft2)
c   = 3.4503            # mean aerodynamic chord in m (11.32 feet)
b   = 9.144             # wingspan in m (30 feet)
m   = 9300              # mass in kg (20,500 lbs)
Ix  = 12875             # kg/m2
Iy  = 75674             # kg/m2
Iz  = 85552             # kg/m2
g   = 9.81              # m/s2
rho = 0.458312441644953 # air density in kg/m^3 at 9,144 m (30,000 ft)

# Aerodynamics
PU   =  p*0.5*b/V
QU   =  q*0.5*c/V
RU   =  r*0.5*b/V
DLEF =  0 # leading edge deflection (fixed at 0 - do not change)
WLEF =  1 - DLEF/25

# 1D tables
CXQ   = CXQ_intp1(alpha*180/pi)
CYR   = CYR_intp1(alpha*180/pi)
CYP   = CYP_intp1(alpha*180/pi)
CZQ   = CZQ_intp1(alpha*180/pi)
DCLB  = DCLB_intp1(alpha*180/pi)
CLR   = CLR_intp1(alpha*180/pi)
CLP   = CLP_intp1(alpha*180/pi)
DCM   = DCM_intp1(alpha*180/pi)
CMQ   = CMQ_intp1(alpha*180/pi)
DCNB  = DCNB_intp1(alpha*180/pi)
DCNDA = DCNDA_intp1(alpha*180/pi)
CNR   = CNR_intp1(alpha*180/pi)
CNP   = CNP_intp1(alpha*180/pi)

# Cx
CXBAS = CXBAS_intp3(alpha*180/pi,beta*180/pi,de)
Cx = CXBAS + QU*CXQ;

# Cy
dCYBAS = dCYBAS_intp2(alpha*180/pi,beta*180/pi)
dvCYDA20 = dvCYDA20_intp2(alpha*180/pi,beta*180/pi)
dvCYDR30 = dvCYDR30_intp2(alpha*180/pi,beta*180/pi)
CY_Da    = (dvCYDA20-dCYBAS)/20
CY_Dr    = (dvCYDR30-dCYBAS)/30
Cy =   (dCYBAS + CY_Da*da)*(1-WLEF) + CY_Dr*dr + RU*CYR + PU*CYP

# Cz
CZBAS = CZBAS_intp3(alpha*180/pi,beta*180/pi,de)
Cz = CZBAS + QU*CZQ

# Cl
dvCLDA20 = dvCLDA20_intp2(alpha*180/pi,beta*180/pi)
dvCLDR30 = dvCLDR30_intp2(alpha*180/pi,beta*180/pi)
vCLDE0   = vCLDE0_intp2(alpha*180/pi,beta*180/pi)
CLBAS = CLBAS_intp3(alpha*180/pi,beta*180/pi,de)
C_l_Da      = (dvCLDA20-vCLDE0)/20
C_l_Dr      = (dvCLDR30-vCLDE0)/30
Cl = CLBAS + DCLB*beta*180/pi + C_l_Da*da + C_l_Dr*dr + RU*CLR + PU*CLP

# Cm
CMBAS = CMBAS_intp3(alpha*180/pi,beta*180/pi,de)
Cm = CMBAS + DCM + QU*CMQ + Cz*(0.35-CG/100)

# Cn
dvCNDA20 = dvCNDA20_intp2(alpha*180/pi,beta*180/pi)
dvCNDR30 = dvCNDR30_intp2(alpha*180/pi,beta*180/pi)
vCNDE0   = vCNDE0_intp2(alpha*180/pi,beta*180/pi)
CNBAS = CNBAS_intp3(alpha*180/pi,beta*180/pi,de)
C_n_Da      = (dvCNDA20-vCNDE0)/20
C_n_Dr      = (dvCNDR30-vCNDE0)/30
Cn = CNBAS + DCNB*beta*180/pi + C_n_Da*da+ DCNDA*da + C_n_Dr*dr + RU*CNR + PU*CNP - Cy*(0.35-CG/100)*c/b

# Equations of motions (wind axis)
# Transform body-axis forces to wind axes: 
CDrag = -cos(alpha)*cos(beta)*Cx - sin(beta)*Cy - sin(alpha)*cos(beta)*Cz
CSide = -cos(alpha)*sin(beta)*Cx + cos(beta)*Cy - sin(alpha)*sin(beta)*Cz
CLift =  sin(alpha)*          Cx                - cos(alpha)*          Cz

qbar = 0.5*rho*V^2

Drag = CDrag*qbar*S
Side = CSide*qbar*S
Lift = CLift*qbar*S
L    = Cl   *qbar*S*b
M    = Cm   *qbar*S*c
N    = Cn   *qbar*S*b

alphadot = -Lift*sec(beta)/m/V + 
            q -
           (p*cos(alpha) + r*sin(alpha))*tan(beta) +
            g/V*sec(beta)*(cos(phi)*cos(theta)*cos(alpha) + sin(theta)*sin(alpha)) -
           (T*sec(beta)/m/V)*sin(alpha)

betadot = Side/m/V +
          p*sin(alpha) -
          r*cos(alpha) +
          g/V*cos(theta)*sin(phi)*cos(beta) +
          g/V*sin(beta) * (cos(alpha)*sin(theta) - sin(alpha)*cos(phi)*cos(theta) + T*cos(alpha)/m/g)

Vdot = -Drag/m +
        g*(cos(phi)*cos(theta)*sin(alpha)*cos(beta) + sin(phi)*cos(theta)*sin(beta) - sin(theta)*cos(alpha)*cos(beta)) +
       (T/m)*cos(beta)*cos(alpha)
   
pdot = ((Iy - Iz)/Ix)*q*r + L/Ix
qdot = ((Iz - Ix)/Iy)*p*r + M/Iy
rdot = ((Ix - Iy)/Iz)*p*q + N/Iz

phidot   = p + q*tan(theta)*sin(phi) + r*tan(theta)*cos(phi)
thetadot =     q*cos(phi)            - r*sin(phi)

dx[1]  = alphadot
dx[2]  = betadot
dx[3]  = Vdot
dx[4]  = pdot
dx[5]  = qdot
dx[6]  = rdot
dx[7]  = phidot
dx[8]  = thetadot

end