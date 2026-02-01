# This base class is for open loop model only, the base system dynammics
# Will define equations of motion here

import numpy as np
from scipy.interpolate import interp1d
from scipy.interpolate import RectBivariateSpline
from numpy.typing import NDArray

class Base:
    """
    This class represents open loop model.
    """
    def __init__(self, aero_data, cbar=3.4503, S=27.87,g=9.81,rho = 0.458312441644953,RM = 1.076e-4,C7= 1.322e-5):
        """
        Initialising mode with aero look up tables and with constants.
        """
        self.aero_data = aero_data
        self.cbar = cbar
        self.S    = S
        self.g    = g
        self.rho  = rho
        self.RM   = RM
        self.C7   = C7
        self.Iyy = 1 / self.C7
        self.m   = 1 / self.RM
    def aero_model(self, x: NDArray, u: NDArray) -> tuple:
        """
        Method for evaluating aerodynamic coefficients from aero data
        ,current state and input.
        
        :param x: ndarray, state variables
        :param u: ndarray, contrtol input variables 

        :returns CX, CZ, CM: tuple, Aerodynamic coefficients
        """
        
        # ---------------- State variables ----------------
        alpha = x[0]   # rad
        v     = x[1]   # m/s
        q     = x[2]   # rad/s
        theta = x[3]   # rad

        # ---------------- Control inputs ----------------
        dstab = u[0]       # stabilator deg
        cg    = u[1]       # %MAC
        thrust     = u[2]       # thrust N

        # Derived quantities
        TVT = 0.5 / v
        CQ  = self.cbar * q * TVT
        QU  = CQ

        # angle to degrees for MATLAB-based tables
        alpha_deg = np.degrees(alpha)

        # ------------------- Retrieve aero tables -------------------

        AoA = self.aero_data['aero_AoA']
        AoA = AoA[0]
        De = self.aero_data['aero_De']
        De = De[0]
        D   = self.aero_data["aero_D"]                # 2D (leave as-is)

        # ------------- 1D interpolations (interp1d) -----------------
        CXQ = interp1d(AoA, D[0, :], kind='cubic', fill_value="extrapolate")(alpha_deg)
        CZQ = interp1d(AoA, D[3, :], kind='cubic', fill_value="extrapolate")(alpha_deg)
        DCM = interp1d(AoA, D[7, :], kind='cubic', fill_value="extrapolate")(alpha_deg)
        CMQ = interp1d(AoA, D[8, :], kind='cubic', fill_value="extrapolate")(alpha_deg)


        # ------------------- Stack coefficient tables -------------------
        CXtot = np.column_stack([
            self.aero_data["aero_CXDEM25"][:, 9],
            self.aero_data["aero_CXDEM10"][:, 9],
            self.aero_data["aero_CXDE0"][:, 9],
            self.aero_data["aero_CXDE10"][:, 9],
            self.aero_data["aero_CXDE25"][:, 9]
        ])

        CZtot = np.column_stack([
            self.aero_data["aero_CZDEM25"][:, 9],
            self.aero_data["aero_CZDEM10"][:, 9],
            self.aero_data["aero_CZDE0"][:, 9],
            self.aero_data["aero_CZDE10"][:, 9],
            self.aero_data["aero_CZDE25"][:, 9]
        ])

        CMtot = np.column_stack([
            self.aero_data["aero_CMDEM25"][:, 9],
            self.aero_data["aero_CMDEM10"][:, 9],
            self.aero_data["aero_CMDE0"][:, 9],
            self.aero_data["aero_CMDE10"][:, 9],
            self.aero_data["aero_CMDE25"][:, 9]
        ])
        CX_spline = RectBivariateSpline(AoA, De, CXtot, kx=3, ky=3)
        CZ_spline = RectBivariateSpline(AoA, De, CZtot, kx=3, ky=3)
        CM_spline = RectBivariateSpline(AoA, De, CMtot, kx=3, ky=3)
        # ------------------- 2D interpolated baseline values -------------------
        CXBAS = CX_spline(alpha_deg, dstab)[0, 0]
        CZBAS = CZ_spline(alpha_deg, dstab)[0, 0]
        CMBAS = CM_spline(alpha_deg, dstab)[0, 0]

        # ------------------- Aerodynamic coefficients -------------------
        CX = CXBAS + QU * CXQ
        CZ = CZBAS + QU * CZQ
        CM = CMBAS + DCM + QU * CMQ + CZ * (0.35 - cg / 100)


        return CX, CZ, CM
    
    def cm_deriv(self, x: NDArray, u: NDArray, eps: float) -> float:
        """
        Calculates the derivative of CM w.r.t dstab using basic finite difference
        at a given state and control inputs.
        
        :param x: the state 
        :type x: NDArray
        :param u: control inputs
        :type u: NDArray
        :param eps: finite difference step to use for numerical derivative
        :type eps: float
        :return: derivative of CM w.r.t dstab
        :rtype: float
        """
        
        dstab = u[0]
        dtsab1 = dstab+eps
        dstab2 = dstab-eps
        u1 = u.copy()
        u2 = u.copy()
        u1[0] = dtsab1
        u2[0] = dstab2
        _, _, CM1 = self.aero_model(x, u1)
        _, _, CM2 = self.aero_model(x, u2)

        dCM_ddstab = (CM1 - CM2) / (2.0 * eps)

        return dCM_ddstab
    
    def eom(self, x: NDArray, u: NDArray) ->NDArray:
        """
        Method for evaluating state derivatives given current state
        and control inputs.
        
        :param x: ndarray, state variables
        :param u: ndarray, control input variables

        :returns x_dot: ndarray, the state derivatives
        """
        # ---------------- Aero Coeffs --------------------
        CX, CZ, CM = self.aero_model(x,u)
        # ---------------- State variables ----------------
        alpha = x[0]   # rad
        v     = x[1]   # m/s
        q     = x[2]   # rad/s
        theta = x[3]   # rad

        # ---------------- Control inputs ----------------
        dstab = u[0]       # stabilator deg
        cg    = u[1]       # %MAC
        thrust     = u[2]       # thrust N
        # ------------------- Dynamics -------------------

        alphadot = q + (
            self.m * self.g * np.cos(theta - alpha)
            + 0.5 * self.rho * v**2 * self.S * (CZ * np.cos(alpha) - CX * np.sin(alpha))
            - thrust * np.sin(alpha)
        ) / (self.m * v)

        vdot = (
            -self.m * self.g * np.sin(theta - alpha)
            + 0.5 * self.rho * v**2 * self.S * (CX * np.cos(alpha) + CZ * np.sin(alpha))
            + thrust * np.cos(alpha)
        ) / self.m

        qdot = 0.5 * self.rho * v**2 * self.S * self.cbar * CM / self.Iyy
        thetadot = q

        return np.array([alphadot, vdot, qdot, thetadot])
    
class ClosedLoop(Base):
    """
    TThis class inherits from the open loop class and adds
    methods for applying controllers. For now only
    pitch controller has been implemented.
    """
    def __init__(self, aero_data):
        super().__init__(aero_data)  # Inheriting

    def dstab_actuator(self, max_rate, dstab_command, dstab_current) -> float:
        """
        Method for determing actuator rate for simulation, has to be given a max rate.
        So far assumes an actuator bandwidth
        
        :param max_rate: float, max actuator rate for dstab
        :param dstab_command: float, commanded dstab
        :param dstab_current: float, current dstab
        :return: dstab rate
        :rtype: float
        """
        
        k_act = 20.0      # actuator bandwidth (1/s) ?
        # max_rate = 60.0   # deg/s

        raw_rate = k_act * (dstab_command - dstab_current)
        dstab_rate = np.clip(raw_rate, -max_rate, max_rate)

        return dstab_rate

    def pitch_NDI(self, t, z, dstab_rate, theta_des, theta_dot_des=0, p_gain=1.0, d_gain=1.0):
        """
        Creates an augmented system for simulating the closed loop NDI controller.
        This method returns the augmented state derivatives which is needed for
        using scipy integrate functions I think. It also allows for modelling
        actuator rates by including their derivatives.
        
        :param t: float, time is unused currently but is needed for solver
        :param z: NDArray, augmented state includes control inputs
        :param dstab_rate: max dstab rate
        :param theta_des: desired theta in radians
        :param theta_dot_des: desired thete dot, I believe this is usually zero
        :param p_gain: NDI proportional gain
        :param d_gain: NDI derivative gain
        """
        # ---------------- State variables ----------------
        alpha = z[0]   # rad
        v     = z[1]   # m/s
        q     = z[2]   # rad/s
        theta = z[3]   # rad

        # ---------------- Control inputs ----------------
        dstab_current = z[4]       # stabilator deg
        cg    = 25       # %MAC
        thrust = 10000      # thrust N

        # ---------------- Linearisation point ----------------
        dstab_guess = dstab_current  # Using current dstab as point
        
        u_lin = np.array([dstab_guess, cg, thrust])
        # Find Aero coeff
        _, _, CM_lin = self.aero_model(z, u_lin)

        
        # Numerical derivative of CM w.r.t dstab
        dCM_ddstab = self.cm_deriv(z, u_lin, eps=0.1)

        # ---------------- NDI formulation ----------------
        common_factor = 0.5 * self.rho * v**2 * self.S * self.cbar / self.Iyy

        # qdot = f(x) + g(x) * dstab
        # f_x = common_factor * (CM - dCM_ddstab * dstab_guess)
        # g_x = common_factor * dCM_ddstab
        f_x = common_factor * (CM_lin - dCM_ddstab * dstab_guess)
        g_x = common_factor * dCM_ddstab
        
        # Desired qdot
        qdot_des = (
            -p_gain * (theta - theta_des)
            -d_gain * (q - theta_dot_des)
        )

        dstab = (qdot_des - f_x) / g_x
        # Saturation (lookup table bounds)
        dstab = np.clip(dstab, -25.0, 25.0)
        u_command = np.array([dstab, cg, thrust]) # New u commanded after control law is applied
        # ---------------- Final aero evaluation ----------------
        dstab_rate = self.dstab_actuator(dstab_rate, dstab, dstab_current)
        x = z[:4]
        dxdt = self.eom(x, u_lin)
        dzdt = np.zeros(5)
        dzdt[:4] = dxdt
        dzdt[4] = dstab_rate  #dstab rate is now included in augmented state

        return dzdt
