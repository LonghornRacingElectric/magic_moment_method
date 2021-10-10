import math
import numpy as np

#paramaters
ClA_tot = 3.955
CdA0    = 0.7155
CdA_tot = 1.512 - CdA0
CsA_tot = 0         #don't know this yet

#distribution of downforce across components
ClA_dist = [0.371, 0.282, 0.347]   #[front, undertray, rear]
CdA_dist = [0.324, 0.186, 0.490]
CsA_dist = [.25, 0, .75]

#pitch, bodyslip, and roll sensitivities
p_sens	= [[.0126, -.0418], [.0466, -.0496], [0,0]]	  # [%/deg], [Cl, Cd, Cs] -> [positive, negative]
bs_sens	= [.00446,  .0019,  0]					      # [%/deg]
r_sens	= [-.0074, -.00404, 0]					      # [%/deg]

in_to_m = 0.0254 #conversion factor

#positions of component CoPs (magnitudes, equation takes signs into account)
front_CoP =     [23.65 * in_to_m, 9.30 * in_to_m]
undertray_CoP = [43.5 * in_to_m,  7.13 * in_to_m]
rear_CoP =      [67.6 * in_to_m, 42.91 * in_to_m]

def calc_CoP(v, bodyslip, pitch, roll, rideheight):

    #gets aero coefficients for each component
    ClA = [ClA_tot * ClA_dist[0], ClA_tot * ClA_dist[1], ClA_tot * ClA_dist[2]]
    CdA = [CdA_tot * CdA_dist[0], CdA_tot * CdA_dist[1], CdA_tot * CdA_dist[2]]
    CsA = [CsA_tot * CsA_dist[0], CsA_tot * CsA_dist[1], CsA_tot * CsA_dist[2]]

    Fx = []
    Fy = []
    Fz = []

    p_dir = pitch <= 0

    #iterates for each aero component
    for i in range (0, 3):

        #sensitivities for Cl, Cd, and Cs
        Cl_sens = (1 + bs_sens[0] * bodyslip) * (1 + p_sens[0][p_dir] * pitch) * (
                1 + r_sens[0] * roll)
        Cd_sens = (1 + bs_sens[1] * bodyslip) * (1 + p_sens[1][p_dir] * pitch) * (
                1 + r_sens[1] * roll)
        Cs_sens = (1 + bs_sens[2] * bodyslip) * (1 + p_sens[2][p_dir] * pitch) * (
                1 + r_sens[2] * roll)

        #Cl, Cd, and Cs for the compenent
        ClA_part = ClA[i] * Cl_sens
        CdA_part = CdA[i] * Cd_sens
        CsA_part = CsA[i] * Cs_sens


        Fl_part = 0.5 * 1.225 * ClA_part * v ** 2
        Fd_part = 0.5 * 1.225 * CdA_part * v ** 2
        Fs_part = 0.5 * 1.225 * CsA_part * (v * math.tan(bodyslip)) ** 2

        Fx.append(Fd_part)
        Fy.append(Fs_part)
        Fz.append(Fl_part)


    drag_no_aero = 0.5 * 1.225 * CdA0 * v ** 2
    Fx.append(drag_no_aero)

    F_tot = np.array([sum(Fx), sum(Fy), sum(Fz)])
    moment_total = -Fz[0]*front_CoP[0] + Fx[0]*front_CoP[1] + Fz[1]*undertray_CoP[0] \
        + Fx[1]*undertray_CoP[1] + Fz[2]*rear_CoP[0] + Fx[2]*rear_CoP[1]

    return F_tot, moment_total

results = calc_CoP(15, 0, 0, 0, 0)

forces = results[0]
moment = results[1]

np.set_printoptions(suppress = True)
np.set_printoptions(precision = 2)

print("Total forces [x,y,z]: {:.2f} N, {:.2f} N, {:.2f} N".format(forces[0], forces[1], forces[2]))
print("Total moment        : {:.2f} Nm".format(moment))
