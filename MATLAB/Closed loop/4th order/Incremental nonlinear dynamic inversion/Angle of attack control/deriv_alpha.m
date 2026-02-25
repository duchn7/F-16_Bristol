function [dCX_ddstab,dCZ_ddstab,dCM_ddstab] = deriv_alpha(alpha, V, q, dstab, CG)
% this dstab is current dstab
% Numerically approximate dCM_ddstab using central difference 
eps = 0.1; % finite difference

[CX1,CZ1,CM1] = aero_model(alpha, V, q, dstab + eps, CG);
[CX2,CZ2,CM2] = aero_model(alpha, V, q, dstab - eps, CG);

dCX_ddstab = (CX1 - CX2) / (2 * eps);
dCZ_ddstab = (CZ1 - CZ2) / (2 * eps);
dCM_ddstab = (CM1 - CM2) / (2 * eps);
end