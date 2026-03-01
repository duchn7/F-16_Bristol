function [dCM_ddstab] = deriv(alpha, V, q, dstab, CG)
% this dstab is current dstab
% Numerically approximate dCM_ddstab using central difference 
eps = 0.1; % finite difference

[~, ~, CM1] = aero_model(alpha, V, q, dstab + eps, CG);
[~, ~, CM2] = aero_model(alpha, V, q, dstab - eps, CG);

dCM_ddstab = (CM1 - CM2) / (2 * eps);
end