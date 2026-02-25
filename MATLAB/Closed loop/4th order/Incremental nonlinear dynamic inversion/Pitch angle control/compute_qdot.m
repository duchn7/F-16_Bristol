function qdot = compute_qdot(x, dstab)
    t = 0;
    CG = 25;
    T = 10000;
    u = [dstab; CG; T];
    dx = eom(t, x, u);
    qdot = dx(3);
end