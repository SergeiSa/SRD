function T = SRD_RotationMatrix3D_y(q)
T = [cos(q),  0, sin(q);
    0,       1, 0;
    -sin(q), 0, cos(q)];
end