function T = SRD_RotationMatrix3D_z(q)
T = [cos(q), -sin(q), 0;
    sin(q), cos(q),  0;
    0,      0,       1];
end