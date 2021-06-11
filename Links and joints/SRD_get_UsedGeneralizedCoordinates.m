function q_used = SRD_get_UsedGeneralizedCoordinates(Link, q)

if isa(q, 'sym') || isa(q, 'double')
    q_used = diag(sign(Link.Joint.UsedGeneralizedCoordinates)) * ...
            reshape(q(abs(Link.Joint.UsedGeneralizedCoordinates)), [], 1);
else
    q_used = diag(sign(Link.Joint.UsedGeneralizedCoordinates)) * ...
            q(abs(Link.Joint.UsedGeneralizedCoordinates));
end