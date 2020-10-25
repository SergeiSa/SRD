function SRD_ForwardKinematics_JointUpdate_RelativeOrientationType(Link)

Link.AbsoluteBase = Link.ParentLink.AbsoluteFollower(:, Link.ParentFollowerNumber);
Link.AbsoluteOrientation = Link.ParentLink.AbsoluteOrientation * Link.RelativeOrientation;

rBaseToFollower = Link.RelativeFollower - repmat(Link.RelativeBase, 1, size(Link.RelativeFollower, 2));
rBaseToCoM      = Link.RelativeCoM - Link.RelativeBase;

Link.AbsoluteFollower = repmat(Link.AbsoluteBase, 1, size(Link.RelativeFollower, 2)) + Link.AbsoluteOrientation*rBaseToFollower;
Link.AbsoluteCoM = Link.AbsoluteBase + Link.AbsoluteOrientation*rBaseToCoM;
end