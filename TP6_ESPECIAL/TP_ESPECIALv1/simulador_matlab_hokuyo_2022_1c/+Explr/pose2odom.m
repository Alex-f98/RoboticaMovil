function odom = pose2odom(pose)
%odom= [dr1, dtras, dr2]
%pose= [x; y; theta];
persistent old_pose;
   
if isempty(old_pose)
    old_pose = zeros(size(pose));
end

%delta_tralacion= sqrt( (x'-x)^2 + (y'-y)^2 )
dtras = sqrt( (pose(1) - old_pose(1)).^2 + (pose(2) - old_pose(1)).^2 );

%delta_rot1= atan2( y'-y, x'-x )
dr1 = atan2( pose(2) - old_pose(2) , pose(1) - old_pose(1));

%delta_rot2= theta' - theta - delta_rot1
dr2 = pose(3) - old_pose(3) - dr1;
    
old_pose= pose;
odom= [dr1, dtras, dr2];
end