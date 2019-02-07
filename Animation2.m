%Set Up intital Bones
angle = 0;
length = 10;
root = Bone([], length,angle);

%Bone1 = Bone(root, 5,pi/4);
%Bone2 = Bone(Bone1, 5,-pi/4);

current = root;
for i=0:2 
    length = length * 0.8;
    next = Bone(current, length, angle);
    current = next;
    root.nBones = root.nBones + 1;
end


Angles = [pi/2,pi/4,-3*pi/4,pi/3];
g1 = [-10,-20,0];
g2 = [0,-20,0];
g3 = [10,-20,0];
g4 = [-10,0,0];
g5 = [-10,20,0];

figure
hold on 
axis equal
axis([-30 30 -30 30]);

%resetBone(root);
%findBoundary(root,pi/5)
%forwardKin(root,[(3/2)*pi,0,0,0]);
%forwardKin(root,Angles+Angles+Angles);

%inverseKin(root,g1);
%inverseKin(root,g2);
%inverseKin(root,g3);

%root.interpRate = 0.1;

%resetBone(root);
%forwardKin(root,[(3/2)*pi,0,0,0]);

%bezierForwardKin(root,[pi,pi,pi,pi]);

%resetBone(root);
%root.controlPoint = 0.6;
%bezierForwardKin(root,[pi,pi,pi,pi]);


% Bone with angle Limitations

limitBone0 = Bone([], 10,0);
limitBone0.nBones = 3;
limitBone0.maxAngle = pi;
limitBone0.minAngle = 0;

limitBone1 = Bone(limitBone0, 5,0);

limitBone1.maxAngle = 3/4 * pi;
limitBone1.minAngle = - 3/4 * pi;

limitBone2 = Bone(limitBone1, 5,0);

limitBone2.maxAngle = 1/3 * pi;
limitBone2.minAngle = - 1/3 * pi;

%resetBone(limitBone0);
%findBoundary(limitBone0, pi/4);
%forwardKin(limitBone0,[(3/2)*pi,0,0,0]);
%bezierForwardKin(limitBone0,[pi,pi/2,pi/3,pi/4]);

%inverseKin(limitBone0,[-10,-10,0]);
%inverseKin(limitBone0,[10,10,0]);











