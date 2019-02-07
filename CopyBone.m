function cp = CopyBone( bone )

b = Bone([], bone.Length,bone.jointAngle);
b.maxAngle = bone.maxAngle ;
b.minAngle = bone.minAngle;
%Bone1 = Bone(root, 5,pi/4);
%Bone2 = Bone(Bone1, 5,-pi/4);

current = b;
while ~isempty(bone.ChildBone) 
    bone = bone.ChildBone;
    next = Bone(current, bone.Length, bone.jointAngle);
    next.selfAngle = bone.selfAngle;
    next.maxAngle = bone.maxAngle ;
    next.minAngle = bone.minAngle;
    current = next;
end

cp = b;


end

