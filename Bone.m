classdef Bone < matlab.mixin.Copyable
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        %Basic Properties
        Position
        Length
        jointAngle
        selfAngle 
        endPosition
        %Parent and Child Infomation
        ParentBone = []
        ChildBone = []
        nBones = 1
        %Physical Properties
        interpRate = 0.05 % radians per update
        controlPoint = 0.9 % where the bezier control points are
        maxAngle = 2*pi;
        minAngle = -(1.999)*pi;
    end
    methods
        %Bone Constructor
        function obj = Bone(Parent, Length,angle)
            obj.Length = Length;
            obj.ParentBone = Parent; 
            obj.jointAngle = angle;
            obj.selfAngle = obj.jointAngle;
            if Parent ~= 0
                obj.Position = Parent.endPosition;
                obj.ParentBone.ChildBone = obj;
            else
                obj.Position = [0,0,0];
            end
             obj.endPosition(1) = obj.Position(1) + (obj.Length * cos(obj.jointAngle));
             obj.endPosition(2) = obj.Position(2) + (obj.Length * sin(obj.jointAngle));
             obj.endPosition(3) = 0;
            
        end
        %calculates new end Position of Bone
        function calcEndPos(obj)
             obj.endPosition(1) = obj.Position(1) + (obj.Length * cos(obj.jointAngle));
             obj.endPosition(2) = obj.Position(2) + (obj.Length * sin(obj.jointAngle));
        end
        %Rotates bone dy dA radians
        function rotate(obj,dA)
            obj.selfAngle = obj.selfAngle + dA;
            obj.selfAngle = mod(obj.selfAngle,2*pi);
            if obj.selfAngle > pi
                %obj.selfAngle = obj.selfAngle - 2*pi;
            end
        end
        %update Base, End Location and Angle
        function update(obj)
            obj.jointAngle = obj.selfAngle;
             if ~isempty(obj.ParentBone)
                obj.Position = obj.ParentBone.endPosition;
                obj.jointAngle = obj.jointAngle + obj.ParentBone.jointAngle;
                obj.jointAngle = mod(obj.jointAngle,2*pi);
                if obj.jointAngle > pi
                %obj.jointAngle = obj.jointAngle - 2*pi;
                end
            end
            calcEndPos(obj);
        end
        %Animates Skeleton for given joint angles using linear Interpolation
        function forwardKin(obj,Angles)
            dA = findDiffernceAngle(obj,Angles(1));
            rate = obj.interpRate;
            while abs(dA)>0
                if dA - rate >rate
                    rotate(obj,rate)
                    dA = dA - rate;
                elseif -(dA-rate)>rate
                    rotate(obj,-rate)
                    dA = dA + rate;
                else
                    rotate(obj,dA);
                    dA = 0;
                end
                next = obj;
                while true
                    update(next)
                    if ~isempty(next.ChildBone)
                        next = next.ChildBone;
                    else
                        break
                    end
                end
                cla;
                drawFromEnd(next)
                drawnow
            end
            if ~isempty(obj.ChildBone)
                Angles = Angles(2:end);
                forwardKin(obj.ChildBone,Angles);
            end
        end
        
        function bezierForwardKin(obj,Angles)
            %dA = Angles(1) - obj.selfAngle;
            dA = findDiffernceAngle(obj,Angles(1));
            if abs(dA) > 0
                nSteps = ceil(abs(dA)/obj.interpRate);
                %Calculate Bezier Curve control points
                P0 = [0,obj.selfAngle];
                P1 = [obj.controlPoint,obj.selfAngle];
                P2 = [1-obj.controlPoint,obj.selfAngle + dA];
                P3 = [1,obj.selfAngle + dA];
                %Calculate Bezier Curve
                Q = bezierCurve(P0,P1,P2,P3,nSteps);
                dA = gradient(Q(:,2));
                for i=1:nSteps-1
                    rotate(obj,dA(i));
                    next = obj;
                    while true
                        update(next)
                        if ~isempty(next.ChildBone)
                            next = next.ChildBone;
                        else
                            break
                        end
                    end
                    cla;
                    drawFromEnd(next)
                    drawnow
                end
            end
            if ~isempty(obj.ChildBone)
                Angles = Angles(2:end);
                bezierForwardKin(obj.ChildBone,Angles);   
            end
        end
        %Finds to End Effector Position for given Joint Angles (Forward Kin with out Animation)
        function pos = findEnd(obj, Angles)
            %obj.selfAngle = Angles(1);
            dA = findDiffernceAngle(obj,Angles(1));
            obj.selfAngle = obj.selfAngle + dA;
            next = obj;
            while true
                update(next)
                if ~isempty(next.ChildBone)
                    next = next.ChildBone;
                else
                    break
                end
            end
            if ~isempty(obj.ChildBone)
                Angles = Angles(2:end);
                pos = findEnd(obj.ChildBone,Angles);   
            else
                pos = obj.endPosition;
            end
        end
        function pos = setMid(obj)
            obj.selfAngle = (obj.maxAngle + obj.minAngle)/2;
            next = obj;
            while true
                update(next)
                if ~isempty(next.ChildBone)
                    next = next.ChildBone;
                else
                    break
                end
            end
            if ~isempty(obj.ChildBone)
                pos = setMid(obj.ChildBone);   
            else
                pos = obj.endPosition;
            end
        end
        %Draws the Bone and its parents
        function drawFromEnd(obj)
            line([obj.Position(1); obj.endPosition(1)],[obj.Position(2); obj.endPosition(2)],'linewidth',3)
            plot(obj.Position(1),obj.Position(2),'-o','MarkerFaceColor', 'r');
            if ~isempty(obj.ParentBone)
                drawFromEnd(obj.ParentBone)
            end
        end
        function drawFromRoot(obj)
            line([obj.Position(1); obj.endPosition(1)],[obj.Position(2); obj.endPosition(2)],'linewidth',3)
            plot(obj.Position(1),obj.Position(2),'-o','MarkerFaceColor', 'r');
            if ~isempty(obj.ChildBone)
                drawFromRoot(obj.ChildBone)
            end
        end
        % Does Inverse Kinematic on Bone
        function [Angles,err] = inverseKin(obj,Goal)
            a = [0,0,1];
            invBone = CopyBone(obj);
            %Set bones to thier mid point angles 
            e = setMid(invBone);
          
            
            %e = findEnd(invBone,);
            deltaE = norm(Goal-e)^2;
            stepLength = 0.1;
            Angles = getAngles(invBone);
            itt = 0;

            while deltaE > 0.001

                next = invBone;
                %J = [];
                J = zeros(3,obj.nBones);
                i = 1 ;
                while true

                    b = e-next.Position;
                    J(:,i) = cross(a,b);
                    i = i +1;
                    if ~isempty(next.ChildBone)
                        next = next.ChildBone;
                    else
                        break
                    end
                end
                pInvJ = pinv(J);
                Jt = J';
                dTheta = Jt * (Goal-e)';

                Angles = Angles + stepLength*dTheta';
                itt = itt +1;
                %forwardKin(obj,Angles);
                e = findEnd(invBone,Angles);
                if norm(Goal-e)^2 >= deltaE
                    stepLength = stepLength * 0.5;
                end 
                diffE = abs(deltaE - norm(Goal-e)^2);
                if diffE < 0.0001
                    break
                end
                deltaE = norm(Goal-e)^2;
    
            end
            %itt
            Angles = mod(Angles,2*pi);
            %Angles(Angles>pi) = Angles(Angles>pi) - 2*pi;
            err = deltaE;
            forwardKin(obj,Angles);
        end
        function EE = getCurrentEE(obj)
            next = obj;
            while true
                EE = next.endPosition;
                if ~isempty(next.ChildBone)
                    next = next.ChildBone;
                else
                    break
                end
            end
        end
        function Angles = getAngles(obj)
            if ~isempty(obj.ChildBone)
                Angles = cat(2,obj.selfAngle,getAngles(obj.ChildBone));
            else
                Angles = obj.selfAngle;
            end
        end
        function resetBone(bone)
            Angles = zeros(1,bone.nBones);
            forwardKin(bone,Angles);
        end
        function dA = findDiffernceAngle(obj,Angles)
            dA = Angles(1) - obj.selfAngle;
            dA = mod(dA,2*pi); %Positive dA
            dA_neg = dA - 2*pi;
            toMax = obj.maxAngle - obj.selfAngle; %Angle needed to Rotate bone its Max Angle
            if toMax < 0
                toMax = 2*pi + toMax;
            end
            toMin = mod(obj.minAngle - obj.selfAngle,2*pi); %Angle needed to Rotate bone its Min Angle
            if toMin > 0 
                toMin = toMin - 2*pi;
            end
            
           
            if dA < toMax && dA_neg > toMin
            %Can reasch Angle with +ve and -ve rotatior choose smallest
                if abs(dA_neg) < abs(dA)
                    dA = dA_neg;
                end
            elseif dA < toMax && dA_neg <= toMin
            %Can only reach desired Angle with positive rotation()
            elseif dA >= toMax && dA_neg > toMin
            %Can reach desired Angle with negative rotation
                dA = dA_neg;
            elseif dA >= toMax && dA_neg <= toMin
            %Desired Angle is out of range to go to closest
                dMaxAngle = abs(mod(Angles,2*pi) - mod(obj.maxAngle,2*pi));
                dMinAngle = abs(mod(Angles,2*pi) - mod(obj.minAngle,2*pi));
                if dMaxAngle < dMinAngle 
                    dA = toMax;
                else
                    dA = toMin;
                end
            end
        end
    end
    
    
end

