function [posit,k] = findBoundary(obj,acc)
   skel = CopyBone(obj);
   posit = findPoints(obj.nBones,[],skel,acc);
   k = boundary(posit(:,1),posit(:,2),0.1); 
   
   figure
   hold on 
   axis equal
   axis([-30 30 -30 30]);
   
   plot(posit(:,1),posit(:,2),'.','MarkerEdgeColor', 'g');
   plot(posit(k,1),posit(k,2));
   drawFromRoot(obj);
end

function posit = findPoints(numBones,Angles,obj,acc)
     posit = [];
     if numBones == 0
         posit = findEnd(obj,Angles);
         return
     else
         if(obj.maxAngle - obj.minAngle >2*pi)
             for i = 0:acc:2*pi-acc
                 p = findPoints(numBones-1,[Angles,i],obj,acc);
                 posit = cat(1,posit,p);
             end
         else
             for i = obj.minAngle:acc:obj.maxAngle
                 p = findPoints(numBones-1,[Angles,i],obj,acc);
                 posit = cat(1,posit,p);
             end
         end
     end  
end

