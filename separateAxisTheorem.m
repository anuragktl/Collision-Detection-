 function flag = separateAxisTheorem (rectA, rectB)
 % Implementation of Separating Axis theorem to check for intersection of
 % two rectangles

 %rectA -> 4 rows of 2 columns each... eg. rectA = [2 1; 7 1; 7 5; 2 5]
 %rectB -> 4 rows of 2 columns each... eg. rectB = [7 1; 10 1; 10 5; 7 5]
% creating a zero vector 
overlapflag = zeros (7,1);   
% calculating the normals along the faces of rectangles
m1 = [((rectA(1,2)-rectA(2,2))/(rectA(1,1)-rectA(2,1))) ((rectA(2,2)-rectA(3,2))/(rectA(2,1)-rectA(3,1))) ...
    ((rectB(1,2)-rectB(2,2))/(rectB(1,1)-rectB(2,1))) ((rectB(2,2)-rectB(3,2))/(rectB(2,1)-rectB(3,1))) ((rectB(3,2)-rectB(4,2))/(rectB(3,1)-rectB(4,1))) ((rectB(4,2)-rectB(1,2))/(rectB(4,1)-rectB(1,1)))];  
m = (1./m1);
for j=1:6
 % projrectA and projrectB hold the projections of each of the vertices of rectA and rectB onto the axis y=mx
 % y = mx is an axis which pases through the origin
 % (x1,y1) is the projection of the point (x0,y0) on the axis y=mx
 % x1 = (m.y0 + x0) / (m^2 + 1)
 % y1 = (m^2.y0 + m.x0) / (m^2 + 1)	
	% Calculating projrectA
	for i = 1:4
		% x1
		projrectA (i,1) = (m(j)* rectA (i,2) + rectA (i,1)) / (m(j)^2 + 1);
		
		%y1
		projrectA (i,2) = ((m(j)^2) * rectA (i,2) + m(j) * rectA (i,1)) / (m(j)^2 + 1);
	end
		
	% Calculating projrectB
	for i = 1:4
		% x1
		projrectB (i,1) = (m(j) * rectB (i,2) + rectB (i,1)) / (m(j)^2 + 1);
		
		%y1
		projrectB (i,2) = ((m(j)^2) * rectB (i,2) + m(j) * rectB (i,1)) / (m(j)^2 + 1);
	end
	
	%Calculating the maximum and minimum X co-ord for the two rectangles
	xMinRectA = min (projrectA(:,1));
	xMaxRectA = max (projrectA(:,1));
	
	xMinRectB = min (projrectB(:,1));
	xMaxRectB = max (projrectB(:,1));
    
    % check for overlap
    if ((xMaxRectB < xMinRectA) || (xMaxRectA < xMinRectB ))
		%no overlap
		overlapflag(j) = 1;	
	else	
		overlapflag(j) = 0;
    end		
 end
 
 if any(overlapflag) == 1
	flag = 0;
 else
    flag = 1;
 end