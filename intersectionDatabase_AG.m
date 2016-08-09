function [fArray]=intersectionDatabase_AG()
% intersectionDatabase - pre-computes the fraction of a polytopes that
% intersects with another polytope of same size, but different position and
% and orientation
%
% Syntax:  
%    [P]=intersectionDatabase(obj)
%
% Inputs:
%    obj - road object
%
% Outputs:
%    fArray - 3-dimensional fraction array
%
% Example: 
%
% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
%
% See also: none

% Author:       Anurag Goyal, Matthias Althoff
% Written:      25-July-2013
%------------- BEGIN CODE --------------
tic

% size of ego vehicle
ego_length = 5;
ego_width = 2;

% centre of ego vehicle
ego_centre = [0,0];

% size of other traffic participant
other_length = 4;
other_width = 2;

% coordinates of ego vehicle
ego_xCoor = [ego_centre(1)+(ego_length/2),ego_centre(1)-(ego_length/2),ego_centre(1)-(ego_length/2),ego_centre(1)+(ego_length/2)];
ego_yCoor = [ego_centre(2)+(ego_width/2),ego_centre(2)+(ego_width/2),ego_centre(2)-(ego_width/2),ego_centre(2)-(ego_width/2)];

% coordinates of obstacle in counter-clockwise direction
other_xCoor = [(other_length/2) -(other_length/2) -(other_length/2) (other_length/2)];
other_yCoor = [(other_width/2) (other_width/2) -(other_width/2) -(other_width/2)];

% radius of enclosing circle of ego vehicle
r = sqrt((ego_yCoor(1) * ego_yCoor(1)) + (ego_xCoor(1) * ego_xCoor(1)));

% gridded area
xInterval = [-r-(other_length/2), r+(other_length/2)];
yInterval = [-r-(other_length/2), r+(other_length/2)];
orientInterval = [0,pi];

% nr of segments
nrOfxSegments = 30;
nrOfySegments = 30;
nrOforientSegments = 60;

% cell size
cell_x = (xInterval(2) - xInterval(1))/nrOfxSegments;
cell_y = (yInterval(2) - yInterval(1))/nrOfySegments;
cell_orient = (orientInterval(2) - orientInterval(1))/nrOforientSegments;

% initial value of y
y = yInterval(1);

for index_y = 1:nrOfySegments+1
    % initial value of x
    x = xInterval(1);
    
    for index_x = 1:nrOfxSegments+1
        % initial value of theta
        theta = orientInterval(1);
        
        for index_orient = 1:nrOforientSegments+1       
            orient = theta;
            
            % bounding orient in the interval [0,pi/2]
            if orient < (pi/2 + 0.00001)
            else
                orient = pi - orient;
            end                    
            ct = cos(orient);
            st = sin(orient);
            % rotation matrix
            R = [ct,-st;st,ct];                
            % calculate the uncertainity of position
            rot = R * [other_xCoor;other_yCoor];
            deltax = max(abs(rot(1,:) - other_xCoor));
            deltay = max(abs(rot(2,:) - other_yCoor));
            % Approximation of the coordinates

            other_xCoor_app = [other_xCoor(1)+(deltax*(ct))+deltay*(st),other_xCoor(2)-(deltax*(ct)+deltay*(st)),...
                                other_xCoor(3)-(deltax*(ct)+deltay*(st)),other_xCoor(4)+(deltax*(ct)+deltay*(st))];        % adding deltax
            other_yCoor_app = [other_yCoor(1)+(deltay*(ct)+deltax*(st)),other_yCoor(2)+(deltay*(ct)+deltax*(st)),...
                                other_yCoor(3)-(deltay*(ct)+deltax*(st)),other_yCoor(4)-(deltay*(ct)+deltax*(st))];        % adding deltay
            % rotating the other participant by theta
            ct = cos(theta);
            st = sin(theta);
            % rotation matrix
            R = [ct,-st;st,ct];                 
            rot1 = R * [other_xCoor_app;other_yCoor_app];
            % shifting the coordinates of the other participant 
            other_x = rot1(1,:) + x;
            other_y = rot1(2,:) + y;
            poly1 = [ego_xCoor(1),ego_yCoor(1); ego_xCoor(2),ego_yCoor(2); ego_xCoor(3),ego_yCoor(3); ego_xCoor(4),ego_yCoor(4)];
            poly2 = [other_x(1),other_y(1);other_x(2),other_y(2);other_x(3),other_y(3);other_x(4),other_y(4)];  
            % plotting the other participant and ego vehicle using convex hull
            asdx = poly1(:,1);
            asdy = poly1(:,2);
            adx = poly2(:,1);
            ady = poly2(:,2);
            k = convhull(asdx,asdy);
            plot(asdx(k),asdy(k))
            hold on;
            lk=convhull(adx,ady);
            plot(adx(lk),ady(lk),'g');
            % increment orient value
            theta = theta + cell_orient;      
            % generating the database
            database(index_y,index_x,index_orient) = separateAxisTheorem(poly1,poly2);
        end
        %increment x value
        x = x + cell_x;
    end
    %increment y value
    y = y + cell_y;
end
% saving the data
data.egolength = ego_length;
data.egowidth = ego_width;
data.obstacleLength = other_length;
data.obstacleWidth = other_width;
data.nrOfxSegments = nrOfxSegments;
data.nrOfySegments = nrOfySegments;
data.nrOforientSegments = nrOforientSegments;
data.xInterval = xInterval;
data.yInterval = yInterval;
data.orientInterval = orientInterval;
data.cellx = cell_x;
data.celly = cell_y;
data.cellAngle = cell_orient;
data.nrOfAngleSegments = nrOforientSegments;
save('data.mat','data');
save('database.mat','database');
toc

% tic
% % cell size
% cellx = 0.5; %m
% celly = 0.5; %m
% nrOfAngleSegments=72;
% cellAngle=pi/nrOfAngleSegments; %rad        
% deltaorient = pi/nrOfAngleSegments;
% i = 1;                  %for y coordinates
% 
% for index_y = round(-r-(carWidth/2)) : celly : round(r+(carWidth/2))
%     p = 1;               %for x coordinates
%     
%     for index_x = round(-r-(carLength/2)) : cellx : round(r+(carLength/2))
%         theta = 0;
%         iAngle = 0;
%         
%         for index_Angle = 1 : cellAngle : nrOfAngleSegments/2
%             xPos = index_x;
%             yPos = index_y;
%             theta=theta+(deltaorient);
%             ct = cos(theta);
%             st = sin(theta);
%             R = [ct,-st;st,ct];
%             %coordinates of obstacle in counter-clockwise direction
%             s = [(carLength/2) -(carLength/2) -(carLength/2) (carLength/2)];
%             t = [(carWidth/2) (carWidth/2) -(carWidth/2) -(carWidth/2)];
%             deltax = [abs(s(1)*(1-cos(theta))-t(1)*sin(theta)) abs(s(2)*(1-cos(theta))-t(2)*sin(theta)) abs(s(3)*(1-cos(theta))-t(3)*sin(theta)) abs(s(4)*(1-cos(theta))-t(4)*sin(theta))];
%             deltay = [abs(s(1)*sin(theta)+t(1)*(cos(theta)-1)) abs(s(2)*sin(theta)+t(2)*(cos(theta)-1)) abs(s(3)*sin(theta)+t(3)*(cos(theta)-1)) abs(s(4)*sin(theta)+t(4)*(cos(theta)-1))];
%             sss = [s(1)+(deltax(1)*(ct))+deltay(1)*(st) s(2)-(deltax(2)*(ct)+deltay(2)*(st)) s(3)-(deltax(3)*(ct)+deltay(3)*(st)) s(4)+(deltax(4)*(ct)+deltay(1)*(st))];        % adding deltax
%             ttt = [t(1)+(deltay(1)*(ct)+deltax(1)*(st)) t(2)+(deltay(2)*(ct)+deltax(2)*(st)) t(3)-(deltay(3)*(ct)+deltax(3)*(st)) t(4)-(deltay(4)*(ct)+deltax(4)*(st))];        % adding deltay
%             if iAngle ~= 0
%                 ssss = R * [sss;ttt]; 
%                 ss = ssss(1,:)+ xPos;         % x coordinates of obstacle
%                 tt = ssss(2,:)+ yPos;         % y coordinates of obstacle
%              else
%                 ss = s + xPos;                  % x coordinates of obstacle
%                 tt = t + yPos;                  % y coordinates of obstacle
%             end
%            
%             
%             S = [ss(1) ss(2) ss(3) ss(4)];
%             T = [tt(1) tt(2) tt(3) tt(4)];
% %             c1 = IHcenter;                      % centre of ego vehicle
% %             c2 = [xPos,yPos];                   % centre of obstacle
%             poly1 = [x(1),y(1); x(2),y(2); x(3),y(3); x(4),y(4)];
%             poly2 = [ss(1),tt(1);ss(2),tt(2);ss(3),tt(3);ss(4),tt(4)];       
%             % plotting the obstacle and ego vehicle
%             asdx = poly1(:,1);
%             asdy = poly1(:,2);
%             adx = poly2(:,1);
%             ady = poly2(:,2);
%             hold on
%             k = convhull(asdx,asdy);
%             plot(asdx(k),asdy(k))
%             lk=convhull(adx,ady);
%             plot(adx(lk),ady(lk),'g');
% %             l = convhull(adx,ady);
% %             plot(adx(l),ady(l),'r')
%             
%             database(i,p,index_Angle) = separateAxisTheorem(poly1,poly2);
%             iAngle=iAngle+1;
%             close
% 
%             
%            
%         end
%         p = p+1;  
%     end
%     i = i+1;
%     
% end
% for iAngleSeg=1:nrOfAngleSeg
%     for iXseg=1:nrOfxSeg
%         iAngleSeg
%         iXseg
%         for iYseg=1:nrOfySeg
%             obtain x,y translations and angle rotations
%             xTrans=(iXseg-1)*deltaX;
%             yTrans=(iYseg-1)*deltaY;
%             angleTrans=(iAngleSeg-1)*deltaAngle;
%             
%             generate rotation matrix
%             Rot=[cos(angleTrans) -sin(angleTrans);...
%                  sin(angleTrans) cos(angleTrans)];
%              
%             modify second polytope
%             P2mod=Rot*P2+[xTrans;yTrans];
%             
%             compute grid of relative positions within the uncertain center
%             regions
%             relativePos=relativeGridPoints(IHcenter,Rot,[0;0],4);
%             nrOfRelativePos=max(size(relativePos(:,1)));
%             
%             h=figure;
%             hold on
%             
%             for iRelPos=1:nrOfRelativePos
%          
%                 intersect P and modified P2
%                 PcarInt=Pcar & (P2mod+relativePos(iRelPos,:)');
%                 PbicycleInt=Pbicycle & (P2mod+relativePos(iRelPos,:)');
%                 
%                 plot(P2mod+relativePos(iRelPos,:)');
% 
%                 check if polytopes intersected
%                 car
%                 carInt(iRelPos) = ~isempty(PcarInt);
%                 bicycle
%                 bicycleInt(iRelPos)=~isempty(PbicycleInt);  
%             end
%             plot(Pcar,[1 2],'b');
%             close(h);
%             
%             save result
%             intersectionCar(iXseg,iYseg)=length(find(carInt))/nrOfRelativePos;
%             intersectionBicycle(iXseg,iYseg)=length(find(bicycleInt))/nrOfRelativePos;
%         end
%     end
    %save results
%     fArray.val.car{iAngleSeg}=sparse(intersectionCar);
%     fArray.val.bicycle{iAngleSeg}=sparse(intersectionBicycle); 
% end


%store segment lengths
%  fArray.segLength.angle=deltaAngle;
% fArray.segLength.x=deltaX;
% fArray.segLength.y=deltaY;
% %store radii
% fArray.radius.r.car=rCar;
% fArray.radius.r.bicycle=rBicycle;
% fArray.radius.rComb.car=rCarComb;
% fArray.radius.rComb.bicycle=rBicycleComb;
% 
% %store IHvehicle
% fArray.IH.car=Pcar;
% fArray.IH.ego=Pego;
% 
%save intersection database
% [file,path] = uiputfile('*.mat','Save Intersection Database As');
% cd(path);
% save(file,'fArray');

%------------- END OF CODE --------------