function value = colission_check( x,y,theta,data,database )
% colission_check Summary of this function goes here
% x and y are the relative coordinates of centre of other participant with respect to ego vehicle.
% theta is the relative orientation of other participant with respect to ego vehicle.
% number of x segments
nrOfxSegments = data.nrOfxSegments;
% number of y segments
nrOfySegments = data.nrOfySegments;
% number of angle segments
nrOforientSegments = data.nrOforientSegments;
% cell sizes
cell_x = data.cellx;
cell_y = data.celly;
cell_orient = data.cellAngle;
% gridded area
xInterval = data.xInterval;
yInterval = data.yInterval;
orientInterval = data.orientInterval;

%   Detailed explanation goes here

clc;
% checking whether the centre of the other participant lies in the gridded area    
if (xInterval(1) < x || xInterval(1) == x) && (x < xInterval(2) || xInterval(2) == x)
    if (yInterval(1) < y || yInterval(1) == y) && (y < yInterval(2) || yInterval(2) == y)
        % initialising y1
        y1 = yInterval(1);
        for index_y = 1:nrOfySegments+1
            % initial value of x1
            x1 = xInterval(1);
            if y > y1-0.00001 && y < y1+0.00001
                i = index_y;
                for index_x = 1:nrOfxSegments+1
                    % initial value of orient
                    orient = orientInterval(1);
                    if x > x1-0.00001 && x < x1+0.00001
                        p = index_x;   
                        theta = round(theta/cell_orient)*(cell_orient);
                        for index_orient = 1:nrOforientSegments+1 
                            % when orient becomes equal to theta
                            if theta > orient-0.00001 && theta < orient+0.00001
                                angle = index_orient;
                                value = database(i,p,angle);
                                break;
                            end
                            % increment orient
                            orient = orient + cell_orient;
                        end                        
                    end
                    %increment x value
                    x1 = x1 + cell_x;
                end
            end
             %increment y value
            y1 = y1 + cell_y;           
        end 
    else        
        value = 0;
    end
else    
    value = 0;
end



%                         if ((theta > -(2*pi)+0.00001) && (theta < (-1.5*pi)-0.00001))
%                             theta = (1.5*pi)+theta;
%                             theta = round((theta)/cell_orient)*(cell_orient);
%                             for index_orient = 1:nrOforientSegments+1                                        
%                                 if theta > orient-0.00001 && theta < orient+0.00001
%                                     angl = index_orient;
%                                 end
%                                 orient = orient + cell_orient;
%                             end
%                         end
%                         if ((theta > -1.5*pi) && (theta < (-pi/2)+0.00001))
%                             theta = pi+theta;
%                             theta = round((theta)/cell_orient)*(cell_orient);
%                             for index_orient = 1:nrOforientSegments+1                                        
%                                 if theta > orient-0.00001 && theta < orient+0.00001
%                                     angl = index_orient;
%                                 end
%                                 orient = orient + cell_orient;
%                             end
%                         end
%                         if (theta > -pi/2) && (theta < (pi/2)+0.00001)
%                             theta = round((theta)/cell_orient)*(cell_orient);
%                             for index_orient = 1:nrOforientSegments+1                                        
%                                 if theta > orient-0.00001 && theta < orient+0.00001
%                                     angl = index_orient;
%                                 end
%                                 orient = orient + cell_orient;                                       
%                             end
%                         end
%                         if ((theta > pi/2) && (theta < (1.5*pi)+0.00001))
%                             theta = (-pi+theta);
%                             theta = round((theta)/cell_orient)*(cell_orient);
%                             for index_orient = 1:nrOforientSegments+1                                        
%                                 if theta > orient-0.00001 && theta < orient+0.00001
%                                     angl = index_orient;
%                                 end
%                                 orient = orient + cell_orient;
%                             end
%                         end
%                                                            
%                         if ((theta > 1.5*pi) && (theta < 2*pi+0.00001))
%                             theta = (-1.5*pi)+theta;
%                             theta = round((theta)/cell_orient)*(cell_orient);
%                             for index_orient = 1:nrOforientSegments+1                                        
%                                 if theta > orient-0.00001 && theta < orient+0.00001
%                                     angl = index_orient;
%                                 end
%                                 orient = orient + cell_orient;
%                             end
%                         end
