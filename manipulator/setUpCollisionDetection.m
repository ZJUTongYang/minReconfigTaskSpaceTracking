function setUpCollisionDetection(ur5)

cy(1, 1) = collisionCylinder(0.07, 0.16);
cy(1, 1).Pose = [1, 0, 0, 0; 
                0, 1, 0, 0; 
                0, 0, 1, 0.06; 
                0, 0, 0, 1];

% addCollision(ur5.Bodies{1},cy(1, 1));

cy(2, 1) = collisionCylinder(0.07, 0.54);
cy(2, 1).Pose = [1, 0, 0, 0; 
                0, 1, 0, 0; 
                0, 0, 1, 0.22; 
                0, 0, 0, 1];
addCollision(ur5.Bodies{4},cy(2, 1));     

cy(3, 1) = collisionCylinder(0.04, 0.48);
cy(3, 1).Pose = [1, 0, 0, 0; 
                0, 1, 0, 0; 
                0, 0, 1, 0.18; 
                0, 0, 0, 1];
addCollision(ur5.Bodies{5},cy(3, 1));   

cy(4, 1) = collisionCylinder(0.04, 0.11);
cy(4, 1).Pose = [1, 0, 0, 0; 
                0, 1, 0, 0; 
                0, 0, 1, -0.01; 
                0, 0, 0, 1];
addCollision(ur5.Bodies{7},cy(4, 1));   

% This is for main_3
% cy(5, 1) = collisionCylinder(0.04, 0.14);
% cy(5, 1).Pose = [1, 0, 0, 0; 
%                 0, 1, 0, 0; 
%                 0, 0, 1, -0.04; 
%                 0, 0, 0, 1];
            
cy(5, 1) = collisionCylinder(0.04, 0.06);
cy(5, 1).Pose = [1, 0, 0, 0; 
                0, 1, 0, 0; 
                0, 0, 1, -0.07; 
                0, 0, 0, 1];

            
% cy(5, 1) = collisionCylinder(0.04, 0.24);
% cy(5, 1).Pose = [1, 0, 0, 0; 
%                 0, 1, 0, 0; 
%                 0, 0, 1, 0.06; 
%                 0, 0, 0, 1];
% addCollision(ur5.Bodies{10},cy(5, 1));   

cy(6, 1) = collisionCylinder(0.04, 0.04);
cy(6, 1).Pose = [1, 0, 0, 0; 
                0, 1, 0, 0; 
                0, 0, 1, -0.12; 
                0, 0, 0, 1];
addCollision(ur5.Bodies{10},cy(6, 1));   

% figure(1)
% show(ur5,[1, 1, 1, 1, 1, 1],'Collisions','on','Visuals','on');

% figure(2)
% show(ur5,[1, 1, 1, 1, 1, 1],'Collisions','off','Visuals','on');
end