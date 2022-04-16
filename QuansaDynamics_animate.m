%% This function will animate the differential steered robot
% 17 July 2020

function QuansaDynamics_animate(out, write_video, every_nth)
close all;

Base_x = 0.1;
Base_y = 0.1;
Base_z = 0.1;
Lr = 0.085;
Lp = 0.129;
rr = 0.0035;
rp = 0.005;

blockColour = 'c';

th = out.th.signals.values();
alph = out.alph.signals.values();
x_arm = out.xr.signals.values();
y_arm = out.yr.signals.values();
x_pen = out.xp.signals.values();
y_pen = out.yp.signals.values();

if write_video
    writerObj = VideoWriter('Pendulum_Simulation.mp4');
    writerObj.FrameRate = 10;
    open(writerObj);
end

% Plot initial Axis
q0x = quiver3(0,0,0.135, 0.13,0,0, 'r');
hold on
q0x.LineWidth = 2;
q0y = quiver3(0,0,0.135, 0,0.13,0, 'r');
q0y.LineWidth = 2;
q0z = quiver3(0,0,0.135, 0,0,0.065, 'r');
q0z.LineWidth = 2;

plot3(-0.05, -0.05, 0, 'rs', LineWidth = 5);
plot3(0.05, -0.05, 0, 'rs', LineWidth = 5);
plot3(-0.05, 0.05, 0, 'rs', LineWidth = 5);
plot3(0.05, 0.05, 0, 'rs', LineWidth = 5);
plot3(-0.05, -0.05, 0.1, 'rs', LineWidth = 5);
plot3(0.05, -0.05, 0.1, 'rs', LineWidth = 5);
plot3(-0.05, 0.05, 0.1, 'rs', LineWidth = 5);
plot3(0.05, 0.05, 0.1, 'rs', LineWidth = 5);

q1x = quiver3(0,0,0.135, 0.075,0,0, 'g');
q1x.LineWidth = 2;
q1y = quiver3(0,0,0.135, 0,0.075,0, 'g');
q1y.LineWidth = 2;
q1z = quiver3(0,0,0.135, 0,0,0.05, 'g');
q1z.LineWidth = 2;

q2x = quiver3(Lr,0,0.135, 0.035,0,0, 'b');
q2x.LineWidth = 2;
q2y = quiver3(Lr,0,0.135, 0,0.035,0, 'b');
q2y.LineWidth = 2;
q2z = quiver3(Lr,0,0.135, 0,0,0.035, 'b');
q2z.LineWidth = 2;
hold off
% you don't really need to understand the code below

%% Draw Base
% Calculate Rotation Matrix - from inertial to body
% Compute propagation of vertices and patches
for i_time = 1:length(th)
    
    % Vertices
    temp = [0, 0, 0];
    VertexData(:,:,i_time) = GeoVerMakeBlock(temp, eye(3), [Base_x,Base_y,Base_z]);
    [X,Y,Z] = GeoPatMakeBlock(VertexData(:,:,i_time));
    
    PatchData_X(:,:,i_time) = X;
    PatchData_Y(:,:,i_time) = Y;
    PatchData_Z(:,:,i_time) = Z;
end

%% Draw Arm
% Calculate Rotation Matrix - from inertial to body
% Compute propagation of vertices and patches
for i_time = 1:length(th)
    
    R01 = Rotz(th(i_time)); %from frame 0 to frame 1
    R10 = transpose(R01); %from 1 to Inertial
    
    %Vertices - may need to change the height
    temp = [0,0,0.135];
    
    Vertex1Data(:,:,i_time) = GeoVerMakeCyl( temp, R10*Rotz(pi/2), rr, Lr );
    [X,Y,Z] = GeoPatMakeCyl(Vertex1Data(:,:,i_time));
    
    Patch1Data_X(:,:,i_time) = X;
    Patch1Data_Y(:,:,i_time) = Y;
    Patch1Data_Z(:,:,i_time) = Z;
    
    % Create wheel caps
    temp =  Patch1Data_X(1:2,:,i_time);
    temp =  reshape(temp,20*2,1);
    Patch1Cap1Data_X(:,i_time) = temp;
    temp =  Patch1Data_Y(1:2,:,i_time);
    temp =  reshape(temp,20*2,1);
    Patch1Cap1Data_Y(:,i_time) = temp;
    temp =  Patch1Data_Z(1:2,:,i_time);
    temp =  reshape(temp,20*2,1);
    Patch1Cap1Data_Z(:,i_time) = temp;

    % Create wheel caps
    temp =  Patch1Data_X(3:4,:,i_time);
    temp =  reshape(temp,20*2,1);
    Patch1Cap2Data_X(:,i_time) = temp;
    temp =  Patch1Data_Y(3:4,:,i_time);
    temp =  reshape(temp,20*2,1);
    Patch1Cap2Data_Y(:,i_time) = temp;
    temp =  Patch1Data_Z(3:4,:,i_time);
    temp =  reshape(temp,20*2,1);
    Patch1Cap2Data_Z(:,i_time) = temp;
end

%% Draw Encoder
% Calculate Rotation Matrix - from inertial to body
% Compute propagation of vertices and patches
for i_time = 1:length(th)
    
    R01 = Rotz(th(i_time)); %from frame 0 to frame 1
    R10 = transpose(R01); %from 1 to Inertial
    
    %Vertices - may need to change the height
    temp = [-Lr*cos(th(i_time))/4,-Lr*sin(th(i_time))/4,0.135];
    
    Vertex4Data(:,:,i_time) = GeoVerMakeCyl( temp, R10*Rotz(pi/2), 0.04/2, Lr/2 );
    [X,Y,Z] = GeoPatMakeCyl(Vertex4Data(:,:,i_time));
    
    Patch4Data_X(:,:,i_time) = X;
    Patch4Data_Y(:,:,i_time) = Y;
    Patch4Data_Z(:,:,i_time) = Z;
    
    % Create wheel caps
    temp =  Patch4Data_X(1:2,:,i_time);
    temp =  reshape(temp,20*2,1);
    Patch4Cap1Data_X(:,i_time) = temp;
    temp =  Patch4Data_Y(1:2,:,i_time);
    temp =  reshape(temp,20*2,1);
    Patch4Cap1Data_Y(:,i_time) = temp;
    temp =  Patch4Data_Z(1:2,:,i_time);
    temp =  reshape(temp,20*2,1);
    Patch4Cap1Data_Z(:,i_time) = temp;

    % Create wheel caps
    temp =  Patch4Data_X(3:4,:,i_time);
    temp =  reshape(temp,20*2,1);
    Patch4Cap2Data_X(:,i_time) = temp;
    temp =  Patch4Data_Y(3:4,:,i_time);
    temp =  reshape(temp,20*2,1);
    Patch4Cap2Data_Y(:,i_time) = temp;
    temp =  Patch4Data_Z(3:4,:,i_time);
    temp =  reshape(temp,20*2,1);
    Patch4Cap2Data_Z(:,i_time) = temp;
end


%% Draw Pendulum
% Calculate Rotation Matrix - from inertial to body
% Compute propagation of vertices and patches
for i_time = 1:length(th)
    
    R01 = Rotz(th(i_time)); %from frame 0 to frame 1
    R10 = transpose(R01); %from 1 to Inertial
    R12 = Rotx(alph(i_time));
    R02 = R12*R01; %from frame 0 to frame 2
    R20 = transpose(R02); %from 2 to Inertial
    
    %Vertices - may need to change the height
    temp = [Lr*cos(th(i_time)), Lr*sin(th(i_time)), 0.135];
    Vertex1Data(:,:,i_time) = GeoVerMakeCyl( temp, R20*Rotx(-pi/2), rp, Lp);
    [X,Y,Z] = GeoPatMakeCyl(Vertex1Data(:,:,i_time));
    
    Patch2Data_X(:,:,i_time) = X;
    Patch2Data_Y(:,:,i_time) = Y;
    Patch2Data_Z(:,:,i_time) = Z;
    
    % Create wheel caps
    temp =  Patch2Data_X(1:2,:,i_time);
    temp =  reshape(temp,20*2,1);
    Patch2Cap1Data_X(:,i_time) = temp;
    temp =  Patch2Data_Y(1:2,:,i_time);
    temp =  reshape(temp,20*2,1);
    Patch2Cap1Data_Y(:,i_time) = temp;
    temp =  Patch2Data_Z(1:2,:,i_time);
    temp =  reshape(temp,20*2,1);
    Patch2Cap1Data_Z(:,i_time) = temp;
    
    % Create wheel caps
    temp =  Patch2Data_X(3:4,:,i_time);
    temp =  reshape(temp,20*2,1);
    Patch2Cap2Data_X(:,i_time) = temp;
    temp =  Patch2Data_Y(3:4,:,i_time);
    temp =  reshape(temp,20*2,1);
    Patch2Cap2Data_Y(:,i_time) = temp;
    temp =  Patch2Data_Z(3:4,:,i_time);
    temp =  reshape(temp,20*2,1);
    Patch2Cap2Data_Z(:,i_time) = temp;
end

%% Draw Hub
% Calculate Rotation Matrix - from inertial to body
% Compute propagation of vertices and patches
for i_time = 1:length(th)

    
    %Vertices - may need to change the height
    temp = [0, 0, 0];
    Vertex3Data(:,:,i_time) = GeoVerMakeCyl( temp, Rotz(th(i_time))*Rotx(-pi/2) , 0.01, 0.115);
    [X,Y,Z] = GeoPatMakeCyl(Vertex3Data(:,:,i_time));
    
    Patch3Data_X(:,:,i_time) = X;
    Patch3Data_Y(:,:,i_time) = Y;
    Patch3Data_Z(:,:,i_time) = Z;
    
    % Create wheel caps
    temp =  Patch3Data_X(1:2,:,i_time);
    temp =  reshape(temp,20*2,1);
    Patch3Cap1Data_X(:,i_time) = temp;
    temp =  Patch3Data_Y(1:2,:,i_time);
    temp =  reshape(temp,20*2,1);
    Patch3Cap1Data_Y(:,i_time) = temp;
    temp =  Patch3Data_Z(1:2,:,i_time);
    temp =  reshape(temp,20*2,1);
    Patch3Cap1Data_Z(:,i_time) = temp;
    
    % Create wheel caps
    temp =  Patch3Data_X(3:4,:,i_time);
    temp =  reshape(temp,20*2,1);
    Patch3Cap2Data_X(:,i_time) = temp;
    temp =  Patch3Data_Y(3:4,:,i_time);
    temp =  reshape(temp,20*2,1);
    Patch3Cap2Data_Y(:,i_time) = temp;
    temp =  Patch3Data_Z(3:4,:,i_time);
    temp =  reshape(temp,20*2,1);
    Patch3Cap2Data_Z(:,i_time) = temp;
end

%% PLOT
% Draw patches
figure(1);
ground.vertices=[10 -10 0; -10 -10 0; -10 10 0; 10 10 0]*0.2; %square
ground.faces=[1 2 3 4]; %connect vertices
hground = patch('Faces',ground.faces,'Vertices',ground.vertices,'FaceColor',[211,211,211]/255);
set(hground,'FaceLighting','phong','EdgeLighting','phong','FaceAlpha',0.5);
hold on;
h0 = patch(PatchData_X(:,:,1),PatchData_Y(:,:,1),PatchData_Z(:,:,1),'FaceColor',blockColour);
set(h0,'FaceLighting','phong','EdgeLighting','phong','FaceAlpha',0.8);

h1 = patch(Patch1Data_X(:,:,1),Patch1Data_Y(:,:,1),Patch1Data_Z(:,:,1),'FaceColor','r');
set(h1,'FaceLighting','phong','EdgeLighting','phong','FaceAlpha',1,'EdgeColor','k');
h1_cap1 = patch(Patch1Cap1Data_X(:,1),Patch1Cap1Data_Y(:,1),Patch1Cap1Data_Z(:,1),'FaceColor','k');
h1_cap2 = patch(Patch1Cap2Data_X(:,1),Patch1Cap2Data_Y(:,1),Patch1Cap2Data_Z(:,1),'FaceColor','k');

h2 = patch(Patch2Data_X(:,:,1),Patch2Data_Y(:,:,1),Patch2Data_Z(:,:,1),'FaceColor','r');
set(h2,'FaceLighting','phong','EdgeLighting','phong','FaceAlpha',1,'EdgeColor','k');
h2_cap1 = patch(Patch2Cap1Data_X(:,1),Patch2Cap1Data_Y(:,1),Patch2Cap1Data_Z(:,1),'FaceColor','k');
h2_cap2 = patch(Patch2Cap2Data_X(:,1),Patch2Cap2Data_Y(:,1),Patch2Cap2Data_Z(:,1),'FaceColor','k');

h3 = patch(Patch3Data_X(:,:,1),Patch3Data_Y(:,:,1),Patch3Data_Z(:,:,1),'FaceColor','r');
set(h3,'FaceLighting','phong','EdgeLighting','phong','FaceAlpha',1,'EdgeColor','k');
h3_cap1 = patch(Patch3Cap1Data_X(:,1),Patch3Cap1Data_Y(:,1),Patch3Cap1Data_Z(:,1),'FaceColor','k');
h3_cap2 = patch(Patch3Cap2Data_X(:,1),Patch3Cap2Data_Y(:,1),Patch3Cap2Data_Z(:,1),'FaceColor','k');

h4 = patch(Patch4Data_X(:,:,1),Patch4Data_Y(:,:,1),Patch4Data_Z(:,:,1),'FaceColor','r');
set(h3,'FaceLighting','phong','EdgeLighting','phong','FaceAlpha',1,'EdgeColor','k');
h4_cap1 = patch(Patch4Cap1Data_X(:,1),Patch4Cap1Data_Y(:,1),Patch4Cap1Data_Z(:,1),'FaceColor','k');
h4_cap2 = patch(Patch4Cap2Data_X(:,1),Patch4Cap2Data_Y(:,1),Patch4Cap2Data_Z(:,1),'FaceColor','k');

FigHandle = gcf;
grid on
hold on;

xlabel({'X Position (m)'},'FontSize',14,'FontName','AvantGarde');
ylabel({'Y Position (m)'},'FontSize',14,'FontName','AvantGarde');
zlabel({'Z Position (m)'},'FontSize',14,'FontName','AvantGarde');

% Create title
title({'Quanser Rotary Pendulum'},'FontWeight','bold','FontSize',24,...
    'FontName','AvantGarde');

simtime = out.tout;
axis([-0.2 0.2 -0.2 0.2 0 0.2])
camlight;
grid on;
view([35,50]);
set(FigHandle, 'Position', [100, 100, 1200, 800]);
%%{
for i = 1:every_nth:length(th)
    disp(simtime(i));
    set(h0,'XData',PatchData_X(:,:,i));
    set(h0,'YData',PatchData_Y(:,:,i));
    set(h0,'ZData',PatchData_Z(:,:,i));
    set(h1,'XData',Patch1Data_X(:,:,i));
    set(h1,'YData',Patch1Data_Y(:,:,i));
    set(h1,'ZData',Patch1Data_Z(:,:,i));
    set(h2,'XData',Patch2Data_X(:,:,i));
    set(h2,'YData',Patch2Data_Y(:,:,i));
    set(h2,'ZData',Patch2Data_Z(:,:,i));
    set(h3,'XData',Patch3Data_X(:,:,i));
    set(h3,'YData',Patch3Data_Y(:,:,i));
    set(h3,'ZData',Patch3Data_Z(:,:,i));
    set(h4,'XData',Patch4Data_X(:,:,i));
    set(h4,'YData',Patch4Data_Y(:,:,i));
    set(h4,'ZData',Patch4Data_Z(:,:,i));
    
    set(h1_cap1,'XData',Patch1Cap1Data_X(:,i));
    set(h1_cap1,'YData',Patch1Cap1Data_Y(:,i));
    set(h1_cap1,'ZData',Patch1Cap1Data_Z(:,i));
    set(h1_cap2,'XData',Patch1Cap2Data_X(:,i));
    set(h1_cap2,'YData',Patch1Cap2Data_Y(:,i));
    set(h1_cap2,'ZData',Patch1Cap2Data_Z(:,i));
    
    set(h1_cap1,'XData',Patch1Cap1Data_X(:,i));
    set(h1_cap1,'YData',Patch1Cap1Data_Y(:,i));
    set(h1_cap1,'ZData',Patch1Cap1Data_Z(:,i));
    set(h1_cap2,'XData',Patch1Cap2Data_X(:,i));
    set(h1_cap2,'YData',Patch1Cap2Data_Y(:,i));
    set(h1_cap2,'ZData',Patch1Cap2Data_Z(:,i));
    
    set(h4_cap1,'XData',Patch4Cap1Data_X(:,i));
    set(h4_cap1,'YData',Patch4Cap1Data_Y(:,i));
    set(h4_cap1,'ZData',Patch4Cap1Data_Z(:,i));
    set(h4_cap2,'XData',Patch4Cap2Data_X(:,i));
    set(h4_cap2,'YData',Patch4Cap2Data_Y(:,i));
    set(h4_cap2,'ZData',Patch4Cap2Data_Z(:,i));
    
    set(h2_cap1,'XData',Patch2Cap1Data_X(:,i));
    set(h2_cap1,'YData',Patch2Cap1Data_Y(:,i));
    set(h2_cap1,'ZData',Patch2Cap1Data_Z(:,i));
    set(h2_cap2,'XData',Patch2Cap2Data_X(:,i));
    set(h2_cap2,'YData',Patch2Cap2Data_Y(:,i));
    set(h2_cap2,'ZData',Patch2Cap2Data_Z(:,i));
    
    q1x.UData = 0.11*cos(th(i));
    q1x.VData = 0.11*sin(th(i));
    q1y.UData = 0.05*cos(th(i)+pi/2);
    q1y.VData = 0.05*sin(th(i)+pi/2);

    q2x.XData = Lr*cos(th(i));
    q2x.YData = Lr*sin(th(i));
    q2x.UData = 0.05*cos(th(i));
    q2x.VData = 0.05*sin(th(i));
    
    q2y.XData = Lr*cos(th(i));
    q2y.YData = Lr*sin(th(i));
    q2y.UData = 0.05*cos(th(i)+pi/2)*cos(alph(i));
    q2y.VData = 0.05*sin(th(i)+pi/2)*cos(alph(i));
    q2y.WData = 0.05*sin(alph(i));
    
    q2z.XData = Lr*cos(th(i));
    q2z.YData = Lr*sin(th(i));
    q2z.UData = 0.15*sin(th(i))*sin(alph(i));
    q2z.VData = 0.15*cos(th(i))*sin(-alph(i));
    q2z.WData = 0.15*cos(alph(i));
    
    
    drawnow;
    
    if write_video
        frame = getframe(gcf);
        writeVideo(writerObj,frame);
    end

    xlim('manual')
    ylim('manual')
end

if write_video
    close(writerObj);
end
%%}
end

function A = Rotx(th)
    A = [1 0        0;...
         0 cos(th)  sin(th);...
         0 -sin(th)  cos(th)];
end

function A = Roty(th)
    A = [cos(th)  0   -sin(th);...
         0        1   0;...
         sin(th)  0   cos(th)];
end 

function A = Rotz(th)
    A = [cos(th)   sin(th) 0;... 
         -sin(th)  cos(th) 0;...
         0        0        1];
end