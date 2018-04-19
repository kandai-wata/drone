function currFrame = draw_3d_animation(T, X_data, PWMs, fandt, dt, len, record, camera_turn, fastforward, az, el, start, bound, message)
%%%% switch buttons!
% record        = Start Recording
% camera_turn   = Turning Camera! Set angle from below
% fastforward   = Make it faster(Skips Frames)
% az, el        = camera angle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                 Camera                                 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% currFrame(round(length(T)/fastforward)) = struct('cdata',[],'colormap',[]);
date=datetime;
file_name = sprintf('[%i%i%i_%i%i]_%s.avi', date.Year, date.Month, date.Day, date.Hour, date.Minute, message(find(~isspace(message))));
vidObj = VideoWriter(char(file_name));
open(vidObj)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                              axis limit                                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if bound(1)==0 && bound(2)==0
    xyz_min = min(min(X_data(:,1:3)))-2*len; 
    xyz_max=max(max(X_data(:,1:3)))+2*len;
else
    xyz_min=bound(1,1);
    xyz_max=bound(1,2);
end
% xyz_min = -0.5; xyz_max=0.5;
t_end = T(end);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                 Color                                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
proposedColor = [0.85 0.325 0.098];
thinPColor = [0.95 0.425 0.198];
a = 0.000109094;
b = 0.0075591;
c = 0.09060951;
maxForce = 4*(a*100^2 + b*100 + c);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                 Main                                   %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure('units','normalized','outerposition',[0 0 1 1])
for i=start/(dt*fastforward):length(T)/fastforward
%%%%%%%%%%%%%%%%%%%% Set data at t, from _datas %%%%%%%%%%%%%%%%%%%%%%%%%%
    t=round(fastforward*i);
    x = X_data(t,1);
    y = X_data(t,2);
    z = X_data(t,3);
    phi = X_data(t,7);
    th = X_data(t,8);
    psi = X_data(t,9);
    pwm = PWMs(t,:);
    thrust = a*pwm.^2 + b*pwm + c;
    force = fandt(t,1);
    torque = fandt(t,2:4);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Plot %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % 3D Plot of Center of Mass
    hd3 = plot3(X_data(1:t,1),X_data(1:t,2),X_data(1:t,3)); grid on;
    set(hd3(1), 'Color', thinPColor,     'LineStyle', ':', 'LineWidth', 1);

    % Rotation Matrix earth2body
    R = getRotationalMatrix(phi, th, psi);
    % quadrotor init arm pos in xy dimension
    topLeft     = R*[1/sqrt(2);	1/sqrt(2);  0];
    topRight    = R*[1/sqrt(2); -1/sqrt(2); 0];
    % Motor Position
    topLeftMotor        = [x;y;z] + len*topLeft;
    topRightMotor       = [x;y;z] + len*topRight;
    bottomLeftMotor     = [x;y;z] - len*topRight;
    bottomRightMotor    = [x;y;z] - len*topLeft;
    % Draw Quadrotor Arm
    line([topLeftMotor(1) bottomRightMotor(1)], [topLeftMotor(2) bottomRightMotor(2)], [topLeftMotor(3) bottomRightMotor(3)], 'Color', proposedColor);
    line([topRightMotor(1) bottomLeftMotor(1)], [topRightMotor(2) bottomLeftMotor(2)], [topRightMotor(3) bottomLeftMotor(3)], 'Color', proposedColor);
    % Motor Thrust Array Position (to animate the thrust size using an arrow)
    topLeftArrow        = topLeftMotor      + thrust(1)/2*R*[0;0;1];
    topRightArrow       = topRightMotor     + thrust(2)/2*R*[0;0;1];
    bottomLeftArrow     = bottomLeftMotor   + thrust(4)/2*R*[0;0;1];
    bottomRightArrow    = bottomRightMotor  + thrust(3)/2*R*[0;0;1];
    % Draw Thrust Arrow according to the amount of thrust each motor is
    % producing
    arrow(topLeftMotor,     topLeftArrow,       'Length', 8);
    arrow(topRightMotor,    topRightArrow,      'Length', 8);
    arrow(bottomLeftMotor,  bottomLeftArrow,    'Length', 8);
    arrow(bottomRightMotor, bottomRightArrow,   'Length', 8);
    % show each pwm
    topLeftTextString     = sprintf('pwm = %0.1f', pwm(1));
    topRightTextString    = sprintf('pwm = %0.1f', pwm(2));
    bottomLeftTextString  = sprintf('pwm = %0.1f', pwm(4));
    bottomRightTextString = sprintf('pwm = %0.1f', pwm(3));
    % prepare text
    scale = xyz_max-xyz_min;
    textSize = 16;
    scale = scale/4;
    % show text
    topLeftText     = topLeftMotor + scale*topLeft;
    topRightText    = topRightMotor + scale*topRight;
    bottomLeftText  = bottomLeftMotor - scale*topRight;
    bottomRightText = bottomRightMotor - scale*topLeft;
    text(topLeftText(1),   topLeftText(2), topLeftText(3),           topLeftTextString, 'FontSize',textSize);
    text(topRightText(1),  topRightText(2), topRightText(3),         topRightTextString, 'FontSize',textSize);
    text(bottomLeftText(1), bottomLeftText(2), bottomRightText(3),   bottomLeftTextString, 'FontSize',textSize);
    text(bottomRightText(1), bottomRightText(2), bottomRightText(3), bottomRightTextString, 'FontSize',textSize);
    % ARROW to show Thrusts and Torques as well as the direction of a Drone
    center      = [x; y; z];
    top         = [x; y; z] + force/maxForce*R*[0; 0; 1];
    txCenter    = [x; y; z] + R*[scale; 0; 0];
    tyCenter    = [x; y; z] + R*[0; scale; 0];
    tzCenter    = [x; y; z] + R*[0; 0; scale];
    radTorque   = scale/4;        magnify=50;
    txFrom      = [x; y; z] + R*[len;                           radTorque;                      0];
    txTo        = [x; y; z] + R*[len;                           radTorque*cos(magnify*torque(1));    radTorque*sin(magnify*torque(1))];
    tyFrom      = [x; y; z] + R*[-radTorque;                    len;                            0];
    tyTo        = [x; y; z] + R*[-radTorque*cos(magnify*torque(2));  len;                            radTorque*sin(magnify*torque(2))];
    tzFrom      = [x; y; z] + R*[radTorque;                     0;                              len];
    tzTo        = [x; y; z] + R*[radTorque*cos(magnify*torque(3));   radTorque*sin(magnify*torque(3));    len];
    arrow(center, top,          'Length', 8);
    arrow(center, txCenter, 	'Length', 8, 'Color', [0 0 1]);
    CurvedArrow3(txFrom, txTo, txCenter);
    CurvedArrow3(tyFrom, tyTo, tyCenter);
    CurvedArrow3(tzFrom, tzTo, tzCenter);
    forceText   = sprintf('force = %0.1f', force);
    txTextString      = sprintf('tx = %0.2f', torque(1));
    tyTextString      = sprintf('ty = %0.2f', torque(2));
    tzTextString      = sprintf('tz = %0.2f', torque(3));
    text(x, y, z,                               forceText,  'FontSize',textSize);
    text(txCenter(1), txCenter(2), txCenter(3), txTextString,     'FontSize',textSize);
    text(tyCenter(1), tyCenter(2), tyCenter(3), tyTextString,     'FontSize',textSize);
    text(tzCenter(1), tzCenter(2), tzCenter(3), tzTextString,     'FontSize',textSize);

    %%% Axis Limit
	xlim([xyz_min xyz_max]); ylim([xyz_min xyz_max]); zlim([xyz_min xyz_max]);
    xlabel('{\itx}, m'); ylabel('{\ity}, m'); zlabel('{\itz}, m');
    title(sprintf('%0.1f/%i [s]',t*dt,t_end));

    % xyz base frame arrows
    arrow([xyz_min+0.1 xyz_min+0.1 xyz_min+0.1], [xyz_min+0.3 xyz_min+0.1 xyz_min+0.1], 'Length', 8);   text(xyz_min+0.4,xyz_min+0.1,xyz_min+0.1,'\itx', 'FontSize',10);
    arrow([xyz_min+0.1 xyz_min+0.1 xyz_min+0.1], [xyz_min+0.1 xyz_min+0.3 xyz_min+0.1], 'Length', 8);   text(xyz_min+0.1,xyz_min+0.4,xyz_min+0.1,'\ity', 'FontSize',10);
    arrow([xyz_min+0.1 xyz_min+0.1 xyz_min+0.1], [xyz_min+0.1 xyz_min+0.1 xyz_min+0.3], 'Length', 8);   text(xyz_min+0.1,xyz_min+0.1,xyz_min+0.4,'\itz', 'FontSize',10);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Switch %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Camera Turns
    if(camera_turn==true)
        if(az>360)
            az=0;
        end
        az=az+1;
    end
    % view(az, el);
    %%% Recording
    if record==true
        currFrame = getframe(gcf);
        writeVideo(vidObj, currFrame)
    end
    drawnow;
end
close(vidObj);
end

function R = getRotationalMatrix(phi, th, psi)
R = [cos(th)*cos(psi)       sin(phi)*sin(th)*cos(psi)-cos(phi)*sin(psi)         cos(phi)*sin(th)*cos(psi)+sin(phi)*sin(psi);
     cos(th)*sin(psi)       sin(phi)*sin(th)*sin(psi)+cos(phi)*cos(psi)         cos(phi)*sin(th)*sin(psi)-sin(phi)*cos(psi);
     -sin(th)               sin(phi)*cos(th)                                    cos(phi)*cos(th)];
end

%% --- Creates a curved arrow
% from: Starting position - (x,y,z) upplet
% to: Final position - (x,y,z) upplet
% center: Center of arc - (x,y,z) upplet => by default the origin
% count: The number of segment to draw the arrow => by default 15
function [h] = CurvedArrow3(from, to, center, count)
%[        
    % Inputs
    if (nargin < 4), count = 15; end
    if (nargin < 3), center = [0 0 0]; end
    center = center(:); from = from(:); to = to(:);

    % Start, stop and normal vectors    
    start = from - center; rstart = norm(start);
    stop = to - center; rstop = norm(stop);
    angle = atan2(norm(cross(start,stop)), dot(start,stop));
    normal = cross(start, stop); normal = normal / norm(normal);

    % Compute intermediate points by rotating 'start' vector
    % toward 'end' vector around 'normal' axis
    % See: http://inside.mines.edu/fs_home/gmurray/ArbitraryAxisRotation/
    phiAngles = linspace(0, angle, count);
    r = linspace(rstart, rstop, count) / rstart;
    intermediates = zeros(3, count);
    a = center(1); b = center(2); c = center(3);
    u = normal(1); v = normal(2); w = normal(3); 
    x = from(1); y = from(2); z = from(3);
    for ki = 1:count,
        phi = phiAngles(ki);
        cosp = cos(phi); sinp = sin(phi);
        T = [(u^2+(v^2+w^2)*cosp)  (u*v*(1-cosp)-w*sinp)  (u*w*(1-cosp)+v*sinp) ((a*(v^2+w^2)-u*(b*v+c*w))*(1-cosp)+(b*w-c*v)*sinp); ...
             (u*v*(1-cosp)+w*sinp) (v^2+(u^2+w^2)*cosp)   (v*w*(1-cosp)-u*sinp) ((b*(u^2+w^2)-v*(a*u+c*w))*(1-cosp)+(c*u-a*w)*sinp); ...   
             (u*w*(1-cosp)-v*sinp) (v*w*(1-cosp)+u*sinp)  (w^2+(u^2+v^2)*cosp)  ((c*(u^2+v^2)-w*(a*u+b*v))*(1-cosp)+(a*v-b*u)*sinp); ...
                      0                    0                      0                                1                               ];
        intermediate = T * [x;y;z;r(ki)];
        intermediates(:,ki) = intermediate(1:3);
    end

    % Draw the curved line
    % Can be improved of course with hggroup etc...
    X = intermediates(1,:);
    Y = intermediates(2,:);
    Z = intermediates(3,:);    
    tf = ishold;
    if (~tf), hold on; end
    h = line(X,Y,Z);       
    quiver3(X(end-1), Y(end-1), Z(end-1), X(end)-X(end-1), Y(end)-Y(end-1), Z(end)-Z(end-1),1);    
    if (~tf), hold off; end
%]
end
