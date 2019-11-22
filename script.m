%% *** Robot (kinematic) model parameters *** 
clear all; 
close all;
L0 = 0;
L1 = 0;
L2 = 2.0;
L3 = 0;
L4 = 5.0;
L5 = 5.0;
zd = 3;  %z=h=L4+L5
%% *** sampling period *** 
%% *** for the robot motion, kinematic simulation: 
dt = 0.01; %dt = 0.001; i.e. 1 msec)   

%% *** Create (or load from file) reference signals *** 
%% *** DESIRED MOTION PROFILE - TASK SPACE *** 
Tf = 10.0; 	% 10sec duration of motion 
t = 0:dt:Tf;  

%xd0,td0,yd1: initial/final end-point position --> desired task-space trajectory  
xd0 = 2.0;	
xd1 = 8.0;
yd0 = 5.00; 
yd1 = -5.00;  

% Example of desired trajectory : linear segment (x0,y0)-->(x1,y1); Time duration: Tf; 
disp('Initialising Desired Task-Space Trajectory (Motion Profile) ...'); %% 
disp(' ');   
xd(1) = xd0; 
yd(1) = yd0; 
lambda_a3 = (((-1)*2 )/(Tf^3)) *sqrt((xd1-xd0)^2+(yd1-yd0)^2 ); 
lambda_a2 = (3/(Tf^2)) *sqrt((xd1-xd0)^2+(yd1-yd0)^2);
kmax=Tf/dt + 1; 
vx(1)=0;
vy(1)=0;
for k=2:length(t)-1   
   xd(k) =xd0+  ((xd1-xd0)/(sqrt((xd1-xd0)^2+(yd1-yd0)^2 )))*(lambda_a2*(t(k)^2))+(lambda_a3*(t(k)^3)) * ((xd1-xd0)/(sqrt((xd1-xd0)^2+(yd1-yd0)^2 )));    
   yd(k) =yd0+  ((yd1-yd0)/(sqrt((xd1-xd0)^2+(yd1-yd0)^2 )))*(lambda_a2*(t(k)^2))+(lambda_a3*((t(k)^3))) * ((yd1-yd0)/(sqrt((xd1-xd0)^2+(yd1-yd0)^2 )));  
   vx(k) = ((xd1-xd0)/(sqrt((xd1-xd0)^2+(yd1-yd0)^2 )))*(2* (lambda_a2*t(k))+(3*lambda_a3*(t(k)^2))) ;  %v(t)= 3*b3*t^3 + 2*b2*t  velocity of P
   vy(k) = ((yd1-yd0)/(sqrt((xd1-xd0)^2+(yd1-yd0)^2 )))*(2* (lambda_a2*t(k))+(3*lambda_a3*(t(k)^2))) ;  
end  
xd(length(t))=xd1;
yd(length(t))=yd1;
vx(length(t))=0;
vy(length(t))=0;

figure(1);  
subplot(2,1,1); 
plot(t,xd); 
ylabel('xd (cm)'); 
xlabel('time t (sec)');  

subplot(2,1,2); 
plot(t,yd); 
ylabel('yd (cm)'); 
xlabel('time t (sec)');  

figure(2);  
subplot(2,1,1); 
plot(t,vx); 
ylabel('velocity of x(cm/sec)'); 
xlabel('time t (sec)');  

subplot(2,1,2); 
plot(t,vy); 
ylabel('velocity of y (cm/sec)'); 
xlabel('time t (sec)');  
 
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%% ****** KINEMATIC SIMULATION - Main loop ****** 
disp('Kinematic Simulation ...'); %% 
disp(' '); %%  

%% ***** INVESRE KINEMATICS  -->  DESIRED MOTION - JOINT SPACE ***** 
qd1 = 2*atan((zd+sqrt(zd^2 + xd.^2 - L2^2))./(xd+L2));
X = sin(qd1).*xd - cos(qd1).*zd;
Y = yd;
qd3 = acos((X.^2 + Y.^2 - L4^2 - L5^2)/(2*L4*L5));
qd2 = atan( Y ./ X ) - asin(L5*sin(qd3)./(sqrt(X.^2 + Y.^2)));


%velocity of q
vq1 = diff(qd1);
vq2 = diff(qd2);
vq3 = diff(qd3);

figure(3);  
subplot(3,1,1); 
plot(t,qd1); 
ylabel('joint 1'); 
xlabel('time t (sec)');  

subplot(3,1,2); 
plot(t,qd2); 
ylabel('joint 2'); 
xlabel('time t (sec)');  

subplot(3,1,3); 
plot(t,qd3); 
ylabel('joint 3'); 
xlabel('time t (sec)');  

figure(4);  
subplot(3,1,1); 
plot(t(2:end),vq1); 
ylabel('velocity joint 1'); 
xlabel('time t (sec)');  

subplot(3,1,2); 
plot(t(2:end),vq2); 
ylabel('velocity of joint 2'); 
xlabel('time t (sec)');  

subplot(3,1,3); 
plot(t(2:end),vq3); 
ylabel('velocity of joint 3'); 
xlabel('time t (sec)');  

%animation

c1=cos(qd1);
c2=cos(qd2);
c23=cos(qd2 + qd3);
s1=sin(qd1);
s2=sin(qd2);
s23=sin(qd2 + qd3);

xp1 = L1;
yp1 = 0;
zp1 = -L0;

xp2 = c1 .* L2 + L1;
yp2 = 0;
zp2 = s1 .* L2 - L0;

xp3 = s1 .* c2 .* L4 + c1 .* L2 + L1;
yp3 = s2 .* L4;
zp3 = -c1 .* c2 .* L4 + s1 .* L2 - L0;

pex = s1 .* (c23 .* L5 + c2 .* L4) + c1 .* L2 + L1;
pey = s23 .* L5 + s2 .* L4;
pez = -c1 .* (c23 .* L5 + c2 .* L4) + s1 .* L2 - L0;

figure(5)
axis([-25 25 -15 15 -10 20])
axis on
hold on
xlabel('x (cm)');
ylabel('y (cm)');
zlabel('z (cm)');

dtime = 100;
plot3([0],[0],[0],'o');
for time=1:100:length(t),    %%% 	
   pause(0.1);	%% pause motion to view successive robot configurations    				
   
   plot3(xp1,yp1,zp1,'o');
   plot3([xp1,xp2(time)],[yp1,yp2],[zp1,zp2(time)]);
   
   plot3(xp2(time),yp2,zp2(time),'o');
   plot3([xp2(time),xp3(time)],[yp2,yp3(time)],[zp2(time),zp3(time)]);

   plot3(xp3(time),yp3(time),zp3(time),'o');
   plot3([xp3(time),pex(time)],[yp3(time),pey(time)],[zp3(time),pez(time)]);
   
   plot3(pex(time),pey(time),pez(time),'x');
end      


