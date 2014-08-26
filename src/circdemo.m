clear all; close all; clc;
figure();
%% Add Robotics Toolbox to MATLAB path
addpath(genpath('../rvctools'));

%% Define robot
% theta -> kinematic: link angle
% d -> kinematic: link offset
% a -> kinematic: link length
% alpha -> kinematic: link twist

L(1)=Link([pi/2 228 0 pi/2]);
L(2)=Link([pi/2 0 0 pi/2]);
L(3)=Link([-pi/2 312.5 0 -pi/2]);
L(4)=Link([pi 0 0 -pi/2]);
L(5)=Link([0 278 0 pi/2]);
L(6)=Link([0 0 0 -pi/2]);
L(7)=Link([0 180 0 pi/2]);

manfred=SerialLink(L, 'name', 'manfredv3');

%% Start position
qn = [0,pi/2,0,pi,pi/2,pi/4,0];
p0=zeros(6,1);

n=800;
rotangle=rotz(2*pi/n, 'rad');
%% Generate trajectory
angles=linspace(0,2*pi,20);

for i=1:n
    %% Original position
    p0_aux=manfred.fkine(qn);

    p0(1:3, end)=p0_aux(1:3,end);
    
    %% Jacobian and Inverse Jacobian on initial point
    jacobian=manfred.jacob0(qn);

    jacobianInv=pinv(jacobian);

    %% Position increment
    p1=zeros(6,1);
    p1(1:3)=rotangle*p0(1:3);
    p1(6)=2*pi/n;

    %% Rotate angles
    p3=p0;
    
    j=jacobianInv*(p1-p3);
    qr=qn'+j;

    p3_aux=manfred.fkine(qr);
    p3=zeros(6,1);
    p3(1:3)=p3_aux(1:3,end);

    error=sqrt((p1(1)-p3(1))^2+(p1(2)-p3(2))^2+(p1(3)-p3(3))^2);
    
    t=0:1;%Time vector & steps
    traj1=jtraj(qn,qr,t); 
    
    res(i).traj=traj1;
    res(i).qr=qr;
    res(i).qn=qn';
    res(i).error=error;
    qn=qr';
end

%% Plot results
for i=1:length(res)
    p_aux=manfred.fkine(res(i).qr);
    p(:,i)=p_aux(1:3,end);
end

p_aux=manfred.fkine(res(1).qn);

for i=1:length(res)
    p2(:,i)=rotangle*p_aux(1:3,end);
    p_aux=p2(:,i);
end

for i=1:length(res)
    error(i)=res(i).error;
end

figure();
manfred.plot(res(1).qn')
for i=1:length(res)
    if i~=length(res)
        plot3([p(1,i) p(1,i+1)], [p(2,i) p(2,i+1)], [p(3,i) p(3,i+1)], 'b-')
        hold on
    end
    manfred.plot(res(i).traj);
    hold on
end
hold off



