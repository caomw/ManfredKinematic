clear all; close all; clc;
figure();
%% Add Robotics Toolbox to MATLAB path
addpath(genpath('../rvctools'));
% startup_rvc

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
qn=[pi/2, pi/2, -pi/4, pi/4, 0, pi/2, 0];
p0=zeros(6,1);

n=20;

data=ones(n,1)*500/n;

for i=1:n
    %% Original position
    p0_aux=manfred.fkine(qn);

    p0(1:3, end)=p0_aux(1:3,end);
    
    %% Jacobian and Inverse Jacobian on initial point
    jacobian=manfred.jacob0(qn);

    jacobianInv=pinv(jacobian);

    %% Rotate angles  
    
    if i<=n/2
        translate=[0, 0, data(i), 0, 0, 0]';
    else
        translate=[data(i), 0, 0, 0, 0, 0]';
    end
    
    j=jacobianInv*translate;

    qr=qn'+j;
    
    p1_aux=manfred.fkine(qn);
    p1=p1_aux(1:3,end)+translate(1:3);
    
    p3_aux=manfred.fkine(qr);
    p3=p3_aux(1:3,end);
    
    error=sqrt((p1(1)-p3(1))^2+(p1(2)-p3(2))^2+(p1(3)-p3(3))^2);

    
    t=0:1;%Time vector & steps
    traj1=jtraj(qn,qr,t); 
    
    res(i).traj=traj1;
    res(i).qr=qr;
    res(i).qn=qn';
    res(i).error=error;
    res(i).p1=p1;
    qn=qr';
    
    
end

%% Plot results
for i=1:length(res)
    p_aux=manfred.fkine(res(i).qr);
    p(:,i)=p_aux(1:3,end);
end

p_aux=manfred.fkine(res(1).qn);

for i=1:length(res)
    p2(:,i)=res(i).p1;
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



