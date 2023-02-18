%Vellios Georgios Serafeim AEM:9471

%results();pause(3);
clc;
close all;
clear;

% create the robotic arm and the table
lwr=lwr_create();
l=plotcube([0.35 0.35 0.6],[1.325 1.325 0],.1,[0 0 1]);

% define the 3d workspace
work=[-1 3 -1 2 0 1.5];

%Plot the cylinder
R = 0.025 ;           % Radius of the cylindrical shell
H = 0.1 ;           % Height of the Cylinder
M = 50 ;N = 100 ;

% Discretizing the Height and Angle of the cylinder
nH = linspace(0,H,M) ;
nT = linspace(0,2*pi,N) ;
[H, T] = meshgrid(nH,nT) ;

%%cylinder at (0,0)
X1 = R*cos(T); 
Y1 = R*sin(T);
Z1 = H ;

X2=X1+1.5;
Y2=Y1+1.5;
Z2=Z1+0.6;
cyl=mesh(X2,Y2,Z2);

% initialize matrixes
z=0:0.08:20;
len2=length(z);
Akroxyz=zeros(len2,3);
Gwnies=zeros(len2,6);
Quant=zeros(len2,1);
Qdot=zeros(len2,6);
xydot=zeros(len2,2);
v=zeros(len2,6);

a=0:0.08:5;
len=length(a);
pinxy=zeros(len,2);

% save the first 5 seconds of the trajectory
i=1;
for t=0:0.08:4.96
    
    q1=2.618;
    q2=-0.6695;
    q3=1.2719;
    q4=3.1416;
    q5=1.2002;
    q6=-0.9821;

    q1dot=0;
    q2dot=0;
    q3dot=0;
    q4dot=0;
    q5dot=0;
    q6dot=0;
    
    v1=0;
    v2=0;
	v3=0;
    v4=0;
    v5=0;
    v6=0;

    x=0.249628*(t^2)-0.0332838*(t^3);
    y=0.0978*(t^2)-0.01304*(t^3);
    
    xdot=0.499256*t-0.09985144*t*t;
    ydot=0.1956*t-0.03912*t*t;
    
    xydot(i,1)=xdot;
    xydot(i,2)=ydot;
    
    pinxy(i,1)=x;
    pinxy(i,2)=y;
    
    xnew=x-0.375;
    ynew=y-0.5;
    
    % move the base of the arm
    lwr.base=SE3(x,y+0.35,0.5);  
    
    % find the angles from the quaternions
    g=lwr.fkine([q1 q2 q3 q4 q5 q6]);
    qua=g.UnitQuaternion;
    h=qua.s;
    theta=2*acos(h);
    Quant(i,1)=theta;
    
    Gwnies(i,1)=q1;
    Gwnies(i,2)=q2;
    Gwnies(i,3)=q3;
    Gwnies(i,4)=q4;
    Gwnies(i,5)=q5;
    Gwnies(i,6)=q6;
    
    Qdot(i,1)=q1dot;
    Qdot(i,2)=q2dot;
    Qdot(i,3)=q3dot;
    Qdot(i,4)=q4dot;
    Qdot(i,5)=q5dot;
    Qdot(i,6)=q6dot;
    
    v(i,1)=v1;
    v(i,2)=v2;
    v(i,3)=v3;
    v(i,4)=v4;
    v(i,5)=v5;
    v(i,6)=v6;
    
    Akroxyz(i,1)=x-0.523;
    Akroxyz(i,2)=y+0.6559;
    Akroxyz(i,3)=0.9049;   

    i=i+1;
end

for t=5.04:0.08:20
    
    % movement from 5 to 2o seconds
    xydot(i,1)=0;
    xydot(i,2)=0;
    
    if t>5 && t <=10
        q1=2.618;
        q2=-0.6695-0.047184*((t-5)^2)+0.0062912*((t-5)^3);
        q3=1.2719-(0.018912*((t-5)^2))+(0.002516*((t-5)^3));
        q4=3.1416;
        q5=1.2002-0.028272*((t-5)^2)+0.003769*((t-5)^3);
        q6=-0.9821;
 
        q1dot=0;
        q2dot=-0.094368*(t-5)+0.0188736*((t-5)^2);
        q3dot=-0.037824*(t-5)+0.007548*((t-5)^2);
        q5dot=-0.056544*(t-5)+0.011309*((t-5)^2);
        q4dot=0;    
        q6dot=0;              
    end
    
    if t>10 && t<=13
       q1=2.618;
       q2=-1.0627+0.04496*((t-10)^2)-0.00999259*((t-10)^3);
       q3=1.1143-(0.004533*((t-10)^2))+(0.001007*((t-10)^3));
       q4=3.1416;
       q5=0.9646+0.0495*((t-10)^2)-0.011*((t-10)^3);
       q6=-0.9821;
                
       q1dot=0;
       q2dot=0.089933*(t-10)-0.0299777*((t-10)^2);
       q3dot=-0.004533*(t-10)+0.0010074*((t-10)^2);
       q4dot=0;
       q5dot=0.099*(t-10)-0.033*((t-10)^2);
       q6dot=0;              
    end
    
    if t>13 && t<=17
        q1=2.618+0.39269*((t-13)^2)-0.06544*((t-13)^3);
        q2=-0.9278;
        q3=1.1007;
        q4=3.1416;
        q5=1.1131;
        q6=-0.9821;
        
        q1dot=0.785395*(t-13)-0.196348*((t-13)^2);
        q2dot=0;
        q3dot=0;
        q4dot=0;
        q5dot=0;
        q6dot=0;        
    end
    
    if t>17 && t<=20
        
        q1=4.712389;
        q2=-0.9278+0.01633*((t-17)^2)-0.00362*((t-17)^3);
        q3=1.1007+0.3514*((t-17)^2)-0.078088*((t-17)^3);
        q4=3.1416;       
        q5=1.1131-0.335066*((t-17)^2)+0.074459*((t-17)^3);
        q6=-0.9821+0.001026*((t-17)^2)-2.28148*(10^(-4))*((t-17)^3);
    
        q1dot=0;
        q2dot=0.03266*(t-17)-0.01088*((t-17)^2);
        q3dot=0.7028*(t-17)-0.234266*((t-17)^2);
        q4dot=0;        
        q5dot=-0.670133*(t-17)+0.223377*((t-17)^2);
        q6dot=0.002053*(t-17)-6.84*(10^(-4))*((t-17)^2);
    end
    
    qdot=[q1dot;q2dot;q3dot;q4dot;q5dot;q6dot];    
    vedot=lwr.jacobe([q1 q2 q3 q4 q5 q6])*qdot;
    v1=vedot(1,1);
    v2=vedot(2,1);
    v3=vedot(3,1);
    v4=vedot(4,1);
    v5=vedot(5,1);
    v6=vedot(6,1);    
      
    qua=g.UnitQuaternion;
    h=qua.s;
    theta=2*acos(h);
    Quant(i,1)=theta;
 
    g=lwr.fkine([q1 q2 q3 q4 q5 q6]); 
    Akroxyz(i,1)=g.t(1);
    Akroxyz(i,2)=g.t(2);
    Akroxyz(i,3)=g.t(3);
    
   
    Gwnies(i,1)=q1;
    Gwnies(i,2)=q2;
    Gwnies(i,3)=q3;
    Gwnies(i,4)=q4;
    Gwnies(i,5)=q5;
    Gwnies(i,6)=q6;
    
    Qdot(i,1)=q1dot;
    Qdot(i,2)=q2dot;
    Qdot(i,3)=q3dot;
    Qdot(i,4)=q4dot;
    Qdot(i,5)=q5dot;
    Qdot(i,6)=q6dot;
    
    v(i,1)=v1;
    v(i,2)=v2;
    v(i,3)=v3;
    v(i,4)=v4;
    v(i,5)=v5;
    v(i,6)=v6;
    
i=i+1;   
end

% simulation of the first 5 seconds
for t=0:0.1:5
    q1=2.618;
    q2=-0.6695;
    q3=1.2719;
    q4=3.1416;
    q5=1.2002;
    q6=-0.9821;
    
    x=0.249628*(t^2)-0.0332838*(t^3);
    y=0.0978*(t^2)-0.01304*(t^3);
       
    xnew=x-0.375;
    ynew=y-0.5;

    %platform and table
    k=plotcube([0.75 1 0.5],[xnew ynew 0],.1,[0 0 1]);   
    l=plotcube([0.35 0.35 0.6],[1.325 1.325 0],.1,[0 0 1]);
    
    % arm and movement of its base
    lwr.base=SE3(x,y+0.35,0.5);  
    lwr.plot([q1 q2 q3 q4 q5 q6],'workspace',work,'nobase','noname','noshadow');    
    
    if t ~= 5
    delete(k);
    end
    
end


s=0:0.08:5;
r=length(s);

a=5.08:0.08:10;
len3=length(a);

a=10.08:0.08:20;
len4=length(a);

Gwnies2=zeros(len3,6);
for i=1:len3
    for j=1:6
    Gwnies2(i,j)=Gwnies(r+i,j);
    end
end

Gwnies3=zeros(len4,6);
for i=1:len4
    for j=1:6
    Gwnies3(i,j)=Gwnies(len3+r+i,j);
    end
end

lwr.plot([Gwnies2(:,1) Gwnies2(:,2) Gwnies2(:,3) Gwnies2(:,4) Gwnies2(:,5) Gwnies2(:,6)],'workspace',work,'nobase','noname','noshadow');
delete(cyl);
lwr.plot([Gwnies3(:,1) Gwnies3(:,2) Gwnies3(:,3) Gwnies3(:,4) Gwnies3(:,5) Gwnies3(:,6)],'workspace',work,'nobase','noname','noshadow');

X2=X1+2.080237;
Y2=Y1+0.815;
Z2=Z1+0.5;


hold on

% plot the movement of the cylinder
cyl=mesh(X2,Y2,Z2);
plot(pinxy(:,1), pinxy(:,2),'linewidth',3);
plot3(Akroxyz(:,1),Akroxyz(:,2),Akroxyz(:,3),'linewidth',3)
hold off

pause(1);

fig=2;

t=0:0.08:20;

% create necessary diagramms
figure(fig);
hold on
title("Apokrisi gwniwn q1 q2 q3");
a1= plot(t, Gwnies(:,1));
M1="q1";
xlabel('t');
 
a2= plot(t, Gwnies(:,2));
M2="q2";
 
a3= plot(t, Gwnies(:,3));
M3="q3";
legend([a1,a2,a3], [M1, M2,M3]);

hold off
fig=fig+1;

figure(fig);
hold on
title("Apokrisi gwniwn q4 q5 q6");
a1= plot(t, Gwnies(:,4));
M1="q4";
xlabel('t');
 
a2= plot(t, Gwnies(:,5));
M2="q5";
 
a3= plot(t, Gwnies(:,6));
M3="q6";
legend([a1,a2,a3], [M1, M2,M3]);

hold off
fig=fig+1;


figure(fig);
hold on
title("Thesi tou akrou ston x");
plot(t,Akroxyz(:,1));
hold off
fig=fig+1;

figure(fig);
hold on
title("Thesi tou akrou ston y");
plot(t,Akroxyz(:,2));
hold off
fig=fig+1;

figure(fig);
hold on
title("Thesi tou akrou ston z");
plot(t,Akroxyz(:,3));
hold off
fig=fig+1;

figure(fig);
hold on
title("Gwnia toy akrou ws Quaternion");
plot(t,Quant(:,1));
hold off
fig=fig+1;

figure(fig);
hold on
title("Taxitita q1");
plot(t,Qdot(:,1));
hold off
fig=fig+1;

figure(fig);
hold on
title("Taxitita q2");
plot(t,Qdot(:,2));
hold off
fig=fig+1;

figure(fig);
hold on
title("Taxitita q3");
plot(t,Qdot(:,3));
hold off
fig=fig+1;

figure(fig);
hold on
title("Taxitita q4");
plot(t,Qdot(:,4));
hold off
fig=fig+1;

figure(fig);
hold on
title("Taxitita q5");
plot(t,Qdot(:,5));
hold off
fig=fig+1;

figure(fig);
hold on
title("Taxitita q6");
plot(t,Qdot(:,6));
hold off
fig=fig+1;

figure(fig);
hold on
title("Taxitita Platformas ston x");
plot(t,xydot(:,1));
hold off
fig=fig+1;

figure(fig);
hold on
title("Taxitita Platformas ston y");
plot(t,xydot(:,2));
hold off
fig=fig+1;

figure(fig);
hold on
title("Taxitita akrou v1");
plot(t,v(:,1));
hold off
fig=fig+1;

figure(fig);
hold on
title("Taxitita akrou v2");
plot(t,v(:,2));
hold off
fig=fig+1;

figure(fig);
hold on
title("Taxitita akrou v3");
plot(t,v(:,3));
hold off
fig=fig+1;

figure(fig);
hold on
title("Taxitita akrou v4");
plot(t,v(:,4));
hold off
fig=fig+1;

figure(fig);
hold on
title("Taxitita akrou v5");
plot(t,v(:,5));
hold off
fig=fig+1;

figure(fig);
hold on
title("Taxitita akrou v6");
plot(t,v(:,6));
hold off
fig=fig+1;