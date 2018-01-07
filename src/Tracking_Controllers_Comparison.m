clc;
clear;
close all;

vr = 1;
wr = 0;
thetar = 0;
xref = 0 ;
yref = 10 ;
xrefvec = xref ;
yrefvec = yref ;

x1 = 0;
y1 = 0;
theta1 = 0;
x1ref = 0;
y1ref = 0;
theta1ref = 0;


deltat = 0.05 ;

% Choice of Controller
ch = 2 ; 


for i = 0:deltat:10
    %Reference
    xref = xref + vr*cos(thetar)*deltat ;
    yref = yref + vr*sin(thetar)*deltat ;
    xrefvec = [xrefvec xref];
    yrefvec = [yrefvec yref];
    
    %Leader
    Te = [ cos(theta1) , sin(theta1) , 0  ;
          -sin(theta1) , cos(theta1) , 0  ;
               0       ,       0     , 1 ];
    error = Te * [xref-x1 ; yref-y1 ; thetar-theta1];
    
    if ch == 1
    
    %Controller 1 (Mobile robot on reference path)
    g = 2 ;
    omega_n  = sqrt(wr^2 + g*vr^2);
    k1 = 2*0.7*omega_n ;
    k2 = 2*0.7*omega_n ;
    k3 = g*abs(vr);
    K = [-k1  ,     0        , 0     ; 
           0  ,-sign(vr)*k2 , -k3 ];
    vt = K*error;
    u = [ vr*cos(error(3)) - vt(1) ; wr - vt(2)];
    
    else
    %Controller 2( Backstepping)
    k1 = 0.4;
    k2 = 5;
    k3 = 2;
    u = [vr*cos(error(3)) + k1*error(1) ; 
         wr + k2*vr*error(2) + k3*vr*sin(error(3))];
    end
    
    %Kinematics
    x1 = x1 + cos(theta1)*u(1)*deltat ;
    y1 = y1 + sin(theta1)*u(1)*deltat ;
    theta1 = theta1 + 1*u(2)*deltat ;
    x1ref = [x1ref x1];
    y1ref = [y1ref y1];
    theta1ref = [theta1ref theta1];
    
end

figure(1);
plot(x1ref,y1ref);grid on;hold on;
plot(xrefvec,yrefvec);
    
    
    