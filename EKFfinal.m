%Full EKF symbolic
%------------------------------------------------------------------------------------
%{
%symbolic function declarations====================================================== 
    %f(xi,ui)
syms f(a_r, v_r, r_r, sag, phi, the, psi, dphi, dthe, dpsi, dt) ...  %process model declaration
     h(v_r,r_r,sag);
     %h(xi_+1)
     %h2(v_r,r_r,sag,phi,psi);
%}
%====================================================================================
%% process function------------------------------------------------------------------
%{
%inputs = a_r, dphi, dthe, dpsi, dt (line acc, euler rates, and dt)
 f(a_r, v_r, r_r, sag, phi, the, psi, dphi, dthe, dpsi, dt)=...
    [v_r + dt*a_r;... % line velocity state
     r_r + dt*v_r;... % line position state
     phi + dt*dphi;...% roll angle state
     the + dt*dthe;...% pitch angle state
     psi + dt*dpsi;...% yaw angle state
     sag;...          % sag parameter state
                        ]'; %note the inverse!
 %-----------------------------------------------------------------------------------
 %process jacobian F
jF=jacobian(f,[v_r;r_r;phi;the;psi;sag]);

%====================================================================================
%% observation function h
%take in only states and produce forms required by measurements
h(v_r,r_r,sag)=...
    [v_r;...               % line velocity state (compared with projected v GPS)
     r_r;...               % line position state (compared with X and Y GPS projected to r)
     sag*r_r.^2+b*r_r+c;...% expected height from parabolic line position (compared to Z GPS)
                       ]'; %note the inverse!
 %--------------------------------------------------------------------------
 %measurement jacobain H
jH=jacobian(h,[v_r;r_r;phi;the;psi;sag]);
%}
%==========================================================================

%% P0,R,Q Selection=========================================================
%initial state error covariance
%Pk=6X6
Pprior=zeros(6,6,1); 
Ppost=zeros(6,6,1); %Ppost=zeros(6,6,length(data));
%pretty sure about the first state estimate (might need to change this later...)
%set this to the first measurement cov R (CHALMERS!: optimal initialization
%Pprior=Pprior+eye(6)*0.01;
P=zeros(6,6,1)+eye(6)*0.1;
Pprior(:,:,1)=P;


%initial measurement covariance
%include dynamically changing GPS HDOP and VDOP later...
% R=3X3
R=[0.01,0,0,;...    %vGPS not very good at low speeds             v
   0,0.0009,0;...   %RTK position pretty good                     r
   0,0,0.001]*100000;      %height Z GPS notoriously bad                 z
   
%initial process covariance
%Q=6X6
Q=[  0.04, 0, 0, 0, 0, 0;...%velocityUncertainty (VAR)
     0, 0.0225,0,0,0,0;...    %positionUncertainty (VAR)
     0,0,0.01,0,0,0;...   %rollUncertainty (VAR)
     0,0,0,0.05,0,0;...   %pitchUncertainty(VAR)
     0,0,0,0,0.01,0;...   %yawUncertainty (VAR)
     0,0,0,0,0,0.00000001.^2];  %sag barely changes
%==========================================================================
%%
%pre alocate space----------------------------------------------------
Xk=zeros(length(T)-1,6); %state vector (6 states)
xk=zeros(length(T)-1,6);
%hk=zeros(length(T)-1,2); %expected measurements based on states 
ys=zeros(length(zk)-1,3); %residual save variable for plotting propagation 

%% initialize first state vector and measurement
i=1;
%prediction
% aa priori state estimate Xo using starting inputs
%f(a_r, v_r, r_r, sag, phi, the, psi, dphi, dthe, dpsi, dt)
Xk(i,:)=f(rAccX(i),0,0,sag0,0,0,0,eulerX(i),eulerY(i),eulerZ(i),dT(i));
%% very first update (measurements available) 
%h(v_r,r_r,sag)
hk = double(h(Xk(i,1),Xk(i,2),Xk(i,6)));
%residual
yk = zk(i,:)' - hk';
%save residual
ys(i,:)=yk; 
%residual covariance
%Sk = jH(Xk(i,1),Xk(i,2),Xk(i,6))*Pprior(:,:,i)*jH(Xk(i,1),Xk(i,2),Xk(i,6))' + R;
Sk = jH(Xk(i,1),Xk(i,2),Xk(i,6))*P*jH(Xk(i,1),Xk(i,2),Xk(i,6))' + R;
%kalman gain
%K = Pprior(:,:,i)*jH(Xk(i,1),Xk(i,2),Xk(i,6))'*inv(Sk);
K = P*jH(Xk(i,1),Xk(i,2),Xk(i,6))'*inv(Sk);
%correct prediction
%aa posteriori state
Xk(i,:) = Xk(i,:)' + K*yk;
%compute state error covariance Pk
%Ppost(:,:,i) = double((eye(6) - K*jH(Xk(i,1),Xk(i,2),Xk(i,6))*Pprior(:,:,i)));
P = double((eye(6) - K*jH(Xk(i,1),Xk(i,2),Xk(i,6)))*P);
%save posteriori state error covariance 
Ppost(:,:,i)=P;
%%

%% Kalman Time =============================================================
for i=2:length(T)-1
%always do state propagation!
%a priori state estimate (based on previous state and current input)
%x=f(xi,ui)
Xk(i,:)=double(f(rAccX(i),...  %line acceleration (INPUT)
                 Xk(i-1,1),... %line velocity state
                 Xk(i-1,2),... %line position state
                 Xk(i-1,6),... %sag state
                 Xk(i-1,3),... %roll state
                 Xk(i-1,4),... %pitch state
                 Xk(i-1,5),... %yaw state
                 eulerX(i),eulerY(i),eulerZ(i),...%rotated gyro rates (INPUT)
                 dT(i))); %incremental change in time (INPUT)
 %xk(i,:)=Xk(i,:);
 % a priori state error covariance using jacobian F (evaluate at current state
 tempF = double(jF(rAccX(i),...  %line acceleration (INPUT)
                 Xk(i-1,1),... %line velocity state
                 Xk(i-1,2),... %line position state
                 Xk(i-1,6),... %sag state
                 Xk(i-1,3),... %roll state
                 Xk(i-1,4),... %pitch state
                 Xk(i-1,5),... %yaw state
                 eulerX(i),eulerY(i),eulerZ(i),...%rotated gyro rates (INPUT)
                 dT(i))); %incremental change in time (INPUT)
 
  %Pprior=FPF'+Q
  %Pprior(:,:,i) = tempF*Ppost(:,:,i-1)*tempF' + Q;
   P = tempF*P*tempF' + Q;
  %save priori state error covariance
   Pprior(:,:,i)=P;
  %if dont have measurement then require Ppost as value
  %Ppost(:,:,i)=Pprior(:,:,i);
  
  
  %% measurement update @ 5Hz
  if(rem(i,40)==0)
      hk = double(h(Xk(i,1),Xk(i,2),Xk(i,6)));
      %residual
      yk = zk(i/40,:)' - hk';
      %save residual
      ys(i,:)=yk;
      %residual covariance
      %Sk = jH(Xk(i,1),Xk(i,2),Xk(i,6))*Pprior(:,:,i)*jH(Xk(i,1),Xk(i,2),Xk(i,6))' + R;
      Sk = jH(Xk(i,1),Xk(i,2),Xk(i,6))*P*jH(Xk(i,1),Xk(i,2),Xk(i,6))' + R;
      %kalman gain
      %K = Pprior(:,:,i)*jH(Xk(i,1),Xk(i,2),Xk(i,6))'*inv(Sk);
      K = P*jH(Xk(i,1),Xk(i,2),Xk(i,6))'*inv(Sk);
      %correct prediction
      %a posteriori state
      Xk(i,:) = Xk(i,:)' + K*yk;
      %compute state error covariance Pk
      %Ppost(:,:,i) = double((eye(6) - K*jH(Xk(i,1),Xk(i,2),Xk(i,6))*Pprior(:,:,i)));
      P = double((eye(6) - K*jH(Xk(i,1),Xk(i,2),Xk(i,6)))*P);
      %save posteriroi state error covariance
      Ppost(:,:,i/40)=P;
  end
  
end

%xOut=zeros(length(Xk),1);
%yOut=zeros(length(Xk),1);
%zOut=zeros(length(Xk),1);

for i=1:length(Xk)
xOut(i)=Xk(i,2)*sin(psiL);
yOut(i)=Xk(i,2)*cos(psiL);
zOut(i)=Xk(1,6)*Xk(i,2).^2+b*Xk(i,2)+c;
end

%{
for i=1:length(Xk)
p(i,1)=atan(2*Xk(i,6)*Xk(i,2)+b);
end
plot(p);
%}

grid on
figure
scatter3(xOut,yOut,zOut,'.r');
%plot3(xOut,yOut,zOut);
 
 
for i=1:length(zk)-1
temp=diag(Ppost(:,:,i));
%Pdiag(i,1)=temp(1,1);  %velocity
%Pdiag(i,2)=temp(2,1);  %position
%Pdiag(i,3)=temp(3,1); 
%Pdiag(i,4)=temp(4,1);
%Pdiag(i,5)=temp(5,1);
Pdiag(i,6)=temp(6,1);
end
%}


figure
hold on
subplot(2,1,1);
plot(Pdiag(:,1));
hold 
plot(Pdiag(:,2));
%plot(Pdiag(:,3));
%plot(Pdiag(:,4));
%plot(Pdiag(:,5));
subplot(2,1,2);
plot(Pdiag(:,6));

                



