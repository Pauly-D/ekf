%rotate test
%time in seconds
T = t/1000;
%delta T in seconds
dT=zeros(length(T),1);

for i = 1:length(T)-1
    %dT(i) = T(i+1)-T(i);
    dT(i)=0.005;
end

%anlges roll-pitch-yaw
roll=zeros(length(T)-1,1);
pitch=zeros(length(T)-1,1);
yaw=zeros(length(T)-1,1);

%eueler rates
eulerX = zeros(length(T)-1,1);
eulerY = zeros(length(T)-1,1);
eulerZ = zeros(length(T)-1,1);

%rotated accelerations
rAccX = zeros(length(T)-1,1);
rAccY = zeros(length(T)-1,1);
rAccZ = zeros(length(T)-1,1);

%velcotites
vX = zeros(length(T)-1,1);
vY = zeros(length(T)-1,1);
vZ = zeros(length(T)-1,1);

%positions
rX = zeros(length(T)-1,1);
rY = zeros(length(T)-1,1);
rZ = zeros(length(T)-1,1);

%get initial pitch and roll angles from gravity vector
%contribution in steady state to the acceleration axes (this helps the
%gyro integration to be better as the initial conditions reflect
%real attitude
pitch(1) = (pi/2)-(acos(accX(1)/9.734)); %pitch
roll(1) = (pi/2)-(acos(accY(1)/-9.734)); %roll
%yaw(1) = psiL;
yaw(1) = 0;


for i=1:length(T)-1
    
    %rotate gyro rates
    temp = (rotGyro(gyroX(i),roll(i),gyroY(i),gyroZ(i),pitch(i)))';
    
    %into euler rates
    eulerX(i) = temp(1);
    eulerY(i) = temp(2);
    eulerZ(i) = temp(3);
    
    %integrate to get euler angles
    roll(i+1)=roll(i)+dT(i)*eulerX(i); %roll
    pitch(i+1)=pitch(i)+dT(i)*eulerY(i); %pitch
    yaw(i+1)=yaw(i)+dT(i)*eulerZ(i); %yaw
    
    
end

%{
hold all
plot(T,roll);
plot(T,pitch);
plot(T,yaw);
%}


for i=1:length(T)-1
    
    temp = (rotAcc(accX(i),accY(i),accZ(i),roll(i),pitch(i),yaw(i)))';
    
    rAccX(i) = temp(1)-0.022*(T(i));
    rAccY(i) = temp(2);%-0.0005*(T(i));
    rAccZ(i) = temp(3);
    
    
    %integrate to get velocity
    vX(i+1)=vX(i)+dT(i)*rAccX(i);
    vY(i+1)=vY(i)+dT(i)*rAccY(i);
    %vZ(i+1)=vZ(i)+dt(i)*rAccZ(i);
    
    %integrate again to get position
    rX(i+1)=rX(i)+dT(i)*vX(i);%+dT(i)*0.5*rAccX(i).^2;
    rY(i+1)=rY(i)+dT(i)*vY(i);
    
end


windowSize = 300;
bF = (1/windowSize)*ones(1,windowSize);
aF = 1;

lpfaccX = filter(bF,aF,accX);
lpfrAccX = filter(bF,aF,rAccX);
lpfvX = zeros(length(T),1);
lpfrX = zeros(length(T),1);

for i=1:length(T)-1
    %integrate to get velocity
    lpfvX(i+1)=lpfvX(i)+dT(i)*lpfrAccX(i);
    %integrate again to get position
    lpfrX(i+1)=lpfrX(i)+dT(i)*lpfvX(i);
end

