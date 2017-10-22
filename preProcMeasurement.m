%run preProcGPS to import lat, long, height values for run
%run before fullEKF to populate GPS measurements
zk=zeros(length(zGPS)-1,3); %measurement vector (3 measurements)

%only GPS measurements :)
for i=1:length(zGPS);
    zk(i,1)=v(i); %GPS velocity pre-processed to give line direction velocity (mappped onto line)
    zk(i,2)=r(i); %GPS X and Y NED deltas projected onto line direction for line position r
    zk(i,3)=zGPS(i);%GPS height (referenced to zero at towerA so expect negative  
end
%Legacy code from different measurement vector
%{ 
k=2;
%loop through longest measurement data set (IMU)
for i=2:length(roll)
    %do rem(i,20) for 100 Hz IMU
    %if i = multiple of 40 (40*5ms = 200ms) then know GPS measurement avail.
    if((rem(i,40) == 0))
        %update measurement vector with GPS data every 5Hz
        zk(i,1)=v(k); %GPS velocity pre-processed to give line direction velocity
        zk(i,2)=r(k); %GPS X and Y NED deltas projected onto line direction for line position r
        zk(i,3)=zNED(k);%GPS height (referenced to zero at towerA so expect negative 
        k=k+1;
    else 
        %set GPS measurements = 0 (use this to check wich hk to use in main
        zk(i,1)=0;
        zk(i,2)=0;
        zk(i,3)=0;
    end
    %always update roll,pitch,yaw at the known rate    
    zk(i,4)= roll(i);%roll measurement (from preprocIMU ie: no noise assumption as EKF does)
    zk(i,5)=pitch(i); %pitch measurement(from preprocIMU ie: no noise assumption as EKF does)
    zk(i,6)=0; %line angle constant zero measurement (compare to local yaw state)
end
%}