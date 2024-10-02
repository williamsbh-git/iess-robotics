clear;
data = readmatrix("Arduino Data (2).xlsx");

data = data(:,2:4);
data = [0,0,0;data];

accelData = data(2:2:end,:);
gyroData = data(1:2:end,:)*pi/180;

%figure;
%subplot(2,1,1);

%plot(accelData);

%subplot(2,1,2);

%plot(gyroData);

T = 1;

pw = 4;
pb = 2;
qw = 100;
qb = 100;
rtheta = 1e-5;
rw = 0.1;

Ad = [1,0,0;T,1,0;0,0,1];

Q = [qw,0,0;0,0,0;0,0,qb];
Qd = [T*qw,0.5*T^2*qw,0;0.5*T^2*qw,(1/3)*T^3*qw,0;0,0,T*qb];

R = [rtheta,0;0,rw];
Rd = R./T;

Cd = [0,1,0;1,0,1];
I = eye(3);

yww = gyroData(:,1);
ywb = gyroData(:,2);
ywn = gyroData(:,3);

yar = accelData(:,1);
yat = accelData(:,2);
yan = accelData(:,3);

P = [T*pw,0.5*T^2*pw,0; 0,(1/3)*T^3*pw,0;0,0,T*pb];

uData = zeros(3,1);
PData = zeros(3,3);
yData = zeros(3,1);

for i = 2:50

    yt = atan2(-yar(i),yat(i)) + yan(i);

    ytPrev = atan2(-yar(i-1),yat(i-1)) + yan(i-1);

    y = [yww(i);yt;ywb(i)];
    yk = Cd * y;

    uPrev = [yww(i-1);ytPrev;ywb(i-1)];
    uPredict = Ad * uPrev;

    P = Ad * P * transpose(Ad) + Qd;

    KalGain = P * transpose(Cd) * inv((Cd * P * transpose(Cd) + Rd));

    uk = uPredict + KalGain * (yk - Cd * uPredict);

    Pk = (I - KalGain * Cd)*P;

    uData = vertcat(uData,uk);
    yData = vertcat(yData,y);

end

figure;

plot(uData);
hold on;
plot(yData);

legend('Prediction Data', 'Measured Data');