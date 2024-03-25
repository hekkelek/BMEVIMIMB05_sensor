clear;

tic;
s = serialport("COM3",115200,"Timeout",10,"FlowControl","hardware");
data = read(s,200*(20),"uint8");
toc;
clear s

startIdx = find(data==85);

arrAcc = zeros(1,3);
arrGyro = zeros(1,3);
arrMag = zeros(1,3);

for i = 1:length(startIdx)
    idx = startIdx(i);

    if( idx + (20)-1 >= length(data))
        break;
    end

    if( 170 ~= data(idx+(20)-1) )
        continue;
    end

    pack8 = uint8(data(idx:(idx+(20)-1)));

    Xa = typecast(pack8(2:3),  'int16');
    Ya = typecast(pack8(4:5),  'int16');
    Za = typecast(pack8(6:7),  'int16');
    X  = typecast(pack8(8:9),  'int16');
    Y  = typecast(pack8(10:11),'int16');
    Z  = typecast(pack8(12:13),'int16');

    Xm = typecast(pack8(14:15),'int16');
    Ym = typecast(pack8(16:17),'int16');
    Zm = typecast(pack8(18:19),'int16');

    arrAcc(i,:) = [Xa Ya Za];
    arrGyro(i,:) = [X Y Z];
    arrMag(i,:) = [Xm Ym Zm];

    disp("Xa = " + num2str(Xa));
    disp("Ya = " + num2str(Ya));
    disp("Za = " + num2str(Za));
    disp("X = " + num2str(X));
    disp("Y = " + num2str(Y));
    disp("Z = " + num2str(Z));
    disp("Xm = " + num2str(Xm));
    disp("Ym = " + num2str(Ym));
    disp("Zm = " + num2str(Zm));
    disp('\n')
end

% Conversion to SI units
% The magnetometer presents values in 0.3 microtesla units
arrMag = arrMag.*0.3*10^-6;  % Conversion to Tesla units
% The accelerometer presents values in Gs with 4096 LSB/G sensitivity
arrAcc = arrAcc.*9.81./4096;  % Conversion to meter/second^2
% The gyroscope presents values in dps with 16.4 LSB/dps sensitivity
arrGyro = arrGyro./16.4./360.*(2*pi);  % Conversion to radians per second

% Plotting figures
figure;
plot(arrGyro(:,1));
title("Gyroscope X [rad/sec]");

figure;
plot(arrGyro(:,2));
title("Gyroscope Y [rad/sec]");

figure;
plot(arrGyro(:,3));
title("Gyroscope Z [rad/sec]");

figure;
plot(arrAcc(:,1));
title("Accelerometer X [m/s^2]");

figure;
plot(arrAcc(:,2));
title("Accelerometer Y [m/s^2]");

figure;
plot(arrAcc(:,3));
title("Accelerometer Z [m/s^2]");

figure;
plot(arrMag(:,1));
title("Magnetometer X [Tesla]");

figure;
plot(arrMag(:,2));
title("Magnetometer Y [Tesla]");

figure;
plot(arrMag(:,3));
title("Magnetometer Z [Tesla]");
