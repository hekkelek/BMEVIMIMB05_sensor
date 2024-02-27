clear;

s = serialport("COM3",115200,"Timeout",10,"FlowControl","hardware");
data = read(s,200*(32),"uint8");
clear s

startIdx = find(data==85);

arrAcc = zeros(1,3);
arrGyro = zeros(1,3);
arrMag = zeros(1,3);

for i = 1:length(startIdx)
    idx = startIdx(i);

    if( idx + (32)-1 >= length(data))
        break;
    end

    if( 170 ~= data(idx+(32)-1) )
        continue;
    end

    pack8 = uint8(data(idx:(idx+(32)-1)));

    Xa = typecast(pack8(2:5),  'single');
    Ya = typecast(pack8(6:9),  'single');
    Za = typecast(pack8(10:13),'single');
    X  = typecast(pack8(14:17),'single');
    Y  = typecast(pack8(18:21),'single');
    Z  = typecast(pack8(22:25),'single');

    Xm = typecast(pack8(26:27),'int16');
    Ym = typecast(pack8(28:29),'int16');
    Zm = typecast(pack8(30:31),'int16');

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

figure;
plot(arrGyro(:,1));
title("Gyroscope X");

figure;
plot(arrGyro(:,2));
title("Gyroscope Y");

figure;
plot(arrGyro(:,3));
title("Gyroscope Z");

figure;
plot(arrAcc(:,1));
title("Accelerometer X");

figure;
plot(arrAcc(:,2));
title("Accelerometer Y");

figure;
plot(arrAcc(:,3));
title("Accelerometer Z");

figure;
plot(arrMag(:,1));
title("Magnetometer X");

figure;
plot(arrMag(:,2));
title("Magnetometer Y");

figure;
plot(arrMag(:,3));
title("Magnetometer Z");
