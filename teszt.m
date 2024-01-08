clear;

s = serialport("COM3",115200,"Timeout",10);
data = read(s,200*(6*4+2),"uint8");
clear s

startIdx = find(data==85);

arrAcc = zeros(1,3);
arrGyro = zeros(1,3);

for i = 1:length(startIdx)
    idx = startIdx(i);

    if( idx + (6*4+2)-1 >= length(data))
        break;
    end

    if( 170 ~= data(idx+(6*4+2)-1) )
        continue;
    end

    pack8 = uint8(data(idx:(idx+(6*4+2)-1)));

    Xa = typecast(pack8(2:5),  'single');
    Ya = typecast(pack8(6:9),  'single');
    Za = typecast(pack8(10:13),'single');
    X  = typecast(pack8(14:17),'single');
    Y  = typecast(pack8(18:21),'single');
    Z  = typecast(pack8(22:25),'single');
    
    arrAcc(i,:) = [Xa Ya Za];
    arrGyro(i,:) = [X Y Z];

    disp("Xa = " + num2str(Xa));
    disp("Ya = " + num2str(Ya));
    disp("Za = " + num2str(Za));
    disp("X = " + num2str(X));
    disp("Y = " + num2str(Y));
    disp("Z = " + num2str(Z));
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
