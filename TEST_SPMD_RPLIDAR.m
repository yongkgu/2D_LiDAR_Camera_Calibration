%%

delete(gcp('nocreate'))
p = parpool(2);

%%

clear; clc; close all;

spmd
    if labindex == 1 % spmdIndex
        
        count = 1;
        Point_number = 820;

        alldistances = zeros([2000 1]);
        allangles = zeros([2000 1]);

        distance = 0;
        angle = 0;
        bNewScan = 0;
        Quality = 0;

        pDistance = libpointer('doublePtr', distance);
        pAngle = libpointer('doublePtr', angle);
        pbNewScan = libpointer('int32Ptr', bNewScan);
        pQuality = libpointer('int32Ptr', Quality);

        [notfound, warnings]=loadlibrary('hardwarex', @hardwarex_proto);
        pRPLIDAR = calllib('hardwarex', 'CreateRPLIDARx');
        result = calllib('hardwarex', 'ConnectRPLIDARx', pRPLIDAR, 'RPLIDAR0.txt');
        disp(result)

        [~] = calllib('hardwarex', 'SetLidarSpinSpeedRequestRPLIDARx', pRPLIDAR, 1200);

        ClearCacheRPLIDAR(pRPLIDAR);

        t_prev = clock;
        t_prev = t_prev(6);
        
        while 1      
            
            t = clock;
            if t(6) < t_prev + 0.1

                [~] = calllib('hardwarex', 'GetScanDataResponseRPLIDARx', pRPLIDAR, pDistance, pAngle, pbNewScan, Quality);
                alldistances(count) = pDistance.value;
                allangles(count) = pAngle.value;
                count = count + 1;

            else

                labSend(alldistances(count-Point_number:count-1), 2);
                labSend(allangles(count-Point_number:count-1), 2);
                count = 1;
                t_prev = clock;
                t_prev = t_prev(6);
                
            end

        end

    elseif labindex == 2
        j = 1;
        Point_number = 820;
        alldistances2 = zeros(Point_number, 3000);
        allangles2 = zeros(Point_number, 3000);
        IMUData = zeros(3000, 4);

        IMU = serialport("COM4", 921600);
        flush(IMU);

        while 1
            received_alldistances2 = labReceive(1);
            received_allangles2 = labReceive(1);
            
            Data = splitlines(read(IMU, IMU.NumBytesAvailable, "string"));
            Data = split(Data(1:end-1, :), ",");
            IMUData(j, :) = str2double(Data(end, 2:end));
            
            alldistances2(:, j) = received_alldistances2;
            allangles2(:, j) = received_allangles2;

            j = j + 1;

        end
    end
end

%% Matrix Indexing

IMUData = IMUData{2};
alldistances = alldistances2{2};
allangles = allangles2{2};
idx = j{2} - 1;
eul_IMUData = quat2eul(IMUData, 'ZYX');

%% Simulate Scan Data and IMU Data

figure(1)

for i=1:idx
    scan = lidarScan(alldistances(:, i), allangles(:, i));
    subplot(1,2,1);
    scan.plot
    title(i);
    
    subplot(1,2,2);
    plot(1:idx, eul_IMUData(1:idx, 1), '.k')
    hold on
    plot(i, eul_IMUData(i, 1), '.r', 'MarkerSize', 40)
    hold off
    pause(0.01);

end

%%
figure
k = 92;
scan = lidarScan(alldistances(:, k), allangles(:, k));
plot(scan)
title(k);

figure
k = 93;
scan = lidarScan(alldistances(:, k), allangles(:, k));
plot(scan)
title(k);



