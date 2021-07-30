startBytes = uint8([49,50]);

receiverNew = uint8(1);
receiverData = int32([101,202,303,404,505,606,707,808]);
receiverDataBytes = typecast(receiverData,'uint8');

lidarNew = uint8(1);
lidarData = int32(99999);
lidarDataBytes = typecast(lidarData,'uint8');

sonarNew = uint8(1);
sonarData = int32(50000);
sonarDataBytes = typecast(sonarData,'uint8');

gpsNew = uint8(1);
gpsData = double([9.09,10.10,11.11]);
gpsDataBytes = typecast(gpsData,'uint8');
numSatsBytes = uint8(5);

imuNew = uint8(1);
imuData = single([10.01,12.02,13.03,14.04,15.05,16.06,17.07,18.08,19.09,20.20]);
imuDataBytes = typecast(imuData,'uint8');

timestamp = uint32(1111111);
timestampBytes = typecast(timestamp,'uint8');

stopBytes = uint8([60,59]);

dataMsg = [startBytes,receiverNew,receiverDataBytes,lidarNew, ...
    lidarDataBytes,sonarNew,sonarDataBytes,gpsNew,gpsDataBytes, ...
    numSatsBytes, imuNew, imuDataBytes, timestampBytes, stopBytes];
dataMsg = [dataMsg,[1,2,3],dataMsg];
%% read file
fileID = fopen('BlackBox/rosebow2/log224.txt');
A = fread(fileID);
dataMsg = uint8(A);

clearvars -except dataMsg
i = 1;
msgLength = 122;
idx = 1;
receiver_idx = 1;
lidar_idx = 1;
sonar_idx = 1;
gps_idx = 1;
imu_idx = 1;
display('reading data!')
badByteCount = 0;
while (i < length(dataMsg))
    if dataMsg(i) == uint8(49) && dataMsg(i+1) == uint8(50) && dataMsg(i+msgLength-2) == uint8(60)...
            && dataMsg(i+msgLength-1) == uint8(59)
        if (dataMsg(i+2) == uint8(1)) % new receiver
            for j = 1:8
                receiver(receiver_idx,j) = typecast(dataMsg(i+3+(j-1)*4:i+3+(j-1)*4+3),'int32');
            end
            receiver_idx = receiver_idx+1;
        end
        
        if (dataMsg(i+35) == uint8(1)) % new lidar
            lidar(lidar_idx,1) = typecast(dataMsg(i+36:i+39),'int32');
            lidar_idx = lidar_idx+1;
        end
        
        if (dataMsg(i+40) == uint8(1)) % new sonar
            sonar(sonar_idx,1) = typecast(dataMsg(i+41:i+44),'int32');
            sonar_idx = sonar_idx+1;
        end
        
        if (dataMsg(i+45) == uint8(1)) % new GPS
            for j= 1:7
                gps(gps_idx,j) = typecast(dataMsg(i+46+(j-1)*4:i+49+(j-1)*4),'int32');
            end
            gps_sat_num(gps_idx,1) = dataMsg(i+74);
            gps_t(gps_idx) = typecast(dataMsg(i+116:i+119),'uint32');
            gps_idx = gps_idx+1;
            
        end
        
        if (dataMsg(i+75) == uint8(1)) % new IMU
            for j= 1:10
                imu(imu_idx,j) = typecast(dataMsg(i+76+(j-1)*4:i+79+(j-1)*4),'single');
            end
            imu_idx = imu_idx+1;
        end
        
        timestamp(idx) = typecast(dataMsg(i+116:i+119),'uint32');
        idx = idx + 1;
        
        i = i + msgLength;
    else
        i = i+1;
        badByteCount = badByteCount+1;
    end
end

receiver;
lidar;
sonar;
gps;
gps_sat_num;
imu;
timestamp;
badByteCount;
fprintf('Bad bytes: %i, IMU frequency: %f, lidar: %f, sonar: %f, gps: %f\n', ...
    badByteCount,imu_idx/(-(single(timestamp(1))-single(timestamp(end)))*1e-6), ...
    lidar_idx/(-(single(timestamp(1))-single(timestamp(end)))*1e-6), ...
    sonar_idx/(-(single(timestamp(1))-single(timestamp(end)))*1e-6), ...
    gps_idx/(-(single(timestamp(1))-single(timestamp(end)))*1e-6));

plot(timestamp(2:end)-timestamp(1:end-1))

%%

gps_f(:,1) = double(gps(1:1:end,1))*1e-7;
gps_f(:,2) = double(gps(1:1:end,2))*1e-7;

gps_v = double(gps(:,4:6))*1e-3;

close all
figure
labels = ["x velocity", "y velocity", "z velocity"];
for i = 1:3
subplot(4,1,i)
plot(gps_v(:,i))
title(labels(i))
xlabel('t')
ylabel('m/s')
end
subplot(4,1,4)
plot(sqrt(gps_v(:,1).^2+gps_v(:,2).^2))
title('magnitude')
xlabel('t')
ylabel('m/s')


%%

% try differntiating position
gps_dx = zeros(size(gps_v)-1); 
gps_dx(:,1) = diff(x)./(double(gps_t(2:end)- gps_t(1:end-1))'./1e6);
gps_dx(:,2) = diff(y)./(double(gps_t(2:end)- gps_t(1:end-1))'./1e6);
gps_dx(:,3) = -diff(double(gps(:,3)))./(double(gps_t(2:end)- gps_t(1:end-1))'./1e3);

%figure(1)

subplot(4,1,1)
hold on
plot(gps_dx(:,2));
subplot(4,1,2)
hold on
plot(gps_dx(:,1));
subplot(4,1,3)
hold on
plot(gps_dx(:,3))
subplot(4,1,4)
hold on
plot(sqrt(gps_dx(:,1).^2+gps_dx(:,2).^2))


%% 

avgLat = mean(gps_f(:,1));
avgLong = mean(gps_f(:,2));
% minLat = min(gps_f(:,1));
% minLong = min(gps_f(:,2));
gps_avg(:,1) = deg2rad(gps_f(:,1)- gps_f(1,1));
gps_avg(:,2) = deg2rad(gps_f(:,2)- gps_f(1,2));

x = 6.3781e6 .* gps_avg(:,2).* cos(avgLat./360.*2*pi);
y = 6.3781e6 .* gps_avg(:,1);
figure(2)
subplot(3,1,1)
plot(x)
subplot(3,1,2);
plot(y)
subplot(3,1,3);
plot(sqrt(x.^2+y.^2));

figure
z = zeros(size(x'));
time = (1:length(x))';
%time = (sqrt(gps_v(:,1).^2+gps_v(:,2).^2)).*2.237;
surface([x';x'],[y';y'],[z;z],[time';time'],...
        'facecol','no',...
        'edgecol','interp',...
        'linew',2);
    
%  h = colorbar;
%  ylabel = get(h,'YTickLabel');
%  mm = repmat(' mph',size(ylabel,1),1);
%  ylabel = [ylabel mm];
%  set(h,'YTickLabel',ylabel);
xlabel("east (m)");
ylabel("west (m)");
title("flight 1 data");


 
%%
lat = double(gps(:, 1:10:end))*1e-7;
lon = double(gps(:,2:10:end))*1e-7;
[latlim, lonlim] = geoquadpt(lat, lon);
[latlim, lonlim] = bufgeoquad(latlim, lonlim, (max(lat) - min(lat)), ...
    (max(lon) - min(lon)));
figure;
usamap(latlim, lonlim)
geoshow(lat, lon, 'DisplayType', 'line')

latlim = [latlim(1)-.05, latlim(2) + .05];
lonlim = [lonlim(1)-.05, lonlim(2) + .05];

%%

%latlim = [37.78 37.84]; 
%lonlim = [-122.53 -122.40];

info = wmsinfo('http://basemap.nationalmap.gov/ArcGIS/services/USGSImageryOnly/MapServer/WMSServer?');
layer = info.Layer(1);



%[A, R] = wmsread(layer, 'Latlim', latlim, 'Lonlim', lonlim);
[A,R] = wmsread(layer,'Latlim',latlim, ...
                           'Lonlim',lonlim, ...
                           'ImageHeight',1024, ...
                           'ImageWidth',1024);
figure
usamap(A,R)
geoshow(A,R)
geoshow(lat, lon, 'DisplayType', 'line', 'Color', 'red', ...
    'Marker', 'diamond', 'MarkerEdgeColor', 'blue');