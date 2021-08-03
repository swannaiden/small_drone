%% read teensy log
fileID = fopen('log234.txt');
A = fread(fileID);
dataMsg = uint8(A);

clearvars -except dataMsg
i = 1;
msgLength = 142;
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
            receiver_t(receiver_idx) =  typecast(dataMsg(i+116:i+119),'uint32');
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
            for j = 1:3
                mag(imu_idx,j) = typecast(dataMsg(i+120+(j-1)*4:i+123+(j-1)*4),'single');
            end
            temp(imu_idx) =  typecast(dataMsg(i+132:i+135),'single');
            bar(imu_idx) =  typecast(dataMsg(i+136:i+139),'single');
            
            imu_t(imu_idx) =  typecast(dataMsg(i+116:i+119),'uint32');
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

fprintf('Bad bytes: %i, IMU frequency: %f, lidar: %f, sonar: %f, gps: %f\n', ...
    badByteCount,imu_idx/(-(single(timestamp(1))-single(timestamp(end)))*1e-6), ...
    lidar_idx/(-(single(timestamp(1))-single(timestamp(end)))*1e-6), ...
    sonar_idx/(-(single(timestamp(1))-single(timestamp(end)))*1e-6), ...
    gps_idx/(-(single(timestamp(1))-single(timestamp(end)))*1e-6));

gps_t_f = double(gps_t - timestamp(1))*1e-6;
imu_t_f = double(imu_t - timestamp(1))*1e-6;
receiver_t_f = double(receiver_t - timestamp(1))*1e-6;

gps_f(:,1) = double(gps(1:1:end,1))*1e-7;
gps_f(:,2) = double(gps(1:1:end,2))*1e-7;
gps_f(:,3) = double(gps(1:1:end,3))*1e-3;
gps_f(:,3) = gps_f(:,3)-gps_f(1,3);


gps_v = double(gps(:,4:6))*1e-3;

%% read black box
Tbl = readtable('BlackBox/rosebow2/flight1rose.csv');
VarNames = Tbl.Properties.VariableNames;
bb_roll = Tbl.heading_0_;
bb_pitch = Tbl.heading_1_;
bb_yaw = Tbl.heading_2_;
bb_rates = [Tbl.gyroADC_0_,Tbl.gyroADC_1_,Tbl.gyroADC_2_];
bb_alt = Tbl.BaroAlt ./100;
bb_t = double(Tbl.time)*1e-6;
bb_t = bb_t-bb_t(1);

%% convert imu data reference frames
clearvars roll pitch yaw
tmp = quatinv(imu(1,1:4));
tmp2 = quaternion(tmp);

for i = 1:length(imu)
   tmp = quaternion(imu(i,1:4));
   tmp3 = tmp2*tmp;
   tmp4 = eulerd(tmp2*tmp,'ZYX','frame');
   roll(i) = tmp4(1)*pi/180;
   pitch(i) = tmp4(2)*pi/180;
   yaw(i) = tmp4(3)*pi/180;
end
%% plot imu data
close all
figure(1);
subplot(3,2,1)
plot(imu_t_f,-roll)
hold on
plot(bb_t,bb_roll-bb_roll(1))
hold off
subplot(3,2,3)
plot(imu_t_f,pitch)
hold on
plot(bb_t,bb_pitch-bb_pitch(1))
hold off
subplot(3,2,5)
plot(imu_t_f,yaw)
hold on
plot(bb_t,bb_yaw-bb_yaw(1))
hold off

subplot(3,2,2)
plot(imu_t_f,-imu(:,5))
hold on
plot(bb_t,-(1/(180/pi))*bb_rates(:,1))
hold off
subplot(3,2,4)
plot(imu_t_f,imu(:,6))
hold on
plot(bb_t,-(1/(180/pi))*bb_rates(:,2))
hold off
subplot(3,2,6)
plot(imu_t_f,imu(:,7))
hold on
plot(bb_t,-(1/(180/pi))*bb_rates(:,3))
hold off

sgtitle('VN-100 (blue) vs onboard IMU (orange)')

%% altitude comparison
figure(2)
plot(gps_t_f,gps_f(:,3))
hold on
plot(bb_t,bb_alt)
hold off
title('gps height vs onboard altimeter');
%%
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
plot(sqrt(gps_v(:,1).^2+gps_v(:,2).^2+gps_v(:,3).^2))

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
close all


avgLat = mean(gps_f(:,1));
avgLong = mean(gps_f(:,2));
% minLat = min(gps_f(:,1));
% minLong = min(gps_f(:,2));
gps_avg(:,1) = deg2rad(gps_f(:,1)- gps_f(1,1));
gps_avg(:,2) = deg2rad(gps_f(:,2)- gps_f(1,2));

x = 6.3781e6 .* gps_avg(:,2).* cos(avgLat./360.*2*pi);
y = 6.3781e6 .* gps_avg(:,1);
z = (double(gps(:, 3)-gps(1,3))./1e3)';

figure(2)
subplot(3,1,1)
plot(x)
subplot(3,1,2);
plot(y)
subplot(3,1,3);
plot(sqrt(x.^2+y.^2));

figure
%z = zeros(size(x'));
%time = (1:length(x))';
time = (sqrt(gps_v(:,1).^2+gps_v(:,2).^2+gps_v(:,3).^2)).*2.237;


patch([180 -1 -1 180], [180 180 -1 -1], [0 0 0 0], 'white')  

surface([x';x'],[y';y'],[z;z],[time';time'],...
        'facecol','no',...
        'edgecol','interp',...
        'linew',2);

h = colorbar;
xlabel("east (m)");
ylabel("west (m)");
title("flight 1 data");

[roll, pitch, yaw] = quat2angle(quats(:, 1:4));
X = cos(yaw);%.*cos(pitch);
Y = sin(yaw);%.*cos(pitch);
%Z = sin(pitch);
Z = zeros(length(X),1);
X = X(1:floor(length(X)/length(gps)):end);
Y = Y(1:floor(length(X)/length(gps)):end);
Z = Z(1:floor(length(X)/length(gps)):end);
X = X(1:length(z));
Y = Y(1:length(z));
Z = Z(1:length(z));

quiver3(x,y,z',X,Y,Z);

% figure
% axis([0 180 0 180 0 35])
% 
% for i = 1:length(gps_t)
%     plot3(x(1:i), y(1:i), z(1:i), 'b')
%     drawnow limitrate
% end

%% Compare desired angle rates
rcCommand = (double(receiver(:,2:4))-172)./((1811-172)./2) -1;
rcCommand = rcCommand .*-1;
d_rates = 200.*rcCommand.*(1./(-1+(abs(rcCommand).*.7)))./360.*2.*pi;

figure
p1 = subplot(3,1,1);
plot(imu_t, imu(:,7))
hold on
plot(receiver_t, d_rates(:,3))

p2 = subplot(3,1,2);
plot(imu_t, imu(:,6))
hold on
plot(receiver_t, -d_rates(:,2))

p3 = subplot(3,1,3);
plot(imu_t, imu(:,5))
hold on
plot(receiver_t, d_rates(:,1))
linkaxes([p1 p2 p3], 'x')

%%
clc
% yaw(yaw < 0) = yaw(yaw < 0) + 2*pi;

%pitch = zeros(1,length(yaw));
%roll = zeros(1,length(yaw));
clear quats_tmp;
quats = imu(:,1:4);
for i = 1:1%length(quats)
    quat2eul(quats(i,1:4))
    tmp = quat2rotm(quats(i,:));
    tmp = rotx(180)*transpose(tmp);
    quats_tmp(i,:) = rotm2quat(tmp);
    quat2eul(quats_tmp(i,1:4))
end
%%
quats_int = interp1(imu_t_f,imu(:,1:4),gps_t_f);

figure
trajectory3(x,y,z, quats_int , .5, 100, 'gripen')
axis equal
axis([0 180 0 180 0 35])


%%



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
