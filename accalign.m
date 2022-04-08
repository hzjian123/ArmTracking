%clear all
%%  Data loading
lf = 0.25;
wrist_offset = [lf, 0, 0]';
load('./data/android-forward.mat')
load('./dataset/kinect-forward.mat');
addpath 'sensorProcessing'
filename = './dataset/forward-android.txt';
M = dlmread(filename,";");
euler = M(:,[3,4,5]);
accel = M(:,6:8);
gyro = M(:,15:17);
%%  Data pre-precessing
%series forward
kinect_start = 500;
android_start = 1;
kinect_end = 9450;
android_end = 8100;
[kinectTimeSeries, KinectWrist2torosr, KinectElbow2torosr ] = dataDrop(kinectTimeSeries, KinectWrist2torosr, KinectElbow2torosr, kinect_start, kinect_end);%Kinect data crop
[andTimeSeries, WristPoinsCloud, ElbowPoinsCloud, numPoints ] = pointDrop(andTimeSeries, WristPoinsCloud, ElbowPoinsCloud, numPoints, android_start, android_end);%Android points cloud data crop
[accel, gyro, euler, andPosPoinsCloud ] = sensorDrop(accel, gyro, euler, WristPoinsCloud, android_start, android_end);%Android readings data crop
M = M(1:android_end,:);%Android readings data crop

times = 3;% Resamping rate: time step = times*Android_time_step(0.037s)

dt = 0.037*times;
gravity = zeros(length(euler),3);
for i = 1:length(euler)
    gravity(i,:) = [0 0 9.8]*(eul2rotm(euler(i,:),'ZXY'));
end
%plot(normal(accel-gravity),'r');hold on;plot(normal(accel),'b')  %Gravity remove test
ori_ser = 1:floor(length(M)/times);% New time index
new_ser = round(ori_ser*times);%Map the new index to the old time series (Down sample)
new_ser(end) = length(ElbowPoinsCloud);
M = M(new_ser,:);
ElbowPoinsCloud = ElbowPoinsCloud(new_ser,:,:);%Down sample
cloudlen = size(ElbowPoinsCloud,3);
numPoints=numPoints(new_ser,:,:);%Down sample
accel = accel-gravity;%Linear acceleration
accel = accel(new_ser,:,:);%Down sample
max_index = length(M);%New vector length
[w2e_pos,acw] = deal(zeros(max_index,3));
alignedKinectElbow =  timeSeriesAlign(andTimeSeries,kinectTimeSeries, KinectElbow2torosr);alignedKinectWrist =  timeSeriesAlign(andTimeSeries,kinectTimeSeries, KinectWrist2torosr);
alignedKinectElbow = alignedKinectElbow(new_ser,:,:);alignedKinectWrist = alignedKinectWrist(new_ser,:,:);
sta = 400; endp = 800;
avg = zeros(endp-sta+1,3);
sum_series = zeros(1,100);
for r = sta:endp
    for e = 1:cloudlen
        cloud = squeeze(ElbowPoinsCloud(r,:,:));
        sum_series(e)= sum(sum(abs(repmat(cloud(:,e)-cloud,1,cloudlen))));
    end
    [~,mpos] = min(sum_series);
    avg(r-sta+1,:) = (cloud(:,mpos))';
end
%elbow = mean(ElbowPoinsCloud(sta:endp,:,:),3);
%% Elbow acceleration
cou = 0;
err = zeros(1,3375);
angp = zeros(3,3375);
refkin = alignedKinectElbow;
for alp = linspace(0,pi,15)
    for bet = linspace(0,pi,15)
        for gam= linspace(0,pi,15)
            for i = sta:endp
                rm = eul2rotm([alp bet gam],'ZYX');%([-1.5;-0.8;0.5]);%Rotation alignment
                alignedKinectElbow(i,:) = (rm*refkin(i,:)')';
            end
            kinect = alignedKinectElbow(sta:endp,:);
            cou = cou + 1;
            err(cou) = sum(sum(abs(kinect-avg)));
            angp(:,cou) = [alp bet gam ];
            
        end
    end
end
[~,minp] = min(err);
%plot(err)
for i = 1:1:max_index
    rm = eul2rotm( angp(:,minp)','ZYX');%([-1.5;-0.8;0.5]);%Rotation alignment   angp(:,minp)'
    alignedKinectElbow(i,:) = (rm*refkin(i,:)')';
end
scatter3(avg(:,1),avg(:,2),avg(:,3))
hold on
kinect = alignedKinectElbow(sta:endp,:);
scatter3(kinect(:,1),kinect(:,2),kinect(:,3))
angp(:,minp)%Rotation angle
mean(normal(abs(kinect-avg)))%Mean error
%cdfplot(normal(abs(kinect-avg)))

