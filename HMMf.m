clear all
lf = 0.25;
wrist_offset = [lf, 0, 0]';
load('./dataset/android-complete-forward36.mat')
load('./dataset/kinect-forward.mat');
addpath 'C:\Users\Eachann\OneDrive - The Hong Kong Polytechnic University\each\ARM\sensorProcessing'
filename = './dataset/forward-android.txt';
M = dlmread(filename,";");
euler = M(:,[2,4,5]);
accel = M(:,6:8);
gyro = M(:,12:14);
%series forward 
kinect_start = 500;
android_start = 1;
kinect_end = 9450;
android_end = 8100;

[kinectTimeSeries, KinectWrist2torosr, KinectElbow2torosr ] = dataDrop(kinectTimeSeries, KinectWrist2torosr, KinectElbow2torosr, kinect_start, kinect_end);
[andTimeSeries, WristPoinsCloud, ElbowPoinsCloud, numPoints ] = pointDrop(andTimeSeries, WristPoinsCloud, ElbowPoinsCloud, numPoints, android_start, android_end);
[accel, gyro, euler, andPosPoinsCloud ] = sensorDrop(accel, gyro, euler, WristPoinsCloud, android_start, android_end);

M = M(1:android_end,:);
times = 5.4;
dt = 0.037*times;
m = zeros(floor(length(M)/times),18);
for i=1:18
    m(:,i) =  resample(M(:,i),length(m),length(M));
end
ori_ser = 1:length(m);
new_ser = round(ori_ser/length(m)*length(M));
new_ser(end) = length(ElbowPoinsCloud);
ElbowPoinsCloud = ElbowPoinsCloud(new_ser,:,:);
M = m;
max_index = length(M);
accel = removeGravity(accel);
[w2e_pos,ac2] = deal(zeros(max_index,3));
alignedKinectElbow =  timeSeriesAlign(andTimeSeries,kinectTimeSeries, KinectElbow2torosr);
alignedKinectElbow = alignedKinectElbow(new_ser,:,:);
for i = 1:1:max_index
    eulerOrientation = M(i,[2,4,5]);
    and_rot = eul2rotm(eulerOrientation');
    wrist_offset = [lf, 0, 0]';
    w2e_pos(i,:)=(and_rot*wrist_offset)';
    ac2(i,:) = (and_rot*accel(i,:)')';
end
newa = [zeros(1,3);diff(diff(w2e_pos))/dt/dt;zeros(1,3)];
ace = ac2 - newa;

Pt1 = zeros(50^3,3);
mxlen = 50^3;
id = zeros(1,mxlen);
for q = 1:50
    for w = 1:50
        for e = 1:50
            pidx = e+50*(w-1)+50*50*(q-1);
            Pt1(pidx,:) = [q,w,e];
        end
    end
end
PI = pi;
stepNum = 36;
stepLen = PI/stepNum;
joint1 = -PI/3:stepLen:PI;
joint2 = -PI/6:stepLen:PI*2/3;
pt1 = combvec(joint1, joint2)';
ptlen = length(pt1);
State = zeros(ptlen,4);
Stposi = zeros(ptlen,6);
Stateind = zeros(ptlen,2);
idx = 1;
for r = 1:ptlen
    pt = pt1(r,:);
    dis = abs(pt1 - pt);
    dis = dis(:,1)+dis(:,2);
    mindis = 20*pi/180;
    id = find(dis<mindis);
    lenid = length(id);
    State(idx:idx+lenid-1,:) = [repmat(pt1(r,:),lenid,1),pt1(id,:)];
    Stateind(idx:idx+lenid-1,:) = [repmat(pt1(r,:),lenid,1)];
    idx=idx + lenid;
end
State = pt1;
st1 = State(:,1);st2 = State(:,2);
po1 = cal_elbow_loc(st1',st2',lf);
[~,stpoi] = min(sum((abs(alignedKinectElbow(460,:)'-po1))));
sig = 1;
emis = ones(ptlen,1);
stnum = 1;
eststate = zeros(1,max_index);
for u = 460:max_index
    trans = zeros(ptlen);
    if u == 460
        stnum = stpoi;%randi([1 idx],1,1);
        eststate(u) = stnum;
    else
        cloud = squeeze(ElbowPoinsCloud(u,:,1:numPoints(u)));%scatter3(cloud(1,:),cloud(2,:),cloud(3,:))
        acc = ace(u,:);%st = 1;endp = 400;scatter3(alignedKinectElbow(st:endp,1),alignedKinectElbow(st:endp,2),alignedKinectElbow(st:endp,3));
        lastpt = State(stnum,:);
        dis = abs(lastpt - pt1);
        dis = dis(:,1)+dis(:,2);
        id = find(dis<mindis);
        lenid = length(id);
        state_is = State(id,:);
        for z = 1:lenid
            state_i = state_is(z,:);
            dis_j = abs(state_i - pt1);
            dis_j = dis_j(:,1)+dis_j(:,2);
            id_j = find(dis_j<mindis);
            lenid_j = length(id_j);
            state_js = State(id_j,:);
            for w = 1:length(state_js)
                state_j = state_js(w,:);
                pr1 = 1;
                pos1 = cal_elbow_loc(lastpt(1),lastpt(2),lf);
                pos2 = cal_elbow_loc(state_i(1),state_i(2),lf);
                pos3 = cal_elbow_loc(state_j(1),state_j(2),lf);%scatter3(pos1(1),pos1(2),pos1(3))
                bias  = sum(abs(cloud - pos2));
                if min(bias)< 0.5
                    pr3 = 1;
                    vi = (pos2-pos1)/dt;
                    vj = (pos3-pos2)/dt;
                    accl = ((vj-vi)/dt)';
                    pr2 = 1/(2*pi)^0.5/sig*exp(-sum(acc-accl).^2/2/sig^2);
                    trans(z,w) = pr2;
                end
            end
        end
        p = zeros(1,ptlen);
        p(stnum) = 1;
        TRANS = [0 p; zeros(size(trans,1),1) trans];
        em = [zeros(1,size(emis,2)); emis];
        fi= hmmviterbi([1,1], TRANS,em)
        stnum = fi(2)-1;
        eststate(u) = stnum;
    end
end
endp = u-1;
st = 460;
po = zeros(endp-st,3);
for q = 1:endp-st
    
    po(q,:) = cal_elbow_loc(State(eststate(q+st-1),1),State(eststate(q+st-1),2),lf);
end
%po = [-po(:,3),-po(:,1),po(:,2)];
plot3(po(:,1),po(:,2),po(:,3),'o')
hold on
plot3(alignedKinectElbow(st:endp,1),alignedKinectElbow(st:endp,2),alignedKinectElbow(st:endp,3),'o')



p=[0.7,10];
TRANS = [0.1,0.9;0.9,0.1];
emis = [1;1];
tran = [0 p; zeros(size(TRANS,1),1) TRANS];
em = [zeros(1,size(emis,2)); emis];
fi= hmmviterbi([1,1,1,1,1], tran,em)