kinect = alignedKinectElbow(st_loc:endp-1,:);
%% Elbow acceleration
cou = 0;
err = zeros(1,3375);
angp = zeros(3,3375);
avg = po;
refkin = kinect;
max_index = length(kinect);
for alp = linspace(0,pi,15)
    for bet = linspace(0,pi,15)
        for gam= linspace(0,pi,15)
            rm = eul2rotm([alp bet gam],'ZXY');%([-1.5;-0.8;0.5]);%Rotation alignment
            for i = 1:max_index
                kinect(i,:) = (rm*refkin(i,:)')';
            end
            cou = cou + 1;
            err(cou) = sum(sum(abs(kinect-avg)));
            angp(:,cou) = [alp bet gam ];
            
        end
    end
end
[~,minp] = min(err);
%plot(err)
for i = 1:1:max_index
    rm = eul2rotm( angp(:,minp)','ZXY');%([-1.5;-0.8;0.5]);%Rotation alignment   angp(:,minp)'
    kinect(i,:) = (rm*refkin(i,:)')';
end
scatter3(avg(:,1),avg(:,2),avg(:,3))
hold on
scatter3(kinect(:,1),kinect(:,2),kinect(:,3))
legend("HMM","Kinect")

rm2 = angp(:,minp)
mean(normal(abs(kinect-avg)))
%cdfplot())

