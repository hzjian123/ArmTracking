function rotm = eul2rotm(eul)
    %% Translate the euler angle to rotation matrix 
    rotm = zeros(3,3);
    
    s_1 = sin(eul(1,1)); % theta_z or theta_z1
    c_1 = cos(eul(1,1));
    s_2 = sin(eul(2,1)); % theta_y
    c_2 = cos(eul(2,1));
    s_3 = sin(eul(3,1)); % theta_x or theta_z2
    c_3 = cos(eul(3,1));
    
    
    %            | c_1*c_2-s_1*s_2*s_3     s_1*s_3     c_1*s_2 + s_1*c_2*s_3|
    % R(Theta) = |-s_1*c_2-c_1*c_2*s_3     c_1*c_3    -s_1*s_2 + c_1*c_2*s_3|
    %            |-s_2*c_3                -s_3                 c_2*c_3      |
    
    rotm(1,1) =  c_1*c_3-s_1*s_3*s_2;
    rotm(1,2) =  s_1*s_2;
    rotm(1,3) =  c_1*s_3 + s_1*c_3*s_2;

    rotm(2,1) =  -s_1*c_3-c_2*c_3*s_2;
    rotm(2,2) =  c_1*c_2;
    rotm(2,3) =  -s_1*s_3 + c_1*c_3*s_2;

    rotm(3,1) = -s_3*c_2;
    rotm(3,2) =  -s_2;
    rotm(3,3) =  c_3*c_2;  
end