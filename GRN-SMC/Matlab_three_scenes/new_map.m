clear;close all;clc;
global cell_wid;
global distent_fish;                        %与目标的安全距离
global distent_capter;                      %与其他机器人的安全距离
global distent_detect ;                     %探测距离
global v ;                                  %机器人运动速度

p_fish = [10,22.5,1
    10,20.5,1];
time_step    = 360; 

% rand('seed',0);
num_targets   = 2;  % 目标个数
num_robot     = 10; % 群体机器人的个数
num_barrier   = 2;  % 障碍物个数

%滑模控制器相关矩阵
x_mat = zeros(time_step,num_robot);
y_mat = zeros(time_step,num_robot);
x_d_mat = zeros(time_step,num_robot);
y_d_mat = zeros(time_step,num_robot);
x_p_mat = zeros(time_step,num_robot);
y_p_mat = zeros(time_step,num_robot);
theta_mat = zeros(time_step, num_robot);
theta_d_mat = zeros(time_step, num_robot);
error = zeros(time_step,num_robot);
rec_time = zeros(time_step,1);

x_d_error = [];
y_d_error = [];
x_point = [];
y_point = [];
% target_point_rec = zeros(time_step, num_robot);
dt = 0.01;

%TH-GRN相关factors设置
factors=[1,1,1.0,1,1,1,1,   1,1,0,0,0,1,1,   0,0,1,0,1,1,1];  
factors = reshape(factors,7,3);
factors = factors';

% 初始群体机器人的位置
p_captor   = textread('point3.txt')';
% p_captor   = 0.1.*rand(num_robot,2);
% p_captor      = 7 * rand(num_robot,3) + 6;
% p_captor(:,2) = p_captor(:,2) + 11;
p_captor(:,3) = 1;


cell_wid = 0.25;                            %细胞的长宽    100*100缩回到25*25
distent_detect = 4.5;                       %探测范围
distent_fish = 1;                           %安全距离
distent_capter = 0.5;                       %安全距离
v = 0.2;

%%  过窄道
length = 0.5;    %模拟障碍物的大小
num_simulation = 0;

% 右窄道
right_wide_1 = 14; right_wide_2 = 15;
right_long_1 = 15.5; right_long_2 = 22;

% 左窄道
left_wide_1 = 5; left_wide_2 = 6;
left_long_1 = 15.5; left_long_2 = 22;

% 左窄道
point1 = 4*[left_wide_1,left_wide_2,left_wide_2,left_wide_1;
    left_long_1,left_long_1,left_long_2,left_long_2;];
barrier1 = myrectangle(point1);
point1 = 4*[left_wide_1,left_wide_2,left_wide_2,left_wide_1;
    left_long_1,left_long_1,left_long_2,left_long_2;]*cell_wid;
simulation1 = [left_wide_1+length, left_wide_2-length, left_wide_2-length, left_wide_1+length, left_wide_1+length;
    left_long_1+length, left_long_1+length, left_long_2-length, left_long_2-length, left_long_1+length];

for i = 1:4         %算各条线与x正方向夹角的大小
    simulation_theta(i) = atand((simulation1(2,i+1)-simulation1(2,i))/(simulation1(1,i+1)-simulation1(1,i)));
    if ((simulation1(1,i+1)-simulation1(1,i))<0 )
        simulation_theta(i) = simulation_theta(i) +180;
        y = simulation1(2,i);
        for x = simulation1(1,i):length*cosd(simulation_theta(i)):simulation1(1,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            y = y + length*sind(simulation_theta(i));
        end
    elseif ((simulation1(1,i+1)-simulation1(1,i))==0) && (simulation1(2,i+1)-simulation1(2,i))>0
        simulation_theta(i) = 90;
        x = simulation1(1,i);
        for y = simulation1(2,i):length:simulation1(2,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            x = x + length*cosd(simulation_theta(i));
        end
    elseif ((simulation1(1,i+1)-simulation1(1,i))==0) && (simulation1(2,i+1)-simulation1(2,i))<0
        simulation_theta(i) = 270;
        x = simulation1(1,i);
        for y = simulation1(2,i):-length:simulation1(2,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            x = x + length*cosd(simulation_theta(i));
        end
    else
        y = simulation1(2,i);
        for x = simulation1(1,i):length*cosd(simulation_theta(i)):simulation1(1,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            y = y + length*sind(simulation_theta(i));
        end
    end
    clear simulation_theta
end
% 
% % 右窄道
% point2 = 4*[right_wide_1,right_wide_2,right_wide_2,right_wide_1;
%     right_long_1,right_long_1,right_long_2,right_long_2;];
% barrier2 = myrectangle(point2);
% point2 = 4*[right_wide_1,right_wide_2,right_wide_2,right_wide_1;
%     right_long_1,right_long_1,right_long_2,right_long_2;]*cell_wid;
% simulation2 = [right_wide_1+length, right_wide_2-length, right_wide_2-length, right_wide_1+length, right_wide_1+length;
%     right_long_1+length, right_long_1+length, right_long_2-length, right_long_2-length, right_long_1+length];
% % num_simulation = 0;
% for i = 1:4         %算各条线与x正方向夹角的大小
%     simulation_theta(i) = atand((simulation2(2,i+1)-simulation2(2,i))/(simulation2(1,i+1)-simulation2(1,i)));
%     if ((simulation2(1,i+1)-simulation2(1,i))<0 )
%         simulation_theta(i) = simulation_theta(i) +180;
%         y = simulation2(2,i);
%         for x = simulation2(1,i):length*cosd(simulation_theta(i)):simulation2(1,i+1)
%             num_simulation = num_simulation + 1;
%             simulation_captor(num_simulation,1)= x;
%             simulation_captor(num_simulation,2)= y;
%             y = y + length*sind(simulation_theta(i));
%         end
%     elseif ((simulation2(1,i+1)-simulation2(1,i))==0) && (simulation2(2,i+1)-simulation2(2,i))>0
%         simulation_theta(i) = 90;
%         x = simulation2(1,i);
%         for y = simulation2(2,i):length:simulation2(2,i+1)
%             num_simulation = num_simulation + 1;
%             simulation_captor(num_simulation,1)= x;
%             simulation_captor(num_simulation,2)= y;
%             x = x + length*cosd(simulation_theta(i));
%         end
%     elseif ((simulation2(1,i+1)-simulation2(1,i))==0) && (simulation2(2,i+1)-simulation2(2,i))<0
%         simulation_theta(i) = 270;
%         x = simulation2(1,i);
%         for y = simulation2(2,i):-length:simulation2(2,i+1)
%             num_simulation = num_simulation + 1;
%             simulation_captor(num_simulation,1)= x;
%             simulation_captor(num_simulation,2)= y;
%             x = x + length*cosd(simulation_theta(i));
%         end
%     else
%         y = simulation2(2,i);
%         for x = simulation2(1,i):length*cosd(simulation_theta(i)):simulation2(1,i+1)
%             num_simulation = num_simulation + 1;
%             simulation_captor(num_simulation,1)= x;
%             simulation_captor(num_simulation,2)= y;
%             y = y + length*sind(simulation_theta(i));
%         end
%     end
%     clear simulation_theta
% end
%% 基地base
inc0_y_1     = 26;
inc0_y_2     = 24.7;
% 右窄道
inc0_piont_1 = 7.5;   inc0_piont_2 = 12.5;
inc0_piont_3 = 7.5;   inc0_piont_4 = 12.5;

inc_0 = 4*[inc0_piont_1,inc0_piont_2,inc0_piont_4,inc0_piont_3;
    inc0_y_1,inc0_y_1,inc0_y_2,inc0_y_2;];
barrier0 = myrectangle(inc_0);
inc_0 = 4*[inc0_piont_1,inc0_piont_2,inc0_piont_4,inc0_piont_3;
    inc0_y_1,inc0_y_1,inc0_y_2,inc0_y_2;]*cell_wid;
% % 加入障碍物 —— 1
% inc_y_1     = 21;
% inc_y_2     = 19;
% 右窄道
% inc2_piont_1 = 12.1;   inc2_piont_2 = 12.6;
% inc2_piont_3 = 13;   inc2_piont_4 = 15;
% 
% 左窄道
% inc1_piont_1 = 7.5;   inc1_piont_2 = 7.9;
% inc1_piont_3 = 5;   inc1_piont_4 = 7;
% 
% 左通道
% 
% inc_1 = 4*[inc1_piont_1,inc1_piont_2,inc1_piont_4,inc1_piont_3;
%     inc_y_1,inc_y_1,inc_y_2,inc_y_2;];
% barrier3 = myrectangle(inc_1);
% inc_1 = 4*[inc1_piont_1,inc1_piont_2,inc1_piont_4,inc1_piont_3;
%     inc_y_1,inc_y_1,inc_y_2,inc_y_2;]*cell_wid;
% simulation6 = [inc1_piont_1+length, inc1_piont_2-length, inc1_piont_4-length, inc1_piont_3+length, inc1_piont_1+length;
%     inc_y_1+length, inc_y_1+length, inc_y_2-length, inc_y_2-length, inc_y_1+length];
% 
% for i = 1:4         %算各条线与x正方向夹角的大小
%     simulation_theta(i) = atand((simulation6(2,i+1)-simulation6(2,i))/(simulation6(1,i+1)-simulation6(1,i)));
%     if ((simulation6(1,i+1)-simulation6(1,i))<0 )
%         simulation_theta(i) = simulation_theta(i) +180;
%         y = simulation6(2,i);
%         for x = simulation6(1,i):length*cosd(simulation_theta(i)):simulation6(1,i+1)
%             num_simulation = num_simulation + 1;
%             simulation_captor(num_simulation,1)= x;
%             simulation_captor(num_simulation,2)= y;
%             y = y + length*sind(simulation_theta(i));
%         end
%     elseif ((simulation6(1,i+1)-simulation6(1,i))==0) && (simulation6(2,i+1)-simulation6(2,i))>0
%         simulation_theta(i) = 90;
%         x = simulation6(1,i);
%         for y = simulation6(2,i):length:simulation6(2,i+1)
%             num_simulation = num_simulation + 1;
%             simulation_captor(num_simulation,1)= x;
%             simulation_captor(num_simulation,2)= y;
%             x = x + length*cosd(simulation_theta(i));
%         end
%     elseif ((simulation6(1,i+1)-simulation6(1,i))==0) && (simulation6(2,i+1)-simulation6(2,i))<0
%         simulation_theta(i) = 270;
%         x = simulation6(1,i);
%         for y = simulation6(2,i):-length:simulation6(2,i+1)
%             num_simulation = num_simulation + 1;
%             simulation_captor(num_simulation,1)= x;
%             simulation_captor(num_simulation,2)= y;
%             x = x + length*cosd(simulation_theta(i));
%         end
%     else
%         y = simulation6(2,i);
%         for x = simulation6(1,i):length*cosd(simulation_theta(i)):simulation6(1,i+1)
%             num_simulation = num_simulation + 1;
%             simulation_captor(num_simulation,1)= x;
%             simulation_captor(num_simulation,2)= y;
%             y = y + length*sind(simulation_theta(i));
%         end
%     end
%     clear simulation_theta
% end
% 
% % 右通道
% inc_2 = 4*[inc2_piont_1,inc2_piont_2,inc2_piont_4,inc2_piont_3;
%     inc_y_1,inc_y_1,inc_y_2,inc_y_2;];
% barrier4 = myrectangle(inc_2);
% inc_2 = 4*[inc2_piont_1,inc2_piont_2,inc2_piont_4,inc2_piont_3;
%     inc_y_1,inc_y_1,inc_y_2,inc_y_2;]*cell_wid;
% simulation7 = [inc2_piont_1+length, inc2_piont_2-length, inc2_piont_4-length, inc2_piont_3+length, inc2_piont_1+length;
%     inc_y_1+length, inc_y_1+length, inc_y_2-length, inc_y_2-length, inc_y_1+length];
% % num_simulation = 0;
% for i = 1:4         %算各条线与x正方向夹角的大小
%     simulation_theta(i) = atand((simulation7(2,i+1)-simulation7(2,i))/(simulation7(1,i+1)-simulation7(1,i)));
%     if ((simulation7(1,i+1)-simulation7(1,i))<0 )
%         simulation_theta(i) = simulation_theta(i) +180;
%         y = simulation7(2,i);
%         for x = simulation7(1,i):length*cosd(simulation_theta(i)):simulation7(1,i+1)
%             num_simulation = num_simulation + 1;
%             simulation_captor(num_simulation,1)= x;
%             simulation_captor(num_simulation,2)= y;
%             y = y + length*sind(simulation_theta(i));
%         end
%     elseif ((simulation7(1,i+1)-simulation7(1,i))==0) && (simulation7(2,i+1)-simulation7(2,i))>0
%         simulation_theta(i) = 90;
%         x = simulation7(1,i);
%         for y = simulation7(2,i):length:simulation7(2,i+1)
%             num_simulation = num_simulation + 1;
%             simulation_captor(num_simulation,1)= x;
%             simulation_captor(num_simulation,2)= y;
%             x = x + length*cosd(simulation_theta(i));
%         end
%     elseif ((simulation7(1,i+1)-simulation7(1,i))==0) && (simulation7(2,i+1)-simulation7(2,i))<0
%         simulation_theta(i) = 270;
%         x = simulation7(1,i);
%         for y = simulation7(2,i):-length:simulation7(2,i+1)
%             num_simulation = num_simulation + 1;
%             simulation_captor(num_simulation,1)= x;
%             simulation_captor(num_simulation,2)= y;
%             x = x + length*cosd(simulation_theta(i));
%         end
%     else
%         y = simulation7(2,i);
%         for x = simulation7(1,i):length*cosd(simulation_theta(i)):simulation7(1,i+1)
%             num_simulation = num_simulation + 1;
%             simulation_captor(num_simulation,1)= x;
%             simulation_captor(num_simulation,2)= y;
%             y = y + length*sind(simulation_theta(i));
%         end
%     end
%     clear simulation_theta
% end


%%
% pattern_barrier = barrier1'+ barrier2'+ barrier3'+ barrier4'+ barrier5'+ barrier6' + barrier7'+ barrier8' ;

[num,dim] = size(simulation_captor);

for index = 1:num
    if isnan(simulation_captor(index))
        simulation_captor(index) = simulation_captor(index+1);
    end
end
pattern_simulatecaptor = generate_simulatepattern(simulation_captor);

%%
[num,dim] = size(p_captor);   % %机器人个数

drawn_captor = [];
for i = 1:time_step
    
%     if i >= 350
%         distent_fish = 1.5;
%     end
    
    num_i = i;
    [pattern_target,pattern_point] = generate_pattern(p_fish,pattern_simulatecaptor);         %形成pattern

    
    temp = pattern_point;
    [num,numb] = size(temp);
    point = temp(round(linspace(1, num, num_robot+1)),:);
    point(end,:) = [];
    
    new_temp = zeros(num,numb);
    diff_x = temp(:, 1) - p_fish(1, 1); % 计算每个点的x坐标与p_fish第一个点的x坐标之间的差距
    valid_indices = find(temp(:, 2) > p_fish(1, 2) & diff_x >= 0); % 找到y坐标大于p_fish第一个点的y坐标并且x坐标大于等于p_fish第一个点的x坐标的点的索引

    
    valid1 = valid_indices(1);
    valid2 = num - valid1+1;
    valid3 = valid2+1;
    valid4 = valid1-1;
    
    new_temp(1:valid2,:) = temp(valid1:num,:);
    new_temp(valid3:num,:) = temp(1:valid4,:);
    sorted_temp = new_temp(valid_indices, :);

    [~, max_y_index] = max(temp(:, 2));
    valid1 = max_y_index(1);
    valid2 = num - valid1+1;
    valid3 = valid2+1;
    valid4 = valid1-1;
    
    new_temp(1:valid2,:) = temp(valid1:num,:);
    new_temp(valid3:num,:) = temp(1:valid4,:);
    sorted_temp = new_temp(max_y_index, :);
    
    point = new_temp(round(linspace(1, num, num_robot+1)),:);
    point(end,:) = [];
% 
%          计算每一份中的点的数量
%      slice_size = floor(size(temp, 1) / num_robot);
%      分组
%      group1 = new_temp(1:slice_size, :);
%      group2 = new_temp(slice_size+1:2*slice_size, :);
%      group3 = new_temp(2*slice_size+1:3*slice_size, :);
%      group4 = new_temp(3*slice_size+1:4*slice_size, :);
%      group5 = new_temp(4*slice_size+1:5*slice_size, :);
%      group6 = new_temp(5*slice_size+1:6*slice_size, :);
%      group7 = new_temp(6*slice_size+1:7*slice_size, :);
%      group8 = new_temp(7*slice_size+1:8*slice_size, :);
%      group9 = new_temp(8*slice_size+1:9*slice_size, :);
%      group10 = new_temp(9*slice_size+1:end, :);
%      
%      point = [group1(1,:);group2(1,:);group3(1,:);group4(1,:);group5(1,:);
%               group6(1,:);group7(1,:);group8(1,:);group9(1,:);group10(1,:)];
%           
%           scatter(point(:,1), point(:,2), 'o', 'MarkerFaceColor', [0, 1, 1]);
    %%

%     scatter(pattern_point(1,:), pattern_point(2,:), 'o', 'MarkerFaceColor', [1, 0.5, 0]); % 第一个点集使用橘红色
%     hold on
%     scatter(point(:,1), point(:,2), 'o', 'MarkerFaceColor', [0, 1, 1]); % 第二个点集使用青蓝色
%     hold off
%     set(gca, 'FontSize', 12);
    

    
    
    r=1;
    k=1;
    j=1;
    temp_captor = [];  % 临时记录
    
    
    while j<=num_robot
        if (p_captor(j,3) == 1) %如果还活着
            
%              if i>1
%             p_compare(:,1) = x_d_mat(i-1,:);
%             p_compare(:,2) = y_d_mat(i-1,:);
%             
%             
%             
%             for k1 = 1:10
%             % 计算当前点与每个组中的点的距离
%             distances = sqrt(sum((p_compare(k1,:) - [group1; group2; group3; group4; group5;
%                                              group6; group7; group8; group9; group10]).^2, 2));
%             % 找到最小距离的索引
%             [~, min_index] = min(distances);
%     
%             % 将对应组中的点赋值给 point
%             switch min_index
%                 
%         case 1
%             point(k1,:) = group1(1,:);
%         case 2
%             point(k1,:) = group2(1,:);
%         case 3
%             point(k1,:) = group3(1,:);
%         case 4
%             point(k1,:) = group4(1,:);
%         case 5
%             point(k1,:) = group5(1,:);
%         case 6
%             point(k1,:) = group6(1,:);
%         case 7
%             point(k1,:) = group7(1,:);
%         case 8
%             point(k1,:) = group8(1,:);
%         case 9
%             point(k1,:) = group9(1,:);
%         case 10
%             point(k1,:) = group10(1,:);
%     end
%             end
%             end
            
            x_mat(i,j) = p_captor(j, 1);
            y_mat(i,j) = p_captor(j, 2);
            x_d_mat(i, j) = point(j, 1);
            y_d_mat(i, j) = point(j, 2);
            
%            if i >= 5
%                x_data1 = [x_mat(i-3,j);x_mat(i-2,j);x_mat(i-1,j);x_mat(i,j);y_mat(i-3,j);y_mat(i-2,j);y_mat(i-1,j);y_mat(i,j)];
%                x_data1 = x_data1';
%                [y_data1] = NeuralNetworkFunction(x_data1);
%                x_p_mat(i,j) = y_data1(1);
%                y_p_mat(i,j) = y_data1(2);
%            end
           %%
           %BP神经网络训练部分
%             if i >= 10
%                x_data1 = [x_mat(i,j);x_mat(i-1,j);x_mat(i-2,j);x_mat(i-3,j);y_mat(i,j);y_mat(i-1,j);y_mat(i-2,j);y_mat(i-3,j)];
%                x_data1 = x_data1';
%                [y_data1] = BPNeuralNetworkFunction(x_data1);
%                x_p_mat(i,j) = y_data1(1);
%                y_p_mat(i,j) = y_data1(2);
%            end
             %%
            %改动控制器部分
%             tic
%             move = UAV_sliding_model_controller(i,j,dt,x_mat,y_mat,x_d_mat,y_d_mat);

%             [move] = Sliding_model_controller(i,j,dt,x_mat,y_mat,x_d_mat,y_d_mat);
% 
            [move] = enhance_Sliding_model_controller(i,j,dt,x_mat,y_mat,x_d_mat,y_d_mat);

% % % 
% %             [move,theta_mat,theta_d_mat] = T_sliding_model_controller(i,j,dt,x_mat,y_mat,x_d_mat,y_d_mat,theta_mat,theta_d_mat);
%             [move] = T_sliding_model_controller(i,j,dt,x_mat,y_mat,x_d_mat,y_d_mat,theta_mat,theta_d_mat);
%           toc
%           disp(['运行时间: ',num2str(toc)]);
%           rec_time(i) = toc;
            error(i,j) = ((x_d_mat(i,j) -x_mat(i,j))^2 + (y_d_mat(i,j) -y_mat(i,j))^2)^0.5;
                        %%
            %TH-GRN控制器（去除die部分）
%             tic
%            move = THtrapping(p_captor,p_fish,j,pattern_target,pattern_barrier,factors);
           
%            move = newTHtrapping(p_captor,p_fish,j,pattern_target,pattern_barrier,factors);
%            move = avoiding(p_captor,move,j,pattern_barrier,p_fish);
%             
%%
            %%GRN-1
%           tic
%             move = trapping(p_captor,p_fish,j,pattern_target,pattern_point);
% 
%             move = newtrapping(p_captor,p_fish,j,pattern_target,pattern_point);
%             move = avoiding(p_captor,move,j,pattern_barrier,p_fish);
%           toc
%           disp(['运行时间: ',num2str(toc)]);
%           rec_time(i) = toc;
%             error(i,j) =  ((p_captor(j, 1) - point(j, 1))^2 + (p_captor(j, 2) - point(j, 2))^2)^0.5;
%              
            
                   % Accumulate errors for each robot
        if i == 1
            cumulative_error(i, j) = error(i, j);
        else
            cumulative_error(i, j) = cumulative_error(i - 1, j) + error(i, j);
        end
            
        
            dis_robotTOpattern = sqrt( (p_captor(j,1)-point(j,1)).^2 + (p_captor(j,2)-point(j,2)).^2 );
            if dis_robotTOpattern <= 0.25
               move(1) = 0;
               move(2) = 0;
            end
            
            if j > 1
             dis_robotTOpattern = sqrt( (p_captor(j,1)-p_captor(j-1,1)).^2 + (p_captor(j,2)-p_captor(j-1,2)).^2 );
            if dis_robotTOpattern <= 0.25
               move(1) = 0;
               move(2) = 0;
            end
            end
            
%             if i< 10
%             p_captor(j,1) = p_captor(j,1)+move(1);
%             p_captor(j,2) = p_captor(j,2)+move(2);
%             p_captor(j,4) = move(1);
%             p_captor(j,5) = move(2);
%             p_organizing(r,:) = p_captor(j,:);
%             end
%             
%             if i >= 10
%             p_move = y_data1;
%             p_move(1) = p_captor(j, 1) - y_data1(1);
%             p_move(2) = p_captor(j, 2) - y_data1(2);
%             p_captor(j,1) = p_captor(j,1) + 0.85 * move(1) + 0.15 * p_move(1);
%             p_captor(j,2) = p_captor(j,2) + 0.85 * move(2) + 0.15 * p_move(2);
%             p_captor(j,4) = 0.85 * move(1) + 0.15 * p_move(1);
%             p_captor(j,5) = 0.85 * move(2) + 0.15 * p_move(2);
%             p_organizing(r,:) = p_captor(j,:);
%             end
            
            p_captor(j,1) = p_captor(j,1)+move(1);
            p_captor(j,2) = p_captor(j,2)+move(2);
            p_captor(j,4) = move(1);
            p_captor(j,5) = move(2);
            p_organizing(r,:) = p_captor(j,:);

            r = r+1;
        else                                    %如果不是组织机器人：与组织机器人运动方向相同
            non_move=[0,0];
            p_captor(j,1) = p_captor(j,1)+non_move(1);
            p_captor(j,2) = p_captor(j,2)+non_move(2);
            p_captor(j,4) = non_move(1);
            p_captor(j,5) = non_move(2);
            p_nonorganizing(k,:) = p_captor(j,:);
            k = k+1;
        end
        trace(2*j-1,i) = p_captor(j,1);
        trace(2*j,i) = p_captor(j,2);
        
        
        robot = [p_captor(j,1),p_captor(j,2)];
        temp_captor = [temp_captor,robot]; 
        
        j=j+1;
    end
    
    drawn_captor = [drawn_captor;temp_captor];

        p1 = line([10,10],[1,22.5],'linestyle','--','LineWidth',2,'color',[0.65,0.65,0.65]);   % 虚线，目标轨迹
        hold on;
    p2 = line([-1,-1],[1,18],'linestyle','-','LineWidth',2,'color',[1.00,0.00,0.00]);   % 红线[1.00,0.41,0.16]，目标轨迹 (界外，代指群体形态)
    hold on ;
    
%     p2 = plot(pattern_point(1,:),pattern_point(2,:),'-','LineWidth',2,'MarkerSize', 15,'color',[1.00,0.41,0.16]);
    hold on;
    p3 = plot(p_fish(:,1),p_fish(:,2),'*','LineWidth',1,'MarkerSize', 8,'color','#FF0000');              %四个target用.画出
    hold on;
    
    for num_i = 1:num_robot
        plot(drawn_captor(:,2*num_i-1),drawn_captor(:,2*num_i),'g-','LineWidth',1);
        hold on;
    end
    

    
    if r~=1
        plot(p_organizing(:,1),p_organizing(:,2),'o','MarkerFaceColor',[0.3,0.74,0.93],'MarkerEdgeColor',[0.00,0.30,1.00],'MarkerSize', 8,'LineWidth',1); % ,'MarkerFaceColor',[1.00,0.07,0.65]                      %organizer用.画出
    end
    if k~=1
        plot(p_nonorganizing(:,1),p_nonorganizing(:,2),'hexagram','color',[0.00,0.00,1.00],'MarkerSize', 8)               %nonorganizer用.画出
    end
    clear p_organizing p_nonorganizing;
    
%     p1 = line([10,10],[1,18],'linestyle','--','LineWidth',2,'color',[0.65,0.65,0.65]);   % 虚线，目标轨迹
%     p2 = line([-1,-1],[1,18],'linestyle','-','LineWidth',2,'color',[1.00,0.00,0.00]);   % 红线[1.00,0.41,0.16]，目标轨迹 (界外，代指群体形态)
%     hold on;
%     p3 = plot(p_fish(:,1),p_fish(:,2),'*','LineWidth',2,'MarkerSize', 8,'color',[1.00,0.00,1.00]);              %四个target用.画出
%     hold on;
%     p4 = patch(point1(1,:),point1(2,:),[0.7,0.7,0.7], 'edgecolor', 'k','LineWidth',1);   % tunnel，通道
%     patch(point2(1,:),point2(2,:),[0.7,0.7,0.7], 'edgecolor', 'k','LineWidth',1);
%     patch(point3(1,:),point3(2,:),[0.7,0.7,0.7], 'edgecolor', 'none');
%     patch(point4(1,:),point4(2,:),[0.7,0.7,0.7], 'edgecolor', 'none');
%     patch(inc_1(1,:),inc_1(2,:),[0.7,0.7,0.7], 'edgecolor', 'k','LineWidth',1);
%     patch(inc_2(1,:),inc_2(2,:),[0.7,0.7,0.7], 'edgecolor', 'k','LineWidth',1);
%     patch(inc_3(1,:),inc_3(2,:),[0.7,0.7,0.7], 'edgecolor', 'k','LineWidth',1);
%     patch(inc_4(1,:),inc_4(2,:),[0.7,0.7,0.7], 'edgecolor', 'k','LineWidth',1);
%     patch(inc_5(1,:),inc_5(2,:),[0.7,0.7,0.7], 'edgecolor', 'k','LineWidth',1);
%     patch(inc_6(1,:),inc_6(2,:),[0.7,0.7,0.7], 'edgecolor', 'k','LineWidth',1);
%     patch(inc_7(1,:),inc_7(2,:),[0.7,0.7,0.7], 'edgecolor', 'k','LineWidth',1);
%     patch(inc_8(1,:),inc_8(2,:),[0.7,0.7,0.7], 'edgecolor', 'k','LineWidth',1);
%     patch(inc_9(1,:),inc_9(2,:),[0.7,0.7,0.7], 'edgecolor', 'k','LineWidth',1);
%     patch(inc_10(1,:),inc_10(2,:),[0.7,0.7,0.7], 'edgecolor', 'k','LineWidth',1);
   p8 =  patch(inc_0(1,:),inc_0(2,:),'c', 'edgecolor', [0,0.44,0.74],'LineWidth',2);
    hold on;
    
%     if i <= 80
%         p_fish(1,2) = p_fish(1,2)-0.04;
%         p_fish(2,2) = p_fish(2,2)-0.05;
%     elseif i > 80 && i <=150
%         p_fish(1,2) = p_fish(1,2)-0.09;
%         p_fish(2,2) = p_fish(2,2)-0.07;
%      elseif i > 150 && i <=180
%         p_fish(1,2) = p_fish(1,2)-0.04;
%         p_fish(2,2) = p_fish(2,2)-0.04;
%      elseif i > 180 && i <=275
%         p_fish(1,2) = p_fish(1,2)-0.09;
%         p_fish(2,2) = p_fish(2,2)-0.09;
%      elseif i > 275 && i <=300
%         p_fish(1,2) = p_fish(1,2);
%         p_fish(2,2) = p_fish(2,2);
%     end

     if i <= 120
        p_fish(1,2) = p_fish(1,2)-0.048;
        p_fish(2,2) = p_fish(2,2)-0.055;
     elseif i > 120 && i <=210
        p_fish(1,2) = p_fish(1,2)-0.045;
        p_fish(2,2) = p_fish(2,2)-0.03;
     elseif i > 210 && i <=310
        p_fish(1,2) = p_fish(1,2)-0.08;
        p_fish(2,2) = p_fish(2,2)-0.08;
     elseif i > 310 && i <=360
        p_fish(1,2) = p_fish(1,2)-0.03;
        p_fish(2,2) = p_fish(2,2)-0.03;
     elseif i > 360 && i <=500
        p_fish(1,2) = p_fish(1,2);
        p_fish(2,2) = p_fish(2,2);
     end
    
   
    
    p7 = plot(point(:,1), point(:,2), 'o', 'color', [0, 0.7, 0.9]);
    
    p10 = plot(x_p_mat(i,:)', y_p_mat(i,:)', 'o', 'MarkerFaceColor', 'r');
%     p9 = plot(point(:,1), point(:,2), 'o', 'color', [0, 0.7, 0.9]);
%      p7 = scatter(point(:,1), point(:,2), 'o', 'MarkerFaceColor', [0, 1, 1]);
    hold on;
    p5 = plot(-2,-2,'o','MarkerFaceColor',[0.3,0.74,0.93],'MarkerEdgeColor',[0.00,0.30,1.00],'MarkerSize', 8,'LineWidth',1); % 目标
        for num_i = 1:num_robot
    arrow_length = 1.5;  % 调整箭头长度
    p6 = quiver(p_captor(num_i,1), p_captor(num_i,2), arrow_length*p_captor(num_i,4), arrow_length*p_captor(num_i,5), 'r', 'LineWidth', 1.3, 'MaxHeadSize', 1, 'AutoScale', 'on', 'ShowArrowHead', 'on');
    hold on;
        end
        
     if p_fish(1,1)-5 <= 0
        x_down = 0;
    else
        x_down = (p_fish(1,1)-5);
    end
    if p_fish(1,1)+5 >= 10
        x_up = 15; 
    else
        x_up = (p_fish(1,1)+ 5 );
    end
    if p_fish(1,2)-3 <= 0
        y_down = 0;
    else
        y_down = (p_fish(1,2)-7.5);
    end
    if p_fish(1,2)+5 >= 25
        y_up = 26;
    else
        y_up = (p_fish(1,2)+5);
    end
    axis([x_down,x_up,y_down,y_up]);
    hold on;  

% if p_fish(1,1)-5 <= 0
%     x_down = 5;
% else
%     x_down = max(0, p_fish(1,1)-2.5); % 保持 x 在 10 个单位范围内
% end
% if p_fish(1,1)+5 >= 10
%     x_up = 10; 
% else
%     x_up = min(10, p_fish(1,1)+2.5); % 保持 x 在 10 个单位范围内
% end
% if p_fish(1,2)-2.5 <= 0
%     y_down = 0;
% else
%     y_down = max(0, p_fish(1,2)-2.5); % 保持 y 在 10 个单位范围内
% end
% if p_fish(1,2)+2.5 >= 10
%     y_up = 10;
% else
%     y_up = min(10, p_fish(1,2)+2.5); % 保持 y 在 10 个单位范围内
% end
% axis([x_down,x_up,y_down,y_up]);
% hold on;

    
    %     title('群体机器人包围目标的效果图','FontSize',10);
%     leg_h = legend([p1 p2 p3 p4 p5 p6 p7],{'Path','Pattern','Targets','Tunnel','Robots','Direction','Point'}) ;
      leg_h = legend([p1 p8 p2 p3 p5 p7 p10],{'Path','Base','Pattern','Targets','Robots','Point','BP'}) ;


%     set(leg_h,'Fontsize',10);
     
    set(gca, 'FontSize', 10);
    hold off;
    drawnow;
    disp(i)
    
%          % 输出下载图表的时刻
%      if i == 1 || i == 150 || i == 200 || i == 400 || i == 500 
%      % 保存图像
%         filename = sprintf('output_image_%d.png', i);
%         saveas(gcf, filename);
%      end

%     plot(1:time_step,x_error(:,5),  'r-', 'LineWidth', 2);
%     hold on;
%     plot(1:time_step,y_error(:,5),  'b-', 'LineWidth', 2);
%     hold off;
end
