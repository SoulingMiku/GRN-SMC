clear;close all;clc;
global cell_wid;
global distent_fish;                        
global distent_capter;                      
global distent_detect ;                     
global v ;                                 


p_fish = [5,20,1];

time_step    = 80; 

% rand('seed',0);
num_targets   = 2;  
num_robot     = 10;
num_barrier   = 2;  


x_mat = zeros(time_step,num_robot);
y_mat = zeros(time_step,num_robot);
x_d_mat = zeros(time_step,num_robot);
y_d_mat = zeros(time_step,num_robot);

x_b_mat = zeros(time_step,num_robot);
y_b_mat = zeros(time_step,num_robot);
x_d_bp_mat = zeros(time_step,num_robot);
y_d_bp_mat = zeros(time_step,num_robot);

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

dt = 0.01;

%CH-GRN factors
factors=[7,1,1,1,1,1,1,   1,1,0,0,0,1,1,   0,0,1,0,1,1,1];  
factors = reshape(factors,7,3);
factors = factors';

p_captor   = textread('point_free.txt')';
pp_captor  = textread('point_free.txt')';

p_captor(:,3) = 1;
pp_captor(:,3) = 1;

cell_wid = 0.25;                            
distent_detect = 4.5;                      
distent_fish = 1;                          
distent_capter = 0.5;                     
v = 0.2;

%%  
length = 0.5;   
num_simulation = 0;

% 
left_wide_1 = 2.5; left_wide_2 = 7.5;
left_long_1 = 24; left_long_2 = 25;

% 
point1 = 4*[left_wide_1,left_wide_2,left_wide_2,left_wide_1;
    left_long_1,left_long_1,left_long_2,left_long_2;];
barrier1 = myrectangle(point1);
point1 = 4*[left_wide_1,left_wide_2,left_wide_2,left_wide_1;
    left_long_1,left_long_1,left_long_2,left_long_2;]*cell_wid;
simulation1 = [left_wide_1+length, left_wide_2-length, left_wide_2-length, left_wide_1+length, left_wide_1+length;
    left_long_1+length, left_long_1+length, left_long_2-length, left_long_2-length, left_long_1+length];

for i = 1:4         
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



%%
pattern_barrier = barrier1';

[num,dim] = size(simulation_captor);

for index = 1:num
    if isnan(simulation_captor(index))
        simulation_captor(index) = simulation_captor(index+1);
    end
end
pattern_simulatecaptor = generate_simulatepattern(simulation_captor);

%%
[num,dim] = size(p_captor);   

drawn_captor = [];
drawn_captor_b = [];


pattern_num = 6;
Dv = zeros(time_step,1);


for i = 1:time_step
    

    
    num_i = i;
    [pattern_target,pattern_point] = generate_pattern(p_fish,pattern_simulatecaptor);        

    
    temp = pattern_point;
    [num,numb] = size(temp);
    point = temp(round(linspace(1, num, num_robot+1)),:);
    point(end,:) = [];
    
    new_temp = zeros(num,numb);
    diff_x = temp(:, 1) - p_fish(1, 1); 
    valid_indices = find(temp(:, 2) > p_fish(1, 2) & diff_x >= 0);

    
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
    temp_captor = [];  
    temp_captor_b = [];
    
    while j<=num_robot
        if (p_captor(j,3) == 1)
            tic

            
            x_mat(i,j) = p_captor(j, 1);
            y_mat(i,j) = p_captor(j, 2);
            
            x_d_mat(i, j) = point(j, 1);
            y_d_mat(i, j) = point(j, 2);
            
            
            x_b_mat(i,j) = pp_captor(j, 1);
            y_b_mat(i,j) = pp_captor(j, 2);
            
            
            x_d_bp_mat(i,j) =  point(j, 1);
            y_d_bp_mat(i,j) =  point(j, 2);
                 
              if i >= 20
               bp_x_d= [x_d_mat(i,j);x_d_mat(i-1,j);x_d_mat(i-2,j);x_d_mat(i-3,j);y_d_mat(i,j);y_d_mat(i-1,j);y_d_mat(i-2,j);y_d_mat(i-3,j)];
               bp_x_d = bp_x_d';
               [bp_d] = x_d_NeuralNetworkFunction(bp_x_d);
               x_d_bp_mat(i,j) = bp_d(1);
               y_d_bp_mat(i,j) = bp_d(3);

              end
             
               if i >= 10
               x_bp = [x_b_mat(i,j);x_b_mat(i-1,j);x_b_mat(i-2,j);x_b_mat(i-3,j);y_b_mat(i,j);y_b_mat(i-1,j);y_b_mat(i-2,j);y_b_mat(i-3,j)];
               x_bp = x_bp';
               [x_b_bp] = x_b_NeuralNetworkFunction(x_bp);
            end

             %%
            %controller part
%             tic

%             [move_b] = enhance_Sliding_model_controller(i,j,dt,x_b_mat,y_b_mat,x_d_mat,y_d_mat);
            [move_b] = unlimit_Sliding_model_controller(i,j,dt,x_b_mat,y_b_mat,x_d_mat,y_d_mat);
%             [move_b] = Sat_Sliding_model_controller(i,j,dt,x_b_mat,y_b_mat,x_d_mat,y_d_mat);
%             [move_b] = Sat1_Sliding_model_controller(i,j,dt,x_b_mat,y_b_mat,x_d_mat,y_d_mat);
%              move_b = UAV_sliding_model_controller(i,j,dt,x_b_mat,y_b_mat,x_d_bp_mat,y_d_bp_mat);
% %           [move,theta_mat,theta_d_mat] = T_sliding_model_controller(i,j,dt,x_mat,y_mat,x_d_mat,y_d_mat,theta_mat,theta_d_mat);
%             [move] = T_sliding_model_controller(i,j,dt,x_mat,y_mat,x_d_mat,y_d_mat,theta_mat,theta_d_mat);
%           toc
%           disp(['runtime: ',num2str(toc)]);
%           rec_time(i) = toc;
            error(i,j) = ((x_d_mat(i,j) -x_mat(i,j))^2 + (y_d_mat(i,j) -y_mat(i,j))^2)^0.5;
                        %%
            %CH-GRN控制器（die部分except）
%             tic
           move = CHtrapping(p_captor,p_fish,j,pattern_target,pattern_barrier,factors);
           
%          move = newCHtrapping(p_captor,p_fish,j,pattern_target,pattern_barrier,factors);
%          move = avoiding(p_captor,move,j,pattern_barrier,p_fish);
%             
%%
            %%GP-GRN
% %           tic
%           move = trapping(p_captor,p_fish,j,pattern_target,pattern_point);
%           move = newtrapping(p_captor,p_fish,j,pattern_target,pattern_point);
            move = avoiding(p_captor,move,j,pattern_barrier,p_fish);
%           toc
%           disp(['runtime: ',num2str(toc)]);
%           rec_time(i) = toc;
%           error(i,j) =  ((p_captor(j, 1) - point(j, 1))^2 + (p_captor(j, 2) - point(j, 2))^2)^0.5;
%              
            
                   % Accumulate errors for each robot
        if i == 1
            cumulative_error(i, j) = error(i, j);
        else
            cumulative_error(i, j) = cumulative_error(i - 1, j) + error(i, j);
        end
            
        
            dis_robotTOpattern = sqrt( (p_captor(j,1)-p_fish(:,1)).^2 + (p_captor(j,2)-p_fish(:,2)).^2 );
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
            
             % Accumulate errors for each robot
        if i == 1
            cumulative_error(i, j) = error(i, j);
        else
            cumulative_error(i, j) = cumulative_error(i - 1, j) + error(i, j);
        end

            
            if j > 1
             dis_robotTOpattern = sqrt( (pp_captor(j,1)-pp_captor(j-1,1)).^2 + (pp_captor(j,2)-pp_captor(j-1,2)).^2 );
            if dis_robotTOpattern <= 0.1
               move_b(1) = 0;
               move_b(2) = 0;
            end
            
             dis_robotTOpattern = sqrt( (pp_captor(j,1)-p_fish(:,1)).^2 + (pp_captor(j,2)-p_fish(:,2)).^2 );
            if dis_robotTOpattern <= 0.1
               move_b(1) = 0;
               move_b(2) = 0;
            end
                          

        end
            
            p_captor(j,1) = p_captor(j,1)+move(1);
            p_captor(j,2) = p_captor(j,2)+move(2);
            p_captor(j,4) = move(1);
            p_captor(j,5) = move(2);
            p_organizing(r,:) = p_captor(j,:);
            
            if i< 10
            pp_captor(j,1) = pp_captor(j,1)+move_b(1);
            pp_captor(j,2) = pp_captor(j,2)+move_b(2);
            pp_captor(j,4) = move_b(1);
            pp_captor(j,5) = move_b(2);
            p_organizing(r,:) = pp_captor(j,:);
            end
            

            if i >= 10
            p_move = x_b_bp;
            p_move(1) = pp_captor(j, 1) - x_b_bp(1);
            p_move(2) = pp_captor(j, 2) - x_b_bp(3);
            pp_captor(j,1) = pp_captor(j,1) + 1 * move_b(1) + 0 * p_move(1);
            pp_captor(j,2) = pp_captor(j,2) + 1 * move_b(2) + 0 * p_move(2);
            pp_captor(j,4) = 1 * move_b(1) + 0 * p_move(1);
            pp_captor(j,5) = 1 * move_b(2) + 0 * p_move(2);
%             p_organizing(r,:) = pp_captor(j,:);
            end
            

            r = r+1;
        else                                   
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
        robot_b = [pp_captor(j,1),pp_captor(j,2)];
        
        temp_captor = [temp_captor,robot]; 
        temp_captor_b = [temp_captor_b,robot_b]; 
        
        j=j+1;
                          toc
          disp(['runtime: ',num2str(toc)]);
          rec_time(i) = toc;
    end

    drawn_captor = [drawn_captor;temp_captor];
    drawn_captor_b = [drawn_captor_b;temp_captor_b];

p1 = line([5,15],[20,5],'linestyle','--','LineWidth',2,'color',[0.65,0.65,0.65]);  
   hold on;
p2 = line([-1,-1],[1,18],'linestyle','-','LineWidth',2,'color',[1.00,0.00,0.00]);   
   hold on ;
p3 = plot(p_fish(:,1),p_fish(:,2),'*','LineWidth',1,'MarkerSize', 8,'color','#FF0000');              
   hold on;
    %%
pos_max = 64;
p_size = [2*ones(1,16), 3*ones(1,16), 4*ones(1,16), 5*ones(1,16)]; 
if i > 1
    for num_j1 = 1:1:size(2*pos_list1, 1)
        tmp_p1 = pos_list1(num_j1, :); 
%         plot(tmp_p1(1), tmp_p1(2), 'o', 'MarkerFaceColor', [0.65, 0.85, 0.65], 'MarkerSize', p_size(num_j1));%green [0.65, 0.85, 0.65] blue[0.3,0.74,0.93]
        hold on;
    end
    
    for num_j2 = 1:1:size(2 * pos_list2, 1)
        tmp_p2 = pos_list2(num_j2, :);  % 直接引用 pos_list 中的行
        plot(tmp_p2(1), tmp_p2(2), 'o', 'MarkerFaceColor', [1 ,0.55 ,0.0], 'MarkerSize', p_size(num_j2));
        hold on;
    end
    
end
    
 if i == 1
    pos_list1 = p_captor(1:10, 1:2);  
    pos_list2 = pp_captor(1:10, 1:2); 
else
    pos_list1 = [pos_list1; p_captor(1:10, 1:2)]; 
    pos_list2 = [pos_list2; pp_captor(1:10, 1:2)]; 
    if size(pos_list1, 1) > pos_max
        pos_list1 = pos_list1(end-pos_max+1:end, :);  
    end
    if size(pos_list2, 1) > pos_max
        pos_list2 = pos_list2(end-pos_max+1:end, :); 
    end
end

 

   
    %%
    if k~=1
        plot(p_nonorganizing(:,1),p_nonorganizing(:,2),'hexagram','color',[0.00,0.00,1.00],'MarkerSize', 8)            
    end
    clear p_organizing p_nonorganizing;
 p4 = patch(point1(1,:),point1(2,:),'c', 'edgecolor', [0,0.44,0.74],'LineWidth',2);   % tunnel，通道

   hold on;

if i <= 80
  
    p_fish(1,1) = 5;
    p_fish(1,1) =  p_fish(1,1) + 10 * (i / 80)^2.5;
    p_fish(1,2) = 20 - 15 * (i / 80)^2.5; 
elseif i > 80 && i <= 160
    p_fish(1,1) =  p_fish(1,1) + 10 * ((i-80) / 80)^2.5; 
elseif i > 87.5 && i <= 100

    p_fish(1,1) = 7.5 + 2.5 * ((i - 87.5) / 12.5); 
    p_fish(1,2) = 2.5 + 2.5 * ((i - 87.5) / 12.5); 
elseif i > 100 && i <= 180
    speed_y = 0.36 * (i - 100) / 80;
    p_fish(1,1) = 10; 
    p_fish(1,2) = 5 + 15 * ((i - 100) / 80);
end

    hold on;
%     p5 = plot(p_captor(:,1),p_captor(:,2),'o','MarkerFaceColor',[0.3,0.74,0.93],'MarkerEdgeColor',[0.00,0.30,1.00],'MarkerSize', 8,'LineWidth',1); 
%     p5 = plot(p_captor(:,1),p_captor(:,2),'o','MarkerFaceColor',[0.65, 0.85, 0.65],'MarkerEdgeColor',[0.05, 0.5, 0.05],'MarkerSize', 8,'LineWidth',1);
    p9 = plot(pp_captor(:,1),pp_captor(:,2),'o','MarkerFaceColor',[1 ,0.55 ,0.0],'MarkerEdgeColor',[0.7,0.13,0.13],'MarkerSize', 8,'LineWidth',1);
    
%     p10 = plot(x_p_mat(i,:)', y_p_mat(i,:)', 'o', 'MarkerFaceColor', 'r');
%     p11 = plot(x_d_bp_mat(i,:)', y_d_bp_mat(i,:)', 'o', 'MarkerFaceColor', 'b');
        for num_i = 1:num_robot
    arrow_length = 1.5;  
%     p6 = quiver(pp_captor(num_i,1), pp_captor(num_i,2), arrow_length*pp_captor(num_i,4), arrow_length*pp_captor(num_i,5), 'r', 'LineWidth', 1.3, 'MaxHeadSize', 1, 'AutoScale', 'on', 'ShowArrowHead', 'on');
%     hold on;
        end
        
        

x_center = p_fish(1,1);
y_center = p_fish(1,2);
x_down = x_center - 5;
x_up = x_center + 5;
y_down = y_center - 5;
y_up = y_center + 5;
axis([x_down, x_up, y_down, y_up]);
hold on;

               leg_h = legend([p1 p2 p3 p4 p9 ],{'Path','Pattern','Target','Base','GRN-SMC'},'NumColumns', 2, 'FontSize', 15, 'FontName', 'Times New Roman','Location', 'south') ;
%              leg_h = legend([p1 p2 p3 p4 p5 ],{'Path','Pattern','Target','Base','GP-GRN',},'NumColumns', 2, 'FontSize', 15, 'FontName',  'Times New Roman','Location', 'south') ;
%              leg_h = legend([p1 p2 p3 p4 p5 ],{'Path','Pattern','Target','Base','CH-GRN'},'NumColumns', 2, 'FontSize', 15, 'FontName', 'Times New Roman','Location', 'south') ;

     set(gca, 'FontSize', 20, 'FontName', 'Times New Roman');  
     xlabel('x/(m)', 'FontName', 'Times New Roman');           
     ylabel('y/(m)', 'FontName', 'Times New Roman');           
    hold off;
    drawnow;
    disp(i)


% if i == 1 |i == 1 || i == 20 || i == 40 || i == 60
%     set(gcf, 'Position', [100, 100, 500, 450]); 
%     
%     set(gcf, 'PaperPositionMode', 'auto');      
%     paperpos = get(gcf, 'PaperPosition');       
%     papersize = paperpos(3:4);                  
%     
% 
%     set(gcf, 'PaperSize', papersize);          
%     set(gcf, 'PaperPosition', [0 0 papersize]); 
%    
%     filename_pdf = sprintf('CH-GRN_80_%d.pdf', i);
%     print(gcf, filename_pdf, '-dpdf', '-r600', '-bestfit', '-painters'); 
%      end

end
