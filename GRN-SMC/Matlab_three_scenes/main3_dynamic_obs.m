clear;close all;clc;
global cell_wid;
global distent_fish;                      
global distent_capter;                      
global distent_detect ;                     
global v1 v2 v;                                 


p_fish = [2.5, 2.5, 1];
v11 = 0.05; 
r11 = 0.5;    


target = zeros(8,2);
target(1,:) = [2.5, 3];
target(2,:) = [11.5, 3];
target(3,:) = [11.5, 18];
target(4,:) = [20.5, 18];
target(5,:) = [20.5, 24.5];
target(6,:) = [28, 24.5];
target(7,:) = [30, 28];
target(8,:) = [30, 35];



distances = zeros(1, 8);
distances(1) = abs(target(1,2) - p_fish(2))-r11; % 0.5
distances(2) = abs(target(2,1) - target(1,1)) - 2*r11; % 8
distances(3) = abs(target(3,2) - target(2,2)) - 2*r11; % 14
distances(4) = abs(target(4,1) - target(3,1)) - 2*r11; % 8
distances(5) = abs(target(5,2) - target(4,2)) - 2*r11; % 5.5
distances(6) = abs(target(6,1) - target(5,1)) - r11 -r11*tan(pi/8); % 6.5
dx7 = target(7,1) - target(6,1); dy7 = target(7,2) - target(6,2);
distances(7) = sqrt(dx7^2 + dy7^2) ; %
distances(8) = abs(target(8,2) - target(7,2)); % 6.5


t_steps = zeros(1, 13); 
t_steps(1) = round(distances(1) / v11); 
t_steps(2) = round((pi*r11/2) / v11); 
t_steps(3) = round(distances(2) / v11); 
t_steps(4) = round((pi*r11/2) / v11); 
t_steps(5) = round(distances(3) / v11); 
t_steps(6) = round((pi*r11/2) / v11); 
t_steps(7) = round(distances(4) / v11); 
t_steps(8) = round((pi*r11/2) / v11); 
t_steps(9) = round(distances(5) / v11); 
t_steps(10) = round((pi*r11/2) / v11); 
t_steps(11) = round(distances(6) / v11); 
t_steps(12) = round(distances(7) / v11); 
t_steps(13) = round(distances(8) / v11); 


t_ends = cumsum(t_steps);
t1_end = t_ends(1);   
t2_end = t_ends(2);   
t3_end = t_ends(3);   
t4_end = t_ends(4);   
t5_end = t_ends(5);   
t6_end = t_ends(6);   
t7_end = t_ends(7);   
t8_end = t_ends(8);   
t9_end = t_ends(9);   
t10_end = t_ends(10); 
t11_end = t_ends(11); 
t12_end = t_ends(12); 
t13_end = t_ends(13); 

total_steps = t13_end;

time_step    = total_steps; 
v_p_fish1 = [];
v_p_fish2 = [];
v_p = [];
Vr_grn1 = [];
Vr_smc = [];
Vr_ch = [];

num_targets   = 2;  
num_robot     = 6; 
num_barrier   = 2;  


x_mat = zeros(time_step,num_robot);
y_mat = zeros(time_step,num_robot);
x_d_mat = zeros(time_step,num_robot);
y_d_mat = zeros(time_step,num_robot);
x_dd_mat = zeros(time_step,num_robot);
y_dd_mat = zeros(time_step,num_robot);


x_b_mat = zeros(time_step,num_robot);
y_b_mat = zeros(time_step,num_robot);
x_bb_mat = zeros(time_step,num_robot);
y_bb_mat = zeros(time_step,num_robot);
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
point_rec1 = [];
point_rec2 = [];
point_change = [];
d_mat = [];
dd_mat = [];
ddd_mat = [];
P_d_change = [];
captor_change = [];
captor_change_b = [];
captor_change_bb = [];

dt = 0.01;

factors=[5,1,1,1,1,1,1,   1,1,0,0,0,1,1,   0,0,1,0,1,1,1];  
factors = reshape(factors,7,3);
factors = factors';


p_captor   = generate_point(num_robot);
pp_captor  = generate_point(num_robot);
ppp_captor  = generate_point(num_robot);

p_captor(:,3) = 1;
pp_captor(:,3) = 1;
ppp_captor(:,3) = 1;

cell_wide = 0.25; 
cell_wid = 0.5;                         
distent_detect = 4.5;                      
distent_fish = 1;                         
distent_capter = 0.5;                     
v1 = 0;
v2 = 0;

length = 0.5;    
num_simulation = 0;


right_wide_1 = 13; right_wide_2 = 20;
right_long_1 = 5; right_long_2 = 9;


left_wide_1 = 5; left_wide_2 = 9.9;
left_long_1 = 5; left_long_2 = 9.9;


point1 = 4*[left_wide_1,left_wide_2,left_wide_2,left_wide_1;
    left_long_1,left_long_1,left_long_2,left_long_2;];
barrier1 = myrectangle(point1);
point1 = 4*[left_wide_1,left_wide_2,left_wide_2,left_wide_1;
    left_long_1,left_long_1,left_long_2,left_long_2;]*cell_wide;
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

% 
point2 = 4*[right_wide_1,right_wide_2,right_wide_2,right_wide_1;
    right_long_1,right_long_1,right_long_2,right_long_2;];
barrier2 = myrectangle(point2);
point2 = 4*[right_wide_1,right_wide_2,right_wide_2,right_wide_1;
    right_long_1,right_long_1,right_long_2,right_long_2;]*cell_wide;
simulation2 = [right_wide_1+length, right_wide_2-length, right_wide_2-length, right_wide_1+length, right_wide_1+length;
    right_long_1+length, right_long_1+length, right_long_2-length, right_long_2-length, right_long_1+length];

for i = 1:4       
    simulation_theta(i) = atand((simulation2(2,i+1)-simulation2(2,i))/(simulation2(1,i+1)-simulation2(1,i)));
    if ((simulation2(1,i+1)-simulation2(1,i))<0 )
        simulation_theta(i) = simulation_theta(i) +180;
        y = simulation2(2,i);
        for x = simulation2(1,i):length*cosd(simulation_theta(i)):simulation2(1,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            y = y + length*sind(simulation_theta(i));
        end
    elseif ((simulation2(1,i+1)-simulation2(1,i))==0) && (simulation2(2,i+1)-simulation2(2,i))>0
        simulation_theta(i) = 90;
        x = simulation2(1,i);
        for y = simulation2(2,i):length:simulation2(2,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            x = x + length*cosd(simulation_theta(i));
        end
    elseif ((simulation2(1,i+1)-simulation2(1,i))==0) && (simulation2(2,i+1)-simulation2(2,i))<0
        simulation_theta(i) = 270;
        x = simulation2(1,i);
        for y = simulation2(2,i):-length:simulation2(2,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            x = x + length*cosd(simulation_theta(i));
        end
    else
        y = simulation2(2,i);
        for x = simulation2(1,i):length*cosd(simulation_theta(i)):simulation2(1,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            y = y + length*sind(simulation_theta(i));
        end
    end
    clear simulation_theta
end


%% 2
up_long1     = 24;
down_long1     = 20;
% 
left_wide1 = 14;   inc2_piont_2 = 18;
inc2_piont_3 = 14;   inc2_piont_4 = 18;

% 
inc1_piont_1 = 5;   inc1_piont_2 = 9;
inc1_piont_3 = 5;   inc1_piont_4 = 9;

% 

inc_3 = 4*[inc1_piont_1,inc1_piont_2,inc1_piont_4,inc1_piont_3;
    up_long1,up_long1,down_long1,down_long1;];
barrier3 = myrectangle(inc_3);
inc_3 = 4*[inc1_piont_1,inc1_piont_2,inc1_piont_4,inc1_piont_3;
    up_long1,up_long1,down_long1,down_long1;]*cell_wide;
simulation3 = [inc1_piont_1+length, inc1_piont_2-length, inc1_piont_4-length, inc1_piont_3+length, inc1_piont_1+length;
    up_long1+length, up_long1+length, down_long1-length, down_long1-length, up_long1+length];

for i = 1:4         
    simulation_theta(i) = atand((simulation3(2,i+1)-simulation3(2,i))/(simulation3(1,i+1)-simulation3(1,i)));
    if ((simulation3(1,i+1)-simulation3(1,i))<0 )
        simulation_theta(i) = simulation_theta(i) +180;
        y = simulation3(2,i);
        for x = simulation3(1,i):length*cosd(simulation_theta(i)):simulation3(1,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            y = y + length*sind(simulation_theta(i));
        end
    elseif ((simulation3(1,i+1)-simulation3(1,i))==0) && (simulation3(2,i+1)-simulation3(2,i))>0
        simulation_theta(i) = 90;
        x = simulation3(1,i);
        for y = simulation3(2,i):length:simulation3(2,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            x = x + length*cosd(simulation_theta(i));
        end
    elseif ((simulation3(1,i+1)-simulation3(1,i))==0) && (simulation3(2,i+1)-simulation3(2,i))<0
        simulation_theta(i) = 270;
        x = simulation3(1,i);
        for y = simulation3(2,i):-length:simulation3(2,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            x = x + length*cosd(simulation_theta(i));
        end
    else
        y = simulation3(2,i);
        for x = simulation3(1,i):length*cosd(simulation_theta(i)):simulation3(1,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            y = y + length*sind(simulation_theta(i));
        end
    end
    clear simulation_theta
end
% patch(inc_3(1,:),inc_3(2,:),[0.7,0.7,0.7], 'edgecolor', 'k','LineWidth',1);

%% 3
up_long1     = 36;
down_long1     = 32;
% 
left_wide1 = 14;   inc2_piont_2 = 18;
inc2_piont_3 = 14;   inc2_piont_4 = 18;

% 
inc1_piont_1 = 5;   inc1_piont_2 = 9;
inc1_piont_3 = 5;   inc1_piont_4 = 9;

% 

inc_5 = 4*[inc1_piont_1,inc1_piont_2,inc1_piont_4,inc1_piont_3;
    up_long1,up_long1,down_long1,down_long1;];
barrier5 = myrectangle(inc_5);
inc_5 = 4*[inc1_piont_1,inc1_piont_2,inc1_piont_4,inc1_piont_3;
    up_long1,up_long1,down_long1,down_long1;]*cell_wide;
simulation5 = [inc1_piont_1+length, inc1_piont_2-length, inc1_piont_4-length, inc1_piont_3+length, inc1_piont_1+length;
    up_long1+length, up_long1+length, down_long1-length, down_long1-length, up_long1+length];

for i = 1:4         
    simulation_theta(i) = atand((simulation5(2,i+1)-simulation5(2,i))/(simulation5(1,i+1)-simulation5(1,i)));
    if ((simulation5(1,i+1)-simulation5(1,i))<0 )
        simulation_theta(i) = simulation_theta(i) +180;
        y = simulation5(2,i);
        for x = simulation5(1,i):length*cosd(simulation_theta(i)):simulation5(1,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            y = y + length*sind(simulation_theta(i));
        end
    elseif ((simulation5(1,i+1)-simulation5(1,i))==0) && (simulation5(2,i+1)-simulation5(2,i))>0
        simulation_theta(i) = 90;
        x = simulation5(1,i);
        for y = simulation5(2,i):length:simulation5(2,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            x = x + length*cosd(simulation_theta(i));
        end
    elseif ((simulation5(1,i+1)-simulation5(1,i))==0) && (simulation5(2,i+1)-simulation5(2,i))<0
        simulation_theta(i) = 270;
        x = simulation5(1,i);
        for y = simulation5(2,i):-length:simulation5(2,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            x = x + length*cosd(simulation_theta(i));
        end
    else
        y = simulation5(2,i);
        for x = simulation5(1,i):length*cosd(simulation_theta(i)):simulation5(1,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            y = y + length*sind(simulation_theta(i));
        end
    end
    clear simulation_theta
end

% 
inc_6 = 4*[left_wide1,inc2_piont_2,inc2_piont_4,inc2_piont_3;
    up_long1,up_long1,down_long1,down_long1;];
barrier6 = myrectangle(inc_6);
inc_6 = 4*[left_wide1,inc2_piont_2,inc2_piont_4,inc2_piont_3;
    up_long1,up_long1,down_long1,down_long1;]*cell_wide;
simulation6 = [left_wide1+length, inc2_piont_2-length, inc2_piont_4-length, inc2_piont_3+length, left_wide1+length;
    up_long1+length, up_long1+length, down_long1-length, down_long1-length, up_long1+length];

for i = 1:4         
    simulation_theta(i) = atand((simulation6(2,i+1)-simulation6(2,i))/(simulation6(1,i+1)-simulation6(1,i)));
    if ((simulation6(1,i+1)-simulation6(1,i))<0 )
        simulation_theta(i) = simulation_theta(i) +180;
        y = simulation6(2,i);
        for x = simulation6(1,i):length*cosd(simulation_theta(i)):simulation6(1,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            y = y + length*sind(simulation_theta(i));
        end
    elseif ((simulation6(1,i+1)-simulation6(1,i))==0) && (simulation6(2,i+1)-simulation6(2,i))>0
        simulation_theta(i) = 90;
        x = simulation6(1,i);
        for y = simulation6(2,i):length:simulation6(2,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            x = x + length*cosd(simulation_theta(i));
        end
    elseif ((simulation6(1,i+1)-simulation6(1,i))==0) && (simulation6(2,i+1)-simulation6(2,i))<0
        simulation_theta(i) = 270;
        x = simulation6(1,i);
        for y = simulation6(2,i):-length:simulation6(2,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            x = x + length*cosd(simulation_theta(i));
        end
    else
        y = simulation6(2,i);
        for x = simulation6(1,i):length*cosd(simulation_theta(i)):simulation6(1,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            y = y + length*sind(simulation_theta(i));
        end
    end
    clear simulation_theta
end



%% 4
% 
up_long1     = 18;
down_long1     = 14;


inc1_piont_1 = 6;   inc1_piont_2 = 10;
inc1_piont_3 = 6;   inc1_piont_4 = 10;

% 

inc_7 = 4*[inc1_piont_1,inc1_piont_2,inc1_piont_4,inc1_piont_3;
    up_long1,up_long1,down_long1,down_long1;];
barrier7 = myrectangle(inc_7);
inc_7 = 4*[inc1_piont_1,inc1_piont_2,inc1_piont_4,inc1_piont_3;
    up_long1,up_long1,down_long1,down_long1;]*cell_wide;
simulation7 = [inc1_piont_1+length, inc1_piont_2-length, inc1_piont_4-length, inc1_piont_3+length, inc1_piont_1+length;
    up_long1+length, up_long1+length, down_long1-length, down_long1-length, up_long1+length];

for i = 1:4        
    simulation_theta(i) = atand((simulation7(2,i+1)-simulation7(2,i))/(simulation7(1,i+1)-simulation7(1,i)));
    if ((simulation7(1,i+1)-simulation7(1,i))<0 )
        simulation_theta(i) = simulation_theta(i) +180;
        y = simulation7(2,i);
        for x = simulation7(1,i):length*cosd(simulation_theta(i)):simulation7(1,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            y = y + length*sind(simulation_theta(i));
        end
    elseif ((simulation7(1,i+1)-simulation7(1,i))==0) && (simulation7(2,i+1)-simulation7(2,i))>0
        simulation_theta(i) = 90;
        x = simulation7(1,i);
        for y = simulation7(2,i):length:simulation7(2,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            x = x + length*cosd(simulation_theta(i));
        end
    elseif ((simulation7(1,i+1)-simulation7(1,i))==0) && (simulation7(2,i+1)-simulation7(2,i))<0
        simulation_theta(i) = 270;
        x = simulation7(1,i);
        for y = simulation7(2,i):-length:simulation7(2,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            x = x + length*cosd(simulation_theta(i));
        end
    else
        y = simulation7(2,i);
        for x = simulation7(1,i):length*cosd(simulation_theta(i)):simulation7(1,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            y = y + length*sind(simulation_theta(i));
        end
    end
    clear simulation_theta
end




%% 5
up_long1     = 30;
down_long1     = 26;

% 
inc1_piont_1 = 8;   inc1_piont_2 = 12;
inc1_piont_3 = 8;   inc1_piont_4 = 12;

inc_8 = 4*[inc1_piont_1,inc1_piont_2,inc1_piont_4,inc1_piont_3;
    up_long1,up_long1,down_long1,down_long1;];
barrier8 = myrectangle(inc_8);
inc_8 = 4*[inc1_piont_1,inc1_piont_2,inc1_piont_4,inc1_piont_3;
    up_long1,up_long1,down_long1,down_long1;]*cell_wide;
simulation8 = [inc1_piont_1+length, inc1_piont_2-length, inc1_piont_4-length, inc1_piont_3+length, inc1_piont_1+length;
    up_long1+length, up_long1+length, down_long1-length, down_long1-length, up_long1+length];

for i = 1:4      
    simulation_theta(i) = atand((simulation8(2,i+1)-simulation8(2,i))/(simulation8(1,i+1)-simulation8(1,i)));
    if ((simulation8(1,i+1)-simulation8(1,i))<0 )
        simulation_theta(i) = simulation_theta(i) +180;
        y = simulation8(2,i);
        for x = simulation8(1,i):length*cosd(simulation_theta(i)):simulation8(1,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            y = y + length*sind(simulation_theta(i));
        end
    elseif ((simulation8(1,i+1)-simulation8(1,i))==0) && (simulation8(2,i+1)-simulation8(2,i))>0
        simulation_theta(i) = 90;
        x = simulation8(1,i);
        for y = simulation8(2,i):length:simulation8(2,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            x = x + length*cosd(simulation_theta(i));
        end
    elseif ((simulation8(1,i+1)-simulation8(1,i))==0) && (simulation8(2,i+1)-simulation8(2,i))<0
        simulation_theta(i) = 270;
        x = simulation8(1,i);
        for y = simulation8(2,i):-length:simulation8(2,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            x = x + length*cosd(simulation_theta(i));
        end
    else
        y = simulation8(2,i);
        for x = simulation8(1,i):length*cosd(simulation_theta(i)):simulation8(1,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            y = y + length*sind(simulation_theta(i));
        end
    end
    clear simulation_theta
end



%% 6
up_long1     = 22.3;
down_long1     = 10;

% 
inc1_piont_1 = 22;   inc1_piont_2 = 28;
inc1_piont_3 = inc1_piont_1;   inc1_piont_4 = inc1_piont_2;

inc_9 = 4*[inc1_piont_1,inc1_piont_2,inc1_piont_4,inc1_piont_3;
    up_long1,up_long1,down_long1,down_long1;];
barrier9 = myrectangle(inc_9);
inc_9 = 4*[inc1_piont_1,inc1_piont_2,inc1_piont_4,inc1_piont_3;
    up_long1,up_long1,down_long1,down_long1;]*cell_wide;
simulation9 = [inc1_piont_1+length, inc1_piont_2-length, inc1_piont_4-length, inc1_piont_3+length, inc1_piont_1+length;
    up_long1+length, up_long1+length, down_long1-length, down_long1-length, up_long1+length];

for i = 1:4       
    simulation_theta(i) = atand((simulation9(2,i+1)-simulation9(2,i))/(simulation9(1,i+1)-simulation9(1,i)));
    if ((simulation9(1,i+1)-simulation9(1,i))<0 )
        simulation_theta(i) = simulation_theta(i) +180;
        y = simulation9(2,i);
        for x = simulation9(1,i):length*cosd(simulation_theta(i)):simulation9(1,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            y = y + length*sind(simulation_theta(i));
        end
    elseif ((simulation9(1,i+1)-simulation9(1,i))==0) && (simulation9(2,i+1)-simulation9(2,i))>0
        simulation_theta(i) = 90;
        x = simulation9(1,i);
        for y = simulation9(2,i):length:simulation9(2,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            x = x + length*cosd(simulation_theta(i));
        end
    elseif ((simulation9(1,i+1)-simulation9(1,i))==0) && (simulation9(2,i+1)-simulation9(2,i))<0
        simulation_theta(i) = 270;
        x = simulation9(1,i);
        for y = simulation9(2,i):-length:simulation9(2,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            x = x + length*cosd(simulation_theta(i));
        end
    else
        y = simulation9(2,i);
        for x = simulation9(1,i):length*cosd(simulation_theta(i)):simulation9(1,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            y = y + length*sind(simulation_theta(i));
        end
    end
    clear simulation_theta
end



%% 10
up_long1     = 31;
down_long1     = 27;

% 
inc1_piont_1 = 21;   inc1_piont_2 = 28;
inc1_piont_3 = inc1_piont_1;   inc1_piont_4 = inc1_piont_2;

inc_10 = 4*[inc1_piont_1,inc1_piont_2,inc1_piont_4,inc1_piont_3;
    up_long1,up_long1,down_long1,down_long1;];
barrier10 = myrectangle(inc_10);
inc_10 = 4*[inc1_piont_1,inc1_piont_2,inc1_piont_4,inc1_piont_3;
    up_long1,up_long1,down_long1,down_long1;]*cell_wide;
simulation10 = [inc1_piont_1+length, inc1_piont_2-length, inc1_piont_4-length, inc1_piont_3+length, inc1_piont_1+length;
    up_long1+length, up_long1+length, down_long1-length, down_long1-length, up_long1+length];

for i = 1:4        
    simulation_theta(i) = atand((simulation10(2,i+1)-simulation10(2,i))/(simulation10(1,i+1)-simulation10(1,i)));
    if ((simulation10(1,i+1)-simulation10(1,i))<0 )
        simulation_theta(i) = simulation_theta(i) +180;
        y = simulation10(2,i);
        for x = simulation10(1,i):length*cosd(simulation_theta(i)):simulation10(1,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            y = y + length*sind(simulation_theta(i));
        end
    elseif ((simulation10(1,i+1)-simulation10(1,i))==0) && (simulation10(2,i+1)-simulation10(2,i))>0
        simulation_theta(i) = 90;
        x = simulation10(1,i);
        for y = simulation10(2,i):length:simulation10(2,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            x = x + length*cosd(simulation_theta(i));
        end
    elseif ((simulation10(1,i+1)-simulation10(1,i))==0) && (simulation10(2,i+1)-simulation10(2,i))<0
        simulation_theta(i) = 270;
        x = simulation10(1,i);
        for y = simulation10(2,i):-length:simulation10(2,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            x = x + length*cosd(simulation_theta(i));
        end
    else
        y = simulation10(2,i);
        for x = simulation10(1,i):length*cosd(simulation_theta(i)):simulation10(1,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            y = y + length*sind(simulation_theta(i));
        end
    end
    clear simulation_theta
end



%% 11
up_long1     = 25;
down_long1     = 20;

% 
inc1_piont_1 = 35;   inc1_piont_2 = 38;
inc1_piont_3 = inc1_piont_1;   inc1_piont_4 = inc1_piont_2;

inc_11 = 4*[inc1_piont_1,inc1_piont_2,inc1_piont_4,inc1_piont_3;
    up_long1,up_long1,down_long1,down_long1;];
barrier11 = myrectangle(inc_11);
inc_11 = 4*[inc1_piont_1,inc1_piont_2,inc1_piont_4,inc1_piont_3;
    up_long1,up_long1,down_long1,down_long1;]*cell_wide;
simulation11 = [inc1_piont_1+length, inc1_piont_2-length, inc1_piont_4-length, inc1_piont_3+length, inc1_piont_1+length;
    up_long1+length, up_long1+length, down_long1-length, down_long1-length, up_long1+length];

for i = 1:4      
    simulation_theta(i) = atand((simulation11(2,i+1)-simulation11(2,i))/(simulation11(1,i+1)-simulation11(1,i)));
    if ((simulation11(1,i+1)-simulation11(1,i))<0 )
        simulation_theta(i) = simulation_theta(i) +180;
        y = simulation11(2,i);
        for x = simulation11(1,i):length*cosd(simulation_theta(i)):simulation11(1,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            y = y + length*sind(simulation_theta(i));
        end
    elseif ((simulation11(1,i+1)-simulation11(1,i))==0) && (simulation11(2,i+1)-simulation11(2,i))>0
        simulation_theta(i) = 90;
        x = simulation11(1,i);
        for y = simulation11(2,i):length:simulation11(2,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            x = x + length*cosd(simulation_theta(i));
        end
    elseif ((simulation11(1,i+1)-simulation11(1,i))==0) && (simulation11(2,i+1)-simulation11(2,i))<0
        simulation_theta(i) = 270;
        x = simulation11(1,i);
        for y = simulation11(2,i):-length:simulation11(2,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            x = x + length*cosd(simulation_theta(i));
        end
    else
        y = simulation11(2,i);
        for x = simulation11(1,i):length*cosd(simulation_theta(i)):simulation11(1,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            y = y + length*sind(simulation_theta(i));
        end
    end
    clear simulation_theta
end



%% 12
up_long1     = 34;
down_long1     = 28;

% 
inc1_piont_1 = 32;   inc1_piont_2 = 37;
inc1_piont_3 = inc1_piont_1;   inc1_piont_4 = inc1_piont_2;

inc_12 = 4*[inc1_piont_1,inc1_piont_2,inc1_piont_4,inc1_piont_3;
    up_long1,up_long1,down_long1,down_long1;];
barrier12 = myrectangle(inc_12);
inc_12 = 4*[inc1_piont_1,inc1_piont_2,inc1_piont_4,inc1_piont_3;
    up_long1,up_long1,down_long1,down_long1;]*cell_wide;
simulation12 = [inc1_piont_1+length, inc1_piont_2-length, inc1_piont_4-length, inc1_piont_3+length, inc1_piont_1+length;
    up_long1+length, up_long1+length, down_long1-length, down_long1-length, up_long1+length];

for i = 1:4         
    simulation_theta(i) = atand((simulation12(2,i+1)-simulation12(2,i))/(simulation12(1,i+1)-simulation12(1,i)));
    if ((simulation12(1,i+1)-simulation12(1,i))<0 )
        simulation_theta(i) = simulation_theta(i) +180;
        y = simulation12(2,i);
        for x = simulation12(1,i):length*cosd(simulation_theta(i)):simulation12(1,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            y = y + length*sind(simulation_theta(i));
        end
    elseif ((simulation12(1,i+1)-simulation12(1,i))==0) && (simulation12(2,i+1)-simulation12(2,i))>0
        simulation_theta(i) = 90;
        x = simulation12(1,i);
        for y = simulation12(2,i):length:simulation12(2,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            x = x + length*cosd(simulation_theta(i));
        end
    elseif ((simulation12(1,i+1)-simulation12(1,i))==0) && (simulation12(2,i+1)-simulation12(2,i))<0
        simulation_theta(i) = 270;
        x = simulation12(1,i);
        for y = simulation12(2,i):-length:simulation12(2,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            x = x + length*cosd(simulation_theta(i));
        end
    else
        y = simulation12(2,i);
        for x = simulation12(1,i):length*cosd(simulation_theta(i)):simulation12(1,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            y = y + length*sind(simulation_theta(i));
        end
    end
    clear simulation_theta
end

% patch(inc_12(1,:),inc_12(2,:),[0.7,0.7,0.7], 'edgecolor', 'k','LineWidth',1);
%% 13
up_long1     = 9;
down_long1     = 4;

% 
inc1_piont_1 = 30;   inc1_piont_2 = 35;
inc1_piont_3 = inc1_piont_1;   inc1_piont_4 = inc1_piont_2;

inc_13 = 4*[inc1_piont_1,inc1_piont_2,inc1_piont_4,inc1_piont_3;
    up_long1,up_long1,down_long1,down_long1;];
barrier13 = myrectangle(inc_13);
inc_13 = 4*[inc1_piont_1,inc1_piont_2,inc1_piont_4,inc1_piont_3;
    up_long1,up_long1,down_long1,down_long1;]*cell_wide;
simulation13 = [inc1_piont_1+length, inc1_piont_2-length, inc1_piont_4-length, inc1_piont_3+length, inc1_piont_1+length;
    up_long1+length, up_long1+length, down_long1-length, down_long1-length, up_long1+length];

for i = 1:4         
    simulation_theta(i) = atand((simulation13(2,i+1)-simulation13(2,i))/(simulation13(1,i+1)-simulation13(1,i)));
    if ((simulation13(1,i+1)-simulation13(1,i))<0 )
        simulation_theta(i) = simulation_theta(i) +180;
        y = simulation13(2,i);
        for x = simulation13(1,i):length*cosd(simulation_theta(i)):simulation13(1,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            y = y + length*sind(simulation_theta(i));
        end
    elseif ((simulation13(1,i+1)-simulation13(1,i))==0) && (simulation13(2,i+1)-simulation13(2,i))>0
        simulation_theta(i) = 90;
        x = simulation13(1,i);
        for y = simulation13(2,i):length:simulation13(2,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            x = x + length*cosd(simulation_theta(i));
        end
    elseif ((simulation13(1,i+1)-simulation13(1,i))==0) && (simulation13(2,i+1)-simulation13(2,i))<0
        simulation_theta(i) = 270;
        x = simulation13(1,i);
        for y = simulation13(2,i):-length:simulation13(2,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            x = x + length*cosd(simulation_theta(i));
        end
    else
        y = simulation13(2,i);
        for x = simulation13(1,i):length*cosd(simulation_theta(i)):simulation13(1,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            y = y + length*sind(simulation_theta(i));
        end
    end
    clear simulation_theta
end


%% 14
up_long1     = 15;
down_long1     = 11;

% 
inc1_piont_1 = 30;   inc1_piont_2 = 35;
inc1_piont_3 = inc1_piont_1;   inc1_piont_4 = inc1_piont_2;

inc_14 = 4*[inc1_piont_1,inc1_piont_2,inc1_piont_4,inc1_piont_3;
    up_long1,up_long1,down_long1,down_long1;];
barrier14 = myrectangle(inc_14);
inc_14 = 4*[inc1_piont_1,inc1_piont_2,inc1_piont_4,inc1_piont_3;
    up_long1,up_long1,down_long1,down_long1;]*cell_wide;
simulation14 = [inc1_piont_1+length, inc1_piont_2-length, inc1_piont_4-length, inc1_piont_3+length, inc1_piont_1+length;
    up_long1+length, up_long1+length, down_long1-length, down_long1-length, up_long1+length];

for i = 1:4         
    simulation_theta(i) = atand((simulation14(2,i+1)-simulation14(2,i))/(simulation14(1,i+1)-simulation14(1,i)));
    if ((simulation14(1,i+1)-simulation14(1,i))<0 )
        simulation_theta(i) = simulation_theta(i) +180;
        y = simulation14(2,i);
        for x = simulation14(1,i):length*cosd(simulation_theta(i)):simulation14(1,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            y = y + length*sind(simulation_theta(i));
        end
    elseif ((simulation14(1,i+1)-simulation14(1,i))==0) && (simulation14(2,i+1)-simulation14(2,i))>0
        simulation_theta(i) = 90;
        x = simulation14(1,i);
        for y = simulation14(2,i):length:simulation14(2,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            x = x + length*cosd(simulation_theta(i));
        end
    elseif ((simulation14(1,i+1)-simulation14(1,i))==0) && (simulation14(2,i+1)-simulation14(2,i))<0
        simulation_theta(i) = 270;
        x = simulation14(1,i);
        for y = simulation14(2,i):-length:simulation14(2,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            x = x + length*cosd(simulation_theta(i));
        end
    else
        y = simulation14(2,i);
        for x = simulation14(1,i):length*cosd(simulation_theta(i)):simulation14(1,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            y = y + length*sind(simulation_theta(i));
        end
    end
    clear simulation_theta
end


%% 14
up_long1     = 16.5;
down_long1     = 12;

% 
inc1_piont_1 = 14;   inc1_piont_2 = 18;
inc1_piont_3 = inc1_piont_1;   inc1_piont_4 = inc1_piont_2;

inc_15 = 4*[inc1_piont_1,inc1_piont_2,inc1_piont_4,inc1_piont_3;
    up_long1,up_long1,down_long1,down_long1;];
barrier15 = myrectangle(inc_15);
inc_15 = 4*[inc1_piont_1,inc1_piont_2,inc1_piont_4,inc1_piont_3;
    up_long1,up_long1,down_long1,down_long1;]*cell_wide;
simulation15 = [inc1_piont_1+length, inc1_piont_2-length, inc1_piont_4-length, inc1_piont_3+length, inc1_piont_1+length;
    up_long1+length, up_long1+length, down_long1-length, down_long1-length, up_long1+length];

for i = 1:4         
    simulation_theta(i) = atand((simulation15(2,i+1)-simulation15(2,i))/(simulation15(1,i+1)-simulation15(1,i)));
    if ((simulation15(1,i+1)-simulation15(1,i))<0 )
        simulation_theta(i) = simulation_theta(i) +180;
        y = simulation15(2,i);
        for x = simulation15(1,i):length*cosd(simulation_theta(i)):simulation15(1,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            y = y + length*sind(simulation_theta(i));
        end
    elseif ((simulation15(1,i+1)-simulation15(1,i))==0) && (simulation15(2,i+1)-simulation15(2,i))>0
        simulation_theta(i) = 90;
        x = simulation15(1,i);
        for y = simulation15(2,i):length:simulation15(2,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            x = x + length*cosd(simulation_theta(i));
        end
    elseif ((simulation15(1,i+1)-simulation15(1,i))==0) && (simulation15(2,i+1)-simulation15(2,i))<0
        simulation_theta(i) = 270;
        x = simulation15(1,i);
        for y = simulation15(2,i):-length:simulation15(2,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            x = x + length*cosd(simulation_theta(i));
        end
    else
        y = simulation15(2,i);
        for x = simulation15(1,i):length*cosd(simulation_theta(i)):simulation15(1,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            y = y + length*sind(simulation_theta(i));
        end
    end
    clear simulation_theta
end







[num,dim] = size(p_captor);   

drawn_captor = [];
drawn_captor_b = [];
drawn_captor_bb = [];


obstacle_v1 = 0.2; 
up_long1 = 24;
down_long1 = 20;

obstacle_v2 = 0.2; 
up_long2 = 24;
down_long2 = 19;

% time = datestr(now, 30); 
% filename = ['new_obs',time];
% writerObj = VideoWriter(filename); 
% writerObj.FrameRate = 5; 
% open(writerObj);

for i = 1:time_step
    
    up_long1     = up_long1 + obstacle_v1;
    down_long1     = down_long1 + obstacle_v1;
    if up_long1 >= 28 || down_long1 <= 20
        obstacle_v1 = - obstacle_v1;
    end
    left_wide1 = 14;   righe_wide1 = 18;
    [inc_4,barrier4,num_simulation1,simulation_captor1] = generate_dynamicObs(left_wide1, righe_wide1, down_long1, up_long1, num_simulation, simulation_captor);


    up_long2     = up_long2 - obstacle_v2;
    down_long2     = down_long2 - obstacle_v2;
    if up_long2 >= 25 || down_long2 <= 16
        obstacle_v2 = - obstacle_v2;
    end
    left_wide2 = 30;   righe_wide2 = 33;
    [inc_16,barrier16,num_simulation1,simulation_captor1] = generate_dynamicObs(left_wide2, righe_wide2, down_long2, up_long2, num_simulation1, simulation_captor1);


    pattern_barrier = barrier1'+ barrier2'+ barrier3'+ barrier4'+ barrier5'+ barrier6'+ barrier7'+ barrier8'+ barrier9' +barrier10'+ barrier11'+ barrier12'+ barrier13'+ barrier14'+ barrier15'+ barrier16';
    
    [num,dim] = size(simulation_captor1);
    
    for index = 1:num
        if isnan(simulation_captor1(index))
            simulation_captor1(index) = simulation_captor1(index+1);
        end
    end
    pattern_simulatecaptor = generate_simulatepattern(simulation_captor1);


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

    
    
    r=1;
    k=1;
    j=1;
    temp_captor = [];  
    temp_captor_b = [];
     temp_captor_bb = [];
    for j=1:num_robot
        if (p_captor(j,3) == 1) 
            


            
            x_mat(i,j) = p_captor(j, 1);
            y_mat(i,j) = p_captor(j, 2);
            
            x_d_mat(i, j) = point(j, 1);
            y_d_mat(i, j) = point(j, 2);
            
            
            x_b_mat(i,j) = pp_captor(j, 1);
            y_b_mat(i,j) = pp_captor(j, 2);
            
            
            x_d_bp_mat(i,j) =  point(j, 1);
            y_d_bp_mat(i,j) =  point(j, 2);
            
            
            x_bb_mat(i,j) = ppp_captor(j, 1);
            y_bb_mat(i,j) = ppp_captor(j, 2);
            
          
            x_dd_mat(i, j) = point(j, 1);
            y_dd_mat(i, j) = point(j, 2);
           
            
              if i >= 20
               bp_x_d= [x_d_mat(i,j);x_d_mat(i-1,j);x_d_mat(i-2,j);x_d_mat(i-3,j);y_d_mat(i,j);y_d_mat(i-1,j);y_d_mat(i-2,j);y_d_mat(i-3,j)];
               bp_x_d = bp_x_d';
               [bp_d] = point_ANN(bp_x_d);
               x_d_bp_mat(i,j) = bp_d(1);
               y_d_bp_mat(i,j) = bp_d(3);
%                x_b_mat(i,j) = bp_d(1);
%                y_b_mat(i,j) = bp_d(2);
              end

               if i >= 10
               x_bp = [x_b_mat(i,j);x_b_mat(i-1,j);x_b_mat(i-2,j);x_b_mat(i-3,j);y_b_mat(i,j);y_b_mat(i-1,j);y_b_mat(i-2,j);y_b_mat(i-3,j)];
               x_bp = x_bp';
               [x_b_bp] = x_b_NeuralNetworkFunction(x_bp);
            end

             %%
            %改动控制器部分
%             tic
%             move = UAV_sliding_model_controller(i,j,dt,x_mat,y_mat,x_d_mat,y_d_mat);
%             [move_b] = Sliding_model_controller(i,j,dt,x_mat,y_mat,x_d_mat,y_d_mat);
%             [move] = enhance_Sliding_model_controller(i,j,dt,x_mat,y_mat,x_d_mat,y_d_mat);
%             [move_b] = BPenhance_Sliding_model_controller(i,j,dt,x_b_mat,y_b_mat,x_d_mat,y_d_mat);           
%             [move_b] = enhance_Sliding_model_controller(i,j,dt,x_mat,y_mat,x_d_mat,y_d_mat);
             [move_b] = unlimit_Sliding_model_controller(i,j,dt,x_b_mat,y_b_mat,x_d_mat,y_d_mat);

                        %%

           move_bb = CHtrapping(ppp_captor,p_fish,j,pattern_target,pattern_barrier,factors);
           move_bb = CHavoiding(ppp_captor,move_bb,j,pattern_barrier,p_fish);
           error(i,j) =  ((ppp_captor(j, 1) - point(j, 1))^2 + (ppp_captor(j, 2) - point(j, 2))^2)^0.5;
%%
            %%GRN-1
%           tic 
            move = trapping(p_captor,p_fish,j,pattern_target,pattern_point);
            move = avoiding(p_captor,move,j,pattern_barrier,p_fish);        
            
                   % Accumulate errors for each robot
        if i == 1
            cumulative_error(i, j) = error(i, j);
        else
            cumulative_error(i, j) = cumulative_error(i - 1, j) + error(i, j);
        end
            
        
            dis_robotTOpattern = sqrt( (p_captor(j,1)-p_fish(:,1)).^2 + (p_captor(j,2)-p_fish(:,2)).^2 );
            if dis_robotTOpattern <= 0.5
               move(1) = 0;
               move(2) = 0;
            end
            if j > 1
             dis_robotTOpattern = sqrt( (p_captor(j,1)-p_captor(j-1,1)).^2 + (p_captor(j,2)-p_captor(j-1,2)).^2 );
            if dis_robotTOpattern <= 0.5
               move(1) = 0;
               move(2) = 0;
            end
            end
            
                        dis_robotTOpattern = sqrt( (ppp_captor(j,1)-p_fish(:,1)).^2 + (ppp_captor(j,2)-p_fish(:,2)).^2 );
            if dis_robotTOpattern <= 0.5
               move_bb(1) = 0;
               move_bb(2) = 0;
            end
            if j > 1
             dis_robotTOpattern = sqrt( (ppp_captor(j,1)-ppp_captor(j-1,1)).^2 + (ppp_captor(j,2)-ppp_captor(j-1,2)).^2 );
            if dis_robotTOpattern <= 0.5
               move_bb(1) = 0;
               move_bb(2) = 0;
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
            if dis_robotTOpattern <= 0.25
               move_b(1) = 0;
               move_b(2) = 0;
            end
                          
        end
            
            p_captor(j,1) = p_captor(j,1)+move(1);
            p_captor(j,2) = p_captor(j,2)+move(2);
            p_captor(j,4) = move(1);
            p_captor(j,5) = move(2);
            p_organizing(r,:) = p_captor(j,:);
            
            ppp_captor(j,1) = ppp_captor(j,1)+move_bb(1);
            ppp_captor(j,2) = ppp_captor(j,2)+move_bb(2);
            ppp_captor(j,4) = move_bb(1);
            ppp_captor(j,5) = move_bb(2);
            p_organizing(r,:) = ppp_captor(j,:);
            
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
        robot_bb = [ppp_captor(j,1),ppp_captor(j,2)];
        
        temp_captor = [temp_captor,robot]; 
        temp_captor_b = [temp_captor_b,robot_b]; 
        temp_captor_bb = [temp_captor_bb,robot_bb]; 
        
         point_rec1(j , i) = point(j , 1);
         point_rec1(j + 10 , i) = point(j , 2);
         point_rec1(:,i+1) = 0;
         
          
         point_rec2(j , i+1) = point_rec1(j , i);
         point_rec2(j + 10 , i+1) = point_rec1(j+10 , i);
         
        point_change(j, i) = point_rec1(j, i) - point_rec2(j, i) ;
        point_change(j+10, i) = point_rec1(j+10, i) - point_rec2(j+10, i) ;
        point_change(:, 1) = 0;
        
        d_zero = zeros(num_robot * 2 ,1);
        d_mat = [x_d_bp_mat';y_d_bp_mat'];
        dd_mat = [d_zero d_mat];
        d_mat = [d_mat d_zero];
        ddd_mat = d_mat - dd_mat;
        ddd_mat(:, 1) = 0;
        ddd_mat(:,end) = [];


        captor_change(j , i) = p_captor(j, 4);
        captor_change(j + 10 , i) = p_captor(j, 5);
    
        captor_change_b(j , i) = pp_captor(j, 4);
        captor_change_b(j + 10, i) = pp_captor(j, 5);
    
        captor_change_bb(j , i) = ppp_captor(j, 4);
        captor_change_bb(j + 10 , i) = ppp_captor(j, 5);
        
        v_p_fish1(j , i) = p_fish(1, 1);
        v_p_fish1(j + 10 , i) = p_fish(1, 2);

        
        if i>1
            v_p(i,1) = v_p_fish1(1,i) - v_p_fish1(1,i-1);
            Vp = v_p';
        end
        
    end

    drawn_captor = [drawn_captor;temp_captor];
    drawn_captor_b = [drawn_captor_b;temp_captor_b];
    drawn_captor_bb = [drawn_captor_bb;temp_captor_bb];

 


    p3 = plot(p_fish(:,1),p_fish(:,2),'*','LineWidth',1,'MarkerSize', 4,'color','#FF0000');              %四个target用.画出
    hold on;
    %%


pos_max = 64;
p_size = [0.01*ones(1,16), 0.1*ones(1,16), 0.2*ones(1,16), 0.5*ones(1,16)]; 
if i > 1
    % 修正后的循环
%     for num_j1 = 1:1:size(2*pos_list1, 1)
%         tmp_p1 = pos_list1(num_j1, :);  % 直接引用 pos_list 中的行
%         plot(tmp_p1(1), tmp_p1(2), 'o', 'MarkerFaceColor', [0.3,0.74,0.93], 'MarkerSize', p_size(num_j1));%green [0.65, 0.85, 0.65] blue[0.3,0.74,0.93]
%         hold on;
%     end
    
    for num_j2 = 1:1:size(2 * pos_list2, 1)
        tmp_p2 = pos_list2(num_j2, :);  % 直接引用 pos_list 中的行
%         plot(tmp_p2(1), tmp_p2(2), 'o', 'MarkerFaceColor', [1 ,0.55 ,0.0], 'MarkerSize', p_size(num_j2));
        hold on;
    end
    
%     for num_j3 = 1:1:size(2*pos_list3, 1)
%         tmp_p3 = pos_list3(num_j3, :);  % 直接引用 pos_list 中的行
%         plot(tmp_p3(1), tmp_p3(2), 'o', 'MarkerFaceColor', [0.65, 0.85, 0.65], 'MarkerSize', p_size(num_j3));%green [0.65, 0.85, 0.65] blue[0.3,0.74,0.93]
%         hold on;
%     end
%     
end
    
 if i == 1
    pos_list1 = p_captor(1:num_robot, 1:2);
    pos_list2 = pp_captor(1:num_robot, 1:2); 
    pos_list3 = ppp_captor(1:num_robot, 1:2); 
else
    pos_list1 = [pos_list1; p_captor(1:num_robot, 1:2)];  
    pos_list2 = [pos_list2; pp_captor(1:num_robot, 1:2)]; 
    pos_list3 = [pos_list3; ppp_captor(1:num_robot, 1:2)]; 
    if size(pos_list1, 1) > pos_max
        pos_list1 = pos_list1(end-pos_max+1:end, :);  
    end
    if size(pos_list2, 1) > pos_max
        pos_list2 = pos_list2(end-pos_max+1:end, :);  
    end
    if size(pos_list3, 1) > pos_max
        pos_list3 = pos_list3(end-pos_max+1:end, :);  
    end
end

 

    %%

    

    if k~=1
        plot(p_nonorganizing(:,1),p_nonorganizing(:,2),'hexagram','color',[0.00,0.00,1.00],'MarkerSize',4)               %nonorganizer用.画出
    end
    clear p_organizing p_nonorganizing;

    patch(point1(1,:),point1(2,:),[0.7,0.7,0.7], 'edgecolor', 'k','LineWidth',1);   % tunnel，通道
    axis([0,40,0,40]);
    patch(point2(1,:),point2(2,:),[0.7,0.7,0.7], 'edgecolor', 'k','LineWidth',1);
    patch(inc_3(1,:),inc_3(2,:),[0.7,0.7,0.7], 'edgecolor', 'k','LineWidth',1);
    patch(inc_4(1,:),inc_4(2,:),[1.00,0.1,0.16], 'edgecolor', 'k','LineWidth',1);
    patch(inc_5(1,:),inc_5(2,:),[0.7,0.7,0.7], 'edgecolor', 'k','LineWidth',1);
    patch(inc_6(1,:),inc_6(2,:),[0.7,0.7,0.7], 'edgecolor', 'k','LineWidth',1);
    patch(inc_7(1,:),inc_7(2,:),[0.7,0.7,0.7], 'edgecolor', 'k','LineWidth',1);
    patch(inc_8(1,:),inc_8(2,:),[0.7,0.7,0.7], 'edgecolor', 'k','LineWidth',1);
    patch(inc_9(1,:),inc_9(2,:),[0.7,0.7,0.7], 'edgecolor', 'k','LineWidth',1);
    patch(inc_10(1,:),inc_10(2,:),[0.7,0.7,0.7], 'edgecolor', 'k','LineWidth',1);
    patch(inc_11(1,:),inc_11(2,:),[0.7,0.7,0.7], 'edgecolor', 'k','LineWidth',1);
    patch(inc_12(1,:),inc_12(2,:),[0.7,0.7,0.7], 'edgecolor', 'k','LineWidth',1);
    patch(inc_13(1,:),inc_13(2,:),[0.7,0.7,0.7], 'edgecolor', 'k','LineWidth',1);
    patch(inc_14(1,:),inc_14(2,:),[0.7,0.7,0.7], 'edgecolor', 'k','LineWidth',1);
    patch(inc_15(1,:),inc_15(2,:),[0.7,0.7,0.7], 'edgecolor', 'k','LineWidth',1);
    patch(inc_16(1,:),inc_16(2,:),[1.00,0.1,0.16], 'edgecolor', 'k','LineWidth',1);
    hold on;


    if i <= t1_end
        p_fish(1,1) = p_fish(1,1);
        p_fish(1,2) = p_fish(1,2) + v11;
    elseif i <= t2_end
        if i == t1_end+1
            center_x = p_fish(1,1) + r11;
            center_y = p_fish(1,2) ;
        end
        t = i - t1_end;
        theta = pi - (pi/2 * (t / t_steps(2))); 
        p_fish(1,1) = center_x + r11 * cos(theta);
        p_fish(1,2) = center_y + r11 * sin(theta);
    elseif i <= t3_end
        p_fish(1,1) = p_fish(1,1) + v11;
        p_fish(1,2) = p_fish(1,2);
    elseif i <= t4_end
        if i == t3_end+1
            center_x = p_fish(1,1) ;
            center_y = p_fish(1,2) + r11;
        end
        t = i - t3_end;
        theta = -pi/2 + (pi/2 *(t / t_steps(4))); 
        p_fish(1,1) = center_x + r11 * cos(theta);
        p_fish(1,2) = center_y + r11 * sin(theta);
    elseif i <= t5_end
        p_fish(1,1) = p_fish(1,1);
        p_fish(1,2) = p_fish(1,2) + v11 ;
    elseif i <= t6_end
        if i == t5_end+1
            center_x = p_fish(1,1) + r11;
            center_y = p_fish(1,2) ;
        end
        t = i - t5_end;
        theta = pi - (pi/2 * (t / t_steps(6))); 
        p_fish(1,1) = center_x + r11 * cos(theta);
        p_fish(1,2) = center_y + r11 * sin(theta);
    elseif i <= t7_end
        p_fish(1,1) = p_fish(1,1) + v11 ;
        p_fish(1,2) = p_fish(1,2) ;
    elseif i <= t8_end
        if i == t7_end+1
            center_x = p_fish(1,1) ;
            center_y = p_fish(1,2) + r11;
        end
        t = i - t7_end;
        theta = -pi/2 + (pi/2 *(t / t_steps(8))); 
        p_fish(1,1) = center_x + r11 * cos(theta);
        p_fish(1,2) = center_y + r11 * sin(theta);
    elseif i <= t9_end
        p_fish(1,1) = p_fish(1,1);
        p_fish(1,2) = p_fish(1,2) + v11;
    elseif i <= t10_end
        if i == t9_end+1
            center_x = p_fish(1,1) + r11;
            center_y = p_fish(1,2) ;
        end
        t = i - t9_end;
        theta = pi - (pi/2 * (t / t_steps(10))); 
        p_fish(1,1) = center_x + r11 * cos(theta);
        p_fish(1,2) = center_y + r11 * sin(theta);
    elseif i <= t11_end
        p_fish(1,1) = p_fish(1,1) + v11 ;
        p_fish(1,2) = p_fish(1,2) ;
    elseif i <= t12_end
        t = i - t11_end;
        dx = target(7,1) - target(6,1); % 2
        dy = target(7,2) - target(6,2); % 3.5
        dist = distances(7); % ≈ 4.031
        p_fish(1,1) = target(6,1) + (dx / dist) * v11 * t;
        p_fish(1,2) = target(6,2) + (dy / dist) * v11 * t;
    elseif i <= t13_end
        p_fish(1,1) = p_fish(1,1);
        p_fish(1,2) = p_fish(1,2) + v11 ;
    end

         
         v1 = 0.3;
         v2 = 0.3; 
         v = 0.3;
         

    hold on;

    p9 = plot(pp_captor(:,1),pp_captor(:,2),'o','MarkerFaceColor',[1 ,0.55 ,0.0],'MarkerEdgeColor',[0.7,0.13,0.13],'MarkerSize', 4,'LineWidth',0.5);
    

        for num_i = 1:num_robot
    arrow_length = 1.5;  

        end
        
set(gca, 'FontSize', 15, 'FontName', 'Times New Roman');  % 同时设置字号和字体
xlabel('x/m','FontSize', 15, 'FontName', 'Times New Roman');            % 设置x轴标签字体
ylabel('y/m','FontSize', 15, 'FontName', 'Times New Roman');            % 设置y轴标签字体
    hold off;
    drawnow;
    disp(i)

    
    
%     if i == 505 || i == 582 || i == 592 || i == 562 || i == 898  || i == 935 || i == 963 || i == 1147
%     set(gcf, 'Position', [100, 100, 500, 450]); % 设置屏幕显示尺寸
%     
%     % 关键修改：配置纸张设置以匹配屏幕尺寸
%     set(gcf, 'PaperPositionMode', 'auto');      % 保持屏幕比例
%     paperpos = get(gcf, 'PaperPosition');       % 获取默认纸张位置
%     papersize = paperpos(3:4);                  % 提取所需纸张尺寸
%     
%     % 设置自定义纸张大小（单位：英寸）
%     set(gcf, 'PaperSize', papersize);           % 自定义纸张尺寸
%     set(gcf, 'PaperPosition', [0 0 papersize]); % 去除边缘空白
%     
%     % 保存PDF（匹配屏幕尺寸）
%     filename_pdf = sprintf('Dynamic_%d.pdf', i);
%     print(gcf, filename_pdf, '-dpdf', '-r600', '-bestfit', '-painters');
%     
%     end
    
       

    
end
% close(writerObj); % 关闭视频文件
