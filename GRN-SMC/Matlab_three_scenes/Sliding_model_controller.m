function move = Sliding_model_controller(i,j,dt,x_mat,y_mat,x_d_mat,y_d_mat)
%parameters defination
C = 5;
alpha = 0.5;
K_1 = 0.01;
k1 = 0.5;
k3 = 20;
ksi = 0.5;
m = 2;
a_max = 0.3;
v_max = 0.3;

% a_max = 2;
% v_max = 2;
    
%start circulate

       
        
if i == 1
   
    x = x_mat(i, j);
    dx = 0;
%     ddx = 0;
    
    x_d = x_d_mat(i, j);
    dx_d = 0;
%     ddx_d = 0;
    
    y = y_mat(i, j);
    dy = 0;
%     ddy = 0;
    
    y_d = y_d_mat(i, j);
    dy_d = 0;
%     ddy_d = 0;
    
%     theta = theta_mat(i,j);
%     dtheta = 0;
%     theta_d = pi/4;
%     dtheta_d = 0;
    
     e_x = x - x_d;
     e_y = y - y_d;
%      e_theta = theta - theta_d ;
     
    de_x = dx - dx_d;
    de_y = dy - dy_d;
%     de_theta = dtheta - dtheta_d ;
    

        % sliding plant
%         s_x = de_x + C*e_x ;
%         s_y = de_y + C*e_y ;
        s_x = e_x;
        s_y = e_y;
%         s_theta = de_theta ;
        
        % control input
        u_x = dx_d  - k1*s_x;
        u_y = dy_d  - k1*s_y;
        if u_x > a_max
           u_x = a_max;
        end
        if u_y > a_max
           u_y = a_max;
        end
%         u_x = dx_d  - k1*s_x - ksi*sign(s_x);
%         u_y = dy_d  - k1*s_y - ksi*sign(s_y);
%         u_theta = dtheta_d - k1*s_theta - ksi*abs(s_theta)^alpha*sign(s_theta);
        
%         %theta and v
%         theta_d = atan(u_y/u_x);
%         v = u_x/cos(theta_d);
        
%         %ddx ddy
%         ddx = u_x - K_1/m * dx;
%         ddy = u_y - K_1/m * dy;
%         
        %dx dy
        dx = u_x;
        dy = u_y;
        if dx > v_max
           dx = v_max;
        end
        if dy > v_max
           dy = v_max;
        end
%         dx = v*cos(theta);
%         dy = v*sin(theta);
%         dtheta = u_theta;
        
        %Location Update
        x = x + dx *dt;
        y = y + dy *dt;
%         theta = theta + dtheta*dt;

%  %姿态子控制器
%    theta = theta_mat(i,j);
%    
%    %通过u_x,u_y得出theta_d
%    theta_d = atan(u_y/u_x);
%    theta_d_mat(i,j) = theta_d;
%    
%    dtheta = 0;
%    dtheta_d = 0;
%    
%    v = u_x/cos(theta_d);
%    %误差
%    e_theta = theta - theta_d;
%    de_theta = dtheta - dtheta_d;
%    %滑模面
%    s_theta = e_theta ;
%    %控制输入
%    u_theta =dtheta_d - k3*s_theta-ksi*sign(s_theta);
%    %更新ddtheta
%    
%    %更新dtheta
%    d_theta = u_theta;
%    %更新theta
%    theta = theta + d_theta*dt;
        
        %output 
        move(1) = dx;
        move(2) = dy;
%         theta_mat(i,j) = theta;
        error = (e_x^2 + e_y^2)^0.5;
else
        

% 获取单个机器人的状态
    x = x_mat(i, j);
    dx = x_mat(i, j)-x_mat(i-1, j);
%     ddx = dx - x_mat(i-1, j) + x_mat(i-2, j);
    
    x_d = x_d_mat(i, j);
    dx_d = x_d_mat(i, j)-x_d_mat(i-1, j);
%     ddx_d = dx_d -x_d_mat(i-1, j) + x_d_mat(i-2, j);
    
    y = y_mat(i, j);y_d = y_d_mat(i, j);
    dy = y_mat(i, j)-y_mat(i-1, j);
%     ddy = dy - y_mat(i-1, j) + y_mat(i-2, j);
    
    y_d = y_d_mat(i, j);
    dy_d = y_d_mat(i, j)-y_d_mat(i-1, j);
%     ddy_d = dy_d -y_d_mat(i-1, j) + y_d_mat(i-2, j);

%     theta = theta_mat(i,j);
%     dtheta = theta_mat(i, j)- theta_mat(i-1, j);
    
%     theta_d = theta_d_mat(i,j);
%     dtheta_d = theta_d_mat(i, j)- theta_d_mat(i-1, j);
    
    e_x = x - x_d;
     e_y = y - y_d;
%      e_theta = theta - theta_d ;
     
    de_x = dx - dx_d;
    de_y = dy - dy_d;
%     de_theta = dtheta - dtheta_d ;
    

        % sliding plant
%         s_x = de_x+C*e_x;
%         s_y = de_y+C*e_y ;
        s_x = e_x;
        s_y = e_y;
%         s_theta = de_theta ;
        
        % control input
        u_x = dx_d  - k1*s_x ;
        u_y = dy_d  - k1*s_y ;
        if u_x > a_max
           u_x = a_max;
        end
        if u_y > a_max
           u_y = a_max;
        end
%         u_x = dx_d  - k1*s_x - ksi*sign(s_x);
%         u_y = dy_d  - k1*s_y - ksi*sign(s_y);
%         u_theta = dtheta_d - k1*s_theta - ksi*abs(s_theta)^alpha*sign(s_theta);
        
%         %theta and v
%         theta_d = atan(u_y/u_x);
%         v = u_x/cos(theta_d);
        
%         %ddx ddy
%         ddx = u_x - K_1/m * dx;
%         ddy = u_y - K_1/m * dy;
%         
        %dx dy
        dx = u_x;
        dy = u_y;
%         dx = v*cos(theta);
%         dy = v*sin(theta);
%         dtheta = u_theta;
        
        %Location Update
        x = x + dx *dt;
        y = y + dy *dt;
%         theta = theta + dtheta*dt;
        if dx > v_max
           dx = v_max;
        end
        if dy > v_max
           dy = v_max;
        end
%     %姿态子控制器
%    theta = theta_mat(i,j);
%    
%    %通过u_x,u_y得出theta_d
%    theta_d = atan(u_y/u_x);
%    theta_d_mat(i,j) = theta_d;
%    
%    dtheta = theta_mat(i,j) - theta_mat(i-1,j);
%    dtheta_d = theta_d_mat(i,j) - theta_d_mat(i-1,j);
%    
%    v = u_x/cos(theta_d);
%    %误差
%    e_theta = theta - theta_d;
%    de_theta = dtheta - dtheta_d;
%    %滑模面
%    s_theta = e_theta ;
%    %控制输入
%    u_theta =dtheta_d - k3*s_theta-ksi*sign(s_theta);
%    %更新ddtheta
%    
%    %更新dtheta
%    d_theta = u_theta;
%    %更新theta
%    theta = theta + d_theta*dt;
        
        %output 
        move(1) = dx;
        move(2) = dy;
%         theta_mat(i,j) = theta;
        error = (e_x^2 + e_y^2)^0.5;
  
    
end