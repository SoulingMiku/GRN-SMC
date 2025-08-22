function move = enhance_Sliding_model_controller(i,j,dt,x_mat,y_mat,x_d_mat,y_d_mat)
%parameters defination
C = 5;
alpha = 0.6;
k1 = 0.004;
k2 = 0.4;
m = 2;
% 
a_max = 1;
% v_max = 0.2;

% a_max = 2;
v_max = 0.1;
    
%start circulate
% if i<= 20
%     v_max = 0.1;
% else
%     v_max = 0.1;
% end
       
        
if i == 1
   
    x = x_mat(i, j);
    dx = 0;
    ddx = 0;
    
    x_d = x_d_mat(i, j);
    dx_d = 0;
    ddx_d = 0;
    
    y = y_mat(i, j);
    dy = 0;
    ddy = 0;
    
    y_d = y_d_mat(i, j);
    dy_d = 0;
    ddy_d = 0;
    
     e_x = x - x_d;
     e_y = y - y_d;
    de_x = dx - dx_d;
    de_y = dy - dy_d;

    

        % sliding plant
        s_x = de_x + C*e_x;
        s_y = de_y + C*e_y;
        
        % control input
        u_x = ddx_d  - C*(dx - dx_d) - k1*s_x - k2*abs(s_x)^alpha*sign(s_x);
        u_y = ddy_d  - C*(dy - dy_d) - k1*s_y - k2*abs(s_x)^alpha*sign(s_x);
        if u_x > a_max
           u_x = a_max;
        end
        if u_y > a_max
           u_y = a_max;
        end
        
        %ddx ddy
        ddx = u_x ;
        ddy = u_y ;
        
        %dx dy
        dx = dx+ddx *dt;
        dy = dy+ddy *dt;
        
        %Location Update
        x = x + dx *dt;
        y = y + dy *dt;
        
        %output 
        
     % Limit dx and dy within [-v_max, v_max]
        if dx > 0 && dx > v_max
           dx = v_max;
        elseif dx < 0 && dx < -v_max
           dx = -v_max;
        end

        if dy > 0 && dy > v_max
           dy = v_max;
        elseif dy < 0 && dy < -v_max
           dy = -v_max;
        end


        move(1) = dx;
        move(2) = dy;
        error = (e_x^2 + e_y^2)^0.5;
        
elseif i == 2

    
     x = x_mat(i, j);
     dx = x_mat(i, j)-x_mat(i-1, j);
     ddx = 0;
    
     x_d = x_d_mat(i, j);
     dx_d = x_d_mat(i, j)-x_d_mat(i-1, j);
     ddx_d = 0;
    
     y = y_mat(i, j);
     dy = y_mat(i, j)-y_mat(i-1, j);
     ddy = 0;
    
     y_d = y_d_mat(i, j);
     dy_d = y_d_mat(i, j)-y_d_mat(i-1, j);
     ddy_d = 0;
    
     e_x = x - x_d;
     e_y = y - y_d;
     de_x = dx - dx_d;
     de_y = dy - dy_d;


        % sliding plant
        s_x = de_x + C*e_x;
        s_y = de_y + C*e_y;
        
        % control input
        u_x = ddx_d  - C*(dx - dx_d) - k1*s_x - k2*abs(s_x)^alpha*sign(s_x);
        u_y = ddy_d  - C*(dy - dy_d) - k1*s_y - k2*abs(s_y)^alpha*sign(s_y);
        
        if u_x > a_max
           u_x = a_max;
        end
        
        if u_y > a_max
           u_y = a_max;
        end
        
        %ddx ddy
        ddx = u_x ;
        ddy = u_y ;
        
        %dx dy
        dx = dx+ddx *dt;
        dy = dy+ddy *dt;
        
     % Limit dx and dy within [-v_max, v_max]
        if dx > 0 && dx > v_max
           dx = v_max;
        elseif dx < 0 && dx < -v_max
           dx = -v_max;
        end

        if dy > 0 && dy > v_max
           dy = v_max;
        elseif dy < 0 && dy < -v_max
           dy = -v_max;
        end

        %Location Update
        x = x + dx *dt;
        y = y + dy *dt;
else 

% 获取单个机器人的状态
    x = x_mat(i, j);
    dx = x_mat(i, j)-x_mat(i-1, j);
    ddx = dx - x_mat(i-1, j) + x_mat(i-2, j);
    
    x_d = x_d_mat(i, j);
    dx_d = x_d_mat(i, j)-x_d_mat(i-1, j);
    ddx_d = dx_d -x_d_mat(i-1, j) + x_d_mat(i-2, j);
    
    y = y_mat(i, j);y_d = y_d_mat(i, j);
    dy = y_mat(i, j)-y_mat(i-1, j);
    ddy = dy - y_mat(i-1, j) + y_mat(i-2, j);
    
    y_d = y_d_mat(i, j);
    dy_d = y_d_mat(i, j)-y_d_mat(i-1, j);
    ddy_d = dy_d -y_d_mat(i-1, j) + y_d_mat(i-2, j);
    
    e_x = x - x_d;
    e_y = y - y_d;
    de_x = dx - dx_d;
    de_y = dy - dy_d;

    
        % sliding plant
        s_x = de_x + C*e_x;
        s_y = de_y + C*e_y;
        
        % control input
        u_x = ddx_d - C*(dx - dx_d) - k1*s_x - k2*abs(s_x)^alpha*sign(s_x);
        u_y = ddy_d - C*(dy - dy_d) - k1*s_y - k2*abs(s_y)^alpha*sign(s_y);
        
        if u_x > a_max
           u_x = a_max;
        end
        
        if u_y > a_max
           u_y = a_max;
        end
        
        %ddx ddy
        ddx = u_x ;
        ddy = u_y ;
        
        %dx dy
        dx = dx+ddx *dt;
        dy = dy+ddy *dt;
        
     % Limit dx and dy within [-v_max, v_max]
        if dx > 0 && dx > v_max
           dx = v_max;
        elseif dx < 0 && dx < -v_max
           dx = -v_max;
        end

        if dy > 0 && dy > v_max
           dy = v_max;
        elseif dy < 0 && dy < -v_max
           dy = -v_max;
        end

        
        %Location Update
        x = x + dx *dt;
        y = y + dy *dt;
      
end

        move(1) = dx;
        move(2) = dy;
        error = (e_x^2 + e_y^2)^0.5;


  
    
end