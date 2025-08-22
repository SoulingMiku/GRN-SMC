function move = Sat1_Sliding_model_controller(i,j,dt,x_mat,y_mat,x_d_mat,y_d_mat)
% 参数定义
C = 5;
alpha = 0.6;
k1 = 0.004;
k2 = 0.5;
m = 2;
a_max = 2;
phi = 0.05;  % 边界层厚度参数

% 速度限制逻辑
if i <= 20
    v_max = 0.2;
else
    v_max = 5;
end

% 辅助函数：饱和函数 (直接内联在代码中)
function s = sat(val, phi)
    if abs(val) > phi
        s = sign(val);
    else
        s = val / phi;
    end
end

if i == 1
    % 初始化状态
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

    % 滑模面
    s_x = de_x + C*e_x;
    s_y = de_y + C*e_y;
    
    % ===== 使用饱和函数替代符号函数 =====
    % X方向
    if abs(s_x) > phi
        sat_x = sign(s_x);
    else
        sat_x = s_x / phi;
    end
    
    % Y方向
    if abs(s_y) > phi
        sat_y = sign(s_y);
    else
        sat_y = s_y / phi;
    end
    
    % 控制输入 (修正了原代码中u_y使用s_x的错误)
    u_x = ddx_d - C*(dx - dx_d) - k1*s_x - k2*abs(s_x)^alpha*sat_x;
    u_y = ddy_d - C*(dy - dy_d) - k1*s_y - k2*abs(s_y)^alpha*sat_y;
    
    % 加速度限幅
    u_x = min(max(u_x, -a_max), a_max);
    u_y = min(max(u_y, -a_max), a_max);
    
    % 状态更新
    ddx = u_x;
    ddy = u_y;
    dx = dx + ddx * dt;
    dy = dy + ddy * dt;
    
    % 速度限幅
    dx = min(max(dx, -v_max), v_max);
    dy = min(max(dy, -v_max), v_max);
    
    % 位置更新
    x = x + dx * dt;
    y = y + dy * dt;

    move(1) = dx;
    move(2) = dy;
    error = norm([e_x, e_y]);
    
elseif i == 2
    % 第二帧数据处理 (类似修改)
    x = x_mat(i, j);
    dx = x_mat(i, j) - x_mat(i-1, j);
    ddx = 0;
    
    x_d = x_d_mat(i, j);
    dx_d = x_d_mat(i, j) - x_d_mat(i-1, j);
    ddx_d = 0;
    
    y = y_mat(i, j);
    dy = y_mat(i, j) - y_mat(i-1, j);
    ddy = 0;
    
    y_d = y_d_mat(i, j);
    dy_d = y_d_mat(i, j) - y_d_mat(i-1, j);
    ddy_d = 0;
    
    e_x = x - x_d;
    e_y = y - y_d;
    de_x = dx - dx_d;
    de_y = dy - dy_d;

    s_x = de_x + C*e_x;
    s_y = de_y + C*e_y;
    
    % ===== 使用饱和函数 =====
    if abs(s_x) > phi
        sat_x = sign(s_x);
    else
        sat_x = s_x / phi;
    end
    
    if abs(s_y) > phi
        sat_y = sign(s_y);
    else
        sat_y = s_y / phi;
    end
    
    u_x = ddx_d - C*(dx - dx_d) - k1*s_x - k2*abs(s_x)^alpha*sat_x;
    u_y = ddy_d - C*(dy - dy_d) - k1*s_y - k2*abs(s_y)^alpha*sat_y;
    
    u_x = min(max(u_x, -a_max), a_max);
    u_y = min(max(u_y, -a_max), a_max);
    
    ddx = u_x;
    ddy = u_y;
    dx = dx + ddx * dt;
    dy = dy + ddy * dt;
    
    dx = min(max(dx, -v_max), v_max);
    dy = min(max(dy, -v_max), v_max);
    
    x = x + dx * dt;
    y = y + dy * dt;
    
    move(1) = dx;
    move(2) = dy;
    error = norm([e_x, e_y]);
    
else 
    % 常规帧处理
    x = x_mat(i, j);
    dx = x_mat(i, j) - x_mat(i-1, j);
    ddx = dx - (x_mat(i-1, j) - x_mat(i-2, j));  % 修正二阶导数计算
    
    x_d = x_d_mat(i, j);
    dx_d = x_d_mat(i, j) - x_d_mat(i-1, j);
    ddx_d = dx_d - (x_d_mat(i-1, j) - x_d_mat(i-2, j));  % 修正
    
    y = y_mat(i, j);
    dy = y_mat(i, j) - y_mat(i-1, j);
    ddy = dy - (y_mat(i-1, j) - y_mat(i-2, j));  % 修正
    
    y_d = y_d_mat(i, j);
    dy_d = y_d_mat(i, j) - y_d_mat(i-1, j);
    ddy_d = dy_d - (y_d_mat(i-1, j) - y_d_mat(i-2, j));  % 修正
    
    e_x = x - x_d;
    e_y = y - y_d;
    de_x = dx - dx_d;
    de_y = dy - dy_d;

    s_x = de_x + C*e_x;
    s_y = de_y + C*e_y;
    
    % ===== 使用饱和函数 =====
    if abs(s_x) > phi
        sat_x = sign(s_x);
    else
        sat_x = s_x / phi;
    end
    
    if abs(s_y) > phi
        sat_y = sign(s_y);
    else
        sat_y = s_y / phi;
    end
    
    u_x = ddx_d - C*(dx - dx_d) - k1*s_x - k2*abs(s_x)^alpha*sat_x;
    u_y = ddy_d - C*(dy - dy_d) - k1*s_y - k2*abs(s_y)^alpha*sat_y;
    
    u_x = min(max(u_x, -a_max), a_max);
    u_y = min(max(u_y, -a_max), a_max);
    
    ddx = u_x;
    ddy = u_y;
    dx = dx + ddx * dt;
    dy = dy + ddy * dt;
    
    dx = min(max(dx, -v_max), v_max);
    dy = min(max(dy, -v_max), v_max);
    
    x = x + dx * dt;
    y = y + dy * dt;
    
    move(1) = dx;
    move(2) = dy;
    error = norm([e_x, e_y]);
end
end