function [pattern,pattern_point] = generate_pattern(p_fish,p2)

global cell_wid;
N_cell_x = 100; N_cell_y = 100;
k      = 1;

% %%%%%%%%%%%% NSGA-2
% nxor = 0.1256; 
% nand = 1.3393;

% %%%%%%%%%%%% PPS-NSGA
% and = 1.902;
% xor = 1.436;
% xnor = 1.064;

% %%%%%
% nor = 0.45;
% orn2 = 0.72;
% orn1 = 1.24;

%%%%%   2
orn  = 0.97;
pos  = 1.65;
or   = 0.58;


for i = 1:N_cell_x
    for j = 1:N_cell_y
        p1(i,j) = estab_mor(p_fish,i,j);     %实际影响的就只有和目标的距离和目标的影响力

%%%%%%%%%%%%  PPS-NSGA2
%            g1(i,j) = sig(d(i,j) * p(i,j),and,k);
%            g2(i,j) = sig(d(i,j) * (1 - p(i,j)),xor,k) + sig(p(i,j) * (1 - d(i,j)),xor,k);
%            g3(i,j) = 1 - sig(g1(i,j) * (1 - g2(i,j)),xnor,k) - sig(g2(i,j) * (1 - g1(i,j)),xnor,k);

%         % ORN
%         g1(i,j) = sig((1-p2(i,j)) * (1-p2(i,j)),nor,k); % NOR
%         g2(i,j) = sig(p2(i,j) + (1-p1(i,j)),orn2,k); % ORN
%         g3(i,j) = sig(g2(i,j) + (1-g1(i,j)),orn1,k); % ORN

          g1(i,j) = sig(p2(i,j) + (1-p1(i,j)),orn,k); % ORN
          g2(i,j) = sig(p2(i,j),pos,k); % POS
          g3(i,j) = sig(g1(i,j) + g2(i,j),or,k); % OR
            
    end
end

colormap HOT;
x_mat = zeros(N_cell_x,N_cell_y);               %空矩阵
y_mat = zeros(N_cell_x,N_cell_y);

for i1 = 1:N_cell_x
    for j1 = 1:N_cell_y
        x_mat(i1,j1)=(i1-0.5)*cell_wid;         %求出每个细胞的中心位置坐标
        y_mat(i1,j1)=(j1-0.5)*cell_wid;
    end
end
g3 = g3 - 0.04 .* p2;

[c,h]=contour(x_mat,y_mat,g3);   
h.LineWidth = 2;
set(gcf,'color',[1,1,1]);
% set(h,'ShowText','on')%显示等高线的值
% set(h,'ShowText','on','LevelList',0.3643)%设定等高线的值为0.3643
set_con = 0.51;
set(h,'LevelList',set_con);%设定等高线的值
axis([0,25,0,25]);  
set(gca,'fontsize',30);
xlabel('x/m'),ylabel('y/m');
axis square;

[~,low]   = find(abs(c(1,:) - set_con) <= 0.0001);
pattern_point = c(:,low+1:low+c(2,low));
pattern_point = pattern_point';

% % 绘制网格图
% mesh(x_mat, y_mat, g3);
% hold on;  % 保持当前图形
% 
% % 绘制三维等高线
% contour3(x_mat, y_mat, g3, 20, 'LineWidth', 2);  % 这里可以调节等高线密度（20）
% 
% % 设置轴标签和图形参数
% xlabel('x');
% ylabel('y');
% zlabel('Concentration');
% axis([0 30 0 30 0 0.5]);  % 调整轴范围来适应显示效果
% set(gca, 'fontsize', 12);
% 
% hold off;  % 结束保持状态

pattern = g3;