function move = trapping(p_captor,p_fish,i,pattern,pattern_point,bp_d)
%% 初始化

global  distent_detect;
global  distent_fish;                        %与目标的安全距离
global  distent_capter;                      %与其他机器人的安全距离
global  v2 ;                                  %机器人运动速度
flag = 0;                                %系数：保证机器人速度一定
move(1)=0;                               %该时刻横坐标移动的距离
move(2)=0;                               %该时刻纵坐标移动的距离
factor=[1 1 1 1 1 1];                %影响因子【远离目标，远离同类，跟随目标，跟随g3最大的,去密度低的方向,机器人进入包围区后降速】
[num1,dim] = size(p_fish);
[num2,dim] = size(p_captor);
xaxis = fix(p_captor(i,1)/0.25)+1;   %当前网格
yaxis = fix(p_captor(i,2)/0.25)+1;
%% 在安全距离以内:立即远离
for j=1:num1
    if(((p_captor(i,1)-p_fish(j,1))^2+(p_captor(i,2)-p_fish(j,2))^2)^0.5 <  distent_fish )
        move(1) = move(1) + factor(1)*(p_captor(i,1)-p_fish(j,1))/((p_captor(i,1)-p_fish(j,1))^2+(p_captor(i,2)-p_fish(j,2))^2)^0.5;
        move(2) = move(2) + factor(1)*(p_captor(i,2)-p_fish(j,2))/((p_captor(i,1)-p_fish(j,1))^2+(p_captor(i,2)-p_fish(j,2))^2)^0.5;
        flag = flag + 1;
    end
end
for j=1:num2
    if(j~=i)                            %不判断自己
        if(((p_captor(i,1)-p_captor(j,1))^2+(p_captor(i,2)-p_captor(j,2))^2)^0.5 < distent_capter)
            move(1) = move(1) + factor(2)*(p_captor(i,1)-p_captor(j,1))/((p_captor(i,1)-p_captor(j,1))^2+(p_captor(i,2)-p_captor(j,2))^2)^0.5;
            move(2) = move(2) + factor(2)*(p_captor(i,2)-p_captor(j,2))/((p_captor(i,1)-p_captor(j,1))^2+(p_captor(i,2)-p_captor(j,2))^2)^0.5;
            flag = flag + 1;
        end
    end
end
if flag~=0
    move(1)=move(1)/flag/2;             %计算最终移动的距离
    move(2)=move(2)/flag/2;
end
%% 向目标前进
nearfish = near(p_fish, p_captor(i,1), p_captor(i,2));      %找最近的目标
if(flag==0)         %首先不能在安全范围内，然后才能移动
    if(xaxis>0 && yaxis>0 && xaxis<101 && yaxis<101)
        move(1) = move(1) - factor(3)*(p_captor(i,1)-p_fish(nearfish,1))/((p_captor(i,1)-p_fish(nearfish,1))^2+(p_captor(i,2)-p_fish(nearfish,2))^2)^0.5;
        move(2) = move(2) - factor(3)*(p_captor(i,2)-p_fish(nearfish,2))/((p_captor(i,1)-p_fish(nearfish,1))^2+(p_captor(i,2)-p_fish(nearfish,2))^2)^0.5;
        flag = flag + factor(3);
        
    end
    
    %% 向探测范围内pattern浓度高且近的点前进
    
    if (xaxis>0 && yaxis>0 && xaxis<101 && yaxis<101 ) % && pattern(xaxis,yaxis)>0.5
        
        %         [bigg3(:,1),bigg3(:,2)] = find( pattern < 0.563 & pattern > 0.560);
        %         bigg3(:,1) = bigg3(:,1)*0.25;
        %         bigg3(:,2) = bigg3(:,2)*0.25;
        bigg3 = pattern_point;
        [num,~] = size(bigg3);
        tem_point = bigg3(round(linspace(1, num, num2+1)),:);
        tem_point(end,:) = [];
        move(1) = move(1) - factor(4)*(p_captor(i,1)-tem_point(i,1))/((p_captor(i,1)-tem_point(i,1))^2+(p_captor(i,2)-tem_point(i,2))^2)^0.5;
        move(2) = move(2) - factor(4)*(p_captor(i,2)-tem_point(i,2))/((p_captor(i,1)-tem_point(i,1))^2+(p_captor(i,2)-tem_point(i,2))^2)^0.5;

        
%         nearpattern = near(bigg3, p_captor(i,1), p_captor(i,2));            %找最近的g3>0.5
%         move(1) = move(1) - factor(4)*(p_captor(i,1)-bigg3(nearpattern,1))/((p_captor(i,1)-bigg3(nearpattern,1))^2+(p_captor(i,2)-bigg3(nearpattern,2))^2)^0.5;
%         move(2) = move(2) - factor(4)*(p_captor(i,2)-bigg3(nearpattern,2))/((p_captor(i,1)-bigg3(nearpattern,1))^2+(p_captor(i,2)-bigg3(nearpattern,2))^2)^0.5;
        flag = flag + factor(4);
        
    end
    %% 向密度低的方向前进
    flagdensity = 0;
    density(1)=0;
    density(2)=0;
    for j=1:num2
        if(j~=i)                            %不判断自己
            if(((p_captor(i,1)-p_captor(j,1))^2+(p_captor(i,2)-p_captor(j,2))^2)^0.5 < distent_detect)
                density(1)=density(1)+(p_captor(i,1)-p_captor(j,1))/((p_captor(i,1)-p_captor(j,1))^2+(p_captor(i,2)-p_captor(j,2))^2)^0.5;
                density(2)=density(2)+(p_captor(i,2)-p_captor(j,2))/((p_captor(i,1)-p_captor(j,1))^2+(p_captor(i,2)-p_captor(j,2))^2)^0.5;
                flagdensity = flagdensity+1;
            end
        end
    end
    if flagdensity~=0
        density(1)=factor(5)*density(1)/flagdensity;
        density(2)=factor(5)*density(2)/flagdensity;
        flag = flag+factor(5);
    end
    
    move(1) = move(1) + density(1);
    move(2) = move(2) + density(2);
    
    if flag~=0
        move(1)=move(1)/flag;             %计算最终移动的距离
        move(2)=move(2)/flag;
    end
end

if(xaxis>0 && yaxis>0 && xaxis<101 && yaxis<101)        %pattern内的移动速度
    if pattern(xaxis,yaxis)>0.2
        v2 = v2 * factor(6);
    end
end

theta = atand(move(2)/move(1));
if (move(1)<0 )
    theta = theta +180;
end
move(1) = v2 * cosd(theta);
move(2) = v2 * sind(theta);
end