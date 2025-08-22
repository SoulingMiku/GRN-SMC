function non_move = non_trapping(p_captor,p_fish,i)
  %% 初始化
global  distent_detect;
global  distent_fish;                        %与目标的安全距离
global  distent_capter;                      %与其他机器人的安全距离
    flag = 0;                       %系数：保证机器人速度一致
    v=0.5;                          %速度
    non_move(1)=0;
    non_move(2)=0;
    factor=[1 1 1 0 0.5];                         %影响因子【远离目标，远离同类，跟随目标，跟随g3最大的,去密度低的方向】
    [num1,dim] = size(p_fish); 
    [num2,dim] = size(p_captor); 
%% 在安全距离以内:立即远离
    for j=1:num1
        if(((p_captor(i,1)-p_fish(j,1))^2+(p_captor(i,2)-p_fish(j,2))^2)^0.5 <  distent_fish )
            non_move(1) = non_move(1) + factor(1)*(p_captor(i,1)-p_fish(j,1))/((p_captor(i,1)-p_fish(j,1))^2+(p_captor(i,2)-p_fish(j,2))^2)^0.5;
            non_move(2) = non_move(2) + factor(1)*(p_captor(i,2)-p_fish(j,2))/((p_captor(i,1)-p_fish(j,1))^2+(p_captor(i,2)-p_fish(j,2))^2)^0.5;  
            flag = flag + 1;
        end
    end
    for j=1:num2
        if(j~=i)                            %不判断自己
            if(((p_captor(i,1)-p_captor(j,1))^2+(p_captor(i,2)-p_captor(j,2))^2)^0.5 < distent_capter)
                non_move(1) = non_move(1) + factor(2)*(p_captor(i,1)-p_captor(j,1))/((p_captor(i,1)-p_captor(j,1))^2+(p_captor(i,2)-p_captor(j,2))^2)^0.5;
                non_move(2) = non_move(2) + factor(2)*(p_captor(i,2)-p_captor(j,2))/((p_captor(i,1)-p_captor(j,1))^2+(p_captor(i,2)-p_captor(j,2))^2)^0.5;
                flag = flag + 1;
            end
        end
    end
    if flag~=0;
        non_move(1)=non_move(1)/flag*v;
        non_move(2)=non_move(2)/flag*v;
    end
    
%% 在探测范围内：跟随，等于邻居们的方向矢量和
if(flag==0)
        for j=1:num2
            if(j~=i && (p_captor(j,3)==2)    )                            %不判断自己，并且只跟随组织机器人
                if(((p_captor(i,1)-p_captor(j,1))^2+(p_captor(i,2)-p_captor(j,2))^2)^0.5 < distent_detect)
                    non_move(1) = non_move(1) + factor(3) * p_captor(j,4);
                    non_move(2) = non_move(2) + factor(3) * p_captor(j,5);
                    flag = flag + 1;
                end
            end
        end
        if flag~=0;
            non_move(1)=non_move(1)/flag;               %用速度算的，不需要再乘速度
            non_move(2)=non_move(2)/flag;
        end
end