function [point,barrier,num_simulation,simulation_captor] = generate_dynamicObs(left_wide, right_wide, under_long, up_long, num_simulation, simulation_captor)

length         = 0.5;    %模拟障碍物的大小
cell_wid       = 0.25;

% 左窄道
point = 4*[left_wide,right_wide,right_wide,left_wide;
    under_long,under_long,up_long,up_long;];
barrier = myrectangle(point);
point = 4*[left_wide,right_wide,right_wide,left_wide;
    under_long,under_long,up_long,up_long;]*cell_wid;
simulation = [left_wide+length, right_wide-length, right_wide-length, left_wide+length, left_wide+length;
    under_long+length, under_long+length, up_long-length, up_long-length, under_long+length];

for i = 1:4         %算各条线与x正方向夹角的大小
    simulation_theta(i) = atand((simulation(2,i+1)-simulation(2,i))/(simulation(1,i+1)-simulation(1,i)));
    if ((simulation(1,i+1)-simulation(1,i))<0 )
        simulation_theta(i) = simulation_theta(i) +180;
        y = simulation(2,i);
        for x = simulation(1,i):length*cosd(simulation_theta(i)):simulation(1,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            y = y + length*sind(simulation_theta(i));
        end
    elseif ((simulation(1,i+1)-simulation(1,i))==0) && (simulation(2,i+1)-simulation(2,i))>0
        simulation_theta(i) = 90;
        x = simulation(1,i);
        for y = simulation(2,i):length:simulation(2,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            x = x + length*cosd(simulation_theta(i));
        end
    elseif ((simulation(1,i+1)-simulation(1,i))==0) && (simulation(2,i+1)-simulation(2,i))<0
        simulation_theta(i) = 270;
        x = simulation(1,i);
        for y = simulation(2,i):-length:simulation(2,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            x = x + length*cosd(simulation_theta(i));
        end
    else
        y = simulation(2,i);
        for x = simulation(1,i):length*cosd(simulation_theta(i)):simulation(1,i+1)
            num_simulation = num_simulation + 1;
            simulation_captor(num_simulation,1)= x;
            simulation_captor(num_simulation,2)= y;
            y = y + length*sind(simulation_theta(i));
        end
    end
    clear simulation_theta
end
end
