
function point = generate_point(num)
% num = 6;
angle = linspace(pi/2,5*pi/2,num+1);
point = zeros(2,num);
point(1,:) = 2.5 + 1.6*cos(angle(1:num));
point(2,:) = 2.5 + 1.6*sin(angle(1:num)); 
point = point';
end