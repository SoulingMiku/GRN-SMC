function gene = sig(p,sita1,k)

%k = 39.55; % 20 26.2822

gene = 1/(1+exp(-k*(p-sita1)));
