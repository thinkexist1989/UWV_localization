t = 0:0.001:1;
for a = [10 20 30 40 50]
    p = 1./(1+exp(-a.*(t-0.4)));
    plot(t,p);
    hold on;
end
