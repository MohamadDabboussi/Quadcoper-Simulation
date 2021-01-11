function [t]=get_time(a,b,N)
h=(b-a)/N;
t(1)=a;
for i=1:N
    t(i+1)=t(i)+h;
end