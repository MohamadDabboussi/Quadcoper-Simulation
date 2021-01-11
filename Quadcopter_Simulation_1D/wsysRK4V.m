function[x,t_]=wsysRK4V(sys_eom,x0,a,b,N)
h=(b-a)/N;
x = zeros(N,2);
t_=zeros(N,1);
x(1,:)=x0;
t_(1)=a;
for i=1:N
t_(i+1,1)=a+i*h;
K1=feval(sys_eom,t_(i),x(i,:));
K2=feval(sys_eom,t_(i)+h/2,x(i,:)+(h/2).*K1');
K3=feval(sys_eom,t_(i)+h/2,x(i,:)+(h/2).*K2');
K4=feval(sys_eom,t_(i)+h,x(i,:)+h.*K3');
x(i+1,:)=x(i,:)+(h/6)*(K1+2*K2+2*K3+K4)';
end

%K1=feval(FV,t(i),Y(i,:));
 %   K2=feval(FV,t(i)+h/2,x(i,:)+(h/2).*K1');
  %  K3=feval(FV,t(i)+h/2,x(i,:)+(h/2).*K2');
  %  K4=feval(FV,t(i)+h,x(i,:)+h.*K3');
  %  Y(i+1,:)=Y(i,:)+(h/6)*(K1+2*K2+2*K3+K4)';