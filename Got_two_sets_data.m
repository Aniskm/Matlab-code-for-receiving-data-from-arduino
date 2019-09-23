if~isempty(instrfind)
fclose(instrfind)
delete(instrfind)
end
global s
s=serial('/dev/cu.usbmodem1421');
set(s,'BaudRate',9600);
fopen(s);
interval=3000;
t=1;
x1=[];
y1=[];
x2=[];
y2=[];
T=[];
g=[];
i=0;
p=0;
while(t<interval)
    i=i+1;
b=str2num(fgetl(s));
if mod(i,2) == 1
    x1=[x1, b];
figure(1);
plot(x1);
grid;
drawnow;
end
hold on

if mod(i,2)==2
    x2=[x2, b];
    figure(2)
  plot(x2);
grid;
t = t+1;
drawnow;
end

end
fclose(s);  
save Another11.mat x
