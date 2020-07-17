path1="/home/swxie/mujoco200_linux/sample_swxie/process_data";
M=dlmread(path1+"/UR5torques.txt");
%%
clear,clc,
path1="/home/swxie/mujoco200_linux/sample_swxie/process_data";
allctrls=dlmread(path1+"/allctrls.txt");
alltimes=dlmread(path1+"/alltimes.txt");
allqs=dlmread(path1+"/allJfMInvJT.txt"); %allJfMInvJT  allqs
%
% [row,col] =find(alltimes(end:-1:1)~=0);
idx = find(alltimes(end:-1:1), 1, 'first');
idend=length(alltimes)-idx;
figure(1),
for k=1:6
    subplot(2,3,k),
    plot(alltimes(1:idend,1),allctrls(1:idend,k),'x-');
    title("Torque of Joint "+num2str(k));
end
figure(2),
for k=1:6
    subplot(2,3,k),
    plot(alltimes(1:idend,1),allqs(1:idend,k));
    title("Angle of Joint "+num2str(k));
end
%%
allEE=dlmread(path1+"/allEE.txt");
allFT=dlmread(path1+"/allFT.txt");
figure(3);
for k=1:3
    subplot(2,3,k),
    plot(alltimes(1:idend,1),allEE(1:idend,k),'red'); %(x1,y1,z1  x2,y2,z2
    hold on,
    plot(alltimes(1:idend,1),allEE(1:idend,k+3),'green'); %(x1,y1,z1  x2,y2,z2
    plot(alltimes(1:idend,1),allEE(1:idend,k+6),'blue'); %(x1,y1,z1  x2,y2,z2
    plot(alltimes(1:idend,1),allEE(1:idend,k+9),'yellow'); %(x1,y1,z1  x2,y2,z2
    title("Position of Tip "+num2str(k));
end

%estimate Torque
estT=zeros(idend,3);
for k=1:idend
    estT(k,:)=-cross(allFT(k,1:3),(allEE(k,1:3)+allEE(k,7:9))/2-allEE(k,13:15));
end

for k=4:6
    subplot(2,3,k),
    plot(alltimes(1:idend,1),allEE(1:idend,k+9)); %(x1,y1,z1  x2,y2,z2      
    title("Position of Tip "+num2str(k));
end


figure(4);
for k=1:6
    subplot(2,3,k),
    plot(alltimes(1:idend,1),allFT(1:idend,k));hold on
    title("Force/Torque sensor "+num2str(k));
end

TorqueN2=zeros(idend,1);
ForceN2=zeros(idend,1);
for k=1:idend
    TorqueN2(k)=norm( allFT(k,4:6) );
    ForceN2(k)=norm( allFT(k,1:3) );
end

figure(5),
%plot(alltimes(1:idend,1),TorqueN2);
plot(alltimes(1:idend,1),ForceN2,'x-');

% for k=1:3
%     subplot(2,3,k+3),
%     plot(alltimes(1:idend,1),estT(1:idend,k),'red');
% end
%%
%end-effect config in workspace
allEEX=dlmread(path1+"/allEEX.txt");
figure(5);
for k=1:6
    subplot(2,3,k),
    plot(alltimes(1:idend,1),allEEX(1:idend,k)); %(x1,y1,z1  x2,y2,z2
    title("EE in workspace of Tip "+num2str(k));
end

%%


%%
%find the parameters of PID

p1p2=conv([1 20],[1 20]);
roots(p1p2)
p1p2p3=conv(p1p2,[1 3]);
roots(p1p2p3)
%%
eul = [pi/5 pi/4 pi*1.2];
rotmZYX = eul2rotm(eul); %ZYX

angs=[-2.51 0.7853 0.6283]/pi*180;
res=rotz(angs(3))*roty(angs(2))*rotx(angs(1));

%%
%plot rotated angle
N1=14/0.001;
data1=allEE(N1:idend,:);
p1=data1(:,1:3);
p2=data1(:,4:6);
p=p2-p1;
plot3(p(:,1),p(:,2),p(:,3)),
axis equal;
%%
% for k=1:length(data1)
    k=1;
x1=[data1(k,1) data1(k,4) data1(k,7) data1(k,10) data1(k,1)];
y1=[data1(k,2) data1(k,5) data1(k,8) data1(k,11) data1(k,2)];
z1=[data1(k,3) data1(k,6) data1(k,9) data1(k,12) data1(k,3)];
plot3(x1,y1,z1,'blue');hold on



    k=length(data1);
x1=[data1(k,1) data1(k,4) data1(k,7) data1(k,10) data1(k,1)];
y1=[data1(k,2) data1(k,5) data1(k,8) data1(k,11) data1(k,2)];
z1=[data1(k,3) data1(k,6) data1(k,9) data1(k,12) data1(k,3)];
plot3(x1,y1,z1,'blue');hold on

%%
k=1;
v1=[data1(k,1) data1(k,2)  data1(k,3)]-[data1(k,4) data1(k,5)  data1(k,6)];
k=length(data1);
v2=[data1(k,1) data1(k,2)  data1(k,3)]-[data1(k,4) data1(k,5)  data1(k,6)];
acos( v1/norm(v1)*v2'/norm(v2) )/pi*180


%%
%low pass filter design
[b,a]=butter(3,0.3);


%%
ang1=50/180*pi;
mu=0.8;
1/(sin(ang1)-mu*cos(ang1))







