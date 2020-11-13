% Code generates magnetic filed based on current and geometry. Measurements
% are rotated into the "line frame" with zero magnetic field in the z-axis
% Two methods are used: One finds an ellipse without reference to the
% source of the points. The second finds direct and quadrature components
% in each axis and hence rotates these into the x-y plane. The second
% method requires knowledge of the line frequency.
% https://en.wikipedia.org/wiki/Rotation_matrix
mu0=4*pi*1e-7;
Iscale=1; % used to scale the current values!

t=0:1/580:0.2; 
wt=50*2*pi*t';
wt_plot=linspace(0,2*pi);

% Sensor orientation
a= 0; ca=cosd(a); sa=sind(a); % z  phi (yaw in funny axis)
b= 10; cb=cosd(b); sb=sind(b); % y theta (pitch in funny axis)
c= 30; cc=cosd(c); sc=sind(c); % x psi (roll in funny axis)
% Ra=[ca -sa 0; sa ca 0; 0 0 1] % z
% Rb=[cb 0 sb; 0 1 0; -sb 0 cb] % y
% Rc=[1 0 0; 0 cc -sc; 0 sc cc] % x
% Ra*Rb*Rc

R=[ca*cb  ca*sb*sc-sa*cc  ca*sb*cc+sa*sc
   sa*cb  sa*sb*sc+ca*cc  sa*sb*cc-ca*sc
    -sb   cb*sc           cb*cc]

% Simulate magnetic fields based on geometry
Ia=Iscale*5*cos(wt+0);
Ib=Iscale*4.5*cos(wt+2/3*pi-0.1);
Ic=Iscale*4.0*cos(wt-2/3*pi-0.2);
% first pole
ra=1.1; rb=1.0; rc=1.05;
ta=75; tb=90; tc=105;   % These are angles to the lines in degrees
M=mu0/(2*pi)*[cosd(ta) cosd(tb) cosd(tc);sind(ta) sind(tb) sind(tc)]*...
       diag([1/ra 1/rb 1/rc]);
B=(M*[Ia,Ib,Ic]')'; % B in T
% Magnetic field in the line co-ordinate system
B=[B,zeros(length(wt),1)];

% Magnetic field in the sensor co-ordinate system
BR=B*R';      %(R*B1')'
BR=BR+0.12E-6*randn(size(BR));  % 0.12 uT rms noise

% Stuff to calculate the angle according to Method 1 but not to be used
% M=[BR(:,1:2),ones(length(BR),1)];
% abc=(M'*M)\(M'*BR(:,3));   % Assume that z=ax+by+c
% 
% theta=atand(-abc(1)); % b = y 
% psi=atand(abc(2)*cosd(theta));  % c = x axis
% phi=0; % c=0 no z-axis rotation
% cc=cosd(psi); sc=sind(psi);
% cb=cosd(theta); sb=sind(theta);
% R=[cb  sb*sc  sb*cc
%    0   cc     -sc
%   -sb  cb*sc  cb*cc]
% BU=BR*R;   % (R'*B1')'

%%
% Method 1
% z=ax+by = -tan(theta)*x+tan(psi)/cos(theta)*y 
% does not need frequency
Bx=BR(:,1);
By=BR(:,2);
Bz=BR(:,3);
Bx_bar=mean(Bx);   Bx=Bx-Bx_bar;
By_bar=mean(By);   By=By-By_bar;
Bz_bar=mean(Bz);   Bz=Bz-Bz_bar;

% Could do this as M=[Bx,By]; ab=(M'*M)\(M'*Bz), etc
Sx2=Bx'*Bx; % Sum Bx^2  
Sy2=By'*By; % Sum By^2
Sxy=Bx'*By;
Sxz=Bx'*Bz;
Syz=By'*Bz;
den=Sx2*Sy2-Sxy^2;
theta=atan2d(Sxy*Syz-Sy2*Sxz,den)
psi=atan2d(cosd(theta)*(Sx2*Syz-Sxy*Sxz),den)
% Now rotate into the x-y plane
cc=cosd(psi); sc=sind(psi);
cb=cosd(theta); sb=sind(theta);
R=[cb  sb*sc  sb*cc
   0   cc     -sc
  -sb  cb*sc  cb*cc]
BU=BR*R;   % (R'*B1')' be careful what is being rotated! BU = unrotated

% now find elipse parameters in x-y plane
BUx=BU(:,1);
BUy=BU(:,2);

M=[BUx.^2,BUx.*BUy,BUy.^2];
ABC1=(M'*M)\(M'*ones(size(BUx)));

% find ellipse parameters:
phi1=atan2(ABC1(2),ABC1(1)-ABC1(3))/2
a1=sqrt((sin(phi1)^2-cos(phi1)^2)/(-ABC1(1)*cos(phi1)^2+ABC1(3)*sin(phi1)^2));
b1=sqrt((sin(phi1)^2-cos(phi1)^2)/(ABC1(1)*sin(phi1)^2-ABC1(3)*cos(phi1)^2));

ra1=a1*b1./(sqrt(b1^2*cos(wt_plot).^2+a1^2*sin(wt_plot).^2));
Bxx=ra1.*cos(wt_plot+phi1);
Byy=ra1.*sin(wt_plot+phi1);
figure(1)
plot3(B(:,1),B(:,2),B(:,3),'.',BU(:,1),BU(:,2),BU(:,3),'x',Bxx,Byy,zeros(size(Bxx)))
axis equal

%%
% Method 2: Find sinusoidal components
Bx=BR(:,1);
By=BR(:,2);
Bz=BR(:,3);
Bx_bar=mean(Bx);   Bx=Bx-Bx_bar;
By_bar=mean(By);   By=By-By_bar;
Bz_bar=mean(Bz);   Bz=Bz-Bz_bar;

Bxd=2*mean(Bx.*sin(wt)); Bxq=2*mean(Bx.*cos(wt));
Byd=2*mean(By.*sin(wt)); Byq=2*mean(By.*cos(wt));
Bzd=2*mean(Bz.*sin(wt)); Bzq=2*mean(Bz.*cos(wt));
[Bxd, Byd, Bzd; Bxq, Byq, Bzq]*1e6


den=Bxd*Byq-Bxq*Byd;
theta=atan2d(Byd*Bzq-Byq*Bzd,den)
psi=atan2d(cosd(theta)*(Bxd*Bzq-Bxq*Bzd),den)
% Now rotate into the x-y plane
cc=cosd(psi); sc=sind(psi);
cb=cosd(theta); sb=sind(theta);
R=[cb  sb*sc  sb*cc
   0   cc     -sc
  -sb  cb*sc  cb*cc]
BU=BR*R;   % (R'*B1')' be careful what is being rotated! BU = unrotated

D=R'*[Bxd;Byd;Bzd]; % Direct and quadrature in projection to line co-ordinates
Q=R'*[Bxq;Byq;Bzq];
Bxx=D(1)*sin(wt_plot)+Q(1)*cos(wt_plot);
Byy=D(2)*sin(wt_plot)+Q(2)*cos(wt_plot);
Bzz=D(3)*sin(wt_plot)+Q(3)*cos(wt_plot);
Bzz=Bzz*0;  % it should be zero anyway

figure(2)
plot3(B(:,1),B(:,2),B(:,3),'.',BU(:,1),BU(:,2),BU(:,3),'x',Bxx,Byy,zeros(size(Bxx)))
axis equal




% plot3(BR(:,1),BR(:,2),BR(:,3),'x',...
%     Bxd.*sin(wt)+Bxq.*cos(wt),Byd.*sin(wt)+Byq.*cos(wt),Bzd.*sin(wt)+Bzq.*cos(wt),'o')
% 
% Bd2=Bxd^2+Byd^2+Bzd^2;
% BdBq=(Bxd*Bxq)+(Byd*Byq)+(Bzd*Bzq);
% Bq2=Bxq^2+Byq^2+Bzq^2;
% 
% theta=atan2(2*BdBq,Bq2-Bd2)/2;
% Bm1=sqrt(Bd2*sin(theta)^2+2*BdBq*sin(theta)*cos(theta)+Bq2*cos(theta)^2);
% theta2=theta+pi/2;
% Bm2=sqrt(Bd2*sin(theta2)^2+2*BdBq*sin(theta2)*cos(theta2)+Bq2*cos(theta2)^2);
% 
% figure(2)
% % plot(B(:,1),B(:,2),'x',sqrt(Bd2).*sin(wt+theta+pi/4),sqrt(Bq2).*cos(wt+theta+pi/4)),shg
% plot(B(:,1),B(:,2),'x',sqrt(Bd2).*sin(wt+theta),sqrt(Bq2).*cos(wt+theta)),shg
% 
% plot(B(:,1),B(:,2),'x',sqrt(Bd2).*sin(wt),sqrt(Bq2).*cos(wt)),shg
% plot(wt,sqrt(Bx.^2+By.^2+Bz.^2),wt_plot,...
%     sqrt(Bd2.*(sin(wt_plot).^2)+2*BdBq.*sin(wt_plot).*cos(wt_plot)+Bq2.*(cos(wt_plot).^2)))
% A=[sin(wt(:)),cos(wt(:)),ones(size(wt(:)))];
% zz=randn(3,3);
% zz(3,:)=0; % to have centre at origin
% B=A*zz;
% plot3(B(:,1),B(:,2),B(:,3))
% 
% 
% % model as B^2 = M_s^2 sin^2 + M_s*M_c sin*cos + M_c^2 cos^2 
% %              = a*sin^2 +b sin*cos + c*cos^2
% % turning point (a-c)*sin(2*wt) +b*cos(2*wt)=0
% 
% a=sum(zz(1,:).^2);
% b=2*zz(1,:)*zz(2,:)';
% c=sum(zz(2,:).^2);
% wt1=atan2(-b,a-c)/2
% wt1=[wt1,wt1+pi/2]
% figure, plot(wt,a*sin(wt).^2+b*sin(wt).*cos(wt)+c*cos(wt).^2, ...
%     wt1,a*sin(wt1).^2+b*sin(wt1).*cos(wt1)+c*cos(wt1).^2,'x')
% % wt2=atan(b/a)
% 
% 
% % try this out
% theta=linspace(0.1,pi/2);
% wt=linspace(0,2*pi)
% 
% theta=linspace(0.1,1,10);
% for k=1:length(theta)
%     ct=cos(theta(k)); st=sin(theta(k));
%     By(1:length(wt),k)=st^2*sin(wt);
%     Bx(1:length(wt),k)=-sin(wt)-ct*st*sqrt(3)*cos(wt);
% end
% plot(Bx,By), shg
