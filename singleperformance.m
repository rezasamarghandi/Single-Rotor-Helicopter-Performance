% Single Rotor Helicopters Performance Analysis
% 
% Author: Reza Samarghandi 
% 
% rezasamarghandi@yahoo.com
%
%February 28, 2021

clear
clc
close all

minvel=input ('Minimum True Airspeed For Analysis (kts) : ');
maxvel=input ('Maximum True Airspeed For Analysis (kts) : ');
velocity=minvel:maxvel;
alfa_deg=input ('Angle Of Attack (Degrees): ');
alfa=0*pi/180; %angle of attack (rad)
velct=input ('Climb Velocity (kts) : ');
velc=velct*1.6878; %Climb Velocity in (ft/s)
rho=input ('Density Of Air in (lbs/ft^3) : ');
m=input ('Mass Of The Helicopter in (lbs) : '); 
r= input ('Radius Of The Main Rotor Blade (ft) : ');
rtr=input ('Radius Of The Tail Rotor Blade (ft) : ');
chord=input ('Chord Of The Main Rotor Blade (ft) : '); 
chordtr=input ('Chord Of The Tail Rotor Blade (ft) : ');  
nb=input ('Number Of Blades Of The Main Rotor : '); 
nbtr=input ('Number Of Blades Of The Tail Rotor : '); 
dtr=input ('Distance From The Tail Rotor Shaft To The Center Of Mass Of The Helicopter (ft) : ');  

cd0=input ('Main Rotor Blade Airfoil Drag Coefficient in Zero Angle Of Attack (cd0) : ');
cd0tr=input ('Tail Rotor Blade Airfoil Drag Coefficient in Zero Angle Of Attack (cd0 tr) : ');
g=32.17; %Gravitational Acceleration
rpm=input ('RPM of The Main Rotor : ');
rpmtr=input ('RPM of The Tail Rotor : ');
hpav=input ('Available power in hourse power : '); 
Pav=hpav*17696; %Available Power in (lb*ft^2/s^3)
mf=input ('fuel mass (lbs) : ');
sfchp=input ('specific fuel consumption (lb/(hp*h) : ');
sfc=sfchp/17696;
n=input ('Load Factor For Turn Radius (Use Maximum Load Factor To Find Minimum Turn Radius : ');
f=input ('Equivalent Flat Plate Area Of The Fuselage (ft^2) : '); 
k=input ('Induced power factor (Preferred Value is 1.15) : ');
kk=input ('Correction parameter (Preferred Value is 4.7) : '); 
omega=rpm*2*pi/60; %Main Rotor Rotational Speed (rad/s)
omegatr=rpmtr*2*pi/60; %Tail Rotor Rotational Speed (rad/s)
vtip=r*omega; %Main Rotor Blade Tip Speed (ft/s)
vtiptr=rtr*omegatr; %Tail Rotor Blade Tip Speed (ft/s)

a=pi*r^2; %Area of The Main Rotor (ft^2)
atr=pi*rtr^2; %Area of the Tail Rotor (ft^2)
sigma=nb*chord/(pi*r); %Solidity of The Main Rotor 
sigmatr=nbtr*chord/(pi*r); %Solidity of The Tail Rotor
w=m*g; %Weight of The Helicopter (lb*ft/s^2)
cw=w/(rho*a*vtip^2); %Coefficient of Weight

shp=17696; %hp to (lb*ft^2/s^3) 
Pc=w*velc; %Climb Power (lb*ft^2/s^3)
hpc=Pc/shp; %Climb Power (hp)
cpc=Pc/(rho*a*vtip^3);  %Climb Power Coefficient


for j=1:length(velocity)
v=velocity(j)*1.6878; %forward velocity in ft/s

miu=v*cos(alfa)/vtip; %Main Rotor Advance Ratio
miutr=miu*vtip/vtiptr; %Tail Rotor Advance Ratio

landa=sqrt(cw/2); %Initial Guess Of The Main Rotor Inflow Ratio

for i=1:50
    landa=miu*tan(alfa)+cw/(2*sqrt(miu^2+landa^2)); %Main Rotor Inflow Ratio
end

cpi=(k*cw^2)/(2*sqrt(miu^2+landa^2)); %Main Rotor Induced Power Coefficient
cp0=sigma*cd0/8*(1+kk*miu^2); %Main Rotor Profile Power Coefficient
cpp=0.5*f/a*miu^3; %Parasite Power Coefficient Of The Helicopter
Pi=cpi*rho*a*vtip^3; %Main Rotor Induced Power
P0=cp0*rho*a*vtip^3; %Main Rotor Profile Power
Pp=cpp*rho*a*vtip^3; %Parasite Power Of The Helicopter


ttr=(Pi+P0+Pp)/(omega*dtr); %Tail Rotor Required Thrust
cttr=ttr/(rho*atr*vtiptr^2); %Thrust Coefficient Of The Tail Rotor
landatr=sqrt(cttr/2); %Initial Guess Of The Tail Rotor Inflow Ratio

for i=1:50
    landatr=v*sin(alfa)/vtiptr+cttr/(2*sqrt(miutr^2+landatr^2)); %Tail Rotor Inflow Ratio
end

cpitr=(k*cttr^2)/(2*sqrt(miutr^2+landatr^2)); %Tail Rotor Induced Power Coefficient
cp0tr=sigmatr*cd0tr/8*(1+kk*miutr^2); %Tail Rotor Profile Power Coefficient

Pitr=cpitr*rho*atr*vtiptr^3/550; %Tail Rotor Induced Power
P0tr=cp0tr*rho*atr*vtiptr^3/550; %Tail Rotor Profile Power

Ptr=Pitr+P0tr; %Tail Rotor Power

cp= cpi+cp0+cpp+cpitr+cp0tr+cpc; %Total Required Power Coefficient
P=Pi+P0+Pp+Ptr+Pc; %Total Required Power
shp=17696;
hpi=Pi/shp; %Induced Power In HP 
hp0=P0/shp; %Profile Power In HP
hpp=Pp/shp; %Parasite Power In HP
hptr=Ptr/shp; %Tail Rotor Power In HP
hp=P/shp; %Total Required Power In HP

descend(j)=-cp/cw*vtip*60; %Vertical Autorotation Rate Of Descend (ft/min)

endurance(j)=mf/(P*sfc); %Endurance Of The Helicopter (Hour)

range(j)=mf*v/(P*sfc/3600); %Range Of The Helicopter (ft)

rc(j)=(Pav-P)/w; %Rate Of Climb ft/s


ld(j)=w*v/P; %L/D Of The Helicopter

radius(j)=v^2/(sqrt(n^2-1)*g); %Turn Radius For specific Load Factor



plot(velocity(j),hpi,'g.','DisplayName','Induced Power')
hold on

plot(velocity(j),hp0,'b.','DisplayName','Profile Power')
hold on

plot(velocity(j),hpp,'c.','DisplayName','Parasite Power')
hold on

plot(velocity(j),hptr,'r.','DisplayName','Tail Rotor Power')
hold on

plot(velocity(j),hp,'m.','DisplayName','Total Power')
hold on

plot(velocity(j),hpav,'k.','DisplayName','Available Power')
hold on
title('Power Curve')
xlabel('True Airspeed (kts)')
ylabel('Power (hp)')
legend('show')
end

figure
plot(velocity,descend,'c.')
title('Autorotation Descend Rate')
xlabel('True Airspeed (kts)')
ylabel('Descend Rate (ft/min)')

figure
plot(velocity,ld,'c.')
title('Lift to Drag Ratio vs Airspeed')
xlabel('True Airspeed (kts)')
ylabel('L/D')


figure
plot(velocity,rc,'c.')
title('Rate Of Climb')
xlabel('True Airspeed (kts)')
ylabel('Rate of Climb (ft/s)')





figure
plot(velocity,range,'c.')
title('Range Of The Helicopter')
xlabel('True Airspeed (kts)')
ylabel('Range (ft)')

figure
plot(velocity,endurance,'c.')
title('Endurance Of The Helicopter')
xlabel('True Airspeed (kts)')
ylabel('Endurance (hour)')

figure
plot(velocity,radius,'c.')
title('Turn Radius')
xlabel('True Airspeed (kts)')
ylabel('Turn Radius (ft)')

