% This file contains the procedure of designing a PETRAS pebble based on
% the data stored in the smart pebble class. Be careful, the output files
% might overwrite parameter files used in SolidWorks.

% return % If you run this script, it will overwrite the current design
clear; close all; clc;

GA = true;

% Inputs from SmartPebble class
pebble = SmartPebble;

% Output file name
fileName = "parameters.txt";

% GA
if GA
	lb = [ones(9,1)*0.003;0;0.001;0.001;500];
	ub = [0.004;0.018;0.025;0.01;0.007;0.01;0.006;0.02;0.015;0;0.003;0.003;3500];
	[optimal,fval,lastPopulation] = process(lb,ub);
	selected = selectBest(lastPopulation);
	PETRAS = lastPopulation(selected,:);
else
	PETRAS = [0.0032;0.0114;0.0157;0.0049;0.0034;0.0075;0.0040;0.0098;0.0041;0.0000;0.0019;0.0014;3400];
end
[Ip,mp,rhop] = pebble_inertia(PETRAS);
[Id,md,~] = ellipsoid_inertia(pebble.a,pebble.b,pebble.c,rhop);
write_parameters(PETRAS,fileName);

% For the GA plot in the paper
return
hold on;
h = findobj(gca,'Type','line');
set(h, 'MarkerSize', 10);
set(gcf,'color','w')
set(gca,'fontsize',20)
xlabel('$\varepsilon_{GA}(-)$','interpreter','latex');
ylabel('$\bar\rho{p}(\mathrm{kg/m}^3)$','interpreter','latex');
plot(0,2526,'ro','linewidth',2);
xlim([0,5e-4])
ylim([2520,2535])
xlim([0,2e-14])
legend('Final generation','Selected candidate','location','southeast');
yticks(2520:2:2535)

%% Report
disp("===================== REPORT =====================");
disp(strcat("PETRAS mass: ", num2str(mp*1000, '%1.1f'),' g'));
disp(strcat("Average density: ", num2str(round(rhop), '%i'),' kg/m^3'));
disp(strcat("MMOI error: ", num2str(system(PETRAS), '%g')));
disp(strcat("Internal shell infill density: ",num2str(round(PETRAS(13)/4000*100),'%i'),'%, (', num2str(round(PETRAS(13)),'%i'),' kg/m^3)'));
disp(strcat('Principal inertias (PETRAS): ', num2str(Ip(1,1)*1e9,'%1.3f'), ', ', num2str(Ip(2,2)*1e9,'%1.3f'), ', ', num2str(Ip(3,3)*1e9,'%1.3f')));
disp(strcat('Principal inertias (Homogeneous): ', num2str(Id(1,1)*1e9,'%1.3f'), ', ', num2str(Id(2,2)*1e9,'%1.3f'), ', ', num2str(Id(3,3)*1e9,'%1.3f')));
disp("===============================================");

%% Plots
% Draw the selected pebble
figure(2)
X = PETRAS(1:3);
Y = PETRAS(4:6);
Z = PETRAS(7:9);
W = PETRAS(10:12);
hm = 0.01;
wm = 0.019;
lm = 0.029;
brassColor = [213,188,118]/255;
[xr,yr,zr] = ellipsoid(0,0,0,pebble.a,pebble.b,pebble.c,100);
[xp,yp,zp] = ellipsoid(0,0,0,pebble.a-pebble.shell_thickness,pebble.b-pebble.shell_thickness,pebble.c-pebble.shell_thickness,100);
surf(xr,yr,zr,'linestyle','none','facealpha',0.05); hold on;
surf(xp,yp,zp,'linestyle','none','facealpha',0.025); hold on;
drawbox(zeros(3,1),pebble.cavity,ones(1,3)*0.3,0.1);
drawbox(zeros(3,1),[hm,wm,lm],[0,1,1],0.3);
drawbox([-pebble.cavity(1)/2-X(1)/2-W(1),0,0],X,brassColor,1);
drawbox([pebble.cavity(1)/2+X(1)/2+W(1),0,0],X,brassColor,1);
drawbox([0,pebble.cavity(2)/2+Y(2)/2+W(2),-Y(3)/2-0.003],Y,brassColor,1);
drawbox([0,-pebble.cavity(2)/2-Y(2)/2-W(2),-Y(3)/2-0.003],Y,brassColor,1);
drawbox([0,pebble.cavity(2)/2+Y(2)/2+W(2),Y(3)/2+0.003],Y,brassColor,1);
drawbox([0,-pebble.cavity(2)/2-Y(2)/2-W(2),Y(3)/2+0.003],Y,brassColor,1);
drawbox([0,0,pebble.cavity(3)/2+Z(3)/2+W(3)],Z,brassColor,1);
drawbox([0,0,-pebble.cavity(3)/2-Z(3)/2-W(3)],Z,brassColor,1);
colormap([0 0 1]);
axis equal
axis off
box off
set(gcf,'color','white')

%% Functions
function error=system(val)
	pebble = SmartPebble;
	[Ip,~,rhop] = pebble_inertia(val);
	[Id,~,~] = ellipsoid_inertia(pebble.a,pebble.b,pebble.c,rhop);
	f = inside(pebble.cavity(1)/2+val(1)+val(10),val(2)/2,val(3)/2,pebble.a,pebble.b,pebble.c);
	f = f && inside(val(4)/2,pebble.cavity(2)/2+val(5)+val(11),val(6)+0.003,pebble.a,pebble.b,pebble.c);
	f = f && inside(val(7)/2,val(8)/2,pebble.cavity(3)/2+val(9)+val(12),pebble.a,pebble.b,pebble.c);
	% error = max(abs(diag(Id-Ip)./diag(Id)))/f*abs(rhop-pebble.rho_desired)/pebble.rho_desired;
    error = max(abs(diag(Id-Ip)./diag(Id))/f)+abs(rhop-pebble.rho_desired)/pebble.rho_desired/f;
end

function [I,m]=cube_inertia(x0,y0,z0,sx,sy,sz,rho)
	V = sz*sy*sx;
	m = rho*V;
	R = [x0;y0;z0];
	I = [sz^2+sy^2,0,0;0,sx^2+sz^2,0;0,0,sy^2+sx^2]*m/12 + m*(eye(3)*(R'*R)-R*R');
end

function [I,m,rho]=ellipsoid_inertia(a,b,c,rho)
	V = 4/3*pi*a*b*c;
	m = rho*V;
	rho = m/V;
	I = [b^2+c^2,0,0;0,a^2+c^2,0;0,0,a^2+b^2]*m/5;
end

function [Ip,mp,rhop]=pebble_inertia(val)
	pebble = SmartPebble;
	rhom = 1083;

	[Ie,me] = ellipsoid_inertia(pebble.a,pebble.b,pebble.c,pebble.rho_resin); % Solid external resin shell inertia
	[Ih,mh] = ellipsoid_inertia(pebble.a-pebble.shell_thickness,pebble.b-pebble.shell_thickness,pebble.c-pebble.shell_thickness,-pebble.rho_resin); % Hollow resin inertia
	Ie = Ie+Ih; me = me+mh; % Resin shell inertia

	[Ii,mi] = ellipsoid_inertia(pebble.a-pebble.shell_thickness,pebble.b-pebble.shell_thickness,pebble.c-pebble.shell_thickness,val(13)); % Solid internal shell inertia
	[Ic,mc] = cube_inertia(0,0,0,pebble.cavity(1),pebble.cavity(2),pebble.cavity(3),-val(13)); % IMU cavity inertia
	Ii = Ii+Ic; mi = mi+mc; % Internal shell inertia

	[Im,mm] = cube_inertia(0,0,0,pebble.cavity(1),pebble.cavity(2),pebble.cavity(3),rhom); % IMU inertia

	% Mass ballasts
	X = val(1:3);
	Y = val(4:6);
	Z = val(7:9);
	d = val(10:12);
	rho_FDM = val(13);
	
	[IX1,mX1] = cube_inertia( pebble.cavity(1)/2+X(1)/2+d(1),0,0,X(1),X(2),X(3),pebble.rho_ballast);
	[IX2,mX2] = cube_inertia(-pebble.cavity(1)/2-X(1)/2-d(1),0,0,X(1),X(2),X(3),pebble.rho_ballast);

	[IY1,mY1] = cube_inertia(0, pebble.cavity(2)/2+Y(2)/2+d(2), Y(3)/2+0.003,Y(1),Y(2),Y(3),pebble.rho_ballast);
	[IY2,mY2] = cube_inertia(0,-pebble.cavity(2)/2-Y(2)/2-d(2), Y(3)/2+0.003,Y(1),Y(2),Y(3),pebble.rho_ballast);
	[IY3,mY3] = cube_inertia(0, pebble.cavity(2)/2+Y(2)/2+d(2),-Y(3)/2-0.003,Y(1),Y(2),Y(3),pebble.rho_ballast);
	[IY4,mY4] = cube_inertia(0,-pebble.cavity(2)/2-Y(2)/2-d(2),-Y(3)/2-0.003,Y(1),Y(2),Y(3),pebble.rho_ballast);

	[IZ1,mZ1] = cube_inertia(0,0, pebble.cavity(3)/2+Z(3)/2+d(3),Z(1),Z(2),Z(3),pebble.rho_ballast);
	[IZ2,mZ2] = cube_inertia(0,0,-pebble.cavity(3)/2-Z(3)/2-d(3),Z(1),Z(2),Z(3),pebble.rho_ballast);
	
	% Subtract missing material from the print
	IX = IX1+IX2-(IX1+IX2)/pebble.rho_ballast*rho_FDM;
	IY = IY1+IY2+IY3+IY4-(IY1+IY2+IY3+IY4)/pebble.rho_ballast*rho_FDM;
	IZ = IZ1+IZ2-(IZ1+IZ2)/pebble.rho_ballast*rho_FDM;
	
	Ip = Ie+Ii+Im+IX+IY+IZ;
	mp = me+mi+mm+(mX1+mX2+mY1+mY2+mY3+mY4+mZ1+mZ2)-(mX1+mX2+mY1+mY2+mY3+mY4+mZ1+mZ2)/pebble.rho_ballast*val(13);
	rhop = mp/pebble.vol;
end

function drawbox(cg,sl,color,fa)
	x = [[cg(1)-sl(1)/2;cg(1)+sl(1)/2],[cg(1)-sl(1)/2;cg(1)+sl(1)/2]];
	y = [[cg(2)-sl(2)/2;cg(2)+sl(2)/2],[cg(2)-sl(2)/2;cg(2)+sl(2)/2]];
	z = [[cg(3)-sl(3)/2;cg(3)+sl(3)/2],[cg(3)-sl(3)/2;cg(3)+sl(3)/2]];
	% bottom
	p = fill3([cg(1)-sl(1)/2;cg(1)-sl(1)/2;cg(1)+sl(1)/2;cg(1)+sl(1)/2],[cg(2)-sl(2)/2;cg(2)+sl(2)/2;cg(2)+sl(2)/2;cg(2)-sl(2)/2],ones(4,1)*(cg(3)-sl(3)/2),color); hold on;
	p.FaceAlpha = fa;
	% top
	p = fill3([cg(1)-sl(1)/2;cg(1)-sl(1)/2;cg(1)+sl(1)/2;cg(1)+sl(1)/2],[cg(2)-sl(2)/2;cg(2)+sl(2)/2;cg(2)+sl(2)/2;cg(2)-sl(2)/2],ones(4,1)*(cg(3)+sl(3)/2),color);
	p.FaceAlpha = fa;
	% front
	p = fill3(ones(4,1)*(cg(1)+sl(1)/2),[cg(2)-sl(2)/2;cg(2)+sl(2)/2;cg(2)+sl(2)/2;cg(2)-sl(2)/2],[cg(3)-sl(3)/2;cg(3)-sl(3)/2;cg(3)+sl(3)/2;cg(3)+sl(3)/2],color);
	p.FaceAlpha = fa;
	% back
	p = fill3(ones(4,1)*(cg(1)-sl(1)/2),[cg(2)-sl(2)/2;cg(2)+sl(2)/2;cg(2)+sl(2)/2;cg(2)-sl(2)/2],[cg(3)-sl(3)/2;cg(3)-sl(3)/2;cg(3)+sl(3)/2;cg(3)+sl(3)/2],color);
	p.FaceAlpha = fa;
	% left
	p = fill3([cg(1)-sl(1)/2;cg(1)+sl(1)/2;cg(1)+sl(1)/2;cg(1)-sl(1)/2],ones(4,1)*(cg(2)-sl(2)/2),[cg(3)-sl(3)/2;cg(3)-sl(3)/2;cg(3)+sl(3)/2;cg(3)+sl(3)/2],color);
	p.FaceAlpha = fa;
	% right
	p = fill3([cg(1)-sl(1)/2;cg(1)+sl(1)/2;cg(1)+sl(1)/2;cg(1)-sl(1)/2],ones(4,1)*(cg(2)+sl(2)/2),[cg(3)-sl(3)/2;cg(3)-sl(3)/2;cg(3)+sl(3)/2;cg(3)+sl(3)/2],color);
	p.FaceAlpha = fa;
	axis equal
end

function f=inside(x,y,z,a,b,c)
	f = (x/a)^2+(y/b)^2+(z/c)^2<0.9^2;
end

function write_parameters(vals,filename)
	pebble = SmartPebble;
	file = fopen(filename,'w');
	fprintf(file,'"z_1" = %1.3f\n',pebble.y1);
	fprintf(file,'"z_2" = %1.3f\n',pebble.y2);
	fprintf(file,'"z_3" = %1.3fm\n',pebble.y3);
	fprintf(file,'"a" = "z_1" * "z_3"\n');
	fprintf(file,'"b" = "z_2" * "z_3"\n');
	fprintf(file,'"c" = "z_3"\n');
	fprintf(file,'"X1" = %1.4fm\n',vals(1));
	fprintf(file,'"X2" = %1.4fm\n',vals(2));
	fprintf(file,'"X3" = %1.4fm\n',vals(3));
	fprintf(file,'"Y1" = %1.4fm\n',vals(4));
	fprintf(file,'"Y2" = %1.4fm\n',vals(5));
	fprintf(file,'"Y3" = %1.4fm\n',vals(6));
	fprintf(file,'"Z1" = %1.4fm\n',vals(7));
	fprintf(file,'"Z2" = %1.4fm\n',vals(8));
	fprintf(file,'"Z3" = %1.4fm\n',vals(9));
	fprintf(file,'"xshift" = %1.4fm\n',vals(10));
	fprintf(file,'"yshift" = %1.4fm\n',vals(11));
	fprintf(file,'"zshift" = %1.4fm\n',vals(12));
	fclose(file);
end

function [x_optimal,fval,lastPopulation]=process(lb,ub)
    lastPopulation = [];
	nvars = length(ub);
	options = optimoptions('ga', 'OutputFcn', @saveLastPopulation, 'PopulationSize', 1000,'MaxGenerations', 100);
	[x_optimal, fval] = ga(@system, nvars, [], [], [], [], lb, ub, [], options);

	function [state, options, optchanged] = saveLastPopulation(options, state, ~)
        lastPopulation = state.Population;
        optchanged = false;
    end
end

function selected=selectBest(population)
	selected = [];
	pebble = SmartPebble;
	for i=1:length(population(:,1))
		[Ip,~,rhop] = pebble_inertia(population(i,:));
		[I,~,~] = ellipsoid_inertia(pebble.a,pebble.b,pebble.c,pebble.rho_desired);
		errI(i) = trace(I-Ip)^2;
		errRho(i) = rhop;
	end
	plot(errI,errRho,'b.','ButtonDownFcn', @(src, event) showIndex(src, event));
	uiwait(gcf);
	function showIndex(src, event)
    	pos = event.IntersectionPoint(1:2);
    	xdata = src.XData;
    	ydata = src.YData;
    	
    	tavolsagok = (xdata - pos(1)).^2 + (ydata - pos(2)).^2;
    	[~, idx] = min(tavolsagok);
    	
    	selected = idx;
    	uiresume(gcf);
	end
end