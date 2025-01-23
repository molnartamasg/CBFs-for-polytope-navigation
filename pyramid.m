%% Simulation of safety-critical navigation for obstacle avoidance
clear; clc;

%% Problem setup
% Setup and parameters
% agent geometry
par.dp=[0.25;0.25;0.25].*...    % vertices of agent relative to center
       [-1, 1,-1, 1,-1, 1,-1,1;
        -1,-1, 1, 1,-1,-1, 1,1;
        -1,-1,-1,-1, 1, 1, 1,1];
par.P=[7,1,6,5,3,3;             % matrix indicating which vertices form faces
       8,3,8,6,4,4;
       6,7,4,2,8,2;
       5,5,2,1,7,1];
% obstacle geometry
par.om=0;                       % angular velocity of obstacle  % 0, 0.2
par.wc=[4;4;4];                 % center of obstacle
par.dw=[1.5;1;1].*...           % vertices of obstacle relative to center
       [-2, 2,-2, 2,-1, 1,-1,1;
        -2,-2, 2, 2,-1,-1, 1,1;
        -1,-1,-1,-1, 1, 1, 1,1];
par.W=[7,1,6,5,3,3;             % matrix indicating which vertices form faces
       8,3,8,6,4,4;             % vertices ordered in sequence along the edges for plotting
       6,7,4,2,8,2;             % order affects which side of the face the normal vector points to
       5,5,2,1,7,1];
par.dw1=par.dw(:,par.W(1,:));   % one vertex of each face
par.n=normals(par.dw,par.W);    % normal vectors of obstacle (as row vectors)
par.A=[eye(5);                  % adjacency matrix showing which boundaries
       0,ones(1,4)];            % of the obstacle form convex sets (at convex corners)
                                % Aij=1 if boundary i is a member of convex set j
                                % Aij=0 otherwise

% desired controller
par.pgoal=[7;1;3];  % goal point, m
par.Kp=1;           % gain of desired controller
par.vmax=1;         % saturation limit of desired controller, m/s
% safety
par.filter='on';	% safety filter 'on', 'off'
par.gamma=2;        % CBF parameter, 1/s
par.kappa=5;        % smoothing parameter
par.b=0;            % buffer

% Simulation settings
t0=0;               % start time, s
tend=20;            % end time, s
dt=0.01;            % time step, s
t=t0:dt:tend;       % time, s

% Initial conditions
x0=[1;7;4];

% Plot settings (ranges, labels, colors)
It=[t0,tend];	Lt='time, $t$ (s)';
Ix1=[0,8];      Lx1='position, $x_1$ (m)';
Ix2=[0,8];      Lx2='position, $x_2$ (m)';
Ix3=[0,8];      Lx3='position, $x_3$ (m)';
Ih=[-1,3];      Lh='CBF, $h$ (m)';
Iu1=[-1,1];     Lu1='input, $u_1$ (m/s)';
Iu2=[-1,1];     Lu2='input, $u_2$ (m/s)';
Iu3=[-1,1];     Lu3='input, $u_3$ (m/s)';
purple=[170,0,170]/256;
darkgreen=[0,170,0]/256;

% Animation settings
animate=1;          % whether animation is done
Nt=10;              % one out of Nt frames is plotted

%% Simulation
% Simulate system
sol=ode45(@(t,x)rhs(t,x,par),[t0,tend],x0,odeset('RelTol',1e-6));
x=deval(sol,t);

% Evaluate solution
u=zeros(size(x));
ud=zeros(size(u));
h=zeros(size(t));
for kt=1:length(t)
    [u(:,kt),ud(:,kt),h(kt)]=k(t(kt),x(:,kt),par);
end

%% Plot results
figure(1); clf;

% Trajectory
subplot(2,4,[1,2,5,6]); hold on; box on;
plot3(x0(1),x0(2),x0(3),'b.','Markersize',20,'DisplayName','Start');    % plot start and goal
plot3(par.pgoal(1),par.pgoal(2),par.pgoal(3),'g.','Markersize',20,'DisplayName','Goal');
kt=(t==0);                                                              % select given time moment
w=obstacle(t(kt),par.dw,par);                                           % plot obstacle
W=[par.W;par.W(1,:)];
w1=reshape(w(1,W),size(W)); w2=reshape(w(2,W),size(W)); w3=reshape(w(3,W),size(W));
plot3(w1,w2,w3,'k','LineWidth',2,'HandleVisibility','off');
p=agent(x(:,kt),par);                                                   % plot agent
P=[par.P;par.P(1,:)];
p1=reshape(p(1,P),size(P)); p2=reshape(p(2,P),size(P)); p3=reshape(p(3,P),size(P));
plot3(p1,p2,p3,'k','LineWidth',2,'HandleVisibility','off');
plot3(x(1,:),x(2,:),x(3,:),'b','LineWidth',2,'HandleVisibility','off'); % plot trajectory
PlotFinalize({Lx1,Lx2,Lx3},[Ix1,Ix2,Ix3]);
view([-30,30]);

% CBF vs time
subplot(2,4,3); hold on; box on;
plot([t0,tend],[0,0],'k','LineWidth',1,'HandleVisibility','off');
plot(t,h,'r','LineWidth',2,'HandleVisibility','off');
PlotFinalize({Lt,Lh},[It,Ih]);

% Horizontal velocity component vs time
subplot(2,4,4); hold on; box on;
plot([t0,tend],[0,0],'k','LineWidth',1,'HandleVisibility','off');
plot(t,ud(1,:),'k','LineWidth',2,'DisplayName','desired');
plot(t,u(1,:),'g','LineWidth',2,'DisplayName','safe');
PlotFinalize({Lt,Lu1},[It,Iu1]);

% Vertical velocity component vs time
subplot(2,4,7); hold on; box on;
plot([t0,tend],[0,0],'k','LineWidth',1,'HandleVisibility','off');
plot(t,ud(2,:),'k','LineWidth',2,'DisplayName','desired');
plot(t,u(2,:),'g','LineWidth',2,'LineWidth',2,'DisplayName','safe');
PlotFinalize({Lt,Lu2},[It,Iu2]);

% Normal velocity component vs time
subplot(2,4,8); hold on; box on;
plot([t0,tend],[0,0],'k','LineWidth',1,'HandleVisibility','off');
plot(t,ud(3,:),'k','LineWidth',2,'DisplayName','desired');
plot(t,u(3,:),'g','LineWidth',2,'LineWidth',2,'DisplayName','safe');
PlotFinalize({Lt,Lu3},[It,Iu3]);

%% Animate motion
if animate

% % Create a video file for animation
% videoname='revolving_door';
% v=VideoWriter(videoname,'MPEG-4');
% v.Quality=100;
% open(v);

% Make the animation
figure(2); clf;
for kt=1:Nt:length(t)
    % vertices of agent
    p=agent(x(:,kt),par);
    P=[par.P;par.P(1,:)];
    p1=reshape(p(1,P),size(P)); p2=reshape(p(2,P),size(P)); p3=reshape(p(3,P),size(P));
    
    % vertices of obstacle
    w=obstacle(t(kt),par.dw,par);
    W=[par.W;par.W(1,:)];
    w1=reshape(w(1,W),size(W)); w2=reshape(w(2,W),size(W)); w3=reshape(w(3,W),size(W));
    
    tic;
    drawnow;
    % plot start and goal
    plot3(x0(1),x0(2),x0(3),'.','Color','b','Markersize',20,'DisplayName','Start');
    hold on;
    plot3(par.pgoal(1),par.pgoal(2),par.pgoal(3),'.','Color',darkgreen,'Markersize',20,'DisplayName','Goal');
   
    % plot obstacle
    plot3(w1,w2,w3,'k','LineWidth',2,'HandleVisibility','off');

    % plot agent
    plot3(p1,p2,p3,'k','LineWidth',2,'HandleVisibility','off');
    plot3(x(1,1:kt),x(2,1:kt),x(3,1:kt),'Color',purple,'LineWidth',2,'HandleVisibility','off');
    plot3(x(1,kt),x(2,kt),x(3,kt),'.','Color',purple,'Markersize',20,'HandleVisibility','off');
    hold off;
    PlotFinalize({Lx1,Lx2,Lx3},[Ix1,Ix2,Ix3]);

%     % Save the animation
%     frame=getframe(gcf);
%     for kframe=1:v.FrameRate/Nt
%         writeVideo(v,frame);
%     end

    T=toc;
    pause(Nt*dt-T); % time the plots according to the frame rate
end

% close(v);

end

%% Functions for geometry
% Rotation matrices and their derivatives
function [R,dR] = rotmat(angle)
    C=cos(angle);
    S=sin(angle);
    R=[C,-S,0;
       S, C,0;
       0, 0,1];
    dR=[-S,-C,0;
         C,-S,0;
         0, 0,0];
end

% Vertices of agent
function p = agent(x,par)
    pc=x;
    p=pc+par.dp;
end

% Vertices and normal vectors of rotated obstacle
function [w,dwdt,n,dndt] = obstacle(t,dw,par)
    % orientation of obstacle
    th=par.om*t;
    [R,dRdth]=rotmat(th);
    dRdt=dRdth*par.om;
    % vertices of obstacle
    w=par.wc+R*dw;
    dwdt=dRdt*dw;
    % normal vectors of obstacle
    n=par.n*R.';
    dndt=par.n*dRdt.';
end

% Normal vectors of obstacle based on its vertices
function n = normals(w,W)
    n=zeros(size(W,2),size(w,1));
    for k=1:size(W,2)
        % vectors connecting vertices
        dw1=w(:,W(1,k))-w(:,W(2,k));
        dw2=w(:,W(3,k))-w(:,W(2,k));
        % normal vectors pointing outside the obstacle
        nk=cross(dw1,dw2);
        % unit normal vectors organized as row vectors in a matrix
        n(k,:)=(nk./vecnorm(nk)).';
    end
end

%% Functions for dynamics
% System model
function [f,g] = sys(x,~)
    f = zeros(size(x,1),1);
    g = eye(size(x,1));
end

% Right-hand side
function dxdt = rhs(t,x,par)
    [f,g] = sys(x,par);
    u = k(t,x,par);
    dxdt = f+g*u;
end

%% Functions for control
% Desired controller (desired velocity)
function ud = kd(x,par)
    p = x;
    ud = par.Kp*(par.pgoal-p);
    if norm(ud)>par.vmax
        ud = ud/norm(ud)*par.vmax;
    end
end

% Smooth max-min function to combine barriers
function [h,dhdpsi] = smaxmin(psi,par)
    Psi=exp(-par.kappa*psi);            % take exponentials for smoothing corners
    sumPsi=sum(Psi);                    % combine vertices of agents
    Phi=1./(sumPsi*par.A);              % combine convex corners of obstacle
    sumPhi=sum(Phi);                    % combine convex sets
    h=(log(sumPhi)-par.b)/par.kappa;    % smoothen with log and buffer
    dh=(Phi.^2*par.A.').*Psi/sumPhi;    % calculate gradient
    dhdpsi=dh(:).';
end

% CBF evaluation
function [h,gradh,dhdt] = CBF(t,x,par)
    % vertices of agent
    p=agent(x,par);
    
    % vertices and normal vectors of obstacle
    [w,dwdt,n,dndt]=obstacle(t,par.dw1,par);
    
    % individual barriers
    psi=(n*p-dot(n,w.',2)).';
    dpsidt=(dndt*p-dot(dndt,w.',2)-dot(n,dwdt.',2)).';
    gradpsi=kron(n,ones(size(p,2),1));

    % combined barrier and its derivatives
    [h,dhdpsi]=smaxmin(psi,par);
    gradh=dhdpsi*gradpsi;
    dhdt=dhdpsi*dpsidt(:);
end

% Controller (safe velocity)
function [u,ud,h] = k(t,x,par)
    % desired controller
    ud = kd(x,par);
    % safety filter
    switch par.filter
        case 'off'
            h = CBF(t,x,par);
            u = ud;
        case 'on'
            [h,gradh,dhdt] = CBF(t,x,par);
            [f,g] = sys(x,par);
            Lfh = gradh*f;
            Lgh = gradh*g;
            u = ud + max(0,-dhdt-Lfh-Lgh*ud-par.gamma*h)*Lgh.'/(Lgh*Lgh.');
    end
end

%% Finalize plot with axis labels, limits, legends
function PlotFinalize(axislabels,axislimits)
    axis(axislimits);
    pbaspect([1,1,1]);
    xlabel(axislabels{1},'Interpreter','latex');
    ylabel(axislabels{2},'Interpreter','latex');
    if length(axislabels)>2
        zlabel(axislabels{3},'Interpreter','latex');
    end
    set(gca,'TickLabelInterpreter','latex','FontSize',14);
    legend('Location','northeast','Interpreter','latex','FontSize',14);
    if isempty(get(get(gca,'Legend'),'String'))
        legend off;
    end
end