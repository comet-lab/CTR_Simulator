close all; clc; clear;
% addpath util/

%% CTR Simulation (Preparation)
kappa = [1/30e-3; 1/40e-3;1/15e-3]; % tube curvature

phi = [0;deg2rad(90);deg2rad(120)]; % desired rotation
ell = [50e-3;70e-3;25e-3];  % desired translation
pts_per_seg = 8; % used for each segmented tubes

phi0 = [0; 0; 0];   % initial motor parameters
ell0 = [0; 0; 0];

g = robotindependentmapping(kappa,phi,ell,pts_per_seg);
g0 = robotindependentmapping(kappa, phi0, ell0, pts_per_seg);

phi1 = [deg2rad(0); deg2rad(00); deg2rad(0)];
ell1 = [2e-3; 8e-3; 3e-3];
g1 = robotindependentmapping(kappa, phi1, ell1, pts_per_seg);

% simulator(kappa, g0, phi0, phi, ell0, ell)
% simulator(kappa, g, phi, phi0, ell, ell0)
% 
% draw_ctcr(g0,[5 10 15],[2e-3 1.5e-3 1e-3])
% draw_ctcr(g,[5 10 15],[2e-3 1.5e-3 1e-3])
% draw_ctcr(g1,[5 10 15],[2e-3 1.5e-3 1e-3])


%% Simulation Space Init

fig = figure(1); 
set(fig, 'Position',[0 0 1280 1024]);
grid on; hold on;

plot3(0.0547, 0.0537, 0.03968, '*', 'MarkerSize', 10, 'Color', 'red')

r = [2e-3 1.5e-3 1e-3];  % tube radius
seg = [8 16 24]; % tube segment points
start_pos = [0.0547, 0.0537, 0.03968];

draw_ctcr(g0, seg, r)
xlim([-0.1 0.1]);  %
ylim([-0.1 0.1]);
zlim([-0.1 0.1]);
set(gca, 'XDir', 'reverse');  % Reverses the X-axis
set(gca, 'YDir', 'reverse');  % Reverses the Y-axis
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')

% Use a 3D view
view(3);


%% Command the Robot

simulator(g0, g, seg, r, start_pos)
simulator(g, g1, seg, r, start_pos)


%% Simulator functions version 1


function simulator(g0, g1, tube_seg, radius, start_pt)
    numFrames = 50;

    
    
    for t = linspace(0, 1, numFrames)
        tic;
        clf;
        plot3(start_pt(1), start_pt(2), start_pt(3), '*', 'MarkerSize', 10, 'Color', 'red')
        current_g = (1 - t) .* g0 + t .* g1;
        draw_ctcr(current_g, tube_seg, radius)
        % drawnow;
        pause(0.1);
        toc;
    end



end