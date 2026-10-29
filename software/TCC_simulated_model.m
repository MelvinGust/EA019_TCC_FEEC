%% Setup State Space Variables

% Determine A
gamma_mb = 0.095;
gamma_bo = 0.095;
gamma_ob = [0.037, 0.04];
gamma_bm = 0.03;
mu_m = [0.1, 0.2];
mu_o = [0.1, 0.15];
mu_b = 0.2;
A_list = {};
it_A = 1;
for it_g = gamma_ob
    for it_m= mu_m
        for it_o = mu_o
            A_temp = [1-gamma_mb-it_m, gamma_bm, 0;
                 gamma_mb, 1-gamma_bo-gamma_bm-mu_b, it_g;
                 0, gamma_bo, 1-it_g-it_o];
            A_list{it_A} = A_temp;
            it_A = it_A + 1;
        end
    end
end

% Determine B
D_m = 0.01;
B = [D_m;
     0;
     0];

% Determine C
c_m = 0;
c_b = 0;
c_o = 1;
C = [c_m, c_b, c_o];

% Determine D
D = [0;
    0;
    0];

%% Closed Loop Variables
r_val = 10;
Ki = 2.2578;
Kp = -16.6825;


figure(1);
clf;
hold on;
grid on;
box on;
colors = lines(numel(A_list)); % Setup colors
vertex_params = zeros(numel(A_list), 3); % Save param combinations

it_vertex = 1;
for it_g = gamma_ob
    for it_m = mu_m
        for it_o = mu_o
            vertex_params(it_vertex,:) = [it_g, it_m, it_o];
            it_vertex = it_vertex + 1;
        end
    end
end

%% Start simulation
% Run our SIMULINK model
for i = 1:numel(A_list)

    A = A_list{i};

    sim('simulated_model');

    time_data = y_out.Time;
    y_data = y_out.Data;

    g = vertex_params(i,1);
    m = vertex_params(i,2);
    o = vertex_params(i,3);

    label = sprintf(['V%d: \\gamma_{ob}=%.3f, ' ...
                     '\\mu_m=%.2f, \\mu_o=%.2f'], ...
                     i, g, m, o);

    plot(time_data, y_data, ...
        'Color', colors(i,:), ...
        'LineWidth', 1.5, ...
        'DisplayName', label);
end
% Draw our reference via "plot" since apparently my matlab version doesnt
% have yline.
plot([time_data(1), time_data(end)], ...
     [r_val, r_val], 'k--', ...
     'LineWidth', 1.5, ...
     'DisplayName', 'Referência');
 
 
%% Figure 1
xlabel('Tempo [s]', 'FontSize', 12);
ylabel('Saída y(k)', 'FontSize', 12);
title('Respostas em malha fechada para os 8 vértices', ...
      'FontSize', 13);      
lgd = legend('show');
set(lgd, 'Location', 'northeastoutside', ...
         'Interpreter', 'tex', ...
         'FontSize', 9);
ylim([0 r_val + 2]);

% Adds the rectangle to the graph
zoom_xlim = [0.8 2.5];
zoom_ylim = [7 10.2];
rectangle('Position', ...
    [zoom_xlim(1), zoom_ylim(1), ...
     diff(zoom_xlim), diff(zoom_ylim)], ...
    'EdgeColor', [0.5 0.5 0.5], ...
    'LineWidth', 1);
set(gca, 'FontSize', 11);
set(gcf, 'Color', 'w');
hold off;

%% Figure 1 : Zoom

ax_original = gca;
ax_zoom = axes('Position', [0.40     0.2 0.3 0.4]);
curves = findobj(ax_original, 'Type', 'line');
copyobj(curves, ax_zoom);

% Apply zoom limits
xlim(ax_zoom, zoom_xlim);
ylim(ax_zoom, zoom_ylim);

grid(ax_zoom, 'on');
box(ax_zoom, 'on');
set(ax_zoom, 'FontSize', 8);
set(ax_zoom, 'Color', 'w');


% Export our figure to get a high quality image

set(gcf, 'Color', 'w');
set(gcf, 'PaperPositionMode', 'auto');
print(gcf, 'tcc_simulink_respostas', '-dpdf', '-painters');