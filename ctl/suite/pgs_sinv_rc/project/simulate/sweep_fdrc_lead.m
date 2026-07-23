function results = sweep_fdrc_lead()
%SWEEP_FDRC_LEAD Rebuild and validate BL5 at each FDRC delay candidate.
% FDRC lead is a C macro: assignin alone cannot change the SIL executable.

root = fileparts(mfilename('fullpath'));
suite = fileparts(fileparts(root));
gmp_root = fullfile(root, '..', '..', '..', '..', '..');
common_req = fullfile(suite, 'sdpe_general', 'sdpe_requirement.json');
common_gen = fullfile(suite, 'sdpe_general', 'sdpe_generate.bat');
platform_gen = fullfile(root, 'sdpe_mgr', 'sdpe_generate.bat');
solution = fullfile(root, 'GMP_Motor_Control_simulink.sln');
msbuild = 'C:\Program Files\Microsoft Visual Studio\18\Community\MSBuild\Current\Bin\MSBuild.exe';
model = 'PGS_STD_SINV_MODEL_Rectifier';
lead_steps = [2.0 2.5 3.0 3.5 4.0];
vin_rms = 43.0;

if ~isfile(msbuild)
    error('SINV:MSBuildMissing', 'MSBuild was not found: %s', msbuild);
end

setenv('GMP_PRO_LOCATION', gmp_root);
configure_sinv_models;
original = fileread(common_req);
results = table(lead_steps(:), nan(numel(lead_steps),1), nan(numel(lead_steps),1), ...
    false(numel(lead_steps),1), nan(numel(lead_steps),1), ...
    'VariableNames', {'lead_steps','thd_percent','vbus_ripple_percent','oscillation','vbus_mean_v'});

for k = 1:numel(lead_steps)
    lead = lead_steps(k);
    fprintf('\n=== FDRC lead %.1f samples ===\n', lead);
    update_lead_requirement(common_req, original, lead);
    run_checked(['call "' common_gen '"']);
    run_checked(['call "' platform_gen '"']);
    run_checked(['"' msbuild '" "' solution '" /m /t:Build /p:Configuration=Debug /p:Platform=x64 /v:minimal']);

    load_system(model);
    set_param([model '/Grid Voltage Command'], 'Amplitude', num2str(sqrt(2) * vin_rms, 17));
    metrics = run_sinv_validation(5, 5.0, sprintf('fdrc_lead_%0.1f', lead));
    results.thd_percent(k) = metrics.iac_thd_percent;
    results.vbus_ripple_percent(k) = metrics.vbus_ripple_percent;
    results.oscillation(k) = logical(metrics.oscillation_detected);
    results.vbus_mean_v(k) = metrics.vbus_mean_v;
end

valid = ~results.oscillation & isfinite(results.thd_percent);
if ~any(valid)
    error('SINV:NoStableCandidate', 'No stable FDRC lead candidate was found.');
end
candidates = find(valid);
[~, index] = min(results.thd_percent(valid));
best = candidates(index);
best_lead = results.lead_steps(best);
update_lead_requirement(common_req, original, best_lead);
run_checked(['call "' common_gen '"']);
run_checked(['call "' platform_gen '"']);
run_checked(['"' msbuild '" "' solution '" /m /t:Build /p:Configuration=Debug /p:Platform=x64 /v:minimal']);

out_dir = fullfile(root, 'validation');
if ~isfolder(out_dir), mkdir(out_dir); end
writetable(results, fullfile(out_dir, 'fdrc_lead_sweep.csv'));
disp(results);
fprintf('\nBest stable lead: %.1f samples, THD %.3f%%\n', best_lead, results.thd_percent(best));

figure('Color', 'w');
tiledlayout(2,1);
nexttile; plot(results.lead_steps, results.thd_percent, '-o', 'LineWidth', 1.2); grid on;
xlabel('SINV FDRC lead steps'); ylabel('Current THD (%)');
nexttile; stem(results.lead_steps, results.oscillation, 'filled'); grid on;
xlabel('SINV FDRC lead steps'); ylabel('Oscillation detected'); ylim([-0.1 1.1]);
exportgraphics(gcf, fullfile(out_dir, 'fdrc_lead_sweep.png'), 'Resolution', 160);
end

function run_checked(command)
[status, output] = system(command);
fprintf('%s', output);
if status ~= 0
    error('SINV:CommandFailed', 'Command failed (%d): %s', status, command);
end
end
