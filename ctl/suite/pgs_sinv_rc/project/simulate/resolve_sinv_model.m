function model = resolve_sinv_model(root, base_model)
%RESOLVE_SINV_MODEL Select a Simulink model compatible with this MATLAB.
%
% R2024a and earlier use the models exported for R2022b.
% R2024b and later use the original models.

arguments
    root (1,:) char
    base_model (1,:) char
end

% MATLAB R2024a is version 24.1; R2024b is version 24.2.
if verLessThan('matlab', '24.2')
    model = [base_model '_2022b'];
else
    model = base_model;
end

model_file = fullfile(root, [model '.slx']);

if ~isfile(model_file)
    error('SINV:CompatibleModelMissing', ...
        'Compatible SINV model was not found: %s', model_file);
end
end