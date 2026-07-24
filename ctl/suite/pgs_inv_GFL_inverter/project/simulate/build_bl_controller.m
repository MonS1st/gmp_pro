function build = build_bl_controller(buildLevel, outputDir, varargin)
%BUILD_BL_CONTROLLER Build one isolated PC SIL controller variant.
%
% BUILD_LEVEL and the validation feature flags are passed to MSBuild without
% modifying generated SDPE headers.  Every variant receives an independent
% OutDir/IntDir and a configuration snapshot.

    p = inputParser;
    addRequired(p, 'buildLevel', @(x) isnumeric(x) && isscalar(x) && x == fix(x) && x >= 0 && x <= 6);
    addRequired(p, 'outputDir', @(x) ischar(x) || isstring(x));
    addParameter(p, 'NegSequenceEnabled', buildLevel >= 3 && buildLevel <= 5, @is_logical_scalar);
    addParameter(p, 'ExternalFeedforwardEnabled', buildLevel >= 4 && buildLevel <= 5, @is_logical_scalar);
    addParameter(p, 'DecouplingEnabled', buildLevel >= 4 && buildLevel <= 5, @is_logical_scalar);
    addParameter(p, 'ActiveDampingEnabled', buildLevel >= 4 && buildLevel <= 5, @is_logical_scalar);
    addParameter(p, 'LeadCompensatorEnabled', buildLevel >= 4 && buildLevel <= 5, @is_logical_scalar);
    addParameter(p, 'PQProfileEnabled', false, @is_logical_scalar);
    addParameter(p, 'MSBuild', '', @(x) ischar(x) || isstring(x));
    parse(p, buildLevel, outputDir, varargin{:});
    cfg = p.Results;

    thisDir = fileparts(mfilename('fullpath'));
    repoRoot = fileparts(fileparts(fileparts(fileparts(fileparts(thisDir)))));
    projectFile = fullfile(thisDir, 'DigialPower_simulink.vcxproj');
    outputDir = char(string(outputDir));
    if ~isfolder(outputDir)
        mkdir(outputDir);
    end
    objectDir = fullfile(outputDir, 'obj');
    if ~isfolder(objectDir)
        mkdir(objectDir);
    end

    msbuild = char(string(cfg.MSBuild));
    if isempty(msbuild)
        candidates = { ...
            'C:\Program Files\Microsoft Visual Studio\18\Community\MSBuild\Current\Bin\amd64\MSBuild.exe', ...
            'C:\Program Files\Microsoft Visual Studio\2022\Community\MSBuild\Current\Bin\amd64\MSBuild.exe'};
        for i = 1:numel(candidates)
            if isfile(candidates{i})
                msbuild = candidates{i};
                break
            end
        end
    end
    assert(isfile(msbuild), 'MSBuild executable was not found: %s', msbuild);
    assert(isfile(projectFile), 'Visual Studio project was not found: %s', projectFile);

    flags = feature_flags(cfg);
    defineList = sprintf([ ...
        'GMP_GFL_ENABLE_NEGATIVE_SEQUENCE=%d;' ...
        'GMP_GFL_ENABLE_EXTERNAL_FEEDFORWARD=%d;' ...
        'GMP_GFL_ENABLE_DECOUPLING=%d;' ...
        'GMP_GFL_ENABLE_ACTIVE_DAMPING=%d;' ...
        'GMP_GFL_ENABLE_LEAD_COMPENSATOR=%d;' ...
        'GMP_GFL_PQ_PROFILE=%d'], ...
        flags.NegSequenceEnabled, flags.ExternalFeedforwardEnabled, ...
        flags.DecouplingEnabled, flags.ActiveDampingEnabled, ...
        flags.LeadCompensatorEnabled, flags.PQProfileEnabled);

    setenv('GMP_PRO_LOCATION', repoRoot);
    outputArg = strrep(outputDir, '\', '/');
    objectArg = strrep(objectDir, '\', '/');
    dependencyInclude = fullfile(thisDir, 'vcpkg_installed', ...
        'x64-windows', 'x64-windows', 'include');
    assert(isfolder(dependencyInclude), ...
        'SIL dependency include directory is missing: %s', dependencyInclude);
    dependencyArg = strrep(dependencyInclude, '\', '/');
    command = sprintf([ ...
        '"%s" "%s" /t:Build /m:1 /nologo ' ...
        '/p:Configuration=Debug /p:Platform=x64 ' ...
        '/p:VcpkgEnableManifest=false /p:GflBuildLevel=%d ' ...
        '/p:GflFeatureFlags="%s" /p:GflDependencyInclude="%s" ' ...
        '/p:OutDir="%s/" /p:IntDir="%s/"'], ...
        msbuild, projectFile, buildLevel, defineList, dependencyArg, outputArg, objectArg);

    startedAt = datetime('now', 'TimeZone', 'local');
    [status, text] = system(command);
    finishedAt = datetime('now', 'TimeZone', 'local');
    logFile = fullfile(outputDir, 'build.log');
    write_text(logFile, text);

    executable = fullfile(outputDir, 'Digital_Power_simulink.exe');
    build = struct();
    build.BuildLevel = double(buildLevel);
    build.Status = status;
    build.Succeeded = status == 0 && isfile(executable);
    build.ProjectFile = projectFile;
    build.OutputDirectory = outputDir;
    build.Executable = executable;
    build.LogFile = logFile;
    build.Command = command;
    build.StartedAt = char(startedAt);
    build.FinishedAt = char(finishedAt);
    build.FeatureFlags = flags;
    build.CompilerDefinitions = defineList;
    write_text(fullfile(outputDir, 'build_config.json'), jsonencode(build, PrettyPrint=true));

    if ~build.Succeeded
        error('build_bl_controller:BuildFailed', ...
            'BL%d build failed (exit %d). See %s', buildLevel, status, logFile);
    end
end

function flags = feature_flags(cfg)
    flags = struct( ...
        'NegSequenceEnabled', logical(cfg.NegSequenceEnabled), ...
        'ExternalFeedforwardEnabled', logical(cfg.ExternalFeedforwardEnabled), ...
        'DecouplingEnabled', logical(cfg.DecouplingEnabled), ...
        'ActiveDampingEnabled', logical(cfg.ActiveDampingEnabled), ...
        'LeadCompensatorEnabled', logical(cfg.LeadCompensatorEnabled), ...
        'PQProfileEnabled', logical(cfg.PQProfileEnabled));
end

function tf = is_logical_scalar(value)
    tf = (islogical(value) || isnumeric(value)) && isscalar(value) && isfinite(double(value));
end

function write_text(fileName, value)
    fid = fopen(fileName, 'w', 'n', 'UTF-8');
    assert(fid >= 0, 'Cannot write %s', fileName);
    cleanup = onCleanup(@() fclose(fid));
    fwrite(fid, value, 'char');
    clear cleanup
end
