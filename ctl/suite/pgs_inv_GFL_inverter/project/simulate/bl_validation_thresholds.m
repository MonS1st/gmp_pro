function thresholds = bl_validation_thresholds()
%BL_VALIDATION_THRESHOLDS Central acceptance gates for the GFL SIL matrix.
%
% Values are per-unit unless stated otherwise.  They are commissioning
% thresholds for this PC plant, not grid-code or hardware certification
% limits.

    thresholds = struct();
    thresholds.TailFraction = 0.20;
    thresholds.PWMCompareMax = 3000;
    thresholds.PWMDisabledMeanMax = 0.01;
    thresholds.PWMBoundaryFractionMax = 0.02;
    thresholds.PLLFrequencyErrorMax = 0.02;
    thresholds.PLLVqAbsMeanMax = 0.02;
    thresholds.CurrentPeakMax = 2.0;
    thresholds.ThreeWireResidualRmsMax = 0.02;
    thresholds.CurrentTrackingRmsMax = 0.08;
    thresholds.NegativeSequenceImprovementMin = 0.10;
    thresholds.PowerPErrorMax = 0.005;
    thresholds.PowerQErrorMax = 0.003;
    thresholds.DivergenceCurrentPeak = 10.0;
end
