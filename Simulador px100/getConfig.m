function cfg = getConfig()
    cfg.cupHeight             = 0.070;      % m
    cfg.wristToFingerCenter   = 0.100;      % m
    cfg.toolPitch             = deg2rad(0); % garra horizontal
    cfg.elbow                 = 'up';
    cfg.approachClearance     = 0.080;      % m acima do alvo

    cfg.descentSegments       = 24;
    cfg.approachSegments      = 30;

    cfg.positionTolerance     = 0.010;      % m
    cfg.maxIkError            = 1e-5;       % m
    cfg.maxWaitSteps          = 500;

    cfg.stepsPerApproachPoint = 2;
    cfg.stepsPerDescentPoint  = 4;
    cfg.stepsGripper          = 60;
end