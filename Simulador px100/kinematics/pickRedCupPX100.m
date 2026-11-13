function pickRedCupPX100(robot, cfg, cup)
    grasp = cup.graspTargetBase;
    approach = grasp + [0 0 cfg.approachClearance];

    fprintf('\nCopo (Visão) [X, Y, Z]: [%.4f, %.4f, %.4f] m\n', grasp);

    fingerPath = minJerkPath(approach, grasp, cfg.descentSegments);
    jointPath  = solveJointPath(robot, cfg, fingerPath);

    % Aproximação em espaço das juntas
    moveJointsLinear(robot, robot.arm.get_joints(), jointPath(1,:), ...
        cfg.approachSegments, cfg.stepsPerApproachPoint);
    waitUntilReached(robot, cfg, fingerPath(1,:), 'aproximacao');

    % Descida«
    followPath(robot, cfg, jointPath(2:end,:), fingerPath(2:end,:), 'descida');
    printGraspValidation(robot, cfg, grasp);

    robot.arm.close_hand();
    stepSim(robot.sim, cfg.stepsGripper);

    % Subida«
    followPath(robot, cfg, flipud(jointPath(1:end-1,:)), flipud(fingerPath(1:end-1,:)), 'subida');
end

function path = minJerkPath(p0, pf, segments)
    tau  = linspace(0, 1, segments+1)';
    s    = 10*tau.^3 - 15*tau.^4 + 6*tau.^5;
    path = p0 + s .* (pf - p0);
end

function jointPath = solveJointPath(robot, cfg, fingerPath)
    n = size(fingerPath, 1);
    jointPath = zeros(n, 4);

    for i = 1:n
        finger = fingerPath(i,:);
        waist  = atan2(finger(2), finger(1));
        wrist  = finger - cfg.wristToFingerCenter * toolDirection(waist, cfg.toolPitch);

        [jointPath(i,:), ikInfo] = inverseKinematicsPX100(wrist, ...
            robot.links, robot.minQ, robot.maxQ, cfg.toolPitch, [], cfg.elbow);

        if ikInfo.positionError > cfg.maxIkError
            error('IK falhou no ponto %d da trajetoria.', i);
        end
    end
end

function d = toolDirection(waist, pitch)
    d = [cos(pitch)*cos(waist), cos(pitch)*sin(waist), -sin(pitch)];
end

function p = fingerCenterFK(robot, cfg)
    q     = robot.arm.get_joints();
    wrist = directKinematicsPX100(q, robot.links);
    pitch = sum(q(2:4));
    p     = wrist + cfg.wristToFingerCenter * toolDirection(q(1), pitch);
end

function moveJointsLinear(robot, qStart, qEnd, segments, stepsPerPoint)
    t = linspace(0, 1, segments+1)';
    for i = 2:numel(t)
        setJoints(robot, qStart + t(i)*(qEnd - qStart));
        stepSim(robot.sim, stepsPerPoint);
    end
end

function followPath(robot, cfg, jointPath, fingerPath, label)
    for i = 1:size(jointPath, 1)
        setJoints(robot, jointPath(i,:));
        stepSim(robot.sim, cfg.stepsPerDescentPoint);
        waitUntilReached(robot, cfg, fingerPath(i,:), sprintf('%s (%d)', label, i));
    end
end

function waitUntilReached(robot, cfg, target, label)
    for k = 1:cfg.maxWaitSteps
        err = norm(fingerCenterFK(robot, cfg) - target);
        if err <= cfg.positionTolerance
            return
        end
        robot.sim.step_simulation();
    end
    error('Timeout em %s (erro %.1f mm).', label, err*1000);
end

function printGraspValidation(robot, cfg, grasp)
    q       = robot.arm.get_joints();
    p_wrist = directKinematicsPX100(q, robot.links);
    p_fk    = fingerCenterFK(robot, cfg);
    err     = norm(p_fk - grasp);

    fprintf('Juntas Lidas (get_joints) [deg]: [%.2f°, %.2f°, %.2f°, %.2f°]\n', rad2deg(q));
    fprintf('Posição Pulso (directKinematicsPX100) [X, Y, Z]: [%.4f, %.4f, %.4f] m\n', p_wrist);
    fprintf('Posição Atingida pela FK [X, Y, Z]: [%.4f, %.4f, %.4f] m\n', p_fk);
    fprintf('Erro de Posicionamento Final: %.2f mm (%.5f m)\n\n', err*1000, err);
end

function setJoints(robot, q)
    if robot.arm.set_joints(q) ~= 0
        error('Configuracao articular rejeitada.');
    end
end

function stepSim(sim, steps)
    for k = 1:steps
        sim.step_simulation();
    end
end