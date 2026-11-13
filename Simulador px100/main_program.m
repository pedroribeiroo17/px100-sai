%   main_program.m
%   Main program for the control of manipulators
%   Author: Luís Louro, luislouro@algoritmi.uminho.pt
%           Estela Bicho, estela.bicho@dei.uminho.pt
%   Copyright (C) 2026
%   2026/09/16
%--------------------------------------------------------------------------
% Before running this script, open the scenario in CoppeliaSim, e.g
% Do not run simulation!
%--------------------------------------------------------------------------

function main()
    clear
    robot_name = '/px100';
    ROSWORKING = 0;
    VISIONWORKING = 1;

    TARGET_GREEN = 1;
    BLUE_CUBE = 2;

    cfg = getConfig();
    addProjectPaths();

    [sim_interface, error_sim] = simulator_interface(VISIONWORKING);
    if error_sim == 1
        return;
    end

    cleanup = onCleanup(@()myCleanupFun(sim_interface));

    [robot_arm, error_man] = arm_interface(sim_interface, robot_name, ROSWORKING);
    if error_man == 1
        sim_interface.terminate();
        return;
    end

    sim_interface.start_simulation();
    sim_interface.step_simulation();

    [~, Links, MinPositionJoint, MaxPositionJoint] = robot_arm.get_RobotCharacteristics();
    robot = struct('arm', robot_arm, 'sim', sim_interface, 'links', Links, ...
        'minQ', MinPositionJoint, 'maxQ', MaxPositionJoint);

    if robot_arm.set_joints(zeros(1, 4)) == 1
        return;
    end

    robot_arm.open_hand();
    for k = 1:cfg.stepsGripper
        sim_interface.step_simulation();
    end

    start = tic;
    told = -1;
    i = 1;
    stop = 0;

    %----------------------------------------------------------------------
    % YOUR CODE
    %----------------------------------------------------------------------

    objectInfo = runVisionCupsPX100(sim_interface, true, cfg.cupHeight);
    cup = findRedCup(objectInfo);

    pickRedCupPX100(robot, cfg, cup);

    stop = 1;

    %%------------- END OF YOUR CODE -------------

    while stop == 0
        ReadArmJoints = robot_arm.get_joints();
        ReadArmJointsDegree = ReadArmJoints * 180/pi;
        armPosition = robot_arm.get_robot_position();

        if VISIONWORKING == 1
            [image, resolution] = sim_interface.get_camera_information();
            imshow(image)
            drawnow;
        end

        objectPosition = sim_interface.get_target_position(BLUE_CUBE);
        targetPosition = sim_interface.get_target_position(TARGET_GREEN);

        sim_interface.step_simulation();
        time = toc(start);
        if time < 0.05
            pause(0.05 - time);
        end
        start = tic;
        t = sim_interface.get_simulation_time();
        if t == told
            sim_interface.terminate();
            disp("Simulation stopped");
            return
        end
        told = t;
        fprintf('Simulation time: %.2f [s]\n', t);

        i = i + 1;
    end

    sim_interface.terminate();
    disp('stop simulation')
end

function myCleanupFun(sim_interface)
    disp('stop simulation')
    sim_interface.terminate();
end

function addProjectPaths()
    root = fileparts(mfilename('fullpath'));
    addpath(root, fullfile(root, 'vision'), fullfile(root, 'kinematics'));
end

function cup = findRedCup(objectInfo)
if isempty(objectInfo.cups)
    error('Nao foram detetados copos com rebordo preto.');
end
idx = find(strcmp({objectInfo.cups.color}, 'red'), 1);
if isempty(idx)
    error('O copo vermelho nao foi detetado.');
end
cup = objectInfo.cups(idx);
end