%   simulator_interface.m
%   Interface to simulator in CoppeliaSim simulator
%   2026/09/16
%   % Copyright (C) 2026
%   Author: Luís Louro, luislouro@algoritmi.uminho.pt
%           Estela Bicho, estela.bicho@dei.uminho.pt

classdef simulator_interface < handle

    properties (Access = private)
        client
        sim
        simVision
        TargetHandle        % handle for the target object
        TargetNames         % list of targets
        CameraHandle        % handler for camera

        VISION_WORKING
    end

    properties
        TARGET_Number       % Number of targets
    end

    methods
        % Create a connection handler to the Simulation scene
        %Start handles for objects
        function [obj,error] = simulator_interface(VISIONWORKING)
            error = 0;
            %Start Communication
            obj.client = RemoteAPIClient();
            obj.sim = obj.client.require('sim');
            obj.simVision = obj.client.require('simVision');

            obj.VISION_WORKING = VISIONWORKING;

            % Two different targets
            obj.TargetNames{1} = '/TargetGreen';
            obj.TargetNames{2} = '/BlueCube';

            [~,obj.TARGET_Number]=size(obj.TargetNames);

            %Get Targets Handle
            for a=1:obj.TARGET_Number
                obj.TargetHandle{a} = obj.sim.getObject(obj.TargetNames{a});
                if(obj.TargetHandle{a}==-1)
                    disp('ERROR: Failed getting target handle');
                    error = 1;
                    return;
                end
            end

            %Get Vision Handle
            if obj.VISION_WORKING==1
                obj.CameraHandle = obj.sim.getObject('/Vision_sensor');
                if (obj.CameraHandle==-1)
                    disp('ERROR: Failed getting camera handle');
                    error = 1;
                    return;
                end
            end
        end

        %Function that allows the simulation to be started in CoppeliaSim
        function start_simulation(obj)
            obj.sim.setStepping(true);
            obj.sim.startSimulation();
        end

        %Function that triggers the next simulation step
        function step_simulation(obj)
            obj.sim.step();
        end
        
        function client = get_connection(obj)
            client = obj.client;
        end

        %Function that allows you to get the position of the target
        %Input:
        %iTarget - target index to know position
        %Output:
        %TargetPosition - array with the target location (x, y, z)
        function targetPosition=get_target_position(obj,iTarget)
            targetPosition_cell = obj.sim.getObjectPosition(obj.TargetHandle{iTarget});
            targetPosition(1)=targetPosition_cell{1};      %m
            targetPosition(2)=targetPosition_cell{2};      %m
            targetPosition(3)=targetPosition_cell{3};      %m
        end

        %Function that allows you to get information from vision system
        % Output:
        % image - matrix with image information
        % resolution - resolution information 
        function [image,resolution] = get_camera_information(obj)
            if obj.VISION_WORKING==1
                [ imageData, res] = obj.sim.getVisionSensorImg(obj.CameraHandle);
                width = res{1};
                height = res{2};
                image = flip(permute(reshape(imageData, 3, width, height), [3 2 1]), 1);  %Convert BGR->RGB
                resolution = [width height];
            end
        end

        function [M, angGraus, W, H] = get_camera_parameters(obj)
            m = obj.sim.getObjectMatrix(obj.CameraHandle, obj.sim.handle_world);
            if iscell(m), m = cell2mat(m); end
            M = reshape(m, 4, 3)';
            ang = obj.sim.getObjectFloatParam(obj.CameraHandle, obj.sim.visionfloatparam_perspective_angle);
            if iscell(ang), ang = ang{end}; end
            angGraus = rad2deg(ang);
            [~, res] = obj.sim.getVisionSensorImg(obj.CameraHandle);
            W = res{1};
            H = res{2};
        end

        function position = get_object_position(obj, nome)
            handle = obj.sim.getObject(nome);
            p = obj.sim.getObjectPosition(handle, obj.sim.handle_world);
            position = [p{1}, p{2}, p{3}];
        end

        %Function that provides information on the elapsed simulation time.
        %Output
        %t - simulation time
        function t = get_simulation_time(obj)
            t = obj.sim.getSimulationTime();
        end

        %Function that ends the class
        function terminate(obj)
            obj.sim.stopSimulation();
        end
    end
end
