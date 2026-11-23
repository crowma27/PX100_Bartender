%   simulator_interface.m
%   Interface to simulator in CoppeliaSim simulator
%   2026/09/16
%   % Copyright (C) 2026
%   Author: Luís Louro, luislouro@algoritmi.uminho.pt
%           Estela Bicho, estela.bicho@dei.uminho.pt

classdef simulator_interface < handle  % Declara a classe simulator_interface.

    properties (Access = private)  % Declara as propriedades deste bloco.
        client  % Especifica client.
        sim  % Especifica sim.
        simVision  % Especifica simVision.
        TargetHandle        % handle for the target object
        TargetNames         % list of targets
        CameraHandle        % handler for camera

        VISION_WORKING  % Especifica VISION_WORKING.
    end  % Fim do bloco.

    properties  % Declara as propriedades deste bloco.
        TARGET_Number       % Number of targets
    end  % Fim do bloco.

    methods  % Declara os métodos deste bloco.
        % Create a connection handler to the Simulation scene
        %Start handles for objects
        function [obj,error] = simulator_interface(VISIONWORKING)  % Define as entradas e saídas desta função.
            error = 0;  % Define error.
            %Start Communication
            obj.client = RemoteAPIClient();  % Atualiza obj.client.
            obj.sim = obj.client.require('sim');  % Atualiza obj.sim.
            obj.simVision = obj.client.require('simVision');  % Atualiza obj.simVision.

            obj.VISION_WORKING = VISIONWORKING;  % Atualiza obj.VISION_WORKING.

            % Two different targets
            obj.TargetNames{1} = '/TargetGreen';  % Atualiza obj.TargetNames{1}.
            obj.TargetNames{2} = '/BlueCube';  % Atualiza obj.TargetNames{2}.

            [~,obj.TARGET_Number]=size(obj.TargetNames);  % Atualiza [~,obj.TARGET_Number].

            %Get Targets Handle
            for a=1:obj.TARGET_Number  % Percorre os valores do índice a.
                obj.TargetHandle{a} = obj.sim.getObject(obj.TargetNames{a});  % Obtém a referência do objeto da cena.
                if(obj.TargetHandle{a}==-1)  % Testa a condição sobre obj.TargetHandle, a.
                    disp('ERROR: Failed getting target handle');  % Chama disp com os parâmetros indicados.
                    error = 1;  % Define error.
                    return;  % Regressa à função que fez a chamada.
                end  % Fim do bloco.
            end  % Fim do bloco.

            %Get Vision Handle
            if obj.VISION_WORKING==1  % Testa a condição sobre obj.VISION_WORKING.
                obj.CameraHandle = obj.sim.getObject('/Vision_sensor');  % Obtém a referência do objeto da cena.
                if (obj.CameraHandle==-1)  % Testa a condição sobre obj.CameraHandle.
                    disp('ERROR: Failed getting camera handle');  % Chama disp com os parâmetros indicados.
                    error = 1;  % Define error.
                    return;  % Regressa à função que fez a chamada.
                end  % Fim do bloco.
            end  % Fim do bloco.
        end  % Fim do bloco.

        %Function that allows the simulation to be started in CoppeliaSim
        function start_simulation(obj)  % Define as entradas e saídas desta função.
            obj.sim.setStepping(true);  % Chama obj.sim.setStepping com os parâmetros indicados.
            obj.sim.startSimulation();  % Chama obj.sim.startSimulation com os parâmetros indicados.
        end  % Fim do bloco.

        %Function that triggers the next simulation step
        function step_simulation(obj)  % Define as entradas e saídas desta função.
            obj.sim.step();  % Chama obj.sim.step com os parâmetros indicados.
        end  % Fim do bloco.
        
        function client = get_connection(obj)  % Define as entradas e saídas desta função.
            client = obj.client;  % Atualiza client.
        end  % Fim do bloco.

        %Function that allows you to get the position of the target
        %Input:
        %iTarget - target index to know position
        %Output:
        %TargetPosition - array with the target location (x, y, z)
        function targetPosition=get_target_position(obj,iTarget)  % Define as entradas e saídas desta função.
            targetPosition_cell = obj.sim.getObjectPosition(obj.TargetHandle{iTarget});  % Lê a posição do objeto no referencial pedido.
            targetPosition(1)=targetPosition_cell{1};      %m
            targetPosition(2)=targetPosition_cell{2};      %m
            targetPosition(3)=targetPosition_cell{3};      %m
        end  % Fim do bloco.

        %Function that allows you to get information from vision system
        % Output:
        % image - matrix with image information
        % resolution - resolution information 
        function [image,resolution] = get_camera_information(obj)  % Define as entradas e saídas desta função.
            if obj.VISION_WORKING==1  % Testa a condição sobre obj.VISION_WORKING.
                [ imageData, res] = obj.sim.getVisionSensorImg(obj.CameraHandle);  % Lê a imagem do sensor.
                width = res{1};  % Atualiza width.
                height = res{2};  % Atualiza height.
                image = flip(permute(reshape(imageData, 3, width, height), [3 2 1]), 1);  %Convert BGR->RGB
                resolution = [width height];  % Atualiza resolution.
            end  % Fim do bloco.
        end  % Fim do bloco.

        function [M, angGraus, W, H] = get_camera_parameters(obj)  % Define as entradas e saídas desta função.
            m = obj.sim.getObjectMatrix(obj.CameraHandle, obj.sim.handle_world);  % Lê a transformação do objeto.
            if iscell(m), m = cell2mat(m); end  % Testa a condição sobre iscell, m.
            M = reshape(m, 4, 3)';  % Reorganiza as dimensões dos dados.
            ang = obj.sim.getObjectFloatParam(obj.CameraHandle, obj.sim.visionfloatparam_perspective_angle);  % Lê um parâmetro geométrico da cena.
            if iscell(ang), ang = ang{end}; end  % Testa a condição sobre iscell, ang.
            angGraus = rad2deg(ang);  % Atualiza angGraus.
            [~, res] = obj.sim.getVisionSensorImg(obj.CameraHandle);  % Lê a imagem do sensor.
            W = res{1};  % Atualiza W.
            H = res{2};  % Atualiza H.
        end  % Fim do bloco.

        function position = get_object_position(obj, nome)  % Define as entradas e saídas desta função.
            handle = obj.sim.getObject(nome);  % Obtém a referência do objeto da cena.
            p = obj.sim.getObjectPosition(handle, obj.sim.handle_world);  % Lê a posição do objeto no referencial pedido.
            position = [p{1}, p{2}, p{3}];  % Atualiza position.
        end  % Fim do bloco.

        %Function that provides information on the elapsed simulation time.
        %Output
        %t - simulation time
        function t = get_simulation_time(obj)  % Define as entradas e saídas desta função.
            t = obj.sim.getSimulationTime();  % Atualiza t.
        end  % Fim do bloco.

        %Function that ends the class
        function terminate(obj)  % Define as entradas e saídas desta função.
            obj.sim.stopSimulation();  % Chama obj.sim.stopSimulation com os parâmetros indicados.
        end  % Fim do bloco.
    end  % Fim do bloco.
end  % Fim do bloco.
