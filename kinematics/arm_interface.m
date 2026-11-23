%   arm_interface.m
%   Interface to a simulated robot in CoppeliaSim simulator
%   2026/09/16
%   % Copyright (C) 2026
%   Author: Luís Louro, luislouro@algoritmi.uminho.pt
%           Estela Bicho, estela.bicho@dei.uminho.pt

classdef arm_interface < handle  % Declara a classe arm_interface.
    properties (Access = private)  % Declara as propriedades deste bloco.
        client  % Especifica client.
        sim  % Especifica sim.

        RobotHandle  % handle for the robot object
        jointHandle  % 1,2,...,4 handle for each of the joints of the manipulator
        HandHandle   % handle for the hand object 
        
        ROS_WORKING    % Especifica ROS_WORKING.
    end  % Fim do bloco.

    properties  % Declara as propriedades deste bloco.
        number_of_joints  % variable indicating the number of joints of the manipulator arm
        robot_name        % variable indicating the name of the handler in CoppeliaSim
        MinJointPos       % vector indicating the minimum position allowed for the joint i
        MaxJointPos       % vector indicating the maximun position allowed for the joint i
    end  % Fim do bloco.

    methods  % Declara os métodos deste bloco.
        % Classe constructor
        function [obj,error] = arm_interface(sim_obj,robot_name,ROSWORKING)  % Define as entradas e saídas desta função.
            obj.robot_name = robot_name;  % Atualiza obj.robot_name.
            obj.ROS_WORKING = ROSWORKING;  % Atualiza obj.ROS_WORKING.
            error = 0;  % Define error.

            obj.client = sim_obj.get_connection();  % Atualiza obj.client.
            obj.sim = obj.client.require('sim');  % Atualiza obj.sim.

            %% Get objects handles

            %Get Arm Handle
            obj.RobotHandle = obj.sim.getObject(robot_name);  % Obtém a referência do objeto da cena.

            if(obj.RobotHandle==-1)  % Testa a condição sobre obj.RobotHandle.
                disp('ERROR: Failed getting robot handle');  % Chama disp com os parâmetros indicados.
                error = 1;  % Define error.
                return;  % Regressa à função que fez a chamada.
            end  % Fim do bloco.

            %arm joints handles
            jointName{1} = '/waist';  % Atualiza jointName{1}.
            jointName{2} = '/shoulder';  % Atualiza jointName{2}.
            jointName{3} = '/elbow';  % Atualiza jointName{3}.
            jointName{4} = '/wrist_angle';  % Atualiza jointName{4}.
            [~,obj.number_of_joints]=size(jointName);  % Atualiza [~,obj.number_of_joints].
            
            for j=1:obj.number_of_joints  % Percorre os valores do índice j.
                obj.jointHandle{j}=obj.sim.getObject(jointName{j});  % Obtém a referência do objeto da cena.
                if(obj.jointHandle{j}==-1)  % Testa a condição sobre obj.jointHandle, j.
                    disp('ERROR: Failed getting joint handle');  % Chama disp com os parâmetros indicados.
                    error = 1;  % Define error.
                    return  % Regressa à função que fez a chamada.
                end  % Fim do bloco.
            end  % Fim do bloco.

            fingerName{1} = '/left_finger';  % Atualiza fingerName{1}.
            fingerName{2} = '/right_finger';  % Atualiza fingerName{2}.
            %Get Fingers Handles
            for m=1:2  % Percorre os valores do índice m.
                obj.HandHandle{m} = obj.sim.getObject(fingerName{m});  % Obtém a referência do objeto da cena.
                if(obj.HandHandle{m}==-1)  % Testa a condição sobre obj.HandHandle, m.
                    disp('ERROR: Failed getting finger handle');  % Chama disp com os parâmetros indicados.
                    error = 1;  % Define error.
                    return;  % Regressa à função que fez a chamada.
                end  % Fim do bloco.
            end  % Fim do bloco.

            if obj.ROS_WORKING==1  % Testa a condição sobre obj.ROS_WORKING.
                %Initialialize comunication with ros
            end  % Fim do bloco.

        end  % Fim do bloco.

        %Function that allows you to get the arm characteristics
        function [nJoints,Link,MinPositionJoint,MaxPositionJoint] = get_RobotCharacteristics(obj)  % Define as entradas e saídas desta função.
            %Number of joints
            nJoints = obj.number_of_joints;  % Atualiza nJoints.
            %Min and max joint value for each joint
            MinPositionJoint(1)=-180*pi/180;  % Atualiza MinPositionJoint(1).
            MinPositionJoint(2)=-111*pi/180;  % Atualiza MinPositionJoint(2).
            MinPositionJoint(3)=-121*pi/180;  % Atualiza MinPositionJoint(3).
            MinPositionJoint(4)=-100*pi/180;  % Atualiza MinPositionJoint(4).
            MaxPositionJoint(1)=180*pi/180;  % Atualiza MaxPositionJoint(1).
            MaxPositionJoint(2)=107*pi/180;  % Atualiza MaxPositionJoint(2).
            MaxPositionJoint(3)=92*pi/180;  % Atualiza MaxPositionJoint(3).
            MaxPositionJoint(4)=123*pi/180;       % Atualiza MaxPositionJoint(4).
            %Links lenghts
            Link(1)=0.08945;  % Define Link(1).
            Link(2)=0.1;  % Define Link(2).
            Link(3)=0.035;  % Define Link(3).
            Link(4)=0.1;  % Define Link(4).
            Link(5)=0.12915;  % Define Link(5).
            obj.MinJointPos = MinPositionJoint;  % Atualiza obj.MinJointPos.
            obj.MaxJointPos = MaxPositionJoint;  % Atualiza obj.MaxJointPos.
        end  % Fim do bloco.

        %Function that allows you to get the position of the arm (x,y,z)
        function armPosition=get_robot_position(obj)  % Define as entradas e saídas desta função.
            armPosition_str = obj.sim.getObjectPosition(obj.RobotHandle);  % Lê a posição do objeto no referencial pedido.
            armPosition(1) = armPosition_str{1};  % Atualiza armPosition(1).
            armPosition(2) = armPosition_str{2};  % Atualiza armPosition(2).
            armPosition(3) = armPosition_str{3};  % Atualiza armPosition(3).
        end  % Fim do bloco.

        %Function that allows you to set the joints from the arm
        function error = set_joints(obj, armJoints) %armJoints (1-4) in rad
            error = 0;  % Define error.
            for i=1:1:obj.number_of_joints  % Percorre os valores do índice i.
                if armJoints(i)<obj.MinJointPos(i) || armJoints(i)>obj.MaxJointPos(i)  % Testa a condição sobre armJoints, i, obj.MinJointPos, obj.MaxJointPos.
                    disp('ERROR: value outside the limits of the joints');  % Chama disp com os parâmetros indicados.
                    error = 1;  % Define error.
                    return;  % Regressa à função que fez a chamada.
                else  % Trata o caso alternativo.
                    obj.sim.setJointTargetPosition(obj.jointHandle{i},armJoints(i));  % Envia a referência ao atuador.
                end  % Fim do bloco.
            end  % Fim do bloco.

            if obj.ROS_WORKING==1  % Testa a condição sobre obj.ROS_WORKING.
                %Send values for real robot
            end  % Fim do bloco.
        end  % Fim do bloco.

        %Function that allows you to get the joints values from the arm
        %armJoints - joint values for arm (in rad)


        %testing
        function gripperPosition = get_gripper_position(obj)  % Define as entradas e saídas desta função.

            leftHandle = obj.sim.getObject('/px100/px100_left_finger_link_visual');  % Obtém a referência do objeto da cena.
            rightHandle = obj.sim.getObject('/px100/px100_right_finger_link_visual');  % Obtém a referência do objeto da cena.

            leftPos = obj.sim.getObjectPosition(leftHandle);  % Lê a posição do objeto no referencial pedido.
            rightPos = obj.sim.getObjectPosition(rightHandle);  % Lê a posição do objeto no referencial pedido.

            gripperPosition(1) = (leftPos{1} + rightPos{1}) / 2;  % Atualiza gripperPosition(1).
            gripperPosition(2) = (leftPos{2} + rightPos{2}) / 2;  % Atualiza gripperPosition(2).
            gripperPosition(3) = (leftPos{3} + rightPos{3}) / 2;  % Atualiza gripperPosition(3).

        end  % Fim do bloco.

        function jointPositions = get_joint_positions(obj)  % Define as entradas e saídas desta função.

            jointPositions = zeros(obj.number_of_joints,3);  % Reserva memória inicializada a zero.

            for i = 1:obj.number_of_joints  % Percorre os valores do índice i.

                p = obj.sim.getObjectPosition(obj.jointHandle{i});  % Lê a posição do objeto no referencial pedido.

                jointPositions(i,1) = p{1};  % Atualiza jointPositions(i,1).
                jointPositions(i,2) = p{2};  % Atualiza jointPositions(i,2).
                jointPositions(i,3) = p{3};  % Atualiza jointPositions(i,3).

            end  % Fim do bloco.

        end  % Fim do bloco.

        %testing

        function armJoints = get_joints(obj)  % Define as entradas e saídas desta função.
            for i = 1:obj.number_of_joints  % Percorre os valores do índice i.
                %Read from joints
                armJoints(i)=obj.sim.getJointPosition(obj.jointHandle{i});  % Atualiza armJoints(i).
            end  % Fim do bloco.

            if obj.ROS_WORKING==1  % Testa a condição sobre obj.ROS_WORKING.
                %Send values for real robot
            end  % Fim do bloco.
        end  % Fim do bloco.

        %Functions that allows close the hand
        function close_hand(obj,speed)  % Define as entradas e saídas desta função.
            if nargin < 2  % Verifica se o argumento opcional foi fornecido.
                speed = 0.05;  % Define speed.
            end  % Fim do bloco.
            obj.sim.setJointTargetVelocity(obj.HandHandle{1},-speed);  % Envia a referência ao atuador.
            obj.sim.setJointTargetVelocity(obj.HandHandle{2},speed);  % Envia a referência ao atuador.

            if obj.ROS_WORKING==1  % Testa a condição sobre obj.ROS_WORKING.
                %Send values to the gripper
            end  % Fim do bloco.
            
        end  % Fim do bloco.

        %Functions that allows open the hand
        function open_hand(obj)  % Define as entradas e saídas desta função.
            obj.sim.setJointTargetVelocity(obj.HandHandle{1},0.05);  % Envia a referência ao atuador.

            obj.sim.setJointTargetVelocity(obj.HandHandle{2},-0.05);  % Envia a referência ao atuador.

            if obj.ROS_WORKING==1  % Testa a condição sobre obj.ROS_WORKING.
                %Send values to the gripper
            end  % Fim do bloco.
        end        % Fim do bloco.
    end  % Fim do bloco.
end  % Fim do bloco.
