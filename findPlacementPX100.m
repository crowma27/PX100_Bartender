function placement = findPlacementPX100(simInterface, cup, supportPath)  % Define as entradas e saídas desta função.
% Localiza os apoios fixos usando o centro do copo detetado pela câmara.
    client = simInterface.get_connection();  % Atualiza client.
    sim = client.require('sim');  % Atualiza sim.
    base = sim.getObject('/px100');  % Obtém a referência do objeto da cena.
    handles = sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0);  % Obtém a referência do objeto da cena.
    if iscell(handles)  % Testa a condição sobre iscell, handles.
        handles = cell2mat(handles);  % Atualiza handles.
    end  % Fim do bloco.

    closestSupport = inf;  % Atualiza closestSupport.
    placement = struct('supportPath', '', 'supportTop', [], 'supportRadius', [], ...  % Reúne os campos numa estrutura.
        'readGripperCenter', []);  % Completa os dados da instrução anterior.
    fingerHandles = zeros(1, 2);  % Reserva os identificadores dos dois dedos.

    for handle = handles(:)'  % Percorre os valores do índice handle.
        alias = sim.getObjectAlias(handle, -1);  % Obtém a referência do objeto da cena.
        if strcmpi(alias, 'fingerLeft')  % Testa a condição sobre strcmpi, alias, fingerLeft.
            fingerHandles(1) = handle;  % Atualiza fingerHandles(1).
        elseif strcmpi(alias, 'fingerRight')  % Testa a condição sobre strcmpi, alias, fingerRight.
            fingerHandles(2) = handle;  % Atualiza fingerHandles(2).
        elseif strcmpi(alias, 'Disc')  % Testa a condição sobre strcmpi, alias, Disc.
            if ~isPurple(sim, handle)  % Testa a condição sobre isPurple, sim, handle.
                continue;  % Passa à iteração seguinte.
            end  % Fim do bloco.
            path = sim.getObjectAlias(handle, 1);  % Obtém a referência do objeto da cena.
            if ~isempty(supportPath) && ~strcmp(path, supportPath)  % Testa a condição sobre isempty, supportPath, strcmp, path.
                continue;  % Passa à iteração seguinte.
            end  % Fim do bloco.
            [top, radius] = supportGeometry(sim, handle, base);  % Atualiza [top, radius].
            distance = norm(top(1:2) - cup.graspTargetBase(1:2));  % Calcula a magnitude do desvio.
            if distance < closestSupport  % Testa a condição sobre distance, closestSupport.
                closestSupport = distance;  % Atualiza closestSupport.
                placement.supportPath = path;  % Atualiza placement.supportPath.
                placement.supportTop = top;  % Atualiza placement.supportTop.
                placement.supportRadius = radius;  % Atualiza placement.supportRadius.
            end  % Fim do bloco.
        end  % Fim do bloco.
    end  % Fim do bloco.

    if isempty(placement.supportPath)  % Verifica se há dados do apoio para esta operação.
        error('Nao encontrei o apoio lilas pedido na cena.');  % Interrompe a execução e indica a causa.
    end  % Fim do bloco.
    if any(fingerHandles == 0)  % Testa a condição sobre any, fingerHandles.
        error('Nao encontrei as formas fingerLeft e fingerRight para alinhar a pega.');  % Interrompe a execução e indica a causa.
    end  % Fim do bloco.
    leftCenter = bboxCenter(sim, fingerHandles(1));  % Atualiza leftCenter.
    rightCenter = bboxCenter(sim, fingerHandles(2));  % Atualiza rightCenter.
    placement.readGripperCenter = @() ( ...  % Guarda a função a chamar mais tarde.
        pointInBase(sim, fingerHandles(1), base, leftCenter) + ...  % Completa os dados da instrução anterior.
        pointInBase(sim, fingerHandles(2), base, rightCenter))/2;  % Completa os dados da instrução anterior.
    fingerHalfSize = [diff(bboxLimits(sim, fingerHandles(1))); ...  % Calcula as meias dimensões das caixas dos dedos.
                      diff(bboxLimits(sim, fingerHandles(2)))] / 2;  % Completa os dados da instrução anterior.
    placement.readFingerBottom = @() min([ ...  % Guarda a função a chamar mais tarde.
        bottomZ(sim, fingerHandles(1), base, leftCenter, fingerHalfSize(1,:)), ...  % Completa os dados da instrução anterior.
        bottomZ(sim, fingerHandles(2), base, rightCenter, fingerHalfSize(2,:))]);  % Completa os dados da instrução anterior.
end  % Fim do bloco.

function z = bottomZ(sim, handle, base, center, halfSize)  % Define as entradas e saídas desta função.
    pose = reshape(asRow(sim.getObjectMatrix(handle, base)), 4, 3)';  % Lê a transformação do objeto.
    % A projecao da caixa do dedo em Z da a altura do seu ponto mais baixo.
    z = pose(3,:)*[center(:); 1] - abs(pose(3,1:3))*halfSize(:);  % Atualiza z.
end  % Fim do bloco.

function center = bboxCenter(sim, handle)  % Define as entradas e saídas desta função.
    center = mean(bboxLimits(sim, handle), 1);  % Atualiza center.
end  % Fim do bloco.

function limits = bboxLimits(sim, handle)  % Define as entradas e saídas desta função.
    limits = [sim.getObjectFloatParam(handle, sim.objfloatparam_objbbox_min_x), ...  % Lê um parâmetro geométrico da cena.
              sim.getObjectFloatParam(handle, sim.objfloatparam_objbbox_min_y), ...  % Completa os dados da instrução anterior.
              sim.getObjectFloatParam(handle, sim.objfloatparam_objbbox_min_z); ...  % Completa os dados da instrução anterior.
              sim.getObjectFloatParam(handle, sim.objfloatparam_objbbox_max_x), ...  % Completa os dados da instrução anterior.
              sim.getObjectFloatParam(handle, sim.objfloatparam_objbbox_max_y), ...  % Completa os dados da instrução anterior.
              sim.getObjectFloatParam(handle, sim.objfloatparam_objbbox_max_z)];  % Completa os dados da instrução anterior.
end  % Fim do bloco.

function point = pointInBase(sim, handle, base, localPoint)  % Define as entradas e saídas desta função.
    pose = reshape(asRow(sim.getObjectMatrix(handle, base)), 4, 3)';  % Lê a transformação do objeto.
    point = (pose * [localPoint(:); 1])';  % Atualiza point.
end  % Fim do bloco.

function purple = isPurple(sim, handle)  % Define as entradas e saídas desta função.
    [ok, rgb] = sim.getShapeColor(handle, '', sim.colorcomponent_ambient_diffuse);  % Atualiza [ok, rgb].
    if ok ~= 1  % Testa a condição sobre ok.
        purple = false;  % Atualiza purple.
        return;  % Regressa à função que fez a chamada.
    end  % Fim do bloco.
    hsv = rgb2hsv(asRow(rgb));  % Atualiza hsv.
    purple = hsv(1) >= 0.68 && hsv(1) <= 0.94 && hsv(2) >= 0.25;  % Atualiza purple.
end  % Fim do bloco.

function [top, radius] = supportGeometry(sim, handle, base)  % Define as entradas e saídas desta função.
    pose = reshape(asRow(sim.getObjectMatrix(handle, base)), 4, 3)';  % Lê a transformação do objeto.
    minX = sim.getObjectFloatParam(handle, sim.objfloatparam_objbbox_min_x);  % Lê um parâmetro geométrico da cena.
    maxX = sim.getObjectFloatParam(handle, sim.objfloatparam_objbbox_max_x);  % Lê um parâmetro geométrico da cena.
    minY = sim.getObjectFloatParam(handle, sim.objfloatparam_objbbox_min_y);  % Lê um parâmetro geométrico da cena.
    maxY = sim.getObjectFloatParam(handle, sim.objfloatparam_objbbox_max_y);  % Lê um parâmetro geométrico da cena.
    maxZ = sim.getObjectFloatParam(handle, sim.objfloatparam_objbbox_max_z);  % Lê um parâmetro geométrico da cena.
    minZ = sim.getObjectFloatParam(handle, sim.objfloatparam_objbbox_min_z);  % Lê um parâmetro geométrico da cena.
    % Os discos sao horizontais. A face de cima depende do sinal do Z local.
    if abs(pose(3,3)) < 0.99  % Testa a condição sobre abs, pose.
        error('O apoio lilas tem de estar horizontal.');  % Interrompe a execução e indica a causa.
    end  % Fim do bloco.
    topZ = maxZ;  % Atualiza topZ.
    if pose(3,3) < 0  % Testa a condição sobre pose.
        topZ = minZ;  % Atualiza topZ.
    end  % Fim do bloco.
    top = (pose * [(minX+maxX)/2; (minY+maxY)/2; topZ; 1])';  % Atualiza top.
    radius = min(maxX-minX, maxY-minY)/2;  % Atualiza radius.
end  % Fim do bloco.

function row = asRow(value)  % Define as entradas e saídas desta função.
    if iscell(value)  % Testa a condição sobre iscell, value.
        value = cell2mat(value);  % Atualiza value.
    end  % Fim do bloco.
    row = double(value(:)');  % Atualiza row.
end  % Fim do bloco.
