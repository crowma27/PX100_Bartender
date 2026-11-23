% this requires JeroMQ
% call javaaddpath('path/to/jeromq.jar') before running client code

classdef RemoteAPIClient  % Declara a classe RemoteAPIClient.
    properties (Access = protected)  % Declara as propriedades deste bloco.
        verbose  % Especifica verbose.
        ctx  % Especifica ctx.
        socket  % Especifica socket.
        uuid  % Especifica uuid.
        VERSION  % Especifica VERSION.
        callbacks  % Especifica callbacks.
    end  % Fim do bloco.

    methods(Static, Access = private)  % Declara os métodos deste bloco.
        function opts = getopts(args, varargin)  % Define as entradas e saídas desta função.
            opts = struct();  % Reúne os campos numa estrutura.
            for i=1:2:numel(varargin)  % Percorre os valores do índice i.
                opts.(varargin{i}) = varargin{i+1};  % Atualiza opts.(varargin{i}).
            end  % Fim do bloco.
            for i=1:2:numel(args)  % Percorre os valores do índice i.
                if isfield(opts, args{i})  % Testa a condição sobre isfield, opts, args, i.
                    opts.(args{i}) = args{i+1};  % Atualiza opts.(args{i}).
                else  % Trata o caso alternativo.
                    error('unknown option: "%s"', args{i});  % Interrompe a execução e indica a causa.
                end  % Fim do bloco.
            end  % Fim do bloco.
        end  % Fim do bloco.
    end  % Fim do bloco.

    methods  % Declara os métodos deste bloco.
        function obj = RemoteAPIClient(varargin)  % Define as entradas e saídas desta função.
            try  % Tenta executar a operação seguinte.
                ZMsg();  % Chama ZMsg com os parâmetros indicados.
            catch ex  % Trata a falha da operação anterior.
                if ~strcmp(ex.identifier, 'MATLAB:UndefinedFunction')  % Testa a condição sobre strcmp, ex.identifier, MATLAB, UndefinedFunction.
                    rethrow(ex);  % Chama rethrow com os parâmetros indicados.
                end  % Fim do bloco.
                jeromqVer = '0.5.2';  % Define jeromqVer.
                jeromqJAR = sprintf('jeromq-%s.jar', jeromqVer);  % Formata o texto.
                if ~isfile(jeromqJAR)  % Testa a condição sobre isfile, jeromqJAR.
                    mvnRepo = 'https://repo1.maven.org/maven2';  % Atualiza mvnRepo.
                    websave(jeromqJAR, sprintf('%s/org/zeromq/jeromq/%s/%s', mvnRepo, jeromqVer, jeromqJAR));  % Formata o texto.
                end  % Fim do bloco.
                javaaddpath(jeromqJAR);  % Chama javaaddpath com os parâmetros indicados.
            end  % Fim do bloco.

            import org.zeromq.*;  % Especifica import.

            opts = RemoteAPIClient.getopts(varargin, ...  % Atualiza opts.
                'host', 'localhost', ...  % Completa os dados da instrução anterior.
                'port', 23000, ...  % Completa os dados da instrução anterior.
                'cntPort', -1, ...  % Completa os dados da instrução anterior.
                'verbose', false ...  % Completa os dados da instrução anterior.
            );  % Completa os dados da instrução anterior.
            tcpaddr = @(host, port) sprintf('tcp://%s:%d', host, port);  % Formata o texto.

            obj.verbose = opts.verbose;  % Atualiza obj.verbose.

            obj.ctx = ZContext();  % Atualiza obj.ctx.

            obj.socket = obj.ctx.createSocket(SocketType.REQ);  % Atualiza obj.socket.
            obj.socket.connect(java.lang.String(tcpaddr(opts.host, opts.port)));  % Chama obj.socket.connect com os parâmetros indicados.

            obj.uuid = char(java.util.UUID.randomUUID);  % Atualiza obj.uuid.
            obj.VERSION = 2;  % Define obj.VERSION.
            obj.callbacks = containers.Map();  % Atualiza obj.callbacks.
        end  % Fim do bloco.

        function delete(obj)  % Define as entradas e saídas desta função.
            req = struct('func', '_*end*_', 'args', {}, 'uuid', {obj.uuid}, 'ver', {obj.VERSION}, 'lang', 'matlab');  % Reúne os campos numa estrutura.
            req_raw = cbor.encode(req);  % Codifica o pedido em bytes CBOR.
            req_frame = ZFrame(req_raw);  % Atualiza req_frame.
            req_msg = ZMsg();  % Atualiza req_msg.
            req_msg.add(req_frame);  % Chama req_msg.add com os parâmetros indicados.
            req_msg.send(obj.socket, 0);  % Chama req_msg.send com os parâmetros indicados.
            ZMsg.recvMsg(obj.socket, 0);  % Chama ZMsg.recvMsg com os parâmetros indicados.

            obj.socket.close();  % Chama obj.socket.close com os parâmetros indicados.
            obj.ctx.close();  % Chama obj.ctx.close com os parâmetros indicados.
        end  % Fim do bloco.

        function addCallback(obj, func, name)  % Define as entradas e saídas desta função.
            obj.callbacks(name) = func;  % Atualiza obj.callbacks(name).
        end  % Fim do bloco.

        function outputArgs = call(obj, fn, inputArgs)  % Define as entradas e saídas desta função.
            % Call function with specified arguments. Is Reentrant
            import org.zeromq.*;  % Especifica import.
            req = struct('func', fn, 'args', {inputArgs}, 'uuid', {obj.uuid}, 'ver', {obj.VERSION}, 'lang', 'matlab', 'argsL', numel(inputArgs));  % Conta os elementos.
            req_raw = cbor.encode(req);  % Codifica o pedido em bytes CBOR.
            req_frame = ZFrame(req_raw);  % Atualiza req_frame.
            req_msg = ZMsg();  % Atualiza req_msg.
            req_msg.add(req_frame);  % Chama req_msg.add com os parâmetros indicados.
            req_msg.send(obj.socket, 0);  % Chama req_msg.send com os parâmetros indicados.

            resp_msg = ZMsg.recvMsg(obj.socket, 0);  % Espera pela resposta do simulador.
            resp_frame = resp_msg.pop();  % Retira a trama da mensagem recebida.
            resp_raw = typecast(resp_frame.getData(), 'uint8');  % Interpreta os dados recebidos como bytes.
            resp = cbor.decode(resp_raw);  % Descodifica a resposta recebida.

            while isfield(resp, 'func')  % Repete enquanto a condição se mantiver.
                args = {};  % Atualiza args.
                % _*repeat*_ not yet implemented!
                if ~strcmp(resp.func, '_*wait*_')  % Testa a condição sobre strcmp, resp.func, wait.
                    if isKey(obj.callbacks, resp.func) % we cannot raise an error if not present: e.g. a custom UI async callback cannot be assigned to a specific client
                        callback = obj.callbacks(resp.func);  % Atualiza callback.
                        try  % Tenta executar a operação seguinte.
                            a = callback(resp.args);  % Atualiza a.
                            if ~isempty(a)  % Testa a condição sobre isempty, a.
                                args = a;  % Atualiza args.
                            end  % Fim do bloco.
                        catch ME  % Trata a falha da operação anterior.
                            error('Error in callback: %s', ME.message);  % Interrompe a execução e indica a causa.
                        end  % Fim do bloco.
                    end  % Fim do bloco.
                end  % Fim do bloco.
                req2 = struct('func', '_*executed*_', 'args', {args}, 'uuid', {obj.uuid}, 'ver', {obj.VERSION}, 'lang', 'matlab', 'argsL', numel(args));  % Conta os elementos.
                req2_raw = cbor.encode(req2);  % Atualiza req2_raw.
                req2_frame = ZFrame(req2_raw);  % Atualiza req2_frame.
                req2_msg = ZMsg();  % Atualiza req2_msg.
                req2_msg.add(req2_frame);  % Chama req2_msg.add com os parâmetros indicados.
                req2_msg.send(obj.socket, 0);  % Chama req2_msg.send com os parâmetros indicados.

                resp_msg = ZMsg.recvMsg(obj.socket, 0);  % Espera pela resposta do simulador.
                resp_frame = resp_msg.pop();  % Retira a trama da mensagem recebida.
                resp_raw = typecast(resp_frame.getData(), 'uint8');  % Interpreta os dados recebidos como bytes.
                resp = cbor.decode(resp_raw);  % Descodifica a resposta recebida.
            end  % Fim do bloco.
            if isfield(resp, 'err')  % Testa a condição sobre isfield, resp, err.
                error(resp.err);  % Interrompe a execução e indica a causa.
            end  % Fim do bloco.
            if numel(resp.ret) == 0  % Testa a condição sobre numel, resp.ret.
                outputArgs = {};  % Atualiza outputArgs.
            else  % Trata o caso alternativo.
                outputArgs = resp.ret;  % Atualiza outputArgs.
            end  % Fim do bloco.
        end  % Fim do bloco.

        function remoteObject = getObject(obj, name)  % Define as entradas e saídas desta função.
            remoteObject = RemoteAPIObject(obj, name);  % Atualiza remoteObject.
        end  % Fim do bloco.

        function remoteObject = require(obj, name)  % Define as entradas e saídas desta função.
            obj.call('zmqRemoteApi.require', {name});  % Chama obj.call com os parâmetros indicados.
            remoteObject = obj.getObject(name);  % Obtém a referência do objeto da cena.
        end  % Fim do bloco.

        function setStepping(obj, enable)  % Define as entradas e saídas desta função.
            arguments  % Declara os argumentos deste bloco.
                obj (1,1) RemoteAPIClient  % Chama obj com os parâmetros indicados.
                enable (1,1) logical = true  % Atualiza enable (1,1) logical.
            end  % Fim do bloco.

            obj.call('sim.setStepping', {enable});  % Chama obj.call com os parâmetros indicados.
        end  % Fim do bloco.

        function step(obj, wait)  % Define as entradas e saídas desta função.
            arguments  % Declara os argumentos deste bloco.
                obj (1,1) RemoteAPIClient  % Chama obj com os parâmetros indicados.
                wait (1,1) logical = true  % Atualiza wait (1,1) logical.
            end  % Fim do bloco.

            obj.call('sim.step', {wait});  % Chama obj.call com os parâmetros indicados.
        end  % Fim do bloco.
    end  % Fim do bloco.
end  % Fim do bloco.

