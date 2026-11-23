classdef RemoteAPIObject  % Declara a classe RemoteAPIObject.
    properties (Access = protected)  % Declara as propriedades deste bloco.
        x__client  % Especifica x__client.
        x__objectName  % Especifica x__objectName.
        x__info  % Especifica x__info.
    end  % Fim do bloco.

    methods  % Declara os métodos deste bloco.
        function self = RemoteAPIObject(client, objectName)  % Define as entradas e saídas desta função.
            self.x__client = client;  % Atualiza self.x__client.
            self.x__objectName = objectName;  % Atualiza self.x__objectName.
            info = client.call('zmqRemoteApi.info', {objectName});  % Atualiza info.
            self.x__info = info{1};  % Atualiza self.x__info.
        end  % Fim do bloco.

        function varargout = subsref(self, S)  % Define as entradas e saídas desta função.
            for i=1:numel(S)-1  % Percorre os valores do índice i.
                if ~strcmp(S(i).type, '.')  % Testa a condição sobre strcmp, S, i, type.
                    error('invalid field access');  % Interrompe a execução e indica a causa.
                end  % Fim do bloco.
            end  % Fim do bloco.
            switch S(end).type  % Escolhe o ramo de acordo com S(end).type.
                case '.'  % Executa o ramo '.'.
                    info = builtin('subsref', self.x__info, S);  % Atualiza info.
                    if ~isfield(info, 'const')  % Testa a condição sobre isfield, info, const.
                        error('not a constant');  % Interrompe a execução e indica a causa.
                    end  % Fim do bloco.
                    varargout = {info.const};  % Atualiza varargout.
                case '()'  % Executa o ramo '()'.
                    info = builtin('subsref', self.x__info, S(1:end-1));  % Atualiza info.
                    if ~isfield(info, 'func')  % Testa a condição sobre isfield, info, func.
                        error('not a function');  % Interrompe a execução e indica a causa.
                    end  % Fim do bloco.
                    fn = self.x__objectName;  % Atualiza fn.
                    for i=1:numel(S)-1  % Percorre os valores do índice i.
                        fn = strcat(fn, S(i).type, S(i).subs);  % Atualiza fn.
                    end  % Fim do bloco.
                    args = S(end).subs;  % Atualiza args.
                    varargout = self.x__client.call(fn, args);  % Atualiza varargout.
                otherwise  % Trata o caso alternativo.
                    error('invalid field access');  % Interrompe a execução e indica a causa.
            end  % Fim do bloco.
        end  % Fim do bloco.
    end  % Fim do bloco.
end  % Fim do bloco.

