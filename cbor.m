% cbor.m - CBOR (RFC 7049) encoder/decoder for MATLAB
% Copyright (C) 2022 Coppelia Robotics AG
%
% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
%
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with this program.  If not, see <http://www.gnu.org/licenses/>.
%
% cbor.m is courtesy of Federico Ferri

classdef cbor  % Declara a classe cbor.
    methods  % Declara os métodos deste bloco.
        function obj = cbor()  % Define as entradas e saídas desta função.
            fprintf('Running tests...\n');  % Apresenta a medição ou o estado na consola.
            cbor.test  % Completa os dados da instrução anterior.
            clear obj  % Especifica clear.
        end  % Fim do bloco.
    end  % Fim do bloco.

    properties(Constant, Access = private)  % Declara as propriedades deste bloco.
        MASK_MAJOR = 0xE0  % Atualiza MASK_MAJOR.
        MASK_INFO = 0x1F  % Atualiza MASK_INFO.
        UNSIGNED_INT = 0  % Define UNSIGNED_INT.
        NEGATIVE_INT = 1  % Define NEGATIVE_INT.
        BYTE_STRING = 2  % Define BYTE_STRING.
        TEXT_STRING = 3  % Define TEXT_STRING.
        ARRAY = 4  % Define ARRAY.
        MAP = 5  % Define MAP.
        SIMPLE_VALUE = 7  % Define SIMPLE_VALUE.
        FALSE = 20  % Define FALSE.
        TRUE = 21  % Define TRUE.
        NULL = 22  % Define NULL.
        UNDEFINED = 23  % Define UNDEFINED.
        SINGLE_BYTE_EXTENSION = 24  % Define SINGLE_BYTE_EXTENSION.
        FLOAT16 = 25  % Define FLOAT16.
        FLOAT32 = 26  % Define FLOAT32.
        FLOAT64 = 27  % Define FLOAT64.
        BREAK = 31  % Define BREAK.
        UINT8 = 24  % Define UINT8.
        UINT16 = 25  % Define UINT16.
        UINT32 = 26  % Define UINT32.
        UINT64 = 27  % Define UINT64.
        UNDEFINED_LENGTH = 31  % Define UNDEFINED_LENGTH.
    end  % Fim do bloco.

    methods(Static, Access = private)  % Declara os métodos deste bloco.
        function opts = getopts(args,varargin)  % Define as entradas e saídas desta função.
            opts = struct();  % Reúne os campos numa estrutura.
            for i=1:2:numel(varargin)  % Percorre os valores do índice i.
                opts.(varargin{i}) = varargin{i+1};  % Atualiza opts.(varargin{i}).
            end  % Fim do bloco.
            for i=1:2:numel(args)  % Percorre os valores do índice i.
                if isfield(opts,args{i})  % Testa a condição sobre isfield, opts, args, i.
                    opts.(args{i}) = args{i+1};  % Atualiza opts.(args{i}).
                else  % Trata o caso alternativo.
                    error('unknown option: "%s"',args{i});  % Interrompe a execução e indica a causa.
                end  % Fim do bloco.
            end  % Fim do bloco.
        end  % Fim do bloco.
    end  % Fim do bloco.

    methods(Static)  % Declara os métodos deste bloco.
        function [o,d1] = decode(d,varargin)  % Define as entradas e saídas desta função.
            opts = cbor.getopts(varargin, ...  % Atualiza opts.
                'array2mat', false ...  % Completa os dados da instrução anterior.
            );  % Completa os dados da instrução anterior.
            assert(isa(d, 'uint8'));  % Confirma o resultado esperado no teste.
            major = bitshift(bitand(d(1), cbor.MASK_MAJOR), -5);  % Desloca os bits.
            info = bitand(d(1), cbor.MASK_INFO);  % Seleciona os bits da máscara.
            d1 = d(2:end);  % Atualiza d1.
            d1 = d1(:)';  % Atualiza d1.
            as = @(x,t) typecast(flip(x), t);  % Reinterpreta os bytes no tipo pedido.
            if major ~= cbor.SIMPLE_VALUE  % Testa a condição sobre major, cbor.SIMPLE_VALUE.
                if info < 24  % Testa a condição sobre info.
                    info1 = double(info);  % Atualiza info1.
                elseif info == cbor.UINT8  % Testa a condição sobre info, cbor.UINT8.
                    info1 = double(d1(1));  % Atualiza info1.
                    d1 = d1(2:end);  % Atualiza d1.
                elseif info == cbor.UINT16  % Testa a condição sobre info, cbor.UINT16.
                    info1 = double(as(d1(1:2), 'uint16'));  % Atualiza info1.
                    d1 = d1(3:end);  % Atualiza d1.
                elseif info == cbor.UINT32  % Testa a condição sobre info, cbor.UINT32.
                    info1 = double(as(d1(1:4), 'uint32'));  % Atualiza info1.
                    d1 = d1(5:end);  % Atualiza d1.
                elseif info == cbor.UINT64  % Testa a condição sobre info, cbor.UINT64.
                    info1 = double(as(d1(1:8), 'uint64'));  % Atualiza info1.
                    d1 = d1(9:end);  % Atualiza d1.
                elseif info == cbor.UNDEFINED_LENGTH  % Testa a condição sobre info, cbor.UNDEFINED_LENGTH.
                    info1 = NaN;  % Atualiza info1.
                elseif info > 27  % Testa a condição sobre info.
                    error('malformed data (info > 27)');  % Interrompe a execução e indica a causa.
                end  % Fim do bloco.
            end  % Fim do bloco.
            if major == cbor.UNSIGNED_INT  % Testa a condição sobre major, cbor.UNSIGNED_INT.
                if isnan(info1) error('malformed data (int + undefined length)'); end  % Testa a condição sobre isnan, info1, error, malformed, data, int, undefined, length, end.
                o = info1;  % Atualiza o.
            elseif major == cbor.NEGATIVE_INT  % Testa a condição sobre major, cbor.NEGATIVE_INT.
                if isnan(info1) error('malformed data (negative int + undefined length)'); end  % Testa a condição sobre isnan, info1, error, malformed, data, negative, int, undefined, length, .
                o = -1 - info1;  % Atualiza o.
            elseif major == cbor.BYTE_STRING  % Testa a condição sobre major, cbor.BYTE_STRING.
                if isnan(info1) error('undefined length byte strings are not supported by this decoder'); end  % Testa a condição sobre isnan, info1, error, undefined, length, byte, strings, are, not, support.
                o = d1(1:info1);  % Atualiza o.
                d1 = d1((info1+1):end);  % Atualiza d1.
            elseif major == cbor.TEXT_STRING  % Testa a condição sobre major, cbor.TEXT_STRING.
                if isnan(info1) error('undefined length text strings are not supported by this decoder'); end  % Testa a condição sobre isnan, info1, error, undefined, length, text, strings, are, not, support.
                o = native2unicode(d1(1:info1), 'UTF-8');  % Atualiza o.
                d1 = d1((info1+1):end);  % Atualiza d1.
            elseif major == cbor.ARRAY  % Testa a condição sobre major, cbor.ARRAY.
                if isnan(info1) info1 = 1e10; end  % Testa a condição sobre isnan, info1, end.
                o = {};  % Atualiza o.
                homogeneous = true;  % Atualiza homogeneous.
                for i=1:info1  % Percorre os valores do índice i.
                    if d1(1) == 255 d1 = d1(2:end); break; end  % Testa a condição sobre d1, end.
                    [o{i}, d1] = cbor.decode(d1);  % Atualiza [o{i}, d1].
                    if homogeneous && i > 1 && ~strcmp(class(o{1}),class(o{i}))  % Testa a condição sobre homogeneous, i, strcmp, class, o.
                        homogeneous = false;  % Atualiza homogeneous.
                    end  % Fim do bloco.
                end  % Fim do bloco.
                if opts.array2mat && homogeneous && (numel(o) == 0 || ~isa(o(1),'char'))  % Testa a condição sobre opts.array2mat, homogeneous, numel, o, isa, char.
                    o = cell2mat(o);  % Atualiza o.
                end  % Fim do bloco.
            elseif major == cbor.MAP  % Testa a condição sobre major, cbor.MAP.
                if isnan(info1) info1 = 1e10; end  % Testa a condição sobre isnan, info1, end.
                n = {};  % Atualiza n.
                v = {};  % Atualiza v.
                for i=1:info1  % Percorre os valores do índice i.
                    if d1(1) == 255 d1 = d1(2:end); break; end  % Testa a condição sobre d1, end.
                    [n{i}, d1] = cbor.decode(d1);  % Atualiza [n{i}, d1].
                    [v{i}, d1] = cbor.decode(d1);  % Atualiza [v{i}, d1].
                    if n{i}(1) == '_'  % Testa a condição sobre n, i.
                        % field names cannot start with underscore
                        n{i} = sprintf('x%s', n{i});  % Formata o texto.
                    end  % Fim do bloco.
                end  % Fim do bloco.
                o = cell2struct(v, n, 2);  % Reúne os campos numa estrutura.
            elseif major == cbor.SIMPLE_VALUE  % Testa a condição sobre major, cbor.SIMPLE_VALUE.
                if info == cbor.SINGLE_BYTE_EXTENSION  % Testa a condição sobre info, cbor.SINGLE_BYTE_EXTENSION.
                    % simple value 32..255 in next byte
                    info = d1(1);  % Atualiza info.
                    d1 = d1(2:end);  % Atualiza d1.
                    if info < 32 error('malformed simple value (0..31 in byte ext)'); end  % Testa a condição sobre info, error, malformed, simple, value, in, byte, ext, end.
                end  % Fim do bloco.

                if info == cbor.FALSE  % Testa a condição sobre info, cbor.FALSE.
                    o = false;  % Atualiza o.
                elseif info == cbor.TRUE  % Testa a condição sobre info, cbor.TRUE.
                    o = true;  % Atualiza o.
                elseif info == cbor.NULL  % Testa a condição sobre info, cbor.NULL.
                    o = NaN;  % Atualiza o.
                elseif info == cbor.UNDEFINED  % Testa a condição sobre info, cbor.UNDEFINED.
                    error('cannot decode undefined value');  % Interrompe a execução e indica a causa.
                elseif info == cbor.FLOAT16  % Testa a condição sobre info, cbor.FLOAT16.
                    f16 = as(d1(1:2), 'uint16');  % Atualiza f16.
                    d1 = d1(3:end);  % Atualiza d1.
                    % due to lack of 16-bit floats support, we decode
                    % IEEE 754 16-bit float manually:
                    sign = (-1) ^ double(bitget(f16,16));  % Atualiza sign.
                    expo = bitshift(bitand(f16,0x7C00),-10);  % Desloca os bits.
                    frac = bitand(f16,0x03FF);  % Seleciona os bits da máscara.
                    if expo == 0  % Testa a condição sobre expo.
                        o = sign * 2^-14 * double(frac) / 1024;  % Atualiza o.
                    elseif expo == 0x1F && frac == 0  % Testa a condição sobre expo, frac.
                        o = sign * inf;  % Atualiza o.
                    elseif expo == 0x1F && frac ~= 0  % Testa a condição sobre expo, frac.
                        o = nan;  % Atualiza o.
                    else  % Trata o caso alternativo.
                        o = sign * 2^(double(expo) - 15) * (1 + double(frac) / 1024);  % Atualiza o.
                    end  % Fim do bloco.
                elseif info == cbor.FLOAT32  % Testa a condição sobre info, cbor.FLOAT32.
                    o = as(d1(1:4), 'single');  % Atualiza o.
                    d1 = d1(5:end);  % Atualiza d1.
                elseif info == cbor.FLOAT64  % Testa a condição sobre info, cbor.FLOAT64.
                    o = as(d1(1:8), 'double');  % Atualiza o.
                    d1 = d1(9:end);  % Atualiza d1.
                elseif info == cbor.BREAK  % Testa a condição sobre info, cbor.BREAK.
                    % not used - will peek for 255 manually where needed
                    error('unexpected break');  % Interrompe a execução e indica a causa.
                else  % Trata o caso alternativo.
                    error('unrecognized simple value: %d', info);  % Interrompe a execução e indica a causa.
                end  % Fim do bloco.
            end  % Fim do bloco.
        end  % Fim do bloco.

        function [d] = encode(o,varargin)  % Define as entradas e saídas desta função.
            opts = cbor.getopts(varargin ...  % Atualiza opts.
            );  % Completa os dados da instrução anterior.
            HDR = @(major, info) cbor.encode({'@HDR', {}; major, info});  % Guarda a função a chamar mais tarde.
            tobytes = @(x) flip(typecast(x, 'uint8'));  % Reinterpreta os bytes no tipo pedido.

            % encoding of header with simple value (length, integers, etc...):
            if isa(o, 'cell') && isequal(size(o), [2, 2]) && strcmp(o{1,1}, '@HDR')  % Testa a condição sobre isa, o, cell, isequal, size, strcmp, HDR.
                major = o{2,1};  % Atualiza major.
                info = o{2,2};  % Atualiza info.
                if info < 0  % Testa a condição sobre info.
                    error('info value cannot be negative');  % Interrompe a execução e indica a causa.
                elseif info < 24 || major == cbor.SIMPLE_VALUE  % Testa a condição sobre info, major, cbor.SIMPLE_VALUE.
                    d = [uint8(info)];  % Atualiza d.
                elseif info < 2^8  % Testa a condição sobre info.
                    d = [cbor.UINT8, uint8(info)];  % Atualiza d.
                elseif info < 2^16  % Testa a condição sobre info.
                    d = [cbor.UINT16, tobytes(uint16(info))];  % Atualiza d.
                elseif info < 2^32  % Testa a condição sobre info.
                    d = [cbor.UINT32, tobytes(uint32(info))];  % Atualiza d.
                elseif info < 2^64  % Testa a condição sobre info.
                    d = [cbor.UINT64, tobytes(uint64(info))];  % Atualiza d.
                else  % Trata o caso alternativo.
                    error('info value too big: %d', info);  % Interrompe a execução e indica a causa.
                end  % Fim do bloco.
                d(1) = uint8( ...  % Atualiza d(1).
                    bitor( ...  % Completa os dados da instrução anterior.
                        bitshift(uint8(major), 5), ...  % Completa os dados da instrução anterior.
                        bitand(cbor.MASK_INFO, uint8(d(1))) ...  % Completa os dados da instrução anterior.
                    ) ...  % Completa os dados da instrução anterior.
                );  % Completa os dados da instrução anterior.
                return  % Regressa à função que fez a chamada.
            end  % Fim do bloco.

            % encode NaN as null
            if isnumeric(o) && numel(o) == 1 && isnan(o)  % Testa a condição sobre isnumeric, o, numel, isnan.
                d = [HDR(cbor.SIMPLE_VALUE, cbor.NULL)];  % Atualiza d.
                return  % Regressa à função que fez a chamada.
            end  % Fim do bloco.

            % array types
            l = numel(o);  % Conta os elementos.
            if l == 0  % Testa a condição sobre l.
                % An empty char vector is text, including API names like ''.
                if ischar(o)  % Testa a condição sobre ischar, o.
                    d = HDR(cbor.TEXT_STRING, 0);  % Atualiza d.
                else  % Trata o caso alternativo.
                    d = HDR(cbor.ARRAY, 0);  % Atualiza d.
                end  % Fim do bloco.
                return  % Regressa à função que fez a chamada.
            elseif (iscell(o) || l > 1) && isvector(o)  % Testa a condição sobre iscell, o, l, isvector.
                if isa(o, 'char')  % Testa a condição sobre isa, o, char.
                    % char vector is handled later as a string
                elseif isa(o, 'uint8')  % Testa a condição sobre isa, o, uint8.
                    % uint8 vector is used for binary data
                    d = [HDR(cbor.BYTE_STRING, l), o(:)'];  % Atualiza d.
                    return  % Regressa à função que fez a chamada.
                elseif iscell(o)  % Testa a condição sobre iscell, o.
                    % cell array encodes as array
                    d = [HDR(cbor.ARRAY, l)];  % Atualiza d.
                    for i=1:l; d = [d, cbor.encode(o{i})]; end  % Percorre os valores do índice i.
                    return  % Regressa à função que fez a chamada.
                else  % Trata o caso alternativo.
                    % regular vector encodes as array
                    d = [HDR(cbor.ARRAY, l)];  % Atualiza d.
                    for i=1:l; d = [d, cbor.encode(o(i))]; end  % Percorre os valores do índice i.
                    return  % Regressa à função que fez a chamada.
                end  % Fim do bloco.
            elseif l > 1  % Testa a condição sobre l.
                error('cannot encode nd-matrix');  % Interrompe a execução e indica a causa.
            end  % Fim do bloco.

            % basic types:
            if isa(o, 'struct')  % Testa a condição sobre isa, o, struct.
                % map
                n = fieldnames(o);  % Atualiza n.
                info = struct2cell(o);  % Atualiza info.
                l = numel(n);  % Conta os elementos.
                d = [HDR(cbor.MAP, l)];  % Atualiza d.
                for i=1:l; d = [d, cbor.encode(n{i}), cbor.encode(info{i})]; end  % Percorre os valores do índice i.
            elseif isa(o, 'single')  % Testa a condição sobre isa, o, single.
                if mod(o, 1) == 0; d = cbor.encode(int64(o)); return; end  % Testa a condição sobre mod, o.
                d = [HDR(cbor.SIMPLE_VALUE, cbor.FLOAT32), tobytes(o)];  % Atualiza d.
            elseif isa(o, 'double')  % Testa a condição sobre isa, o, double.
                if mod(o, 1) == 0; d = cbor.encode(int64(o)); return; end  % Testa a condição sobre mod, o.
                d = [HDR(cbor.SIMPLE_VALUE, cbor.FLOAT64), tobytes(o)];  % Atualiza d.
            elseif isa(o, 'integer')  % Testa a condição sobre isa, o, integer.
                if o < 0  % Testa a condição sobre o.
                    d = [HDR(cbor.NEGATIVE_INT, -1 - o)];  % Atualiza d.
                else  % Trata o caso alternativo.
                    d = [HDR(cbor.UNSIGNED_INT, o)];  % Atualiza d.
                end  % Fim do bloco.
            elseif isa(o, 'char')  % Testa a condição sobre isa, o, char.
                u = unicode2native(o, 'UTF-8');  % Atualiza u.
                l = numel(u);  % Conta os elementos.
                d = [HDR(cbor.TEXT_STRING, l), u];  % Atualiza d.
            elseif isa(o, 'string')  % Testa a condição sobre isa, o, string.
                d = cbor.encode(char(o));  % Atualiza d.
            elseif isa(o, 'logical')  % Testa a condição sobre isa, o, logical.
                if o  % Testa a condição sobre o.
                    d = [HDR(cbor.SIMPLE_VALUE, cbor.TRUE)];  % Atualiza d.
                else  % Trata o caso alternativo.
                    d = [HDR(cbor.SIMPLE_VALUE, cbor.FALSE)];  % Atualiza d.
                end  % Fim do bloco.
            else  % Trata o caso alternativo.
                error('unsupported type: %s', class(o));  % Interrompe a execução e indica a causa.
            end  % Fim do bloco.
        end  % Fim do bloco.

        function test()  % Define as entradas e saídas desta função.
            % encode/decode to hex-string:
            enc = @(o) sprintf('%02x', cbor.encode(o));  % Formata o texto.
            dec = @(s) cbor.decode(uint8(sscanf(s, '%02x')'));  % Guarda a função a chamar mais tarde.

            % like isequaln(), but can mix cell with num arrays
            isequalx = @(a,b) isequaln(a,b) ...  % Guarda a função a chamar mais tarde.
                || (iscell(a) && ~iscell(b) && isequaln(a,num2cell(b))) ...  % Completa os dados da instrução anterior.
                || (~iscell(a) && iscell(b) && isequaln(num2cell(a),b));  % Completa os dados da instrução anterior.

            T = @(a,b) assert(strcmpi(enc(a),b) && isequalx(a,dec(b)));  % Guarda a função a chamar mais tarde.

            T(false,'F4');  % Chama T com os parâmetros indicados.
            T(true,'F5');  % Chama T com os parâmetros indicados.
            T(NaN,'F6');  % Chama T com os parâmetros indicados.

            T(1,'01');  % Chama T com os parâmetros indicados.
            T(100,'1864');  % Chama T com os parâmetros indicados.
            T(1000,'1903E8');  % Chama T com os parâmetros indicados.
            T(1000000,'1A000F4240');  % Chama T com os parâmetros indicados.
            T(1000000000000,'1B000000E8D4A51000');  % Chama T com os parâmetros indicados.
            T(23,'17');  % Chama T com os parâmetros indicados.
            T(24,'1818');  % Chama T com os parâmetros indicados.
            T(32,'1820');  % Chama T com os parâmetros indicados.
            T(2^8,'190100');  % Chama T com os parâmetros indicados.
            T(2^16,'1A00010000');  % Chama T com os parâmetros indicados.
            T(2^32,'1B0000000100000000');  % Chama T com os parâmetros indicados.

            T(3.14159,'FB400921F9F01B866E');  % Chama T com os parâmetros indicados.

            T('abc','63616263');  % Chama T com os parâmetros indicados.

            T(struct('a', 10.52),'A16161FB40250A3D70A3D70A');  % Reúne os campos numa estrutura.
            T(struct('b', true),'A16162F5');  % Reúne os campos numa estrutura.
            T(struct('c', 'foo'),'A1616363666F6F');  % Reúne os campos numa estrutura.
            T(struct('a', 10.52, 'b', true, 'c', 'foo'),'A36161FB40250A3D70A3D70A6162F5616363666F6F');  % Reúne os campos numa estrutura.
            T(struct('x', struct('y', struct('z', 42))),'A16178A16179A1617A182A');  % Reúne os campos numa estrutura.

            T({1,2,3},'83010203');  % Chama T com os parâmetros indicados.
            T([1,2,3],'83010203');  % Chama T com os parâmetros indicados.

            T({1.23,4.56,7.89},'83FB3FF3AE147AE147AEFB40123D70A3D70A3DFB401F8F5C28F5C28F');  % Chama T com os parâmetros indicados.

            T(num2cell(2.^(0:16)),'9101020408101820184018801901001902001904001908001910001920001940001980001A00010000');  % Chama T com os parâmetros indicados.

            T(["a","b","c"],'83616161626163');  % Chama T com os parâmetros indicados.
            T({"a","b","c"},'83616161626163');  % Chama T com os parâmetros indicados.
            T({'a','b','c'},'83616161626163');  % Chama T com os parâmetros indicados.

            T({0,"b",true,-1,3.98},'85006162F520FB400FD70A3D70A3D7');  % Chama T com os parâmetros indicados.

            T(uint8([1,2,3]),'43010203');  % Chama T com os parâmetros indicados.
            T(uint8(zeros([1,32])),'58200000000000000000000000000000000000000000000000000000000000000000');  % Reserva memória inicializada a zero.

            T({true,false,false,true,false},'85F5F4F4F5F4');  % Chama T com os parâmetros indicados.
            T([true,false,false,true,false],'85F5F4F4F5F4');  % Chama T com os parâmetros indicados.

            T([NaN,NaN],'82F6F6');  % Chama T com os parâmetros indicados.

            T({1},'8101');  % Chama T com os parâmetros indicados.

            % testcases from https://en.wikipedia.org/wiki/Half-precision_floating-point_format#Half_precision_examples
            Td = @(a,b) assert(abs(a-dec(b))<1e-8);  % Guarda a função a chamar mais tarde.
            Td(1.5,'F93E00');  % Chama Td com os parâmetros indicados.
            Td(0,'F90000');  % Chama Td com os parâmetros indicados.
            Td(0.000000059604645,'F90001');  % Chama Td com os parâmetros indicados.
            Td(0.000060975552,'F903FF');  % Chama Td com os parâmetros indicados.
            Td(0.00006103515625,'F90400');  % Chama Td com os parâmetros indicados.
            Td(0.33325195,'F93555');  % Chama Td com os parâmetros indicados.
            Td(0.99951172,'F93BFF');  % Chama Td com os parâmetros indicados.
            Td(1,'F93C00');  % Chama Td com os parâmetros indicados.
            Td(1.00097656,'F93C01');  % Chama Td com os parâmetros indicados.
            Td(65504,'F97BFF');  % Chama Td com os parâmetros indicados.
            assert(inf==dec('F97C00'));  % Atualiza assert(inf=.
            assert(-inf==dec('F9FC00'));  % Atualiza assert(-inf=.
            Td(0,'F98000'); % -0
            Td(-2,'F9C000');  % Chama Td com os parâmetros indicados.

            % test for more than 24 items:

            T(struct('a', 1000, 'b', 1001, 'c', 1002, 'd', 1003, 'e', 1004, 'f', 1005, 'g', 1006, 'h', 1007, 'i', 1008, 'j', 1009, 'k', 1010, 'l', 1011, 'm', 1012, 'n', 1013, 'o', 1014, 'p', 1015, 'q', 1016, 'r', 1017, 's', 1018, 't', 1019, 'u', 1020, 'v', 1021, 'w', 1022, 'x', 1023, 'y', 1024, 'z', 1025),'B81A61611903E861621903E961631903EA61641903EB61651903EC61661903ED61671903EE61681903EF61691903F0616A1903F1616B1903F2616C1903F3616D1903F4616E1903F5616F1903F661701903F761711903F861721903F961731903FA61741903FB61751903FC61761903FD61771903FE61781903FF6179190400617A190401');  % Reúne os campos numa estrutura.

            % indefinite-length array and map items: (decode-only)

            assert(isequalx(struct('a', 1, 'b', 2), dec('BF616101616202FF')));  % Confirma o resultado esperado no teste.
            assert(isequalx([1,2,3], dec('9F010203FF')));  % Confirma o resultado esperado no teste.

            fprintf('All tests passed successfully.\n');  % Apresenta a medição ou o estado na consola.
        end  % Fim do bloco.
    end  % Fim do bloco.
end  % Fim do bloco.
