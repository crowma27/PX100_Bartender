# PX100 Bartender

## Executar a demonstração atual

1. Abre `Cenario_px100_vision.ttt` no CoppeliaSim e deixa a simulação parada.
2. No MATLAB, abre esta pasta como **Current Folder**. A demonstração usa Image Processing Toolbox, Computer Vision Toolbox e a ligação ZeroMQ do CoppeliaSim.
3. Executa `result = main_program();`. O programa abre os gráficos da deteção e a imagem em direto da câmara. Depois tenta pousar o copo vermelho no apoio lilás mais próximo. Cada execução trata de um copo.
4. Para escolher outra cor, usa `main_program('blue')` ou `main_program('green')`. O segundo argumento permite escolher o caminho exato de um apoio, tal como aparece na consola em «Apoio escolhido».

## Código para apresentar

| Ficheiro | O que explicar |
| --- | --- |
| `main_program.m` | Liga ao simulador, manda captar a imagem, escolhe o copo vermelho e chama a rotina de pega. |
| `vision/detectCupsPX100.m` | Usa a cor para localizar até um copo de cada cor e procura o rebordo escuro para estimar o centro. |
| `vision/runVisionCupsPX100.m` | Usa a calibração para cruzar o raio da câmara com o plano do topo do copo. Depois baixa metade da altura para obter o alvo da garra na base do PX100. |
| `pickConfigPX100.m` | Reúne as dimensões, tolerâncias e número de passos usados na pega. |
| `graspAndLiftPX100.m` | O `switch` organiza a aproximação, pega, subida, transporte, largada e retorno. A cinemática inversa (IK) calcula os ângulos de cada alvo e o programa espera que as juntas atinjam esses ângulos. |
| `findPlacementPX100.m` | Lê a posição dos apoios lilás fixos da cena e associa o copo detetado ao objeto `Cup` mais próximo para verificar o resultado da pega. |
| `vision/detectBlueStripePX100.m` e `vision/runVisionPX100.m` | Ensaio separado para detetar a faixa azul e estimar dois cantos. A projeção do ensaio usa a pose do marcador no CoppeliaSim para definir o plano. |
| `simulator_interface.m` e `kinematics/arm_interface.m` | Fazem a comunicação com o simulador, a leitura da câmara e o envio dos comandos do braço. |

Uma forma de explicar a demonstração é dizer **«A câmara dá-nos o centro do copo em píxeis. Com a calibração e a altura conhecida do copo, calculamos o ponto de pega em metros. A cinemática inversa dá-nos os ângulos de cada pose e só mudamos de estado quando as juntas chegam ao alvo.»**

## Estado do projeto

| Etapa | Estado nesta versão |
| --- | --- |
| Câmara e calibração | Captura de imagem e calibração guardada incluídas no projeto. |
| Visão dos copos | O código procura copos vermelhos, azuis e verdes. A cor predefinida é o vermelho. |
| Pega | O programa mede quanto o copo se deslocou antes de fechar a garra e confirma que ficou suspenso depois de subir. |
| Transporte e largada | Cada movimento comanda um alvo de juntas. Ao chegar ao apoio, o código confirma a posição medida do copo antes de largar. A garra recua e o braço volta aos ângulos iniciais. |
| Faixa azul e dispensador | A faixa já tem um ensaio próprio. Falta usá-la no programa principal para calcular os alvos do dispensador. |
| Serviço | Falta acionar o botão ou a alavanca, representar o enchimento na simulação e devolver o copo à posição inicial. |

O código do transporte foi revisto com o analisador estático do MATLAB. A execução do transporte na cena do CoppeliaSim ficou por verificar.

## Próxima versão

1. Guardar a posição inicial do copo detetado para o devolver no fim.
2. Usar os dois cantos da faixa como referência do dispensador. Com o dispensador nivelado e a altura da faixa conhecida, projetar esses cantos na base do braço e medir, uma vez, os deslocamentos da faixa até ao apoio e ao botão de cada bebida. Testar se o detetor distingue a faixa do copo azul e do cubo azul.
3. Completar primeiro o ciclo com o copo vermelho e uma bebida. Confirmar o alcance do braço para pousar, premir o botão, retomar a pega e devolver o copo.

O gráfico de alcance da pega vertical e o ensaio da faixa azul são ferramentas de estudo independentes.
