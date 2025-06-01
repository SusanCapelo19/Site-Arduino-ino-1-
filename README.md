# Site-Arduino-ino--1-
Site para documentação do projeto de Arduino - Analisador de Frequancia de Audio

MICROFONE

MAX 4466
Potenciômetro do microfone controla a resistência, que determina a quantidade de ganho (o quanto o módulo amplifica o sinal)
Rodar o potenciômetro anti-horário aumenta o ganho
Alimentamos o microfone com 3.3V de tensão (um valor mais baixo indicado pelo fabricante)
A saída do módulo é um sinal de variação de tensão (voltagem)
A medição de vários valores nos permite descobrir os maiores e menores valores da voltagem (amplitude peak-to-peak)

Conexões
O Arduino alimenta o microfone com 3.3V. O ground (terra) do microfone se conecta com o ground do Arduino. A saída do microfone se conecta com a entrada analógica A0 do Arduino.
3.3V -> VCC
GND -> GND
A0 -> AO

Funcionamento microfones eletreto

Dentro do microfone, há um diafragma e uma placa traseira. O diafragma é uma membrana móvel que vibra com o som. Juntos, a placa e o diafragma agem como um capacitor, um componente que acumula carga elétrica.
A vibração do ar faz a capacitância entre o diafragma e a placa variar, o que gera uma mudança de tensão. Interpretamos esta mudança de tensão como a amplitude da onda sonora.

Como a voltagem de saída é muito pequena, usamos o módulo de microfone, que contém uma placa que amplifica essa voltagem.
Além disso, o módulo polariza a voltagem de saída, fazendo com que a tensão varie a partir de um certo valor. Este valor é exatamente metade da tensão de alimentação: VCC/2. Assim, como usamos 3.3V como fonte de alimentação, a saída do módulo está centralizada em 3.3V÷2 = 1.65V.

(Se você se lembra das aulas de funções periódicas, isso é equivalente a somar/subtrair um constante à função, ou seja, deslocar a onda verticalmente.)

Isso é especialmente útil, pois o conversor ADC (analógico->digital) do Arduino lê apenas sinais de 0V a 5V.

Assim, se as ondas do nosso microfone estivessem centradas em 0V, todos os sinais abaixo de 0V não seriam lidos corretamente pelo Arduino.

Recebendo nosso sinal analógico, o Arduino converte a voltagem para um valor digital entre 0 e 1023 (1 byte).

Caso não tivéssemos o módulo, poderíamos polarizar a saída usando um divisor de tensão. Segundo a fórmula, voltagem de saída = (R2÷(R1+R2))*voltagem de entrada

Com dois resistores de 100k ohms, precisaríamos alimentar o circuito com 3.3V, o que retornaria uma saída com metade da tensão original (3.3÷2=1.65).

CIRCUITO PARA POLARIZAR

Uma consequência dessa polarização é que o sinal que entra no pino de entrada do Arduino tem 1.65V mesmo quando não há som (variação de tensão). Assim, o ADC retorna sempre o ponto médio (1023÷2 ≈ 512).

Além disso, temos que considerar que, por padrão, o Arduino usa uma referência de voltagem de 5V para o ADC. Ou seja, uma entrada de 5V retorna 1023 e uma entrada de 0V retorna 0.

Como estamos trabalhando com uma entrada de 3.3V, dizemos à placa do Arduino que nossa referência vai ser esta voltagem, e não o padrão de 5V. Para isso, enviamos ao pino AREF do Arduino o próprio sinal de 3.3V que o Arduino nos fornece.



LEDs
Configurações
Bibliotecas: MD_MAX72xx e SPI
Tipo do nosso hardware: FC16_HW
MAX DEVICES = 4 módulos
D13 -> CLK
D10 -> CS
D11 -> DIN
GND -> GND
5V -> 5V

Notação:
(n,m) = n⁰ linha e m⁰ coluna
Para acender os LEDs, vamos iterar as 32 colunas e para cada coluna acender a quantidade apropriada de linhas.
Cada coluna recebe um byte para indicar a quantidade de linhas a serem acendidas. Por exemplo, 127 = 0b01111111 = todos os LEDs acesos exceto o último. 0b00000000 = nenhuma LED da coluna é aceso.

Por exemplo, para acender todas as linhas da coluna 2. Podemos usar:

mx.setColumn(2, 0b11111111);

Nosso display é uma matriz 8x32 de LEDs. Cada quadrado 8x8 é controlado por um módulo MAX7219, que é um circuito integrado projetado para o controle de LEDs.

Cada saída do circuito integrado está conectada com a entrada do próximo circuito, totalizando 4 módulos que controlam cada um 64 LEDs.

Com o MAX7219, não precisamos conectar cada LED separadamente e podemos manipular a matriz de forma serial.

Os displays poderiam ser ativados enviando dados de forma paralela para cada coluna ou linha. O circuito integrado, contudo, nos permite enviar os comandos de forma serial e ter como saída dados em paralelo.

Os pinos do módulo funcionam assim:
VCC: fonte de alimentação; recebe 5V
GND: ground (terra)
DIN: pino de entrada digital (recebe sinais do Arduino)
CS: seletor de chip (determina quando um chip se comunica com o Arduino)
CLK: entrada de sinal de relógio (para sincronizar o Arduino e os módulos)

O protocolo Serial Peripheral Interface (SPI) serve como meio de comunicação rápido entre um microcontrolador e outros componentes. Usamos os pinos 13 (SCLK), 11 (COPI) e 10 (CS) do Arduino para nos comunicarmos em série com os módulos MAX7219.

Além disso, vamos usar uma biblioteca de Arduino para controlar os módulos MAX7219 (MD_MAX72xx). Assim, controlar cada LED individualmente fica bem mais fácil.



FFT


Vamos usar no projeto uma biblioteca para fazer a transformada rápida de Fourier (FFT), a arduinoFFT. Baixamos a versão 3.5 (de 2019), já que vários projetos de referência usaram essa versão (e as versões mais novas capitalizam os nomes das classes de forma diferente).

Sem conhecimento profundo em séries, vou tentar explicar o algoritmo (simplificando e errando, mas faz parte).

A série de Fourier transforma uma função periódica numa soma de senos e cossenos com períodos diferentes.

A transformada de Fourier é uma transformação linear (ai, ai, álgebra linear, nos encontramos novamente) que relaciona o comportamento de uma função em relação ao tempo com o seu comportamento em relação à frequência.

Lidando com valores contínuos, a transformada faz a integral de coeficientes para representar uma função em termos de valores sinusoidais (aí que entra os coeficientes complexos e a fórmula de Euler).

Para não precisarmos lidar com valores contínuos (analógicos), fazemos a transformada discreta de Fourier. Ela relaciona uma faixa de frequências com amostras (finitas) de som. 

Cada faixa de frequências é o resultado do produto da somatória de cada frequência e valores complexos. O tamanho de cada faixa de frequências é determinado pelo número de amostras e as frequências analisadas.

A transformada rápida de Fourier (finalmente!) é uma forma de fazer esses cálculos com computadores. Ela divide as somatórias das amostras em duas (as de amostras pares e ímpares), o que acelera o processo.

Usando a fórmula de Euler, expandimos cada amostra no produto entre a própria amostra e uma soma de senos e cossenos. 

A parte real desses valores nos diz o quão bem os sinais da amostra correspondem ao cosseno de determinada frequência. Já a parte imaginária faz o mesmo, mas com o seno de uma frequência.

No algoritmo, os coeficientes complexos nos ajudam a formar pontos no plano imaginário, o que nos dá informações sobre a amplitude e a fase dos sinais analisados.

Usando os vetores reais e imaginários, formamos um triângulo retângulo. Aplicando pitágoras, voilá, a magnitude deste terceiro vetor formado entre o vetor real e o imaginário nos diz a intensidade de uma determinada frequência no sinal analisado.

No entanto, informações sobre as frequências são limitadas pela frequência de amostragem (quantas amostras num determinado tempo) que usamos.

O limite de Nyquist nos diz que podemos analisar frequências que sejam até a metade da frequência de amostragem.

Assim, por exemplo, se capturamos amostras com uma frequência de 8000 Hz, a maior frequência que podemos detectar é 8000÷2 = 4000 Hz.

Aqui, nós nos deparamos com uma limitação técnica. A função de leitura de valores analógicos do Arduino — analogRead() — demora 3 ciclos de relógio para ser feita (na verdade, demora um pouco mais). Além disso, o relógio do Arduino tem uma frequência de 16 MHz.

Com isso, a maior frequência de amostragem que conseguimos está em torno de 9kHz. Aplicando Nyquist, descobrimos que a frequência máxima que podemos analisar é em torno de 9÷2 = 4.5kHz.

A audição humana, na sua maior glória, percebe de 20Hz a 20kHz. Então temos um valor razoável.

Quanto à precisão, sabemos que o tamanho de cada faixa de frequências é determinado pela divisão entre a taxa de amostragem e o número de amostras.

Por exemplo, capturando 128 amostras com uma frequência de 9kHz, temos 9000÷128 ≈ 70Hz. Ou seja, cada faixa tem uma largura aproximada de 70Hz.

Já o número de faixas (FFT frequency bins) é dado pela metade do número de amostras. Com 128 amostras, temos 64 faixas com comprimento de 70Hz cada uma.


Sintaxe C

#include
importa arquivo de uma biblioteca para um sketch

#define
escopo global
sem type checking
não tem memória alocada
criada durante preprocessing (antes do compilador)

vetor
tipo nomeVetor[] = {elementos, …};
Para determinar o tamanho:
tipo nomeVetor[tamanho];
int meuVetor[2];

float
4 bytes, até 7 dígitos
double
8 bytes, até 15 dígitos
long
int de 32 bits
uint8_t
int de 8 bits, unsigned (apenas positivos)
const
declara que o valor de uma variável não vai mudar
diferentemente de #define, aloca espaço para esta variável

função
tipo_saida nomeFuncao(tipo_parametro parametro, …){
corpo_funcao;
return
}

for loop
for (inicializacao_variaveis; condicao; mudar_variaveis) {
… }
Incrementar em 1 (++)
Diminuir em 1 (--)

if em uma só linha
if (condicao) operacoes;
else if (condicao) operacoes;

Algumas funções de Arduino
Serial.println() - printar no terminal
delay(n) - esperar n microssegundos
constrain(num, limite_inferior, limite_superior) - restringe um valor a um intervalo
map(valor, inferior_original, superior_original, novo_inferior, novo_superior) - remapeia um valor dentro de um intervalo para um novo valor dentro de outro intervalo
micros() – número de microsegundos a partir da execução do programa (reseta em 70min)



RESUMO DO CÓDIGO


Vamos usar uma taxa de amostragem de 10kHZ e analisaremos 64 amostras por vez (mais que isso e o Arduino chora). 

Os nomes globais que vamos usar:
vReal e vImag – vetores para armazenar os 64 valores reais e os 64 valores imaginários
valorFinalColunas – vetor para armazenar a magnitude média de cada uma das 32 colunas
picos – vetor para armazenar a magnitude máxima (pico) de cada coluna
colunasSuavizadas – vetor para armazenar a média ponderada entre o valor anterior e o novo valor da coluna
ganhoDinamico – variável para armazenar o fator multiplicativo de amplificação
padraoLinhas – vetor para ajudar a iluminar as linhas de cada coluna
mx – instância de uma classe para manipular nosso display
FFT – instância de uma classe para realizar o FFT
entradaAnterior e saidaAnterior – variáveis para realizar o filtro high-pass

Módulo MAX7219

Nosso display tem 32 colunas e 8 linhas.
Para iluminar a quantidade certa de linhas de cada coluna, vamos usar bytes para representar a quantidade certa de linhas:
0 = 0b00000000 (nenhuma linha)
1 = 0b00000001 (primeira linha)
3 = 0b00000011 (primeira e segunda linha)
…
255 = 0b1111111 (todas as linhas)

Filtro high-pass

Os filtros high-pass (HPF) são filtros de frequência que atenuam frequências menores que uma determinada frequência (cutoff frequency). Assim, no nosso caso, por exemplo, frequências abaixo de 100Hz são atenuadas pelo filtro.
Escolhemos usar um filtro, pois o microfone estava captando muito ruído no low-end (provavelmente devido à má conexão com o GND).

Os parâmetros do filtro entradaAnterior e saidaAnterior são variáveis globais, pois precisamos que os seus valores sejam acessados pela função highPass() e atualizados a cada chamada da função.

A função funciona como um filtro digital baseado num circuito Resistor-Capacitor (RC) analógico. Para os valores do nosso filtro, usamos a fórmula:

freq. cutoff = 12RC

Sabemos que a nossa frequência de cutoff é 100Hz. Assim, podemos determinar o valor adequado para o produto da resistência e da capacitância (RC).

RC=12100

Sabendo RC, podemos reconstruir a fórmula de um filtro RC, que relaciona as tensões de entrada e saída em função do tempo:

Vout(t) = RCd(Vin(t))dt

Em que d(Vin(t))/dt é a derivada da voltagem em função do tempo.

Como estamos usando computadores e precisamos lidar com valores discretos, aproximamos essa derivada:

d(Vin(t))dt[Vin(t) - Vin(t-1)]dt

Substituindo esta forma simplificada na equação, temos:

Vout(t) = RC[Vin(t) -Vin(t-1)]dt-Vout(t)

Manipulando a equação, definimos um fator alfa: 

=RC(RC+dt)

Adaptando a equação para valores discretos, temos:

Vout(t)= [((Vin(t)-Vin(t-1))+Vout(t-1)]


Alfa é uma valor entre 0 e 1 que determina a força do filtro. Quanto mais perto de 1, menos agressivo é o filtro.
[Obs. Se o filtro fosse um LPF, alfa seria dt/(RC+dt)]

Somando a saída anterior [Vout(t-1)] à mudança entre duas amostras de voltagem [Vin(t) - Vin(t-1)], preservamos o sinal que já foi filtrado e evitamos que o filtro acumule energia a cada iteração (já que o fator alfa vai atenuar a saída a cada chamada).

Sabendo RC, basta definirmos dt, que será o tempo entre cada amostra. Com uma taxa de amostragem de 10kHz, temos 10k amostras por segundo ou 1/10k = 0.0001 amostra por segundo. Passando para microssegundo, temos dt = 100 microssegundos.

Finalmente, interpretamos (Vin(t)-Vin(t-1) como a quantidade de mudança entre uma amostra e outra. Uma intuição para entender porque isso funciona para um HPF é perceber que sinais lentos (frequências baixas) causam pouca mudança entre duas amostras quaisquer.

No final da função, atualizamos as últimas entradas e saídas para a próxima amostra, já que a saída de cada amostra depende dos valores da amostra anterior.

Setup

Aqui determinamos que o pino A0 vai ser a entrada analógica e que o pino AREF vai receber a voltagem de referência para o ADC. Inicializamos a istância do display, definimos a intensidade do brilho e resetamos o display.

Loop

Primeiro, inicializamos as variáveis que armazenam o tempo de amostragem e o tempo percorrido – micros() retorna o tempo percorrido em microssegundos desde a inicialização do programa. Como já falamos, uma taxa de amostragem de 10kHz determina um tempo de 100 microsegundos para cada amostra.

Loop interno (Captura de amostras)

Para respeitarmos o tempo de cada amostra, usamos um while loop para esperar um pouco caso não seja a hora de capturar uma amostra. O loop funciona ao comparar o tempo passado (diferença entre agora e o tempo da última medição).

Se este tempo for menor que o tempo de amostragem (100 microssegundos), o programa espera (já que não queremos captar mais/menos amostras do que a quantidade prevista, que é de 10 mil amostras por segundo).

Em seguida, o loop:
Atualiza a nova medição (que indica qual foi a última vez em que o loop fez uma amostra)
Lê o sinal analógico de A0
Retira a polarização da voltagem do microfone (se lembra de que o microfone retorna um valor próxima de 512, mesmo sem som?)
Filtra os sinais graves da amostra
Salva o sinal filtrado no vetor de valores reais (e salva 0 no vetor de valores imaginários)


Além disso, ao passar o sinal centrado para a função highPass, dividimos o valor por uma constante (8, no nosso caso). Isso porque
Usamos números menores ao trabalhar com floats (gastamos menos memória).
O FFT não precisa de voltagens absolutas, já que calcula o conteúdo de frequências de forma relativa. Se dividirmos todas as amostras por um mesmo valor, o FFT ainda funciona.
Quanto maior os valores da voltagem, mais responsivo é o FFT (pode retornar mais ruídos).


FFT

Windowing function: dá pesos diferentes para partes do sinal de cada amostra dependendo da posição da amostra no vetor de valores reais.
Quando um sinal não é periódico dentro da janela de amostragem, ocorrem distorções na FFT.


A janela suaviza as bordas do sinal para reduzir essas distorções.
FFT_FORWARD: aplica janela (obs. Reverse remove janela)
FFT.Compute(vReal, vImag, AMOSTRAS, FFT_FORWARD);
Calcula as partes real e imaginária de cada componente de frequência. Usa algoritmo Cooley-Tukey.
FFT.ComplexToMagnitude(vReal, vImag, AMOSTRAS);
Calcula a magnitude (ou módulo) de cada componente de frequência, baseado nos valores complexos.
Hamming window: limpa onda seno de sinais laterais (ruídos devido a onda seno ser representada por dados finitos).

O FFT retorna as magnitudes das frequências no vetor de valores reais vReal.


Processando valores do FFT

Vamos trabalhar agora com os dados de retorno do FFT. Calculamos os grupos de frequências e resetamos o valor total da amostragem.

Depois de rodar o FFT, o algoritmo nos retorna N/2 grupos de frequências, em que N é o número de amostras. Como trabalhamos com 64 amostras, o FFT nos devolve 32 grupos de frequências.

Cada grupo representa a magnitude de uma faixa de frequências (por ex., magnitude das frequências de 0-20Hz).

Com a taxa de amostragem e a quantidade de amostras, podemos achar o tamanho de cada grupo de frequências.

grupo frequência=taxa de amostragemnúmero de amostras=1000064=156.25Hz

Assim, temos 32 grupos de frequências, cada uma com 156.25Hz. Como temos 32 colunas de LEDs, há a feliz coincidência de que cada coluna corresponde a um grupo de frequências! :)

Calculando o tamanho de cada faixa de frequências, (64/2)/32 = 1. Ou seja, cada coluna corresponde a um grupo de frequências do FFT.

Loop interno 2 (Procressamento)

O primeiro loop itera todas as colunas (c) e, para cada iteração, passamos também por um grupo de frequências 
((64/2)/32 = 1). 

Assim, para cada coluna, a gente soma os valores dos grupos de frequências (no nosso caso, há uma coluna para cada grupo)* e divide essa soma pela quantidade de grupos, fazendo assim a média dos valores do grupo.

Por exemplo: se em vez de uma coluna para cada grupo tivéssemos uma coluna para cada dois grupos, o código iria:
iterar cada coluna,
somar os dois grupos de frequências de cada coluna
e dividir pelo número de grupos (2).
Assim, cada coluna receberia a média dos valores dos seus grupos.

O segundo loop é quem faz a soma dos grupos de frequências: soma += vReal[i + k];

Aqui, i determina o primeiro grupo de frequências de cada coluna e k determina o tamanho de cada coluna. Como nós temos apenas um grupo de frequências por coluna, o loop interno roda apenas uma vez.

Mas é interessante entender o funcionamento para outros casos. Suponhamos que, como no exemplo anterior, temos 2 grupos de frequências para cada coluna. Então o loop funcionaria assim:
primeira coluna, primeiro grupo – vReal[0+0] (vReal[0])
primeira coluna, segundo grupo – vReal[0+1] (vReal[1])
média primeira coluna é igual a (vReal[0] + vReal[1])/2
segunda coluna, primeiro grupo – vReal[1+0] (vReal[1])
segunda coluna, segundo grupo – vReal[1+1] (vReal[2])
média segunda coluna é igual a (vReal[1] + vReal[2])/2
…e assim por diante até a 32º coluna

*O que torna o nosso loop interno um pouco redundante, mas útil caso usássemos outra quantidade de amostras.

Suavizar valores das faixas

Para suavizar as colunas, armazenamos dentro de um vetor os valores da magnitude da faixa, sendo que 30% desse valor corresponde ao valor médio dos grupos de frequências e os 70% restantes ao valor anterior (o valor inicial foi definido como 0).

Ao misturar o valor médio dos grupos de frequências com o valor anterior, deixamos a transição entre valores mais suaves a cada loop principal do programa. Os valores de 0.7 e 0.3 são arbitrários, mas funcionaram bem com o Arduino Blackboard e nossa taxa de amostragem.

Depois, vamos aplicar um ganho para aumentar os valores das magnitudes de cada coluna. Multiplicamos o valor suavizado da faixa por um fator de aumento (ganhoDinamico).

Restringimos esse valor para um valor entre 0 e 80 (já que a matriz tem 8 linhas).
Com esses números entre 0 e 80, mapeamos para valores entre 0 e 8, que vão representar a quantidade de LEDs acesas. (Esta conversão transforma os valores decimais de colunaComGanho para valores inteiros entre 0 e 8.)

Finalmente, vamos adicionar este valor a uma variável que armazena a quantidade total de intensidade de todas as colunas.

Ganho dinâmico

No final de uma amostragem, calculamos o level (altura) médio de todas as colunas.
Como a amplitude total é de 8, usamos 3 e 5 como valores arbitrários para servir de referência.
Se o valor for acima de 5, diminuímos levemente o ganho em 0.05.
Se o valor for abaixo de 3, aumentamos levemente o ganho em 0.05.

O ganho inicial é de 2 (amplificação de duas vezes), e os valores possíveis estão entre 1 (nenhuma amplificação) e 5 (amplificação de cinco vezes).

Depois de calcular a intensidade total de todas as colunas de uma amostragem completa, atualizamos o ganho para a próxima amostragem, restringindo=o para um valor entre 1 e 5.

Atualizando o display

Com uma amostragem completa, vamos atualizar o display. Primeiro resetamos o display. Iteramos cada uma das 32 colunas. Para cada coluna, pegamos o valor médio dela e salvamos numa variável.
Agora atualizamos os picos:
Se o pico da coluna c for maior que zero, diminuímos o pico em 1 (picos[c]--).*
Se média atual for maior que o pico anterior, atualizamos o pico.
Usamos o pico da coluna para iluminar a quantidade certa de linhas.
Por fim, atualizamos o display para exibir o valor.

*O pico de cada coluna cai gradualmente. Cada vez que o código completa um loop, diminuímos o valor do pico. Isso faz com que a altura da coluna vá diminuindo LED por LED (caso os novos loops não atualizem/aumentem o valor do pico). 



CÓDIGO

#include <arduinoFFT.h>
#include <MD_MAX72xx.h>
#include <SPI.h>

#define AMOSTRAS 64
#define TIPO_HARDWARE MD_MAX72XX::FC16_HW
#define NUM_MODULOS 4
#define CLK_PIN 13
#define DATA_PIN 11
#define CS_PIN 10
#define coluna 32
#define linha 8

double vReal[AMOSTRAS];
double vImag[AMOSTRAS];

uint8_t valorFinalColunas[coluna];
uint8_t picos[coluna] = {0};
float colunasSuavizadas[coluna] = {0};

float ganhoDinamico = 2.0;
const uint8_t padraoLinhas[] = {0, 1, 3, 7, 15, 31, 63, 127, 255};

MD_MAX72XX mx = MD_MAX72XX(TIPO_HARDWARE, CS_PIN, NUM_MODULOS);
arduinoFFT FFT = arduinoFFT();

float entradaAnterior = 0;
float saidaAnterior = 0;

float highPass(float entrada, float cutoff, float taxaAmostragem) {
  float RC = 1.0 / (2.0 * 3.1416 * cutoff);
  float dt = 1.0 / taxaAmostragem;
  float alfa = RC / (RC + dt);
  float saida = alfa * (saidaAnterior + (entrada - entradaAnterior));
  entradaAnterior = entrada;
  saidaAnterior = saida;
  return saida;
}


void setup() {
  pinMode(A0, INPUT);
  analogReference(EXTERNAL);
  mx.begin();
  mx.control(MD_MAX72XX::INTENSITY, 5);
  mx.clear();
  delay(50); // espera um pouco para o Arduino ler a voltagem de referência
}


void loop() {
  const unsigned long tempoAmostragem = 100;
  unsigned long ultimaMedicao = micros();

  for (int i = 0; i < AMOSTRAS; i++) {
	while (micros() - ultimaMedicao < tempoAmostragem);
	ultimaMedicao = micros();
	int sinalAnalogico = analogRead(A0);
	float sinalCentralizado = sinalAnalogico - 509.0;
	float sinalFiltrado = highPass((sinalCentralizado / 8.0), 100.0, 10000.0);
	vReal[i] = sinalFiltrado;
	vImag[i] = 0;
  }


  FFT.Windowing(vReal, AMOSTRAS, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, AMOSTRAS, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, AMOSTRAS);


  int gruposPorColuna = (AMOSTRAS / 2) / coluna;
  float intensidadeTotal = 0;

  for (int i = 0, c = 0; c < coluna; i += gruposPorColuna, c++) {
	float soma = 0;
	for (int k = 0; k < gruposPorColuna; k++) {
  	soma += vReal[i + k];
	}

	float mediaGrupos = soma / gruposPorColuna;
	colunasSuavizadas[c] = (colunasSuavizadas[c] * 0.7) + (mediaGrupos * 0.3);

	float colunaComGanho = colunasSuavizadas[c] * ganhoDinamico;
	colunaComGanho = constrain(colunaComGanho, 0, 80);
	valorFinalColunas[c] = map(colunaComGanho, 0, 80, 0, linha);
	intensidadeTotal += valorFinalColunas[c];
  }

  float intensidadeMedia = intensidadeTotal / coluna;
  if (intensidadeMedia < 3) ganhoDinamico += 0.05;
  else if (intensidadeMedia > 5) ganhoDinamico -= 0.05;
  ganhoDinamico = constrain(ganhoDinamico, 1.0, 5.0);


  mx.clear();


  for (int c = 0; c < coluna; c++) {
  	if (picos[c] > 0) picos[c]--;
  	uint8_t altura = valorFinalColunas[c];
  	if (altura > picos[c]) picos[c] = altura;
  	mx.setColumn(c, padraoLinhas[picos[c]]);
  }

  mx.update();
}

