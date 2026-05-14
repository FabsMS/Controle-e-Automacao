# Resumo Teórico – Etapa 01

**Disciplina:** Controle e Automação I
**Instituição:** IFPB – Campus Campina Grande
**Curso:** Engenharia da Computação
**Professor:** Dr. Moacy Pereira da Silva
**Autores:** Fabrício Moreno da Silva · Ynnayron Juan Lopes da Silva

---

## Sumário

1. [Introdução a Sistemas Dinâmicos](#1-introdução-a-sistemas-dinâmicos)
2. [Função de Transferência](#2-função-de-transferência)
3. [Polos e Zeros](#3-polos-e-zeros)
4. [Estabilidade](#4-estabilidade)
5. [Resposta Temporal](#5-resposta-temporal)
6. [Aplicação Prática: Motor CC (PBL)](#6-aplicação-prática-motor-cc-pbl)
7. [Referências](#7-referências)

---

## 1. Introdução a Sistemas Dinâmicos

Um **sistema dinâmico** é um sistema cujo comportamento depende do tempo e cujas variáveis internas evoluem em resposta a sinais de entrada. Em engenharia de controle, o objetivo é **modelar matematicamente** esse comportamento para prever, analisar e, quando necessário, **manipular** a resposta do sistema de modo que ele atenda a especificações de desempenho.

A representação mais comum de um sistema dinâmico linear e invariante no tempo (LTI) é por meio de uma **equação diferencial ordinária (EDO)** de coeficientes constantes:

$$
a_n \frac{d^n y(t)}{dt^n} + a_{n-1} \frac{d^{n-1} y(t)}{dt^{n-1}} + \cdots + a_0 \, y(t) \;=\; b_m \frac{d^m u(t)}{dt^m} + \cdots + b_0 \, u(t)
$$

onde $u(t)$ é a entrada (excitação) e $y(t)$ é a saída (resposta). Essa EDO descreve completamente a dinâmica do sistema, mas sua análise direta é trabalhosa — especialmente para sistemas de ordem elevada. Por isso, a disciplina se apoia na **transformada de Laplace**, que converte equações diferenciais em equações algébricas, simplificando drasticamente a análise.

### 1.1 Classificação básica

Os sistemas podem ser classificados segundo diversas propriedades estruturais:

- **Linearidade:** obedece ao princípio da superposição e da homogeneidade.
- **Invariância no tempo:** os coeficientes da EDO não variam com o tempo.
- **Causalidade:** a saída em um instante depende apenas da entrada presente e passada.
- **Estabilidade:** a saída permanece limitada para qualquer entrada limitada (BIBO).

Nesta etapa tratamos exclusivamente de **sistemas LTI causais**, que constituem a base da teoria clássica de controle (Ogata; Nise; Dorf & Bishop).

---

## 2. Função de Transferência

### 2.1 Definição

A **função de transferência** $G(s)$ de um sistema LTI é definida como a razão entre a transformada de Laplace da saída e a transformada de Laplace da entrada, **considerando condições iniciais nulas**:

$$
\boxed{\; G(s) \;=\; \frac{Y(s)}{U(s)} \;\Bigg|_{\text{C.I.} = 0} \;}
$$

Aplicando-se a transformada de Laplace à EDO geral da seção anterior, obtém-se:

$$
G(s) \;=\; \frac{Y(s)}{U(s)} \;=\; \frac{b_m s^m + b_{m-1} s^{m-1} + \cdots + b_1 s + b_0}{a_n s^n + a_{n-1} s^{n-1} + \cdots + a_1 s + a_0}
$$

O polinômio do numerador é $N(s)$; o do denominador é $D(s)$. A condição $n \geq m$ caracteriza sistemas **próprios** (a imensa maioria dos sistemas físicos reais).

### 2.2 Forma polinomial e forma fatorada

A função de transferência pode ser apresentada em duas formas equivalentes:

$$
G(s) \;=\; \frac{N(s)}{D(s)} \;=\; K \cdot \frac{(s - z_1)(s - z_2)\cdots(s - z_m)}{(s - p_1)(s - p_2)\cdots(s - p_n)}
$$

onde:

- $z_i$ são as raízes de $N(s) = 0$, chamadas **zeros** do sistema;
- $p_i$ são as raízes de $D(s) = 0$, chamadas **polos** do sistema;
- $K$ é um ganho constante.

A equação $D(s) = 0$ é conhecida como **equação característica** e é central para a análise de estabilidade.

### 2.3 Significado físico

A função de transferência descreve completamente o comportamento **entrada-saída** de um sistema LTI em condições nulas. Ela não contém informação sobre condições iniciais ou estados internos, mas permite prever a resposta a qualquer entrada por meio da relação:

$$
Y(s) \;=\; G(s) \cdot U(s) \quad \Longleftrightarrow \quad y(t) \;=\; g(t) * u(t)
$$

onde $g(t) = \mathcal{L}^{-1}\{G(s)\}$ é a **resposta ao impulso** e "$*$" denota convolução temporal.

### 2.4 Exemplos fundamentais

- **Sistema de 1ª ordem:** $\;G(s) = \dfrac{K}{\tau s + 1}$, onde $\tau$ é a constante de tempo.
- **Sistema de 2ª ordem:** $\;G(s) = \dfrac{K\,\omega_n^2}{s^2 + 2\zeta\omega_n s + \omega_n^2}$, onde $\omega_n$ é a frequência natural e $\zeta$ é o fator de amortecimento.

Esses dois modelos cobrem uma parcela expressiva dos sistemas de interesse prático e serão retomados nas simulações.

---

## 3. Polos e Zeros

### 3.1 Definições

Os **polos** de $G(s)$ são os valores complexos de $s$ que tornam $D(s) = 0$, ou seja, que levam $|G(s)| \to \infty$. Os **zeros** de $G(s)$ são os valores de $s$ que tornam $N(s) = 0$, ou seja, que levam $G(s) = 0$.

Formalmente:

$$
p_i \;:\; D(p_i) = 0, \qquad z_j \;:\; N(z_j) = 0
$$

### 3.2 Interpretação no plano complexo

Polos e zeros podem ser representados no **plano s** (plano complexo), em que o eixo horizontal é a parte real ($\sigma$) e o eixo vertical é a parte imaginária ($j\omega$). Por convenção:

- polos são marcados com o símbolo **×**;
- zeros são marcados com o símbolo **○**.

A localização dos polos dita o comportamento dinâmico do sistema, enquanto os zeros afetam a magnitude e a forma da resposta transitória (mas não determinam estabilidade).

### 3.3 Influência dos polos na resposta temporal

Considere um polo genérico $p = \sigma + j\omega$. A contribuição desse polo na resposta temporal é proporcional a $e^{\sigma t}\cos(\omega t + \phi)$. Daí decorrem os seguintes casos:

| Localização do polo | Resposta no tempo |
|---|---|
| Real negativo ($\sigma < 0$, $\omega = 0$) | Decaimento exponencial |
| Real positivo ($\sigma > 0$, $\omega = 0$) | Crescimento exponencial |
| Complexo no SPE ($\sigma < 0$, $\omega \neq 0$) | Oscilação amortecida |
| Complexo no SPD ($\sigma > 0$, $\omega \neq 0$) | Oscilação divergente |
| Sobre o eixo imaginário ($\sigma = 0$, $\omega \neq 0$) | Oscilação sustentada (marginal) |
| Na origem ($\sigma = 0$, $\omega = 0$) | Integrador puro |

Esse mapeamento está ilustrado na **Simulação 2**.

### 3.4 Zeros e seu efeito

Os zeros não afetam a estabilidade, mas modificam a amplitude das parcelas modais na resposta. Zeros no semiplano direito (SPD) caracterizam sistemas de **fase não mínima**, que apresentam resposta inicial no sentido oposto ao valor final — comportamento típico, por exemplo, em conversores boost e em algumas plantas térmicas (Ogata; Nise).

---

## 4. Estabilidade

### 4.1 Definição BIBO

Um sistema LTI é **BIBO-estável** (*Bounded Input, Bounded Output*) se, para toda entrada limitada, a saída é também limitada. Matematicamente:

$$
\forall \, u(t):\; |u(t)| \leq M_u < \infty \;\Longrightarrow\; |y(t)| \leq M_y < \infty
$$

### 4.2 Critério dos polos

Para sistemas LTI representados por função de transferência, a condição **necessária e suficiente** de estabilidade BIBO é:

$$
\boxed{\;\text{Re}(p_i) < 0 \quad \forall \, i \;}
$$

ou seja, **todos os polos devem estar estritamente no semiplano esquerdo (SPE)** do plano complexo. Um único polo no SPD torna o sistema instável; polos sobre o eixo imaginário (e que não sejam cancelados por zeros coincidentes) caracterizam **estabilidade marginal**.

### 4.3 Critério de Routh-Hurwitz

Na prática, nem sempre se calcula explicitamente os polos (especialmente em sistemas de ordem elevada ou paramétricos). O **critério de Routh-Hurwitz** permite determinar a estabilidade **diretamente a partir dos coeficientes de $D(s)$**, sem resolver a equação característica.

Dado $D(s) = a_n s^n + a_{n-1} s^{n-1} + \cdots + a_1 s + a_0$, constrói-se a tabela de Routh. Um sistema é estável se, e somente se, **todos os elementos da primeira coluna da tabela forem positivos**. O número de raízes com parte real positiva é igual ao número de trocas de sinal nessa coluna (Nise; Franklin & Powell).

### 4.4 Resumo visual

| Cenário | Polos | Resposta ao degrau |
|---|---|---|
| **Estável** | Todos no SPE | Converge a um valor finito |
| **Marginal** | Sobre o eixo $j\omega$ | Oscila com amplitude constante |
| **Instável** | Ao menos um no SPD | Diverge exponencialmente |

Os três cenários são demonstrados na **Simulação 3**.

---

## 5. Resposta Temporal

A resposta temporal é o comportamento de $y(t)$ ao longo do tempo para entradas padronizadas — tipicamente o **impulso** $\delta(t)$ ou o **degrau unitário** $u_s(t)$. Essas respostas revelam características cruciais como velocidade, oscilação e erro em regime permanente.

### 5.1 Sistemas de 1ª ordem

Um sistema de 1ª ordem tem função de transferência:

$$
G(s) \;=\; \frac{K}{\tau s + 1}
$$

A resposta ao degrau unitário é:

$$
y(t) \;=\; K\!\left(1 - e^{-t/\tau}\right), \quad t \geq 0
$$

Parâmetros característicos:

- **Constante de tempo** $\tau$: instante em que $y$ atinge 63,2 % do valor final;
- **Tempo de subida** ($10\% \to 90\%$): $\;t_r \approx 2{,}2\tau$;
- **Tempo de acomodação (2%)**: $\;t_s \approx 4\tau$;
- **Ganho DC**: $\;y(\infty) = K$.

Quanto menor $\tau$, mais rápido o sistema. Ilustrado no painel esquerdo da **Simulação 4**.

### 5.2 Sistemas de 2ª ordem

A forma canônica é:

$$
G(s) \;=\; \frac{\omega_n^2}{s^2 + 2\zeta\omega_n s + \omega_n^2}
$$

onde $\omega_n$ é a **frequência natural não amortecida** e $\zeta$ é o **fator de amortecimento**. Os polos são:

$$
p_{1,2} \;=\; -\zeta\omega_n \;\pm\; \omega_n\sqrt{\zeta^2 - 1}
$$

A classificação segue o valor de $\zeta$:

| Faixa de $\zeta$ | Classificação | Natureza dos polos |
|---|---|---|
| $\zeta = 0$ | Não amortecido | Imaginários puros |
| $0 < \zeta < 1$ | Subamortecido | Complexos conjugados (SPE) |
| $\zeta = 1$ | Criticamente amortecido | Reais iguais (SPE) |
| $\zeta > 1$ | Superamortecido | Reais distintos (SPE) |

### 5.3 Parâmetros da resposta subamortecida

Para $0 < \zeta < 1$ na resposta ao degrau, definem-se:

- **Sobre-sinal (overshoot)**: $\;M_p \;=\; e^{\,-\zeta\pi/\sqrt{1-\zeta^2}} \times 100\,\%$
- **Tempo de pico**: $\;t_p \;=\; \dfrac{\pi}{\omega_n\sqrt{1-\zeta^2}}$
- **Tempo de subida (0→100%)**: $\;t_r \;=\; \dfrac{\pi - \arctan(\sqrt{1-\zeta^2}/\zeta)}{\omega_n\sqrt{1-\zeta^2}}$
- **Tempo de acomodação (2%)**: $\;t_s \;\approx\; \dfrac{4}{\zeta\omega_n}$

Um valor típico adotado em projetos práticos é $\zeta \approx 0{,}707$, que oferece um bom compromisso entre velocidade de resposta e sobre-sinal. Esse comportamento é ilustrado no painel direito da **Simulação 4**.

### 5.4 Erro em regime permanente

Para entrada em degrau unitário, o erro em regime permanente em malha aberta é:

$$
e_{ss} \;=\; \lim_{t \to \infty}\, [\,r(t) - y(t)\,] \;=\; 1 - K_{DC}, \quad K_{DC} = G(0)
$$

Este conceito será retomado na Etapa 3, quando discutirmos controle em malha fechada e ações integrais.

---

## 6. Aplicação Prática: Motor CC (PBL)

Como sistema **fio-condutor** de todo o estudo dirigido, adotamos o **motor de corrente contínua (CC) controlado por tensão de armadura**. Esse modelo é amplamente utilizado em laboratórios de controle (Franklin; Ogata) e apresenta todas as propriedades que desejamos explorar: EDO de 2ª ordem, dois polos reais bem separados, ganho DC finito e estabilidade natural.

### 6.1 Modelagem elétrica e mecânica

Aplicando a lei de Kirchhoff das tensões na malha de armadura:

$$
v(t) \;=\; R\,i(t) + L\,\frac{di(t)}{dt} + e_b(t),
\qquad e_b(t) \;=\; K_b\,\omega(t)
$$

e a 2ª lei de Newton para o rotor:

$$
J\,\frac{d\omega(t)}{dt} + b\,\omega(t) \;=\; K_m\,i(t)
$$

onde $v$ é a tensão de entrada, $i$ a corrente de armadura, $\omega$ a velocidade angular, $e_b$ a força contra-eletromotriz, $R$ e $L$ a resistência e indutância da armadura, $J$ o momento de inércia, $b$ o atrito viscoso e $K_m$, $K_b$ as constantes de torque e de fcem.

### 6.2 Função de transferência

Aplicando Laplace com condições iniciais nulas e eliminando $I(s)$:

$$
\boxed{\;\frac{\Omega(s)}{V(s)} \;=\; \frac{K_m}{(L s + R)(J s + b) + K_m K_b}\;}
$$

### 6.3 Parâmetros adotados

Utilizamos um conjunto de parâmetros clássicos de motor CC de laboratório:

| Parâmetro | Símbolo | Valor | Unidade |
|---|---|---|---|
| Resistência de armadura | $R$ | 1,0 | Ω |
| Indutância de armadura | $L$ | 0,5 | H |
| Momento de inércia | $J$ | 0,01 | kg·m² |
| Atrito viscoso | $b$ | 0,1 | N·m·s |
| Constante de torque | $K_m$ | 0,01 | N·m/A |
| Constante de fcem | $K_b$ | 0,01 | V·s/rad |

Substituindo:

$$
G(s) \;=\; \frac{0{,}01}{0{,}005\,s^2 + 0{,}06\,s + 0{,}1001}
$$

### 6.4 Análise preliminar

As raízes do denominador (polos) são:

$$
p_1 \approx -2{,}003, \qquad p_2 \approx -9{,}997
$$

Ambos são **reais e negativos**, portanto o motor em malha aberta é **BIBO-estável**. O ganho DC (valor final da resposta ao degrau unitário) é:

$$
G(0) \;=\; \frac{K_m}{R\,b + K_m K_b} \;=\; \frac{0{,}01}{0{,}1001} \;\approx\; 0{,}0999 \;\; \text{rad}/(\text{s·V})
$$

O polo dominante é $p_1 \approx -2{,}003$, cuja constante de tempo associada é $\tau_1 \approx 0{,}5$ s. Esses resultados são confirmados na **Simulação 5**, que mostra a resposta ao degrau convergindo para $\approx 0{,}0999$ rad/s em cerca de 2 s — consistente com $4\tau_1$.

### 6.5 Conexões com as próximas etapas

- **Etapa 2:** aprofundaremos a modelagem, analisando o comportamento frente a variações paramétricas (efeito de $J$, $b$, $L$, etc.).
- **Etapa 3:** projetaremos um controlador PID para regular a velocidade do motor com erro nulo em regime.
- **Etapa 4:** discutiremos a implementação prática desse controle em CLPs e microcontroladores.
- **Projeto Final:** apresentaremos o motor CC completamente controlado como sistema integrador.

---

## 7. Referências

- **OGATA, Katsuhiko.** *Engenharia de Controle Moderno*. 5. ed. São Paulo: Pearson, 2010.
- **DORF, Richard C.; BISHOP, Robert H.** *Sistemas de Controle Modernos*. 13. ed. Rio de Janeiro: LTC, 2018.
- **NISE, Norman S.** *Engenharia de Sistemas de Controle*. 7. ed. Rio de Janeiro: LTC, 2017.
- **KUO, Benjamin C.** *Sistemas de Controle Automático*. 7. ed. Rio de Janeiro: LTC, 1995.
- **FRANKLIN, Gene F.; POWELL, J. David; EMAMI-NAEINI, Abbas.** *Sistemas de Controle para Engenharia*. 6. ed. Porto Alegre: Bookman, 2013.
- **SCHAUM.** *Teoria e Problemas de Sistemas de Controle*. 2. ed. Porto Alegre: Bookman (McGraw-Hill), 1998.