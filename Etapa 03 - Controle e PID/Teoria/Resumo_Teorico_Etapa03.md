# Resumo Teórico – Etapa 03

**Disciplina:** Controle e Automação I
**Instituição:** IFPB – Campus Campina Grande
**Curso:** Engenharia de Computação
**Professor:** Dr. Moacy Pereira da Silva
**Autores:** Fabrício Moreno da Silva · Ynnayron Juan Lopes da Silva

---

## Sumário

1. [Introdução](#1-introdução)
2. [Controle em Malha Fechada](#2-controle-em-malha-fechada)
3. [Controlador PID](#3-controlador-pid)
4. [Ação Proporcional](#4-ação-proporcional)
5. [Ação Integral](#5-ação-integral)
6. [Ação Derivativa](#6-ação-derivativa)
7. [Sintonia do PID](#7-sintonia-do-pid)
8. [PID Digital](#8-pid-digital)
9. [Aplicações Práticas](#9-aplicações-práticas)
10. [Referências](#10-referências)

---

## 1. Introdução

Esta etapa dá continuidade direta ao trabalho desenvolvido na **Etapa 02 – Modelagem e Simulação**, na qual obtivemos a função de transferência do motor CC:

$$G(s) = \frac{2{,}0}{s^2 + 12{,}0 \cdot s + 20{,}02}$$

com polos em $p_1 \approx -2{,}003$ e $p_2 \approx -9{,}997$, ganho DC $G(0) \approx 0{,}0999$ rad/(s·V) e tempo de acomodação $t_s \approx 2{,}0$ s. Embora o sistema seja BIBO-estável em malha aberta, ele apresenta três limitações práticas identificadas na Etapa 01:

- **Baixo ganho DC** ($\approx 0{,}0999$): para $v = 1$ V, a velocidade de regime é apenas $0{,}1$ rad/s;
- **Erro em regime permanente** diante de distúrbios de carga;
- **Tempo de acomodação** de $\approx 2$ s, que pode ser insuficiente para aplicações dinâmicas.

O objetivo desta etapa é projetar um **controlador PID em malha fechada** que corrija essas limitações, analisando o efeito de cada ação de controle e discutindo a implementação digital do controlador.

### 1.1 Problema Norteador

> Como projetar um controlador PID para o motor CC de modo que: (i) o erro em regime permanente seja nulo para referência em degrau; (ii) o tempo de acomodação seja reduzido; (iii) o sobressinal seja mantido abaixo de um valor aceitável?

---

## 2. Controle em Malha Fechada

### 2.1 Estrutura de controle

Na malha fechada com realimentação unitária, o erro $e(t)$ é continuamente calculado e utilizado para corrigir a entrada da planta:

$$e(t) = r(t) - y(t)$$

A função de transferência de malha fechada é:

$$T(s) = \frac{C(s) \cdot G(s)}{1 + C(s) \cdot G(s)}$$

onde $C(s)$ é o controlador e $G(s)$ é a planta (motor CC).

### 2.2 Erro em regime permanente

Para entrada em degrau unitário $R(s) = 1/s$, o erro em regime permanente é dado pelo **Teorema do Valor Final**:

$$e_{ss} = \lim_{s \to 0} \, s \cdot E(s) = \lim_{s \to 0} \, s \cdot \frac{1}{1 + C(s)G(s)} \cdot \frac{1}{s} = \frac{1}{1 + K_p}$$

onde $K_p = \lim_{s \to 0} C(s)G(s)$ é o **ganho de posição**. Para $e_{ss} = 0$, é necessário que $K_p \to \infty$, o que exige que $C(s)G(s)$ tenha ao menos um **polo na origem** — justamente o papel da ação integral.

### 2.3 Função de sensibilidade

A **função de sensibilidade** $S(s) = 1/(1 + C(s)G(s))$ mede a capacidade da malha fechada de rejeitar distúrbios. Um bom controlador deve minimizar $|S(j\omega)|$ na faixa de frequências de interesse.

---

## 3. Controlador PID

### 3.1 Definição

O **controlador PID** (Proporcional-Integral-Derivativo) combina três ações de controle:

$$u(t) = K_p \, e(t) + K_i \int_0^t e(\tau)\,d\tau + K_d \frac{de(t)}{dt}$$

No domínio de Laplace (forma paralela):

$$C(s) = K_p + \frac{K_i}{s} + K_d \cdot s = \frac{K_d s^2 + K_p s + K_i}{s}$$

### 3.2 Forma com filtro derivativo

Na prática, a ação derivativa pura $K_d \cdot s$ amplifica ruído de alta frequência. Por isso, utiliza-se um **filtro de primeira ordem** com constante de filtragem $N$:

$$C(s) = K_p + \frac{K_i}{s} + \frac{K_d \cdot N \cdot s}{s + N}$$

O valor típico de $N$ está entre 5 e 20. O numerador resultante é:

$$C(s) = \frac{(K_d N + K_p)s^2 + (K_p N + K_i)s + K_i N}{s(s + N)}$$

### 3.3 Papel de cada ação

| Ação | Parâmetro | Efeito principal | Efeito colateral |
|---|---|---|---|
| Proporcional | $K_p$ | Reduz erro, aumenta velocidade | Não elimina $e_{ss}$ |
| Integral | $K_i$ | Elimina $e_{ss}$ | Pode causar sobressinal e instabilidade |
| Derivativa | $K_d$ | Reduz sobressinal, aumenta amortecimento | Amplifica ruído |

---

## 4. Ação Proporcional

O controlador proporcional puro tem $C(s) = K_p$. A função de malha fechada torna-se:

$$T(s) = \frac{K_p \cdot G(s)}{1 + K_p \cdot G(s)}$$

O erro em regime permanente para degrau unitário é:

$$e_{ss} = \frac{1}{1 + K_p \cdot G(0)} = \frac{1}{1 + K_p \cdot 0{,}0999}$$

Portanto, $e_{ss}$ nunca é nulo — apenas se reduz com o aumento de $K_p$. Contudo, valores muito altos de $K_p$ aproximam os polos de malha fechada do eixo imaginário, podendo tornar o sistema subamortecido ou instável.

**Compromisso do ganho proporcional:**

$$K_p \uparrow \;\Rightarrow\; e_{ss} \downarrow,\; t_s \downarrow,\; M_p \uparrow \;\text{(risco de instabilidade)}$$

---

## 5. Ação Integral

A adição de $K_i/s$ ao controlador garante **tipo 1** em malha aberta — condição suficiente para $e_{ss} = 0$ frente a referências em degrau. Isso ocorre porque o integrador acumula o erro ao longo do tempo e incrementa a ação de controle até que $e(t) = 0$.

O controlador PI tem $C(s) = K_p + K_i/s$. Para o motor CC com $K_p = 20$ fixo:

$$e_{ss} = 0 \quad \forall K_i > 0$$

Porém, $K_i$ elevado introduz **atraso de fase**, podendo causar:

- Aumento do sobressinal $M_p$;
- Oscilações persistentes (*wind-up* do integrador);
- Instabilidade para $K_i$ excessivo.

**Wind-up do integrador:** quando a saída do atuador satura, o integrador continua acumulando erro, causando sobrecomando. A solução prática é o **anti-windup**: limitar a saída do integrador ou redefini-la quando há saturação.

---

## 6. Ação Derivativa

A ação derivativa $K_d \cdot s$ atua sobre a **taxa de variação do erro**, antecipando sua tendência e adicionando amortecimento à malha. Ela desloca os polos de malha fechada para a esquerda no plano $s$, reduzindo o sobressinal.

Para o controlador PID com $K_p = 20$, $K_i = 10$ fixos:

$$K_d \uparrow \;\Rightarrow\; M_p \downarrow,\; \zeta \uparrow$$

O efeito é análogo ao de aumentar o amortecimento $b$ na planta (Etapa 02), porém aplicado externamente pelo controlador — sem afetar o regime permanente.

**Limitação:** a ação derivativa não deve ser aplicada a sinais ruidosos sem filtragem adequada ($N$), pois $d/dt$ amplifica componentes de alta frequência.

---

## 7. Sintonia do PID

### 7.1 Método de Ziegler-Nichols (malha fechada)

O método clássico de Ziegler-Nichols em malha fechada segue os passos:

1. Zerar $K_i$ e $K_d$; aumentar $K_p$ até o sistema oscilar com amplitude constante;
2. Registrar o **ganho último** $K_u$ e o **período de oscilação** $T_u$;
3. Calcular os ganhos pela tabela:

| Controlador | $K_p$ | $K_i$ | $K_d$ |
|---|---|---|---|
| P | $0{,}5 \cdot K_u$ | — | — |
| PI | $0{,}45 \cdot K_u$ | $1{,}2 \cdot K_p / T_u$ | — |
| **PID** | $0{,}6 \cdot K_u$ | $2 \cdot K_p / T_u$ | $K_p \cdot T_u / 8$ |

**Observação:** o método Z-N tende a gerar respostas com sobressinal elevado ($\approx 25\%$). Na prática, os ganhos obtidos são usados como ponto de partida para ajuste fino.

### 7.2 Critério de sintonia manual adotado

Para o motor CC desta etapa, adotamos os seguintes ganhos após tentativa e erro:

$$K_p = 30, \qquad K_i = 15, \qquad K_d = 0{,}2$$

Esses valores foram escolhidos para satisfazer simultaneamente: $e_{ss} = 0$, $M_p < 10\%$ e $t_s < 2$ s.

---

## 8. PID Digital

### 8.1 Necessidade de discretização

Em sistemas embarcados (Arduino, STM32, CLPs), o controlador é executado com período de amostragem $T_s$. A equação contínua do PID deve ser convertida para uma **equação de diferenças**.

### 8.2 Método de Euler progressivo (forward)

Aproximando a integral por soma de Riemann e a derivada por diferença progressiva:

$$\int e \, dt \approx T_s \sum_{k} e[k], \qquad \frac{de}{dt} \approx \frac{e[k] - e[k-1]}{T_s}$$

A equação de diferenças do PID na forma de velocidade (**incremental**) é:

$$\Delta u[k] = q_0 \cdot e[k] + q_1 \cdot e[k-1] + q_2 \cdot e[k-2]$$

$$u[k] = u[k-1] + \Delta u[k]$$

onde os coeficientes são:

$$q_0 = K_p + K_i T_s + \frac{K_d}{T_s}, \qquad q_1 = -K_p - \frac{2K_d}{T_s}, \qquad q_2 = \frac{K_d}{T_s}$$

### 8.3 Efeito do período de amostragem

| $T_s$ (s) | Comportamento |
|---|---|
| $T_s \ll \tau_1$ | Muito próximo do contínuo |
| $T_s \approx \tau_1 / 10$ | Adequado para a maioria das aplicações |
| $T_s \approx \tau_1 / 2$ | Degradação visível da resposta |
| $T_s > \tau_1$ | Instabilidade ou resposta inaceitável |

Para o motor CC com $\tau_1 \approx 0{,}5$ s, recomenda-se $T_s \leq 0{,}05$ s (20 Hz ou mais).

### 8.4 Regra de Nyquist aplicada ao controle

O teorema de amostragem exige $f_s \geq 2 f_{max}$. Para sistemas de controle, a prática recomenda $f_s \geq 10 \cdot f_{bw}$, onde $f_{bw}$ é a largura de banda da malha fechada.

---

## 9. Aplicações Práticas

- **Controle de velocidade de motores:** o PID é o controlador mais utilizado industrialmente para regulação de velocidade em bombas, ventiladores, esteiras e robôs.

- **Controle de temperatura:** fornos industriais, impressoras 3D e sistemas HVAC utilizam PID para manter temperatura de referência com erro nulo.

- **Controle de posição em CNC:** a ação derivativa do PID é essencial para suavizar a resposta e evitar oscilações em movimentos de alta precisão.

- **Drones e veículos autônomos:** cada eixo de controle (altitude, pitch, roll, yaw) é regulado por um PID independente executado em microcontrolador.

### 9.1 Conexão com as próximas etapas

- **Etapa 4:** os conceitos de PID digital serão retomados na implementação em CLP (linguagem Ladder/Structured Text), onde o bloco PID é um elemento padrão de programação.
- **Projeto Final:** o motor CC será apresentado com o controlador PID projetado nesta etapa integrado ao modelo completo do sistema.

---

## 10. Referências

- **OGATA, Katsuhiko.** *Engenharia de Controle Moderno*. 5. ed. São Paulo: Pearson, 2010.
- **DORF, Richard C.; BISHOP, Robert H.** *Sistemas de Controle Modernos*. 13. ed. Rio de Janeiro: LTC, 2018.
- **NISE, Norman S.** *Engenharia de Sistemas de Controle*. 7. ed. Rio de Janeiro: LTC, 2017.
- **FRANKLIN, Gene F.; POWELL, J. David; EMAMI-NAEINI, Abbas.** *Sistemas de Controle para Engenharia*. 6. ed. Porto Alegre: Bookman, 2013.
- **ÅSTRÖM, Karl J.; HÄGGLUND, Tore.** *PID Controllers: Theory, Design and Tuning*. 2. ed. ISA, 1995.
- **SCIPY.** *scipy.signal – Signal Processing*. Disponível em: https://docs.scipy.org/doc/scipy/reference/signal.html. Acesso em: maio 2026.
