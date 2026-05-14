# Resumo Teórico – Etapa 02

**Disciplina:** Controle e Automação I
**Instituição:** IFPB – Campus Campina Grande
**Curso:** Engenharia da Computação
**Professor:** Dr. Moacy Pereira da Silva
**Autores:** Fabrício Moreno da Silva · Ynnayron Juan Lopes da Silva

---

## Sumário

1. [Introdução](#1-introdução)
2. [Descrição Física do Motor CC](#2-descrição-física-do-motor-cc)
3. [Equações Diferenciais do Sistema](#3-equações-diferenciais-do-sistema)
4. [Função de Transferência](#4-função-de-transferência)
5. [Representação em Espaço de Estados](#5-representação-em-espaço-de-estados)
6. [Análise Paramétrica](#6-análise-paramétrica)
7. [Aplicações Práticas](#7-aplicações-práticas)
8. [Referências](#8-referências)

---

## 1. Introdução

Esta etapa dá continuidade direta ao trabalho desenvolvido na **Etapa 01 – Fundamentos Teóricos**. Lá, estabelecemos os pilares conceituais de sistemas LTI: função de transferência, polos e zeros, estabilidade BIBO e resposta temporal de sistemas de 1ª e 2ª ordens. Agora, o foco se desloca para a **modelagem aprofundada** do sistema fio-condutor definido anteriormente — o motor de corrente contínua (CC) controlado por tensão de armadura.

O objetivo central desta etapa é derivar formalmente as equações que governam o motor CC, transformá-las em função de transferência e investigar como a variação de cada parâmetro físico afeta a posição dos polos e o comportamento dinâmico do sistema.

### 1.1 Problema Norteador

> Como a variação dos parâmetros físicos de um motor CC — momento de inércia $J$, atrito viscoso $b$, indutância $L$, resistência $R$ e constante de torque $K_m$ — altera a posição dos polos, o ganho DC e a velocidade de resposta do sistema? E quais dessas variações podem comprometer a estabilidade ou tornar o sistema subamortecido?

**Resposta resumida:** a posição dos polos depende diretamente dos parâmetros físicos. Aumentar $J$ desloca o polo dominante para próximo da origem, tornando o sistema mais lento sem afetar o ganho DC. Aumentar $b$ move os polos para a esquerda e reduz o ganho DC. Aumentar $K_m$ eleva o ganho, mas pode introduzir componentes imaginárias nos polos, tornando o sistema subamortecido. Em todos os cenários com parâmetros físicos positivos, o sistema permanece **BIBO-estável** em malha aberta — justificativa formal apresentada na Seção 6.5.

---

## 2. Descrição Física do Motor CC

Um motor CC de excitação separada é composto por dois subsistemas fisicamente distintos, porém matematicamente acoplados:

- **Subsistema elétrico:** circuito de armadura, regido pela Lei de Kirchhoff das Tensões (LKT);
- **Subsistema mecânico:** eixo rotativo com carga, regido pela 2ª Lei de Newton para sistemas rotativos.

O acoplamento entre os dois subsistemas ocorre por meio do **torque eletromagnético** $T_{em} = K_m \cdot i(t)$ e da **força contra-eletromotriz (fcem)** $e_b(t) = K_b \cdot \omega(t)$.

### 2.1 Parâmetros do modelo

| Símbolo | Descrição | Valor nominal | Unidade |
|---|---|---|---|
| $v(t)$ | Tensão de armadura (entrada) | — | V |
| $i(t)$ | Corrente de armadura | variável | A |
| $\omega(t)$ | Velocidade angular do eixo (saída) | variável | rad/s |
| $R$ | Resistência de armadura | 1,0 | Ω |
| $L$ | Indutância de armadura | 0,5 | H |
| $J$ | Momento de inércia do rotor | 0,01 | kg·m² |
| $b$ | Coeficiente de atrito viscoso | 0,1 | N·m·s |
| $K_m$ | Constante de torque | 0,01 | N·m/A |
| $K_b$ | Constante de fcem | 0,01 | V·s/rad |

Os valores nominais são os mesmos adotados na Etapa 01 (Franklin; Ogata) e servem de referência para todas as simulações desta etapa.

---

## 3. Equações Diferenciais do Sistema

### 3.1 Subsistema Elétrico

Aplicando a **Lei de Kirchhoff das Tensões (LKT)** na malha de armadura e substituindo a expressão da fcem:

$$v(t) = R \cdot i(t) + L \cdot \frac{di(t)}{dt} + K_b \cdot \omega(t)$$

Esta é uma EDO de 1ª ordem em $i(t)$, com $\omega(t)$ atuando como termo de acoplamento. A fcem representa a conversão de energia mecânica em tensão elétrica e constitui o mecanismo de **realimentação interna** do motor.

### 3.2 Subsistema Mecânico

Aplicando a **2ª Lei de Newton para sistemas rotativos** ao eixo do motor, com torque eletromagnético $T_{em} = K_m \cdot i(t)$ e torque resistivo de atrito viscoso $b \cdot \omega(t)$:

$$J \cdot \frac{d\omega(t)}{dt} + b \cdot \omega(t) = K_m \cdot i(t)$$

Esta é uma EDO de 1ª ordem em $\omega(t)$, com $i(t)$ como termo de entrada proveniente do subsistema elétrico.

### 3.3 Sistema Acoplado

As duas equações formam um **sistema acoplado de duas EDOs de 1ª ordem**. A corrente $i(t)$ é a variável de ligação: ela é determinada pela equação elétrica e, ao mesmo tempo, gera o torque na equação mecânica. Esse acoplamento bidirecional é o que torna o modelo de 2ª ordem — e não a simples soma de dois sistemas de 1ª ordem independentes.

---

## 4. Função de Transferência

### 4.1 Derivação via Transformada de Laplace

Aplicando a Transformada de Laplace nas equações das Seções 3.1 e 3.2, com condições iniciais nulas:

$$V(s) = (Ls + R) \cdot I(s) + K_b \cdot \Omega(s)$$

$$I(s) = \frac{(Js + b) \cdot \Omega(s)}{K_m}$$

Substituindo a segunda na primeira para eliminar $I(s)$ e isolando $\Omega(s)/V(s)$:

$$\frac{\Omega(s)}{V(s)} = \frac{K_m}{(Ls + R)(Js + b) + K_m K_b}$$

### 4.2 Forma expandida

Expandindo o denominador:

$$G(s) = \frac{K_m}{LJ \cdot s^2 + (Lb + RJ) \cdot s + (Rb + K_m K_b)}$$

### 4.3 Substituição numérica

Calculando os coeficientes com os valores nominais da Tabela 1:

$$LJ = 0{,}005 \qquad Lb + RJ = 0{,}06 \qquad Rb + K_m K_b = 0{,}1001$$

$$G(s) = \frac{0{,}01}{0{,}005 \cdot s^2 + 0{,}06 \cdot s + 0{,}1001}$$

Dividindo numerador e denominador por $0{,}005$ para obter a forma canônica:

$$G(s) = \frac{2{,}0}{s^2 + 12{,}0 \cdot s + 20{,}02}$$

### 4.4 Polos e ganho DC

Os polos são as raízes de $D(s) = s^2 + 12{,}0 \cdot s + 20{,}02 = 0$:

$$p_1 \approx -2{,}003 \quad \text{(polo dominante,} \quad \tau_1 \approx 0{,}499 \text{ s)}$$

$$p_2 \approx -9{,}997 \quad \text{(polo rápido,} \quad \tau_2 \approx 0{,}100 \text{ s)}$$

O ganho DC (valor final da resposta ao degrau unitário) é:

$$G(0) = \frac{K_m}{R \cdot b + K_m K_b} = \frac{0{,}01}{0{,}1001} \approx 0{,}0999 \text{ rad/(s·V)}$$

Ambos os polos são **reais e negativos**, confirmando a estabilidade BIBO em malha aberta. O polo dominante $p_1$ determina a dinâmica predominante: o tempo de acomodação é $t_s \approx 4\tau_1 \approx 2{,}0$ s, e o sistema é **superamortecido** (sem sobressinal).

---

## 5. Representação em Espaço de Estados

### 5.1 Definição das variáveis de estado

Definindo $x_1 = i(t)$ e $x_2 = \omega(t)$ como variáveis de estado e $u = v(t)$ como entrada, o sistema acoplado (Seção 3) pode ser reescrito na forma matricial $\dot{x} = Ax + Bu$:

```math
\begin{bmatrix} \dot{x}_1 \\ \dot{x}_2 \end{bmatrix}
=
\begin{bmatrix} -R/L & -K_b/L \\ K_m/J & -b/J \end{bmatrix}
\begin{bmatrix} x_1 \\ x_2 \end{bmatrix}
+
\begin{bmatrix} 1/L \\ 0 \end{bmatrix} u
```

### 5.2 Forma numérica

Substituindo os valores nominais da Tabela 1:

```math
\begin{bmatrix} \dot{x}_1 \\ \dot{x}_2 \end{bmatrix}
=
\begin{bmatrix} -2{,}0 & -0{,}02 \\ 1{,}0 & -10{,}0 \end{bmatrix}
\begin{bmatrix} x_1 \\ x_2 \end{bmatrix}
+
\begin{bmatrix} 2{,}0 \\ 0 \end{bmatrix} u
```

### 5.3 Relação com a função de transferência

Os **autovalores da matriz $A$** coincidem com os polos da função de transferência, pois ambos são raízes do mesmo polinômio característico $D(s)$:

```math
\det(\lambda I - A) = 0 \implies \lambda_1 \approx -2{,}003, \quad \lambda_2 \approx -9{,}997
```

Essa equivalência confirma que as duas representações descrevem o mesmo sistema. A diferença é que a representação em espaço de estados fornece acesso às variáveis internas (corrente e velocidade), enquanto a função de transferência descreve apenas a relação entrada–saída.

---

## 6. Análise Paramétrica

A principal contribuição desta etapa é investigar como cada parâmetro físico afeta os polos e o comportamento dinâmico. A tabela a seguir resume os efeitos qualitativos:

| Parâmetro | Efeito no polo dominante $p_1$ | Efeito no ganho DC $G(0)$ | Efeito no $t_s$ |
|---|---|---|---|
| $J \uparrow$ | Aproxima-se da origem | Nenhum | Aumenta |
| $b \uparrow$ | Afasta-se da origem (mais negativo) | Diminui | Diminui |
| $L \uparrow$ | Pequeno efeito | Nenhum | Pequeno efeito |
| $R \uparrow$ | Pequeno efeito | Diminui | Pequeno efeito |
| $K_m \uparrow$ | Pode tornar-se complexo | Aumenta | Diminui (inicialmente) |

### 6.1 Variação do momento de inércia $J$

O parâmetro $J$ aparece nos coeficientes $LJ$ (de $s^2$) e $RJ$ (de $s$) do denominador. Aumentar $J$ torna o rotor mais resistente à aceleração angular, **desacelerando a resposta** sem alterar o valor em regime permanente — pois $G(0) = K_m/(Rb + K_m K_b)$ independe de $J$.

| $J$ (kg·m²) | $p_1$ | $p_2$ | $t_s \approx 4/|p_1|$ |
|---|---|---|---|
| 0,005 | −2,50 | −19,99 | 1,6 s |
| 0,01 (nominal) | −2,003 | −9,997 | 2,0 s |
| 0,05 | −0,50 | ≈ −10,0 | 8,0 s |
| 0,10 | −0,25 | ≈ −10,0 | 16,0 s |

### 6.2 Variação do atrito viscoso $b$

O coeficiente $b$ aparece no denominador e no ganho DC. Aumentar $b$ move os polos para a esquerda (**resposta mais rápida**) e simultaneamente **reduz o ganho DC** — há um compromisso intrínseco: maior atrito resulta em menor velocidade de regime e resposta mais veloz.

### 6.3 Variação da indutância $L$

A indutância governa principalmente a **dinâmica elétrica** do sistema, com constante de tempo $\tau_e = L/R$. Para $L$ pequeno, $\tau_e \ll \tau_m$ e o modelo se aproxima de uma função de transferência de 1ª ordem dominada pelo polo mecânico. O polo dominante $p_1$ e o ganho DC permanecem praticamente inalterados.

### 6.4 Variação da constante de torque $K_m$

$K_m$ é o parâmetro de acoplamento entre os subsistemas. Para valores suficientemente altos, o produto $K_m K_b$ domina o termo constante do denominador e os termos cruzados introduzem componentes imaginárias nos polos — **o sistema transita de superamortecido para subamortecido**:

| $K_m$ (N·m/A) | $G(0)$ (rad/s/V) | Natureza dos polos | Comportamento |
|---|---|---|---|
| 0,001 | ≈ 0,0100 | Reais distintos | Superamortecido |
| 0,01 (nominal) | ≈ 0,0999 | Reais distintos | Superamortecido |
| 0,1 | ≈ 0,476 | Reais distintos | Superamortecido |
| 1,0 | ≈ 0,909 | Complexos conjugados | Subamortecido |

### 6.5 Estabilidade robusta — critério de Routh-Hurwitz

Para o denominador $D(s) = LJ \cdot s^2 + (Lb + RJ) \cdot s + (Rb + K_m K_b)$, todos os coeficientes são **estritamente positivos** para quaisquer valores físicos positivos dos parâmetros. A tabela de Routh para um polinômio de 2ª ordem é:

| Linha | Coluna 1 | Coluna 2 |
|---|---|---|
| $s^2$ | $LJ > 0$ | $Rb + K_m K_b > 0$ |
| $s^1$ | $Lb + RJ > 0$ | $0$ |
| $s^0$ | $Rb + K_m K_b > 0$ | — |

Não há trocas de sinal na primeira coluna, portanto **todas as raízes têm parte real negativa**. Conclui-se:

> **O motor CC em malha aberta é sempre BIBO-estável para parâmetros físicos positivos.**

---

## 7. Aplicações Práticas

Os modelos e resultados obtidos nesta etapa têm presença direta em inúmeras aplicações:

- **Acionamentos industriais de precisão (CNC):** o modelo de 2ª ordem é o ponto de partida para o projeto de controladores com especificações rigorosas de sobressinal e tempo de acomodação. A análise de sensibilidade paramétrica orienta a robustez frente a variações de carga ($J$ e $b$).

- **Robótica industrial e colaborativa:** juntas de manipuladores robóticos empregam motores com controle em cascata de torque, velocidade e posição. O polo dominante e o ganho DC calculados aqui determinam os parâmetros de sintonia de cada malha.

- **Veículos elétricos e frenagem regenerativa:** a análise da resposta transitória é essencial para o projeto de sistemas de controle de torque que garantam conforto de aceleração e eficiência na recuperação de energia.

- **Sistemas embarcados e drones:** ESCs (*Electronic Speed Controllers*) implementam controle de velocidade de motores em tempo real. O modelo dinâmico obtido nesta etapa é incorporado ao projeto do controlador embarcado no microcontrolador de voo.

### 7.1 Conexões com as próximas etapas

- **Etapa 3:** a função de transferência $G(s)$ obtida aqui será utilizada para projetar um controlador PID que elimine o erro em regime permanente, atinja tempo de acomodação especificado e limite o sobressinal.
- **Etapa 4:** a implementação digital do controlador em CLPs e microcontroladores exigirá a discretização do modelo contínuo derivado nesta etapa.
- **Projeto Final:** o motor CC completamente modelado e controlado será o sistema integrador de todo o estudo dirigido.

---

## 8. Referências

- **OGATA, Katsuhiko.** *Engenharia de Controle Moderno*. 5. ed. São Paulo: Pearson, 2010.
- **DORF, Richard C.; BISHOP, Robert H.** *Sistemas de Controle Modernos*. 13. ed. Rio de Janeiro: LTC, 2018.
- **NISE, Norman S.** *Engenharia de Sistemas de Controle*. 7. ed. Rio de Janeiro: LTC, 2017.
- **KUO, Benjamin C.** *Sistemas de Controle Automático*. 7. ed. Rio de Janeiro: LTC, 1995.
- **FRANKLIN, Gene F.; POWELL, J. David; EMAMI-NAEINI, Abbas.** *Sistemas de Controle para Engenharia*. 6. ed. Porto Alegre: Bookman, 2013.
- **SCHAUM.** *Teoria e Problemas de Sistemas de Controle*. 2. ed. Porto Alegre: Bookman (McGraw-Hill), 1998.
- **SCIPY.** *scipy.signal – Signal Processing*. Disponível em: https://docs.scipy.org/doc/scipy/reference/signal.html. Acesso em: maio 2026.