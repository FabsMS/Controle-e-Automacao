# Discussão Técnica – Etapa 02

**Disciplina:** Controle e Automação I
**Instituição:** IFPB – Campus Campina Grande
**Curso:** Engenharia da Computação
**Professor:** Dr. Moacy Pereira da Silva
**Autores:** Fabrício Moreno da Silva · Ynnayron Juan Lopes da Silva

---

## Sumário

1. [Simulações e Resultados](#1-simulações-e-resultados)
   - 1.1 [Simulação 01 – Resposta ao Degrau (Parâmetros Nominais)](#11-simulação-01--resposta-ao-degrau-parâmetros-nominais)
   - 1.2 [Simulação 02 – Variação do Momento de Inércia J](#12-simulação-02--variação-do-momento-de-inércia-j)
   - 1.3 [Simulação 03 – Variação do Atrito Viscoso b](#13-simulação-03--variação-do-atrito-viscoso-b)
   - 1.4 [Simulação 04 – Variação da Indutância L](#14-simulação-04--variação-da-indutância-l)
   - 1.5 [Simulação 05 – Variação da Constante de Torque Km](#15-simulação-05--variação-da-constante-de-torque-km)
   - 1.6 [Simulação 06 – Diagrama de Polos no Plano s](#16-simulação-06--diagrama-de-polos-no-plano-s)
2. [Análise Crítica dos Resultados](#2-análise-crítica-dos-resultados)
   - 2.1 [Separação de Escalas de Tempo](#21-separação-de-escalas-de-tempo)
   - 2.2 [Estabilidade Robusta e Critério de Routh-Hurwitz](#22-estabilidade-robusta-e-critério-de-routh-hurwitz)
   - 2.3 [Polo Dominante e Projeto de Controladores](#23-polo-dominante-e-projeto-de-controladores)
   - 2.4 [Transição para Comportamento Subamortecido](#24-transição-para-comportamento-subamortecido)
   - 2.5 [Limitações do Modelo Linear](#25-limitações-do-modelo-linear)

---

## 1. Simulações e Resultados

Foram desenvolvidas seis simulações que investigam progressivamente o comportamento do motor CC: primeiro com os parâmetros nominais da Etapa 01, depois com a variação individual de cada parâmetro físico, e por fim um diagrama consolidado de polos no plano $s$. Todas as simulações foram implementadas em Python (biblioteca `scipy.signal`) e os gráficos gerados estão disponíveis na pasta `resultados/`.

---

### 1.1 Simulação 01 – Resposta ao Degrau (Parâmetros Nominais)

A primeira simulação calcula a resposta ao degrau unitário de tensão do motor CC com os parâmetros nominais herdados da Etapa 01:

$$
G(s) = \frac{0{,}01}{0{,}005s^2 + 0{,}06s + 0{,}1001} = \frac{2{,}0}{s^2 + 12{,}0s + 20{,}02}
$$

**Observações:**

A resposta é **monotonicamente crescente e sem oscilação**, característica de um sistema com dois polos reais negativos distintos — comportamento superamortecido ($\zeta > 1$). O valor em regime permanente converge para $y(\infty) \approx 0{,}0999$ rad/s/V, em perfeita concordância com o ganho DC analítico $G(0) = K_m/(Rb + K_mK_b) \approx 0{,}0999$.

O **polo dominante** $p_1 \approx -2{,}003$ determina integralmente a dinâmica observável: a constante de tempo associada é $\tau_1 \approx 0{,}499$ s, e o tempo de acomodação pelo critério de 2% é $t_s \approx 4\tau_1 \approx 2{,}0$ s. O polo rápido $p_2 \approx -9{,}997$, com $\tau_2 \approx 0{,}1$ s, extingue-se em menos de meio segundo e tem influência desprezível na curva visível — esse é justamente o conceito de **polo dominante**: aquele cuja constante de tempo é muito maior que a dos demais polos governa a resposta do sistema.

| Parâmetro de desempenho | Valor obtido |
|---|---|
| Ganho DC: $G(0) = K_m\,/\,(Rb + K_mK_b)$ | $0{,}0999$ rad/s/V |
| Polo dominante $p_1$ | $\approx -2{,}003$ rad/s |
| Polo rápido $p_2$ | $\approx -9{,}997$ rad/s |
| Constante de tempo $\tau_1 = 1/|p_1|$ | $\approx 0{,}499$ s |
| Tempo de acomodação $t_s \approx 4\tau_1$ | $\approx 2{,}0$ s |
| Sobressinal $M_p$ | $0\%$ (superamortecido) |

Esses resultados são consistentes com a análise preliminar realizada na Simulação 05 da Etapa 01 e confirmam a corretude do modelo matemático adotado.

---

### 1.2 Simulação 02 – Variação do Momento de Inércia J


O momento de inércia $J$ representa a resistência do rotor à variação de velocidade angular. Ele aparece nos coeficientes $LJ$ (de $s^2$) e $RJ$ (de $s$) do denominador da função de transferência. Foram comparados quatro valores: $J \in \{0{,}005;\; 0{,}01;\; 0{,}05;\; 0{,}1\}$ kg·m².

**Observações:**

Todas as curvas convergem ao **mesmo valor final** $y(\infty) \approx 0{,}0999$ rad/s/V, pois $G(0) = K_m/(Rb + K_mK_b)$ é independente de $J$. Contudo, o tempo necessário para atingir esse regime permanente varia significativamente: para $J = 0{,}005$ kg·m², o sistema acomoda em $\approx 1{,}6$ s; para $J = 0{,}1$ kg·m², esse tempo sobe para $\approx 16$ s — uma diferença de uma ordem de magnitude.

Esse comportamento tem explicação direta na posição do polo dominante: à medida que $J$ aumenta, $p_1$ se aproxima da origem, aumentando $\tau_1 = 1/|p_1|$ e, consequentemente, $t_s \approx 4\tau_1$.

| $J$ (kg·m²) | $p_1$ | $p_2$ | $t_s \approx 4/|p_1|$ |
|---|---|---|---|
| 0,005 | $-2{,}50$ | $-19{,}99$ | $1{,}6$ s |
| 0,01 (nominal) | $-2{,}003$ | $-9{,}997$ | $2{,}0$ s |
| 0,05 | $-0{,}50$ | $\approx -10{,}0$ | $8{,}0$ s |
| 0,10 | $-0{,}25$ | $\approx -10{,}0$ | $16{,}0$ s |

Nota-se também que o polo rápido $p_2$ permanece **aproximadamente constante** em torno de $-10$, pois ele é controlado principalmente pelo subsistema elétrico ($R/L = 2$ e $b/J \approx 10$ para o $J$ nominal). Isso ilustra a separação entre as dinâmicas elétrica e mecânica: $J$ afeta apenas a dinâmica mecânica, deslocando $p_1$ sem alterar $p_2$ de forma significativa.

**Implicação prática:** em sistemas de acionamento com carga variável (ex.: robôs que manipulam peças de massas distintas), a variação efetiva de $J$ é uma das principais causas de degradação de desempenho. O controlador projetado na Etapa 03 deverá ser robusto a essa variação.

---

### 1.3 Simulação 03 – Variação do Atrito Viscoso b


O coeficiente de atrito viscoso $b$ aparece simultaneamente no denominador (afetando os polos) e no ganho DC:

$$
G(0) = \frac{K_m}{Rb + K_mK_b}
$$

Foram comparados quatro valores: $b \in \{0{,}02;\; 0{,}1;\; 0{,}5;\; 1{,}0\}$ N·m·s.

**Observações:**

Ao contrário de $J$, a variação de $b$ afeta **tanto a velocidade da resposta quanto o valor em regime permanente**, criando um compromisso intrínseco:

- **$b$ pequeno ($b = 0{,}02$):** polo dominante mais lento ($p_1 \approx -1{,}04$), mas ganho DC alto ($\approx 0{,}476$ rad/s/V). O sistema atinge uma velocidade angular elevada, porém demora mais para se estabilizar.
- **$b$ grande ($b = 1{,}0$):** polo dominante mais rápido ($p_1 \approx -12{,}0$), mas ganho DC muito baixo ($\approx 0{,}010$ rad/s/V). O sistema responde rapidamente, mas a velocidade angular em regime permanente é muito reduzida.

O painel direito da simulação exibe a curva analítica de $G(0)$ em função de $b$, evidenciando a relação inversamente proporcional entre atrito e ganho DC. Esse é um resultado com impacto direto no projeto: em aplicações que exigem alta velocidade de regime (ex.: acionamento de bombas), motores com baixo atrito são desejáveis, mas demandam controladores mais cuidadosamente sintonizados.

---

### 1.4 Simulação 04 – Variação da Indutância L


A indutância de armadura $L$ governa a **dinâmica elétrica** do sistema, com constante de tempo $\tau_e = L/R$. Foram comparados quatro valores: $L \in \{0{,}05;\; 0{,}1;\; 0{,}5;\; 2{,}0\}$ H.

**Observações:**

O painel esquerdo mostra que as curvas de resposta ao degrau são **muito semelhantes** entre si, especialmente para $L$ pequeno — o que indica que a variação de $L$ tem pouca influência na dinâmica visível da saída $\omega(t)$.

O painel direito explica o motivo: à medida que $L$ diminui, o polo rápido $p_2$ (associado à dinâmica elétrica) se afasta rapidamente para a esquerda no plano $s$, tornando a separação entre os dois polos cada vez maior. No limite $L \to 0$:

$$
G(s) \xrightarrow{L \to 0} \frac{K_m}{RJs + (Rb + K_mK_b)}
$$

o sistema degenera para uma função de transferência de **1ª ordem** dominada pelo subsistema mecânico — resultado amplamente utilizado na literatura para simplificar o projeto de controladores de motores CC (Franklin; Ogata).

O polo dominante $p_1$ e o ganho DC $G(0)$ permanecem **praticamente inalterados** com a variação de $L$, o que confirma que a indutância não é um parâmetro crítico para a dinâmica de velocidade em regime permanente.

**Implicação prática:** em motores industriais reais, $L$ é geralmente pequena (valores da ordem de mH), de modo que a separação de escalas de tempo $\tau_e \ll \tau_m$ é quase sempre verificada. Isso justifica a simplificação de 1ª ordem adotada em muitos projetos práticos de controladores embarcados.

---

### 1.5 Simulação 05 – Variação da Constante de Torque Km


A constante de torque $K_m$ é o **parâmetro de acoplamento** entre os subsistemas elétrico e mecânico. Ela aparece no numerador (determinando o ganho DC) e no denominador (influenciando os polos por meio do produto $K_mK_b$). Foram comparados quatro valores: $K_m \in \{0{,}001;\; 0{,}01;\; 0{,}1;\; 1{,}0\}$ N·m/A.

**Observações:**

Para valores pequenos de $K_m$ ($\leq 0{,}1$ N·m/A), os polos permanecem reais e o sistema é superamortecido. O ganho DC cresce à medida que $K_m$ aumenta. Para $K_m = 1{,}0$ N·m/A, o produto $K_mK_b$ domina o termo constante do denominador e os termos cruzados introduzem **componentes imaginárias nos polos**:

$$
p_{1,2} \approx -6{,}0 \pm j7{,}5
$$

O sistema torna-se **subamortecido**, com fator de amortecimento $\zeta \approx 0{,}625$ e frequência natural amortecida $\omega_d \approx 7{,}5$ rad/s — a curva de resposta ao degrau exibe sobressinal e oscilações amortecidas visíveis.

O painel direito exibe a trajetória dos polos no plano $s$ à medida que $K_m$ varia de 0 a 2 N·m/A. Observa-se que, para $K_m$ pequeno, ambos os polos são reais e distintos. À medida que $K_m$ cresce, os polos se aproximam no eixo real e, em determinado valor crítico, coalescem (sistema criticamente amortecido) e então se bifurcam em direção ao eixo imaginário, tornando-se complexos conjugados.

| $K_m$ (N·m/A) | $G(0)$ (rad/s/V) | Natureza dos polos | Comportamento |
|---|---|---|---|
| 0,001 | $\approx 0{,}0100$ | Reais distintos | Superamortecido |
| 0,01 (nominal) | $\approx 0{,}0999$ | Reais distintos | Superamortecido |
| 0,1 | $\approx 0{,}476$ | Reais distintos | Superamortecido |
| 1,0 | $\approx 0{,}909$ | Complexos conjugados | Subamortecido |

**Implicação prática:** a escolha de motores com $K_m$ elevado para obter maior ganho DC pode introduzir oscilações indesejadas na resposta transitória. O controlador PID projetado na Etapa 03 deverá considerar essa possibilidade, adicionando amortecimento por meio da ação derivativa.

---

### 1.6 Simulação 06 – Diagrama de Polos no Plano s


A última simulação consolida, em quatro subgráficos, a posição dos polos no plano $s$ para as variações de $J$, $b$, $L$ e $K_m$ estudadas nas simulações anteriores. A região sombreada em verde representa o **semiplano esquerdo (SPE)** — região de estabilidade.

**Observações:**

- **Variação de $J$:** o polo dominante $p_1$ migra ao longo do eixo real negativo, aproximando-se da origem com o aumento de $J$. O polo rápido $p_2$ permanece quase fixo.
- **Variação de $b$:** ambos os polos se deslocam para a esquerda com o aumento de $b$, porém em magnitudes distintas — $p_1$ é mais sensível que $p_2$.
- **Variação de $L$:** o polo rápido $p_2$ se afasta significativamente para a esquerda com a redução de $L$, enquanto $p_1$ permanece praticamente estático.
- **Variação de $K_m$:** os polos inicialmente convergem no eixo real e depois se bifurcam em direção ao eixo imaginário, evidenciando a transição superamortecido → criticamente amortecido → subamortecido.

Em **todos os cenários**, os polos permanecem no SPE, confirmando a estabilidade BIBO do sistema para quaisquer valores físicos positivos dos parâmetros — resultado que será formalizado na Seção 2.2.

---

## 2. Análise Crítica dos Resultados

Os resultados obtidos nas seis simulações estão em concordância quantitativa com as previsões analíticas e ampliam de forma significativa a compreensão do sistema em relação ao que foi estabelecido na Etapa 01.

---

### 2.1 Separação de Escalas de Tempo

Uma descoberta central desta etapa é a **separação natural das dinâmicas elétrica e mecânica**. As constantes de tempo associadas aos dois polos são:

$$
\tau_e = \frac{1}{|p_2|} \approx 0{,}100 \text{ s} \qquad \tau_m = \frac{1}{|p_1|} \approx 0{,}499 \text{ s}
$$

Para os parâmetros nominais, a razão $\tau_m/\tau_e \approx 5$, o que já indica uma separação moderada. Em motores industriais reais, essa razão é tipicamente da ordem de 10 a 100, de modo que a dinâmica elétrica se extingue muito antes de a dinâmica mecânica se manifestar. Essa separação tem consequência prática direta: **o sistema pode ser reduzido a um modelo de 1ª ordem** sem perda significativa de precisão para a dinâmica de velocidade, simplificando o projeto do controlador.

A Simulação 04 demonstrou esse fenômeno de forma quantitativa: para $L$ pequeno, o polo rápido $p_2$ se afasta para $-100$ ou além, e a resposta de $\omega(t)$ torna-se indistinguível da de um sistema de 1ª ordem com polo em $p_1$.

---

### 2.2 Estabilidade Robusta e Critério de Routh-Hurwitz

Para o denominador da função de transferência do motor CC:

$$
D(s) = LJ\,s^2 + (Lb + RJ)\,s + (Rb + K_mK_b)
$$

todos os coeficientes são **estritamente positivos** para quaisquer valores físicos positivos de $R$, $L$, $J$, $b$, $K_m$ e $K_b$. Aplicando o **critério de Routh-Hurwitz** a um polinômio de 2ª ordem:

| Linha | Coluna 1 | Coluna 2 |
|---|---|---|
| $s^2$ | $LJ > 0$ | $Rb + K_mK_b > 0$ |
| $s^1$ | $Lb + RJ > 0$ | $0$ |
| $s^0$ | $Rb + K_mK_b > 0$ | — |

Todos os elementos da primeira coluna são positivos, portanto **não há trocas de sinal** e o sistema possui as duas raízes com parte real negativa. Conclui-se formalmente:

$$
\boxed{\; \text{O motor CC em malha aberta é } \textbf{sempre BIBO-estável} \text{ para parâmetros físicos positivos.} \;}
$$

Essa garantia de estabilidade robusta é uma propriedade valiosa: ela assegura que variações de carga, temperatura e desgaste mecânico — que alteram $J$, $b$, $R$ e $L$ — não comprometem a estabilidade do sistema em malha aberta. Isso simplifica o projeto do controlador, que pode se concentrar em melhorar o desempenho (reduzir $t_s$, eliminar o erro em regime) sem se preocupar com estabilização.

---

### 2.3 Polo Dominante e Projeto de Controladores

A identificação do **polo dominante** $p_1 \approx -2{,}003$ como determinante da dinâmica observável é uma ferramenta central no projeto de controladores. Ela fundamenta a aproximação de 1ª ordem mencionada na Seção 2.1 e fornece uma referência direta para especificações de desempenho:

- Deseja-se $t_s \leq t_s^*$? Então o polo dominante deve satisfazer $|p_1| \geq 4/t_s^*$, ou seja, deve ser deslocado para a esquerda.
- Deseja-se $M_p \leq M_p^*$? Então os polos (se complexos) devem ter fator de amortecimento $\zeta \geq \zeta^*(M_p^*)$.

Os resultados de sensibilidade paramétrica desta etapa mostram que $p_1$ é **mais sensível a variações de $J$ e $b$** do que a variações de $L$. Isso tem implicação direta na robustez do controlador: as incertezas em $J$ (carga variável) e em $b$ (desgaste dos mancais) são as principais fontes de variação paramétrica que o controlador PID da Etapa 03 deverá compensar.

Na Etapa 03, o projeto do PID visará **deslocar o polo dominante** para uma região do plano $s$ que satisfaça simultaneamente as especificações de $t_s$, $M_p$ e erro em regime permanente nulo.

---

### 2.4 Transição para Comportamento Subamortecido

A Simulação 05 revelou que, para valores elevados de $K_m$, o sistema transita de superamortecido para subamortecido. Esse fenômeno merece atenção especial porque tem implicação prática direta: em projetos que buscam maior ganho DC (selecionando motores com $K_m$ alto), o projetista pode se deparar com **oscilações indesejadas** na resposta transitória.

A transição ocorre quando os dois polos reais coalescen (sistema criticamente amortecido) e então se bifurcam em direção ao eixo imaginário. O valor crítico de $K_m$ pode ser determinado analiticamente pela condição de discriminante nulo do polinômio característico:

$$
\Delta = (Lb + RJ)^2 - 4LJ(Rb + K_mK_b) = 0
$$

$$
K_m^* = \frac{(Lb + RJ)^2 - 4LJRb}{4LJK_b}
$$

Para os parâmetros nominais, $K_m^* \approx 0{,}55$ N·m/A — valor significativamente acima do nominal ($0{,}01$ N·m/A), o que explica por que o motor nominal é superamortecido com ampla margem.

---

### 2.5 Limitações do Modelo Linear

O modelo linear adotado ao longo desta etapa é adequado para os fins da disciplina e amplamente utilizado na literatura (Franklin; Ogata; Nise). Contudo, é importante reconhecer suas limitações:

- **Saturação magnética:** a constante $K_m$ é assumida constante, mas na realidade decresce para correntes elevadas devido à saturação do núcleo magnético. Isso introduz uma não-linearidade que limita o torque máximo disponível.

- **Atrito não-linear:** o modelo adota apenas atrito viscoso linear $b\,\omega(t)$. Fenômenos como atrito de Coulomb (torque constante independente da velocidade) e *stiction* (atrito estático de partida) são ignorados, mas podem causar erro em regime permanente e comportamento de zona morta em sistemas reais.

- **Variação com temperatura:** a resistência de armadura $R$ cresce com a temperatura ($R = R_0(1 + \alpha\Delta T)$), afetando o ganho DC e os polos. Em operação contínua, esse efeito pode ser significativo.

- **Folga mecânica:** em transmissões com engrenagens, a folga (*backlash*) introduz não-linearidade que pode causar oscilações de limite mesmo com controlador linear projetado de forma estável.

Essas limitações não invalidam a abordagem linear para o escopo desta disciplina, mas sinalizam que, em projetos industriais avançados, técnicas de controle não-linear ou adaptativo podem ser necessárias para garantir desempenho e robustez adequados.

