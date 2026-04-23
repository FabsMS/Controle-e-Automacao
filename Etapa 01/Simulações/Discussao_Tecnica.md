# Discussão Técnica – Simulações da Etapa 01

**Disciplina:** Controle e Automação I
**Etapa:** 01 – Fundamentos Teóricos
**Autores:** Fabrício Moreno da Silva · Ynnayron Juan Lopes da Silva

Este documento apresenta a análise crítica dos resultados obtidos em cada uma das cinco simulações desenvolvidas em Python (`simulacoes_controle.py`) e em Scilab (`simulacoes_controle.sce`). Os resultados numéricos e gráficos foram idênticos em ambas as ferramentas, evidenciando a consistência dos modelos implementados.

---

## Simulação 1 – Função de Transferência

**Arquivo:** `resultados/sim1_funcao_transferencia.png`

### Sistemas simulados

- $G_1(s) = \dfrac{1}{s + 2}$ — sistema de 1ª ordem, polo real em $s = -2$;
- $G_2(s) = \dfrac{4}{s^2 + 2s + 4}$ — sistema de 2ª ordem com $\omega_n = 2$ rad/s e $\zeta = 0{,}5$.

### Observações

**Para $G_1(s)$:**

- A resposta ao impulso é uma exponencial decrescente pura, $y(t) = e^{-2t}$, consistente com a transformada inversa de $1/(s+2)$.
- A resposta ao degrau converge monotonicamente a $0{,}5$, exatamente o valor do ganho DC $G_1(0) = 1/2$.
- Não há sobre-sinal nem oscilação — característica típica de sistemas com polo real negativo único.

**Para $G_2(s)$:**

- A resposta ao impulso oscila e decai, com um pico inicial seguido de uma pequena reversão de sinal — comportamento próprio de sistemas subamortecidos.
- A resposta ao degrau apresenta sobre-sinal (aprox. 16 %) e se acomoda em torno do valor final $G_2(0) = 1$.
- A frequência das oscilações corresponde à frequência natural amortecida $\omega_d = \omega_n\sqrt{1-\zeta^2} \approx 1{,}73$ rad/s.

### Análise crítica

A simulação confirma a ligação direta entre a estrutura algébrica de $G(s)$ e o comportamento temporal. O ganho DC é facilmente obtido por $G(0)$, e a natureza da resposta (monotônica vs. oscilatória) é inferida imediatamente da ordem do denominador e da presença (ou não) de parte imaginária nos polos. Esses dois sistemas servirão como referência visual ao longo de todo o semestre.

---

## Simulação 2 – Polos e Zeros no Plano s

**Arquivo:** `resultados/sim2_polos_zeros_plano_s.png`

### Configurações avaliadas

| Painel | Função de Transferência | Polos | Classe |
|---|---|---|---|
| (a) | $G(s) = 2/(s+2)$ | $s = -2$ | Real negativo |
| (b) | $G(s) = 4/(s^2 + s + 4)$ | $s = -0{,}5 \pm j\,1{,}94$ | Complexos (SPE) |
| (c) | $G(s) = 4/(s^2 + 4)$ | $s = 0 \pm j\,2$ | Imaginários puros |
| (d) | $G(s) = 2/(s-1)$ | $s = +1$ | Real positivo |

### Observações

- A região sombreada em verde representa o **semiplano esquerdo** (SPE), isto é, o **espaço de estabilidade** para sistemas LTI contínuos.
- No painel (a), o polo está dentro da região estável; o sistema é BIBO-estável.
- No painel (b), ambos os polos têm parte real negativa; o sistema é estável mas apresentará oscilação amortecida.
- No painel (c), os polos estão exatamente sobre o eixo imaginário, caracterizando **estabilidade marginal** — oscilação sustentada sem crescimento.
- No painel (d), o polo está no SPD; o sistema é instável.

### Análise crítica

A visualização dos polos é uma das ferramentas mais poderosas da teoria clássica de controle. Em apenas um diagrama conseguimos antecipar: estabilidade, presença ou não de oscilações, velocidade relativa da resposta e natureza do transitório. Essa simulação reforça a ideia de que **projetar um controlador é, em última instância, manipular a localização dos polos em malha fechada** — tema que será central nas Etapas 3 e 4.

---

## Simulação 3 – Estabilidade

**Arquivo:** `resultados/sim3_estabilidade.png`

### Sistemas comparados

| Caso | FT | Polos | Re($p$) |
|---|---|---|---|
| Estável | $5/(s^2 + 2s + 5)$ | $-1 \pm j\,2$ | $-1 < 0$ |
| Marginal | $4/(s^2 + 4)$ | $\pm j\,2$ | $0$ |
| Instável | $5/(s^2 - 2s + 5)$ | $+1 \pm j\,2$ | $+1 > 0$ |

### Observações

- O **sistema estável** apresenta uma resposta ao degrau com sobre-sinal inicial que se amortece até atingir o valor final $G(0) = 1$. Esse comportamento é coerente com $\zeta \approx 0{,}447$ (calculado a partir da forma canônica de 2ª ordem).
- O **sistema marginalmente estável** oscila indefinidamente com amplitude constante entre 0 e 2, exatamente como previsto para polos imaginários puros. Observe que o valor médio da oscilação é $G(0) = 1$.
- O **sistema instável** diverge rapidamente. O gráfico foi truncado em $t = 3{,}5$ s para preservar a legibilidade — caso contrário, as oscilações crescentes chegariam rapidamente a amplitudes da ordem de centenas.

### Análise crítica

Esta simulação evidencia visualmente por que a regra "**todos os polos no SPE**" é tão rigorosa na engenharia. A diferença entre o sistema estável e o instável é apenas o sinal do coeficiente de $s$ no denominador ($+2$ vs. $-2$) — uma mudança que em um circuito real poderia decorrer de um simples erro de polaridade em uma realimentação. O resultado é a diferença entre um sistema que cumpre sua função e um que se autodestrói em segundos. Esse entendimento é a base de toda a **verificação de estabilidade em malha fechada**, assunto central da Etapa 3.

---

## Simulação 4 – Resposta Temporal

**Arquivo:** `resultados/sim4_resposta_temporal.png`

### Sistemas de 1ª ordem — painel esquerdo

Avaliaram-se três valores de constante de tempo $\tau \in \{0{,}5;\; 1{,}0;\; 2{,}0\}$ s para $G(s) = 1/(\tau s + 1)$.

- Todas as curvas convergem para o mesmo valor final (ganho DC unitário).
- Em $t = \tau$, $y(t) = 1 - e^{-1} \approx 0{,}632$ — a linha pontilhada vermelha confirma esse ponto para todos os três casos.
- O tempo de acomodação de 2% é $\approx 4\tau$, valendo portanto $\approx 2$ s, 4 s e 8 s, respectivamente. A curva com $\tau = 2{,}0$ s ainda não se estabilizou em 6 s, em conformidade com a teoria.

### Sistemas de 2ª ordem — painel direito

Avaliaram-se cinco valores de $\zeta$ para $\omega_n = 1$ rad/s:

| $\zeta$ | Classificação | $M_p$ teórico | Observação |
|---|---|---|---|
| 0,200 | Subamortecido | $\approx 52{,}7\%$ | Oscilações pronunciadas |
| 0,500 | Subamortecido | $\approx 16{,}3\%$ | Sobre-sinal moderado |
| 0,707 | Subamortecido | $\approx 4{,}3\%$ | **Compromisso ótimo** |
| 1,000 | Criticamente amortecido | 0 % | Resposta mais rápida sem sobre-sinal |
| 2,000 | Superamortecido | 0 % | Resposta lenta, sem oscilação |

### Análise crítica

Os resultados batem quantitativamente com as fórmulas analíticas apresentadas no resumo teórico. O valor $\zeta = 0{,}707$ é particularmente interessante: é adotado como referência em muitos projetos de controle porque atinge um compromisso clássico entre **velocidade** (tempo de subida pequeno) e **conforto transitório** (sobre-sinal abaixo de 5 %). Esse parâmetro reaparecerá na Etapa 3, quando ajustarmos o PID para o motor CC.

Uma observação relevante: o tempo de acomodação **não diminui monotonicamente** com o aumento de $\zeta$. Para $\zeta < 0{,}7$ o sistema é rápido porém oscilatório; para $\zeta > 1$ o sistema é lento e superamortecido. O ponto de mínimo global de $t_s$ ocorre próximo a $\zeta = 0{,}7$ — exatamente a razão para seu uso como referência prática.

---

## Simulação 5 – Motor CC (PBL)

**Arquivo:** `resultados/sim5_motor_cc_preview.png`

### Resumo do sistema

$$
\frac{\Omega(s)}{V(s)} = \frac{0{,}01}{0{,}005\,s^2 + 0{,}06\,s + 0{,}1001}
$$

Polos calculados: $p_1 \approx -2{,}003$, $p_2 \approx -9{,}997$.

### Observações

- **Resposta ao degrau (1 V):** a velocidade angular converge para $\omega(\infty) \approx 0{,}0999$ rad/s, em perfeita concordância com o ganho DC calculado $G(0) = K_m/(Rb + K_m K_b)$.
- **Resposta ao impulso:** apresenta um pico inicial rápido (aprox. $0{,}13$ rad/s em $t \approx 0{,}2$ s) seguido de decaimento monotônico, típico de sistemas de 2ª ordem superamortecidos.
- **Mapa de polos:** ambos os polos são reais, negativos e bem separados. O polo em $-2{,}003$ domina a resposta (constante de tempo $\tau_1 \approx 0{,}5$ s), enquanto o polo em $-9{,}997$ influencia apenas a parte inicial da resposta ($\tau_2 \approx 0{,}1$ s, cinco vezes mais rápido).

### Análise crítica

O motor CC em malha aberta é **estável naturalmente** — os dois polos estão confortavelmente no SPE. No entanto, três limitações motivam o uso de controle:

1. **Baixo ganho DC:** uma entrada de 1 V produz apenas $\approx 0{,}1$ rad/s. Para atingir velocidades úteis seriam necessárias tensões altas, com consumo excessivo.
2. **Erro em regime permanente:** em malha aberta, o motor não rastreia velocidades de referência — apenas responde a tensões. Qualquer distúrbio de carga altera a velocidade final.
3. **Tempo de acomodação longo:** aprox. 2 s é aceitável para acionamentos, mas pode ser melhorado com realimentação e controle proporcional + integral.

Essas três limitações são exatamente o **problema de controle** que vamos resolver no restante do estudo dirigido. A Etapa 2 formalizará ainda mais a modelagem (incluindo posição angular e torque de carga). A Etapa 3 introduzirá um controlador PID que eliminará o erro em regime e tornará a resposta mais rápida. A Etapa 4 discutirá como implementar esse controle em tempo real em um CLP ou microcontrolador.

---

## Conclusão geral da Etapa 01

As cinco simulações formam uma **cadeia lógica progressiva**:

1. Introdução ao conceito de função de transferência (**Sim. 1**);
2. Visualização de polos e zeros no plano complexo (**Sim. 2**);
3. Critério de estabilidade aplicado a três casos (**Sim. 3**);
4. Caracterização quantitativa da resposta temporal (**Sim. 4**);
5. Aplicação de toda a teoria a um sistema físico real (**Sim. 5**).

O motor CC analisado na Simulação 5 será o **sistema fio-condutor** das próximas etapas: a mesma função de transferência será retomada para modelagem detalhada (Etapa 2), projeto de PID (Etapa 3) e implementação industrial (Etapa 4), culminando no Projeto Final Integrado.

Os resultados obtidos estão em concordância com a literatura clássica (Ogata; Nise; Dorf & Bishop) e confirmam, numericamente, as propriedades previstas pela teoria. A paridade entre os resultados obtidos em Python (`scipy.signal`) e em Scilab (`csim`) valida também a robustez da implementação computacional.