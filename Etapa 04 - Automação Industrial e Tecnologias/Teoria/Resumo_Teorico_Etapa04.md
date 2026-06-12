# Resumo Teórico – Etapa 04

**Disciplina:** Controle e Automação I
**Instituição:** IFPB – Campus Campina Grande
**Curso:** Engenharia da Computação
**Professor:** Dr. Moacy Pereira da Silva
**Autores:** Fabrício Moreno da Silva · Ynnayron Juan Lopes da Silva

---

## Sumário

1. [Introdução](#1-introdução)
2. [Pirâmide de Automação Industrial](#2-pirâmide-de-automação-industrial)
3. [Elementos Fundamentais](#3-elementos-fundamentais)
4. [Linguagens de Programação para CLPs](#4-linguagens-de-programação-para-clps)
5. [Ferramentas: CodeSys e OpenPLC](#5-ferramentas-codesys-e-openplc)
6. [Protocolos de Comunicação](#6-protocolos-de-comunicação)
7. [Controle Preditivo (MPC)](#7-controle-preditivo-mpc)
8. [Inteligência Artificial em Automação](#8-inteligência-artificial-em-automação)
9. [Integração: do Modelo ao Sistema Industrial](#9-integração-do-modelo-ao-sistema-industrial)
10. [Referências](#10-referências)

---

## 1. Introdução

Esta etapa fecha o ciclo do Estudo Dirigido conectando os conceitos teóricos e de controle desenvolvidos nas Etapas 01–03 com as **tecnologias reais utilizadas na indústria**. O motor CC, modelado na Etapa 02 e controlado pelo PID na Etapa 03, passa agora a ser contextualizado dentro de um sistema de automação industrial completo: desde os sensores de campo até os sistemas corporativos de gestão.

A Etapa 04 responde à seguinte questão central:

> Como os conceitos de modelagem, função de transferência e controle PID se manifestam dentro de uma arquitetura industrial real, e quais tecnologias — CLPs, SCADA, protocolos de comunicação — os implementam na prática?

---

## 2. Pirâmide de Automação Industrial

A **pirâmide de automação** (ISA-95 / IEC 62264) organiza os sistemas industriais em cinco níveis hierárquicos, cada um com função, tecnologia e taxa de atualização características:

| Nível | Nome | Tecnologias típicas | Taxa de atualização |
|---|---|---|---|
| 0 | Campo | Sensores, atuadores, motor CC | 1 kHz – 10 kHz |
| 1 | Controle | CLP, PID, Ladder, ST | 10 ms – 100 ms |
| 2 | Supervisório | SCADA, IHM, OPC-UA | 1 s – 10 s |
| 3 | Gestão de produção | MES, OEE, rastreabilidade | 1 min – 1 h |
| 4 | Corporativo | ERP, SAP, BI | 1 h – 1 dia |

A comunicação entre níveis é realizada por protocolos específicos, com latência e largura de banda adequadas à taxa de atualização de cada camada.

---

## 3. Elementos Fundamentais

### 3.1 Sensores Industriais

Sensores convertem grandezas físicas em sinais elétricos processáveis pelo CLP. Os principais tipos relevantes para o motor CC são descritos a seguir.

**Encoder incremental** — mede posição e velocidade angular por contagem de pulsos. Para $N_{PPR}$ pulsos por revolução e período de amostragem $T_s$:

```math
\omega[k] = \frac{2\pi \cdot \Delta P[k]}{N_{PPR} \cdot T_s} \quad \text{[rad/s]}
```

**Termopar tipo K** — mede a temperatura da armadura, fundamental para proteção contra sobrecarga. A relação tensão–temperatura segue:

```math
V(T) \approx 3{,}908 \times 10^{-2} T - 1{,}178 \times 10^{-5} T^2 \quad \text{[mV]}
```

**Sensor de corrente (shunt / Hall)** — mede $i(t)$ para proteção e controle de torque.

**Tacômetro DC** — gerador de tensão proporcional à velocidade, solução simples para malha de velocidade.

### 3.2 Controladores Lógico-Programáveis (CLPs)

O CLP é o elemento central do nível de controle. Suas características principais são:

- **Ciclo de varredura:** leitura das entradas → execução do programa → atualização das saídas. Tempos típicos: 1–20 ms.
- **Linguagens IEC 61131-3:** Ladder (LD), Structured Text (ST), Function Block Diagram (FBD), Instruction List (IL), Sequential Function Chart (SFC).
- **Módulos de E/S:** analógicos (4–20 mA, 0–10 V) e digitais (24 VDC), com conversores A/D de 12–16 bits.
- **Redundância e segurança:** CLPs de segurança (SIL 2/3) executam verificações internas e trip de emergência.

### 3.3 Sistemas Supervisórios (SCADA) e IHMs

- **SCADA** (*Supervisory Control and Data Acquisition*): software que coleta dados de múltiplos CLPs, gera histórico, alarmes e relatórios. Exemplos: Wonderware, Ignition, WinCC.
- **IHM** (*Interface Homem-Máquina*): painel local com tela touch para operação e diagnóstico. Comunicação com CLP via PROFIBUS ou Ethernet industrial.
- **OPC-UA:** protocolo padrão para integração entre SCADA, CLPs e sistemas corporativos, com suporte a segurança e semântica de dados.

---

## 4. Linguagens de Programação para CLPs

A norma **IEC 61131-3** define cinco linguagens padronizadas:

### 4.1 Ladder (LD)

Baseada em diagramas de relés elétricos. Ideal para lógica de intertravamento e partida de motores. Exemplo — partida direta com selo:

```
  ──┤S1├──┬──┤F1├──(KM1)──
           │
           └──┤KM1├─────────
```

### 4.2 Structured Text (ST)

Linguagem de alto nível similar a Pascal. Permite implementar algoritmos complexos como o PID:

```pascal
(* Bloco PID – IEC 61131-3 ST *)
q0 := Kp + Ki*Ts + Kd/Ts;
q1 := -Kp - 2.0*Kd/Ts;
q2 := Kd/Ts;

e0 := Setpoint - MedVelocidade;
DeltaU := q0*e0 + q1*e1 + q2*e2;
Saida := LIMIT(0.0, Saida + DeltaU, Umax);
e2 := e1; e1 := e0;
```

### 4.3 Function Block Diagram (FBD)

Representação gráfica por blocos funcionais interconectados. O bloco `PID_COMPACT` (Siemens S7) ou `CTRL_PID` (Beckhoff) encapsula toda a lógica de controle.

### 4.4 Sequential Function Chart (SFC)

Baseada em redes de Petri. Ideal para automação de processos sequenciais (partida de máquina, receitas de produção).

---

## 5. Ferramentas: CodeSys e OpenPLC

### 5.1 CodeSys

Ambiente de desenvolvimento IEC 61131-3 amplamente utilizado na indústria (Beckhoff, Wago, Schneider, ABB). Recursos principais:

- Suporte a todas as 5 linguagens IEC 61131-3;
- Simulador de CLP integrado (SoftPLC);
- Biblioteca de blocos para PID, comunicação, segurança;
- Target para Linux, Windows e RTOS embarcados.

### 5.2 OpenPLC

Implementação open-source do runtime IEC 61131-3. Executa em Raspberry Pi, Arduino (com adaptações) e Linux industrial. Permite:

- Programação em Ladder e ST via editor web;
- Integração com Modbus TCP/RTU;
- Prototipagem de sistemas de controle a baixo custo;
- Uso educacional sem licença.

---

## 6. Protocolos de Comunicação

| Protocolo | Camada | Topologia | Taxa | Aplicação típica |
|---|---|---|---|---|
| MODBUS RTU | Serial (RS-485) | Mestre-escravo | 9,6–115 kbps | CLPs, inversores |
| MODBUS TCP | Ethernet | Cliente-servidor | 100 Mbps | Integração SCADA |
| PROFIBUS DP | Serial | Mestre-escravo | até 12 Mbps | Automação de fábrica |
| PROFINET | Ethernet | Estrela/anel | 100 Mbps–1 Gbps | Controle em tempo real |
| EtherNet/IP | Ethernet | Qualquer | 100 Mbps | Rockwell, Allen-Bradley |
| OPC-UA | Ethernet | Cliente-servidor | variável | Integração N2–N4 |
| IO-Link | Ponto-a-ponto | Mestre-escravo | 230 kbps | Sensores inteligentes |

### 6.1 MODBUS RTU — Frame de requisição

A função 03 (Read Holding Registers) utiliza o frame:

```
[Endereço][Função][Reg Hi][Reg Lo][Qtd Hi][Qtd Lo][CRC Lo][CRC Hi]
   01h       03h    00h     00h     00h     04h     xx h    xxh
```

A latência por transação a 9600 baud é aproximadamente:

```math
t_{frame} = \frac{8 \text{ bytes} \times 10 \text{ bits/byte}}{9600 \text{ bps}} \approx 8{,}3 \text{ ms}
```

---

## 7. Controle Preditivo (MPC)

### 7.1 Formulação

O MPC (*Model Predictive Control*) resolve, a cada período $T_s$, um problema de otimização sobre um horizonte finito $N$:

```math
\min_{u} \sum_{k=0}^{N-1} \left[ (y_k - r)^2 Q + u_k^2 R \right]
```

sujeito ao modelo discreto da planta e às restrições de atuador.

Apenas o primeiro elemento da sequência ótima $u^*(0)$ é aplicado (**princípio do horizonte deslizante**). No instante seguinte, o horizonte avança e o problema é resolvido novamente.

### 7.2 Vantagens sobre o PID

- **Restrições explícitas:** o MPC incorpora nativamente limites de atuador ($u_{min} \leq u \leq u_{max}$) e de estado, evitando saturações.
- **Antecipação:** se a referência futura é conhecida, o MPC pode pré-atuar, reduzindo o sobressinal.
- **MIMO:** sistemas com múltiplas entradas e saídas são tratados de forma natural.

### 7.3 LQR como MPC com horizonte infinito

O regulador LQR (*Linear Quadratic Regulator*) é equivalente ao MPC com $N \to \infty$ e sem restrições. O ganho ótimo $K$ é calculado pela equação de Riccati discreta:

```math
K = (R + B^T P B)^{-1} B^T P A
```

onde $P$ é a solução da equação de Riccati: $P = Q + A^T P A - A^T P B K$.

---

## 8. Inteligência Artificial em Automação

### 8.1 Aprendizado por Reforço (RL)

Algoritmos como DQN e PPO aprendem políticas de controle por interação com o ambiente (simulação ou planta real), sem necessidade de modelo explícito. Aplicações: controle de robôs, otimização de processos.

### 8.2 Redes Neurais para Identificação de Sistema

Redes neurais recorrentes (LSTM) podem ser treinadas para identificar a dinâmica de sistemas não-lineares, substituindo o modelo analítico no MPC (*Neural MPC*).

### 8.3 Manutenção Preditiva

Algoritmos de *machine learning* (Random Forest, Isolation Forest) analisam sinais de vibração, corrente e temperatura para detectar falhas incipientes em motores antes que causem parada.

### 8.4 Digital Twin

Réplica virtual do sistema físico, atualizada em tempo real com dados do CLP. Permite simulação de cenários, otimização de parâmetros e treinamento de operadores sem risco à planta real.

---

## 9. Integração: do Modelo ao Sistema Industrial

O percurso completo do Estudo Dirigido pode ser mapeado diretamente nos níveis da pirâmide de automação:

| Etapa | Conceito | Nível ISA-95 | Tecnologia industrial |
|---|---|---|---|
| 01 | Função de transferência, polos, estabilidade | N0 / N1 | Modelo do motor CC |
| 02 | Modelagem física, análise paramétrica | N0 | Datasheet, identificação |
| 03 | Controlador PID, sintonia, PID digital | N1 | Bloco PID em CLP (ST) |
| 04 | CLP, SCADA, protocolos, MPC, IA | N0 a N4 | Sistema industrial completo |

A integração final — **Projeto Final** — apresentará o motor CC completamente controlado, com CLP (OpenPLC), supervisório e comunicação MODBUS, como sistema integrador de todo o estudo dirigido.

---

## 10. Referências

- **OGATA, Katsuhiko.** *Engenharia de Controle Moderno*. 5. ed. São Paulo: Pearson, 2010.
- **NISE, Norman S.** *Engenharia de Sistemas de Controle*. 7. ed. Rio de Janeiro: LTC, 2017.
- **BOLTON, William.** *Programmable Logic Controllers*. 5. ed. Elsevier, 2009.
- **MORAES, Cícero Couto de; CASTRUCCI, Plínio.** *Engenharia de Automação Industrial*. 2. ed. Rio de Janeiro: LTC, 2007.
- **CAMACHO, Eduardo F.; BORDONS, Carlos.** *Model Predictive Control*. 2. ed. Springer, 2004.
- **IEC 61131-3.** *Programmable Controllers – Part 3: Programming Languages*. 3. ed. IEC, 2013.
- **MODBUS.ORG.** *MODBUS Application Protocol Specification V1.1b3*. 2012.
- **CODESYS.** Documentação oficial. Disponível em: https://www.codesys.com. Acesso em: jun. 2026.
- **OPENPLC.** Documentação oficial. Disponível em: https://openplcproject.com. Acesso em: jun. 2026.