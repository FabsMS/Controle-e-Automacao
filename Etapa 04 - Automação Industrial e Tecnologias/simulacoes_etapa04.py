"""


Simulações

  Sim 01 – Lógica Ladder: simulação de uma partida direta de motor (CLP)
  Sim 02 – Sensores industriais: curvas características (termopar, encoder)
  Sim 03 – Protocolo MODBUS RTU: simulação de troca de dados CLP–supervisório
  Sim 04 – Controle PID em CLP (Structured Text): discretização e resposta
  Sim 05 – Controle Preditivo (MPC) vs. PID no motor CC
  Sim 06 – Integração: pirâmide de automação e fluxo de dados
=============================================================================
"""

import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.gridspec as gridspec
from scipy import signal
from scipy.linalg import solve_discrete_are

OUTPUT_DIR = ".\\Etapa 04 - Automação Industrial e Tecnologias\\Resultados"
os.makedirs(OUTPUT_DIR, exist_ok=True)

plt.rcParams.update({
    "font.family":    "serif",
    "font.size":      11,
    "axes.titlesize": 12,
    "axes.labelsize": 11,
    "axes.grid":      True,
    "grid.alpha":     0.35,
    "grid.linestyle": "--",
    "lines.linewidth": 2.0,
    "figure.dpi":     150,
    "savefig.dpi":    200,
    "savefig.bbox":   "tight",
})

CORES = ["#1f77b4","#e07b00","#2ca02c","#d62728","#9467bd","#8c564b","#17becf","#bcbd22"]

# Motor CC nominal (Etapa 02)
R, L, J, b, Km, Kb = 1.0, 0.5, 0.01, 0.1, 0.01, 0.01

def salvar(fig, nome):
    p = os.path.join(OUTPUT_DIR, nome)
    fig.savefig(p)
    plt.close(fig)
    print(f"  [✓] {p}")


# ─────────────────────────────────────────────────────────────────────────────
# SIM 01 – Lógica Ladder: partida direta de motor (simulação de CLP)
# ─────────────────────────────────────────────────────────────────────────────
def simulacao_01():
    """
    Simula a lógica de uma partida direta de motor em Ladder:
      - Botão Liga (S1) → Contator (KM1) latch
      - Botão Desliga (S2) → Reset
      - Proteção térmica (F1) → intertravamento
    Exibe: diagrama Ladder + diagrama de temporização (timing diagram).
    """
    print("\n[Sim 01] Lógica Ladder – partida direta de motor")

    # Temporização simulada
    t = np.arange(0, 10.0, 0.01)
    S1 = np.zeros_like(t)   # Liga
    S2 = np.zeros_like(t)   # Desliga
    F1 = np.zeros_like(t)   # Proteção térmica (dispara em t=7)
    KM = np.zeros_like(t)   # Contator (saída)

    # Eventos
    S1[(t >= 1.0) & (t < 1.5)] = 1
    S2[(t >= 5.0) & (t < 5.5)] = 1
    F1[(t >= 7.0)] = 1

    # Lógica Ladder (simulada ciclo a ciclo)
    latch = False
    for i in range(len(t)):
        if F1[i]: latch = False
        if S1[i]: latch = True
        if S2[i]: latch = False
        KM[i] = 1 if latch else 0

    fig = plt.figure(figsize=(13, 8))
    gs  = gridspec.GridSpec(2, 2, figure=fig, width_ratios=[1.4, 1])

    # Painel esquerdo: diagrama Ladder (ASCII art em texto)
    ax_lad = fig.add_subplot(gs[:, 0])
    ax_lad.axis("off")
    ladder = (
        "  DIAGRAMA LADDER – Partida Direta de Motor\n"
        "  ─────────────────────────────────────────\n\n"
        "  L1 ──┤S1├──┬──┤F1_NA├──────────────(KM1)── L2\n"
        "       Liga  │  Prot.térmica\n"
        "             │\n"
        "             ├──┤KM1├─────────────────────────\n"
        "               (selo/latch)\n\n"
        "  L1 ──┤S2├──┤KM1├──────────────────(KM1_R)── L2\n"
        "       Desliga\n\n"
        "  L1 ──┤F1├──────────────────────────(ALM)─── L2\n"
        "       Prot.  (NA=norm.aberto)\n\n"
        "  ─────────────────────────────────────────\n\n"
        "  CÓDIGO STRUCTURED TEXT equivalente:\n\n"
        "  IF S1 AND NOT F1 THEN\n"
        "      KM1 := TRUE;\n"
        "  END_IF;\n\n"
        "  IF S2 OR F1 THEN\n"
        "      KM1 := FALSE;\n"
        "  END_IF;\n\n"
        "  KM1 := KM1 OR (KM1_latch AND NOT S2\n"
        "                 AND NOT F1);\n\n"
        "  ALM := F1;"
    )
    ax_lad.text(0.02, 0.97, ladder, transform=ax_lad.transAxes,
                fontsize=9.5, va="top", fontfamily="monospace",
                bbox=dict(boxstyle="round", fc="#F8F8F8", alpha=0.9))
    ax_lad.set_title("Diagrama Ladder – Partida Direta", fontsize=11)

    # Painel direito: timing diagram
    signals = [("S1 (Liga)",     S1,  CORES[2]),
               ("S2 (Desliga)",  S2,  CORES[3]),
               ("F1 (Proteção)", F1,  CORES[1]),
               ("KM1 (Contator)",KM,  CORES[0])]

    for idx, (lbl, sig, cor) in enumerate(signals):
        ax = fig.add_subplot(gs[idx // 2, 1] if idx < 4 else None)

    axes_td = []
    for idx, (lbl, sig, cor) in enumerate(signals):
        ax = fig.add_subplot(4, 2, idx*2+2)
        ax.fill_between(t, sig, step="post", alpha=0.4, color=cor)
        ax.step(t, sig, where="post", color=cor, linewidth=1.8)
        ax.set_ylim(-0.2, 1.4)
        ax.set_yticks([0, 1]); ax.set_yticklabels(["OFF","ON"], fontsize=8)
        ax.set_ylabel(lbl, fontsize=9, rotation=0, labelpad=70, va="center")
        ax.set_xlim(0, 10)
        if idx < 3: ax.set_xticklabels([])
        else: ax.set_xlabel("Tempo [s]")
        ax.grid(True, alpha=0.3)
        axes_td.append(ax)

    # Anotações de eventos
    for ax in axes_td:
        ax.axvline(1.0, color="gray", linestyle=":", linewidth=0.9, alpha=0.6)
        ax.axvline(5.0, color="gray", linestyle=":", linewidth=0.9, alpha=0.6)
        ax.axvline(7.0, color="red",  linestyle=":", linewidth=0.9, alpha=0.6)

    axes_td[0].annotate("Liga",    xy=(1.25, 1.15), fontsize=8, color="gray")
    axes_td[1].annotate("Desliga", xy=(5.05, 1.15), fontsize=8, color="gray")
    axes_td[2].annotate("Falha",   xy=(7.05, 1.15), fontsize=8, color="red")

    fig.suptitle("Simulação 01 – Lógica Ladder: Partida Direta de Motor (CLP)",
                 fontsize=13, y=1.01)
    fig.tight_layout()
    salvar(fig, "sim01_ladder_partida_motor.png")


# ─────────────────────────────────────────────────────────────────────────────
# SIM 02 – Sensores industriais: características e condicionamento
# ─────────────────────────────────────────────────────────────────────────────
def simulacao_02():
    """
    Curvas características de dois sensores industriais relevantes
    para o motor CC:
      - Termopar tipo K (temperatura da armadura → V)
      - Encoder incremental (pulsos → velocidade angular)
    """
    print("\n[Sim 02] Sensores industriais")

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    # ── Termopar tipo K ──────────────────────────────────────────────────────
    # Aproximação polinomial NIST (0–1372°C)
    T_range  = np.linspace(0, 400, 500)
    # Coef. NIST tipo K, faixa 0-500°C (simplificada)
    c = [0, 3.9083e-2, -1.1775e-5, -2.3029e-8]
    V_K = sum(c[i]*T_range**i for i in range(len(c)))  # mV

    axes[0].plot(T_range, V_K, color=CORES[0], linewidth=2)
    axes[0].set_xlabel("Temperatura [°C]")
    axes[0].set_ylabel("Tensão de saída [mV]")
    axes[0].set_title("Termopar Tipo K – Característica de saída\n(monitoramento da armadura do motor)")

    # Marcações de pontos de interesse
    for T_ref, cor in [(25,"gray"),(100,CORES[1]),(200,CORES[2]),(300,CORES[3])]:
        V_ref = sum(c[i]*T_ref**i for i in range(len(c)))
        axes[0].annotate(f"  {T_ref}°C\n  {V_ref:.2f} mV",
                         xy=(T_ref, V_ref), fontsize=8, color=cor,
                         arrowprops=dict(arrowstyle="->", color=cor),
                         xytext=(T_ref+30, V_ref-1.5))
        axes[0].plot(T_ref, V_ref, "o", color=cor, markersize=6)

    # ── Encoder incremental ──────────────────────────────────────────────────
    # Simula leitura de encoder: pulsos por revolução → velocidade
    PPR   = 1000          # pulsos por revolução
    Ts    = 0.01          # período de amostragem [s]
    t_enc = np.arange(0, 3.0, Ts)

    # Velocidade real (degrau + ruído)
    omega_real = np.where(t_enc < 1.0, 0.0,
                 np.where(t_enc < 2.0, 10*(1-np.exp(-(t_enc-1)/0.5)), 10.0))
    rng = np.random.default_rng(42)
    omega_real += rng.normal(0, 0.05, len(t_enc))

    # Contagem de pulsos simulada → velocidade medida
    pulsos       = omega_real * PPR / (2*np.pi) * Ts
    omega_medida = pulsos * (2*np.pi) / (PPR * Ts)

    # Filtro passa-baixa (MA de 5 amostras)
    kernel       = np.ones(5)/5
    omega_filt   = np.convolve(omega_medida, kernel, mode='same')

    axes[1].plot(t_enc, omega_real,   color=CORES[3], linewidth=1.2,
                 alpha=0.5, label="Velocidade real")
    axes[1].plot(t_enc, omega_medida, color=CORES[1], linewidth=1.0,
                 alpha=0.7, label="Encoder (com ruído)")
    axes[1].plot(t_enc, omega_filt,   color=CORES[0], linewidth=2.0,
                 label="Encoder (filtro MA-5)")
    axes[1].set_xlabel("Tempo [s]")
    axes[1].set_ylabel(r"$\omega$ [rad/s]")
    axes[1].set_title(f"Encoder Incremental ({PPR} PPR)\nMedição de velocidade do eixo do motor")
    axes[1].legend(fontsize=9)

    fig.suptitle("Simulação 02 – Sensores Industriais: Termopar Tipo K e Encoder Incremental",
                 fontsize=13, y=1.01)
    fig.tight_layout()
    salvar(fig, "sim02_sensores_industriais.png")


# ─────────────────────────────────────────────────────────────────────────────
# SIM 03 – Protocolo MODBUS RTU: simulação de troca de dados
# ─────────────────────────────────────────────────────────────────────────────
def simulacao_03():
    """
    Simula um ciclo de comunicação MODBUS RTU entre CLP (mestre)
    e inversor de frequência (escravo), mostrando:
      - Estrutura do frame de requisição e resposta
      - Latência de comunicação vs. frequência de polling
      - Mapa de registradores típico
    """
    print("\n[Sim 03] Protocolo MODBUS RTU")

    fig, axes = plt.subplots(1, 2, figsize=(13, 5))

    # ── Frame MODBUS (visualização) ──────────────────────────────────────────
    ax = axes[0]
    ax.axis("off")

    frame_req = [
        ("Endereço\nescravo", "01h", CORES[0]),
        ("Código\nfunção", "03h", CORES[1]),
        ("Reg. início\n(Hi)", "00h", CORES[2]),
        ("Reg. início\n(Lo)", "00h", CORES[2]),
        ("Qtd. regs\n(Hi)", "00h", CORES[4]),
        ("Qtd. regs\n(Lo)", "04h", CORES[4]),
        ("CRC\n(Lo)", "xxh", CORES[3]),
        ("CRC\n(Hi)", "xxh", CORES[3]),
    ]
    frame_rsp = [
        ("Endereço\nescravo", "01h", CORES[0]),
        ("Código\nfunção", "03h", CORES[1]),
        ("Byte\ncount", "08h", CORES[5]),
        ("Reg 0\n(Hi)", "00h", CORES[2]),
        ("Reg 0\n(Lo)", "64h", CORES[2]),
        ("Reg 1\n(Hi)", "00h", CORES[2]),
        ("Reg 1\n(Lo)", "0Ah", CORES[2]),
        ("...", "...", "white"),
        ("CRC\n(Lo)", "xxh", CORES[3]),
        ("CRC\n(Hi)", "xxh", CORES[3]),
    ]

    for row_idx, (frame, titulo, y0) in enumerate([
        (frame_req, "REQUISIÇÃO (Mestre → Escravo)", 0.78),
        (frame_rsp, "RESPOSTA   (Escravo → Mestre)", 0.38),
    ]):
        ax.text(0.01, y0+0.08, titulo, transform=ax.transAxes,
                fontsize=9.5, fontweight="bold", color=CORES[0] if row_idx==0 else CORES[2])
        n = len(frame)
        w = 0.88 / n
        for i,(lbl,val,cor) in enumerate(frame):
            x0 = 0.05 + i*w
            rect = mpatches.FancyBboxPatch((x0, y0-0.16), w-0.005, 0.20,
                   boxstyle="round,pad=0.01", fc=cor, ec="white", alpha=0.85,
                   transform=ax.transAxes)
            ax.add_patch(rect)
            ax.text(x0+w/2, y0-0.06, lbl, ha="center", va="center",
                    fontsize=6.5, transform=ax.transAxes, color="white", fontweight="bold")
            ax.text(x0+w/2, y0-0.14, val, ha="center", va="center",
                    fontsize=7, transform=ax.transAxes, color="white")

    # Mapa de registradores
    mapa = (
        "MAPA DE REGISTRADORES (Inversor):\n\n"
        "  40001  Setpoint velocidade [rpm]\n"
        "  40002  Velocidade medida   [rpm]\n"
        "  40003  Corrente armadura   [0.1 A]\n"
        "  40004  Status / Alarmes    [bitmap]\n"
        "  40005  Saída PID           [%]\n"
        "  40006  Temperatura motor   [0.1 °C]"
    )
    ax.text(0.01, 0.28, mapa, transform=ax.transAxes, fontsize=8.5,
            va="top", fontfamily="monospace",
            bbox=dict(boxstyle="round", fc="#F5F5F5", alpha=0.9))
    ax.set_title("Frame MODBUS RTU – Função 03 (Read Holding Registers)", fontsize=10)

    # ── Latência vs. frequência de polling ───────────────────────────────────
    ax2 = axes[1]
    freq_poll = np.array([1, 2, 5, 10, 20, 50, 100])   # Hz
    # Latência MODBUS RTU: T_frame + T_proc + T_prop
    # T_frame = 8 bytes * 10 bits / baud_rate
    baud = 9600
    T_frame_ms = (8 * 10 / baud) * 1000   # ms por byte × n_bytes
    latencia_ms = T_frame_ms * 8 + 2       # frame req + proc ~2ms
    utilizacao  = (1/freq_poll * 1000) / (1/freq_poll * 1000 + latencia_ms) * 100

    ax2b = ax2.twinx()
    l1, = ax2.plot(freq_poll, 1/freq_poll*1000, color=CORES[0],
                   marker="o", label="Período de polling [ms]")
    l2, = ax2b.plot(freq_poll, utilizacao, color=CORES[2],
                    marker="s", linestyle="--", label="Utilização do barramento [%]")
    ax2.axhline(latencia_ms, color=CORES[3], linestyle=":", linewidth=1.5,
                label=f"Latência frame ≈ {latencia_ms:.1f} ms")
    ax2.set_xlabel("Frequência de polling [Hz]")
    ax2.set_ylabel("Período [ms]", color=CORES[0])
    ax2b.set_ylabel("Utilização [%]", color=CORES[2])
    ax2.set_title("Frequência de polling vs. latência\n(MODBUS RTU @ 9600 baud)")
    ax2.set_xscale("log")
    lines = [l1, l2,
             plt.Line2D([0],[0], color=CORES[3], linestyle=":", label=f"Latência ≈ {latencia_ms:.1f} ms")]
    ax2.legend(handles=lines, fontsize=8.5, loc="upper right")

    fig.suptitle("Simulação 03 – Protocolo MODBUS RTU: Frame e Latência de Comunicação",
                 fontsize=13, y=1.01)
    fig.tight_layout()
    salvar(fig, "sim03_modbus_rtu.png")


# ─────────────────────────────────────────────────────────────────────────────
# SIM 04 – PID em Structured Text (CLP): resposta digital
# ─────────────────────────────────────────────────────────────────────────────
def simulacao_04():
    """
    Implementa o bloco PID em Structured Text (IEC 61131-3) e simula
    a resposta do motor CC controlado pelo CLP, comparando com o
    PID contínuo de referência (Etapa 03).
    Exibe: código ST + curva de resposta digital (Ts = 20 ms).
    """
    print("\n[Sim 04] PID em Structured Text (CLP)")

    Kp, Ki, Kd, Ts = 30, 15, 0.2, 0.02   # 50 Hz (típico de CLP)
    T_sim = 6.0
    t_cont = np.linspace(0, T_sim, 5000)

    # Referência: PID contínuo (Etapa 03)
    def pid_cont():
        num_pid = [Kd*100 + Kp, Kp*100 + Ki, Ki*100]
        den_pid = [1, 100, 0]
        C = signal.TransferFunction(num_pid, den_pid)
        num_G = [Km]; den_G = [L*J, L*b+R*J, R*b+Km*Kb]
        G = signal.TransferFunction(num_G, den_G)
        nL = np.polymul(C.num, G.num)
        dL = np.polymul(C.den, G.den)
        nT = nL; dT = np.polyadd(dL, nL)
        T = signal.TransferFunction(nT, dT)
        _, y = signal.step(T, T=t_cont)
        return y

    y_cont = pid_cont()

    # PID digital em ST — simulação numérica
    Ac, Bc, Cc, Dc = signal.tf2ss([Km], [L*J, L*b+R*J, R*b+Km*Kb])
    sys_d  = signal.cont2discrete((Ac, Bc, Cc, Dc), Ts, method='zoh')
    Ad, Bd, Cd, Dd = sys_d[0], sys_d[1], sys_d[2], sys_d[3]

    q0 = Kp + Ki*Ts + Kd/Ts
    q1 = -Kp - 2*Kd/Ts
    q2 = Kd/Ts
    n_steps = int(T_sim/Ts)
    t_d = np.arange(n_steps)*Ts
    x = np.zeros((Ad.shape[0],1)); u=0.0; e=[0.0,0.0,0.0]; y_d=np.zeros(n_steps)
    for k in range(n_steps):
        yk = float((Cd@x).item() + float(Dd.flat[0])*u)
        y_d[k] = yk
        e[2]=e[1]; e[1]=e[0]; e[0]=1.0-yk
        u += q0*e[0]+q1*e[1]+q2*e[2]
        x = Ad@x + Bd*u

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))

    ax1.plot(t_cont, y_cont, color=CORES[0], linewidth=2.5,
             label="PID contínuo (ref. Etapa 03)")
    ax1.step(t_d, y_d, color=CORES[2], linewidth=2, where="post",
             label=f"PID digital ST  Ts = {Ts*1000:.0f} ms (50 Hz)")
    ax1.axhline(1.0, color="gray", linestyle=":", linewidth=1.2, label="r = 1 rad/s")
    ax1.set_xlabel("Tempo [s]"); ax1.set_ylabel(r"$\omega(t)$ [rad/s]")
    ax1.set_title("Resposta: PID Contínuo vs. PID em CLP (ST)")
    ax1.legend(fontsize=9); ax1.set_xlim(0, T_sim); ax1.set_ylim(bottom=0)

    ax2.axis("off")
    st_code = (
        "// BLOCO PID – Structured Text (IEC 61131-3)\n"
        "// Executado a cada ciclo do CLP (Ts = 20 ms)\n\n"
        "VAR\n"
        "  Kp : REAL := 30.0;\n"
        "  Ki : REAL := 15.0;\n"
        "  Kd : REAL := 0.2;\n"
        "  Ts : REAL := 0.02;\n"
        "  q0, q1, q2 : REAL;\n"
        "  e0, e1, e2 : REAL;\n"
        "  u, y      : REAL;\n"
        "END_VAR\n\n"
        "// Inicialização (uma vez)\n"
        "q0 := Kp + Ki*Ts + Kd/Ts;\n"
        "q1 := -Kp - 2.0*Kd/Ts;\n"
        "q2 := Kd/Ts;\n\n"
        "// Ciclo de controle\n"
        "y  := LerEncoder();    // lê sensor\n"
        "e2 := e1; e1 := e0;\n"
        "e0 := Setpoint - y;    // erro atual\n"
        "u  := u + q0*e0 + q1*e1 + q2*e2;\n"
        "u  := LIMIT(0.0, u, 24.0); // anti-windup\n"
        "EscreverDAC(u);        // aplica tensão"
    )
    ax2.text(0.03, 0.97, st_code, transform=ax2.transAxes,
             fontsize=8.8, va="top", fontfamily="monospace",
             bbox=dict(boxstyle="round", fc="#F0F0F0", alpha=0.95))
    ax2.set_title("Código Structured Text – Bloco PID (IEC 61131-3)", fontsize=10)

    fig.suptitle("Simulação 04 – PID em Structured Text: Motor CC Controlado por CLP",
                 fontsize=13, y=1.01)
    fig.tight_layout()
    salvar(fig, "sim04_pid_structured_text.png")


# ─────────────────────────────────────────────────────────────────────────────
# SIM 05 – Controle Preditivo (MPC) vs. PID no motor CC
# ─────────────────────────────────────────────────────────────────────────────
def simulacao_05():
    """
    Implementa um MPC linear simplificado (regulador LQR discreto)
    e compara com o PID da Etapa 03 no motor CC.
    O MPC minimiza J = Σ(y-r)²·Q + u²·R ao longo do horizonte N.
    """
    print("\n[Sim 05] Controle Preditivo MPC vs. PID")

    Ts = 0.02; T_sim = 6.0
    Ac, Bc, Cc, Dc = signal.tf2ss([Km],[L*J,L*b+R*J,R*b+Km*Kb])
    sys_d = signal.cont2discrete((Ac,Bc,Cc,Dc), Ts, method='zoh')
    Ad, Bd, Cd, Dd = sys_d[0], sys_d[1], sys_d[2], sys_d[3]
    n = Ad.shape[0]; t_d = np.arange(int(T_sim/Ts))*Ts

    # LQR (aproximação MPC com horizonte infinito)
    Q_lqr = Cd.T @ np.array([[500]]) @ Cd    # penaliza saída
    R_lqr = np.array([[0.1]])                 # penaliza controle
    try:
        P  = solve_discrete_are(Ad, Bd, Q_lqr, R_lqr)
        K_lqr = np.linalg.inv(R_lqr + Bd.T@P@Bd) @ (Bd.T@P@Ad)
    except Exception:
        K_lqr = np.array([[5.0, 0.5]])        # fallback

    # Simulação MPC/LQR
    def sim_lqr(ref=1.0):
        x = np.zeros((n,1)); u=0.0; y_out=np.zeros(len(t_d))
        # Estado de referência (ponto de equilíbrio para y=ref)
        # x_ref: Ac*x_r = -Bc*u_r, Cc*x_r = ref
        # Solução simplificada: x_ref proporcional
        # Ganho feedforward para rastrear referência
        ff = float(np.linalg.pinv(Cd @ np.linalg.inv(np.eye(n)-Ad) @ Bd).flat[0]) * ref
        for k in range(len(t_d)):
            yk = float((Cd@x).item())
            y_out[k] = yk
            u_lqr = float((-K_lqr @ x).flat[0]) + ff
            x = Ad@x + Bd*u_lqr
        return y_out

    # Simulação PID (Etapa 03)
    Kp,Ki_c,Kd_c = 30,15,0.2
    q0=Kp+Ki_c*Ts+Kd_c/Ts; q1=-Kp-2*Kd_c/Ts; q2=Kd_c/Ts
    def sim_pid(ref=1.0):
        x=np.zeros((n,1)); u=0.0; e=[0.0,0.0,0.0]; y_out=np.zeros(len(t_d))
        for k in range(len(t_d)):
            yk=float((Cd@x).item()+float(Dd.flat[0])*u)
            y_out[k]=yk
            e[2]=e[1]; e[1]=e[0]; e[0]=ref-yk
            u+=q0*e[0]+q1*e[1]+q2*e[2]
            x=Ad@x+Bd*u
        return y_out

    y_lqr = sim_lqr()
    y_pid = sim_pid()

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    axes[0].plot(t_d, y_pid, color=CORES[0], linewidth=2, label="PID (Etapa 03)")
    axes[0].plot(t_d, y_lqr, color=CORES[2], linewidth=2,
                 linestyle="--", label="MPC/LQR (Q=500, R=0.1)")
    axes[0].axhline(1.0, color="gray", linestyle=":", linewidth=1.2, label="r = 1")
    axes[0].set_xlabel("Tempo [s]"); axes[0].set_ylabel(r"$\omega(t)$ [rad/s]")
    axes[0].set_title("Resposta ao degrau: PID vs. MPC (LQR)")
    axes[0].legend(fontsize=9); axes[0].set_xlim(0, T_sim); axes[0].set_ylim(bottom=0)

    # Painel direito: conceito do MPC
    axes[1].axis("off")
    mpc_txt = (
        "CONTROLE PREDITIVO (MPC) – Conceito\n"
        "─────────────────────────────────────\n\n"
        "Objetivo:\n"
        "  Minimizar J = Σ[(y(k)-r)²·Q + u(k)²·R]\n"
        "  sujeito a: modelo da planta, restrições\n\n"
        "Algoritmo (a cada período Ts):\n\n"
        "  1. Medir estado atual x(k)\n"
        "  2. Resolver otimização p/ horizonte N\n"
        "     → sequência ótima u*(k), ..., u*(k+N)\n"
        "  3. Aplicar apenas u*(k)  (princípio\n"
        "     do horizonte deslizante)\n"
        "  4. Avançar k → k+1, repetir\n\n"
        "Vantagens sobre PID:\n"
        "  ✔ Lida com restrições (saturação, segurança)\n"
        "  ✔ Antecipa referências futuras\n"
        "  ✔ Multi-variável (MIMO)\n\n"
        "Desvantagem:\n"
        "  ✘ Maior custo computacional\n"
        "  ✘ Necessita modelo preciso da planta"
    )
    axes[1].text(0.03, 0.97, mpc_txt, transform=axes[1].transAxes,
                 fontsize=9, va="top", fontfamily="monospace",
                 bbox=dict(boxstyle="round", fc="#F5F5F5", alpha=0.9))
    axes[1].set_title("Formulação do MPC Linear", fontsize=10)

    fig.suptitle("Simulação 05 – Controle Preditivo (MPC/LQR) vs. PID: Motor CC",
                 fontsize=13, y=1.01)
    fig.tight_layout()
    salvar(fig, "sim05_mpc_vs_pid.png")


# ─────────────────────────────────────────────────────────────────────────────
# SIM 06 – Pirâmide de automação e integração de sistemas
# ─────────────────────────────────────────────────────────────────────────────
def simulacao_06():
    """
    Visualiza a pirâmide de automação industrial (ISA-95) e o fluxo
    de dados entre os níveis, contextualizando todos os conceitos
    do estudo dirigido dentro da arquitetura industrial moderna.
    """
    print("\n[Sim 06] Pirâmide de automação e integração")

    fig, axes = plt.subplots(1, 2, figsize=(13, 7))

    # ── Pirâmide de automação ────────────────────────────────────────────────
    ax = axes[0]
    ax.axis("off")
    ax.set_xlim(0, 10); ax.set_ylim(0, 10)

    niveis = [
        (5, 0.5, 10, 1.6, CORES[0], "Nível 0 – Campo\nSensores, Atuadores, Motor CC"),
        (4, 2.3,  8, 1.6, CORES[1], "Nível 1 – Controle\nCLPs, PID, Ladder, ST"),
        (3, 4.1,  6, 1.6, CORES[2], "Nível 2 – Supervisório\nSCADA, IHM, Alarmes"),
        (2, 5.9,  4, 1.6, CORES[4], "Nível 3 – Gestão de Produção\nMES, OEE, Qualidade"),
        (1, 7.7,  2, 1.6, CORES[5], "Nível 4 – Corporativo\nERP, SAP, BI"),
    ]

    for x0, y0, w, h, cor, lbl in niveis:
        # Trapézio aproximado com retângulo arredondado
        left  = (10-w)/2
        rect  = mpatches.FancyBboxPatch((left, y0), w, h,
                boxstyle="round,pad=0.1", fc=cor, ec="white",
                alpha=0.85, linewidth=1.5)
        ax.add_patch(rect)
        ax.text(5, y0+h/2, lbl, ha="center", va="center",
                fontsize=8, color="white", fontweight="bold")

    # Protocolos entre níveis
    protocolos = [
        (1.7, "MODBUS / PROFIBUS / IO-Link"),
        (3.5, "OPC-UA / PROFINET / EtherNet/IP"),
        (5.3, "OPC-UA / REST API / MQTT"),
        (7.1, "SQL / Web Services / ERP API"),
    ]
    for y, txt in protocolos:
        ax.annotate("", xy=(4.5, y+0.5), xytext=(4.5, y-0.5),
                    arrowprops=dict(arrowstyle="<->", color="gray", lw=1.5))
        ax.text(5.5, y, txt, fontsize=7, color="gray", va="center")

    ax.set_title("Pirâmide de Automação Industrial (ISA-95)\nContextualização do Estudo Dirigido",
                 fontsize=11)

    # ── Fluxo de dados e latência por nível ─────────────────────────────────
    ax2 = axes[1]
    levels = ["Campo\n(N0)", "Controle\n(N1)", "Supervisório\n(N2)",
              "MES\n(N3)", "ERP\n(N4)"]
    taxa_hz   = [1000, 50, 1, 1/60, 1/3600]   # Hz
    latencia  = [1,    20, 1000, 60000, 3600000]  # ms

    x_pos = np.arange(len(levels))
    bars = ax2.bar(x_pos, np.log10(taxa_hz)+4,
                   color=CORES[:5], alpha=0.8, edgecolor="white", linewidth=1.2)

    ax2.set_xticks(x_pos)
    ax2.set_xticklabels(levels, fontsize=9)
    ax2.set_ylabel("Taxa de atualização (escala log)", fontsize=10)
    ax2.set_title("Taxa de atualização por nível\n(escala log, ref. 0,001 Hz)")

    # Anotações com valores reais
    taxa_str = ["1 kHz","50 Hz","1 Hz","1/min","1/h"]
    for i,(bar,ts) in enumerate(zip(bars, taxa_str)):
        ax2.text(bar.get_x()+bar.get_width()/2, bar.get_height()+0.05,
                 ts, ha="center", va="bottom", fontsize=8.5, fontweight="bold")

    # Legenda com protocolos
    from matplotlib.patches import Patch
    leg_elems = [Patch(fc=CORES[i], label=l, alpha=0.8)
                 for i,l in enumerate(["IO-Link/MODBUS","PROFIBUS/CAN","OPC-UA","REST/SQL","ERP API"])]
    ax2.legend(handles=leg_elems, fontsize=8, title="Protocolo típico",
               loc="lower right")

    fig.suptitle("Simulação 06 – Pirâmide de Automação e Integração de Sistemas Industriais",
                 fontsize=13, y=1.01)
    fig.tight_layout()
    salvar(fig, "sim06_piramide_automacao.png")


# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("="*65)
    print("  Controle e Automação I – Etapa 04: Automação Industrial")
    print(f"  Imagens salvas em: ./{OUTPUT_DIR}/")
    print("="*65)
    simulacao_01()
    simulacao_02()
    simulacao_03()
    simulacao_04()
    simulacao_05()
    simulacao_06()
    print("\n"+"="*65)
    print("  Todas as simulações concluídas com sucesso.")
    print("="*65)
