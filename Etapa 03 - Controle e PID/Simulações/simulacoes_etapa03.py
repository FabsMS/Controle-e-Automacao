"""
Simulações
----------
  Sim 01 – Malha fechada: resposta com e sem controle (P puro)
  Sim 02 – Efeito do ganho proporcional Kp
  Sim 03 – Efeito da ação integral Ki (eliminação do erro em regime)
  Sim 04 – Efeito da ação derivativa Kd (redução do sobressinal)
  Sim 05 – PID completo: sintonia por tentativa e Ziegler-Nichols
  Sim 06 – PID digital (discretizado, método de Euler)
=============================================================================
"""

import os
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

# ── Configuração global ───────────────────────────────────────────────────────
OUTPUT_DIR = ".\\Etapa 03 - Controle e PID\\Resultados"
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

CORES = ["#1f77b4", "#e07b00", "#2ca02c", "#d62728",
         "#9467bd", "#8c564b", "#17becf"]

# ── Parâmetros nominais do motor CC ───────────────────────────────────────────
R, L, J, b, Km, Kb = 1.0, 0.5, 0.01, 0.1, 0.01, 0.01

def planta():
    """G(s) = Km / [LJ s² + (Lb+RJ) s + (Rb+Km·Kb)]"""
    num = [Km]
    den = [L*J, L*b + R*J, R*b + Km*Kb]
    return signal.TransferFunction(num, den)

def pid_tf(Kp, Ki, Kd, N=100):
    """
    PID com filtro derivativo (forma paralela):
      C(s) = Kp + Ki/s + Kd·N·s/(s+N)
    Retorna TransferFunction do controlador.
    """
    # Numerador e denominador somados em frações parciais
    # C(s) = [Kp·s(s+N) + Ki·(s+N) + Kd·N·s²] / [s(s+N)]
    num = [Kp*1 + Kd*N,           # s²
           Kp*N + Ki + 0,          # s¹  (Kd·N·s term cancels with Kp·N·s)
           Ki*N]                   # s⁰
    # Corrigindo: C(s) numerador exato
    # = (Kd*N + Kp)*s² + (Kp*N + Ki)*s + Ki*N
    num = [Kd*N + Kp, Kp*N + Ki, Ki*N]
    den = [1, N, 0]
    return signal.TransferFunction(num, den)

def malha_fechada_OLD(G, C=None):
    """
    Retorna T(s) = C(s)G(s) / (1 + C(s)G(s)).
    Se C=None, retorna G/(1+G) (realimentação unitária sem controlador).
    """
    if C is None:
        T = signal.feedback(G)
    else:
        L_sys = series(C, G)
        T = signal.feedback(L_sys)
    return T

def series(C, G):
    """Produto em série de duas TransferFunctions."""
    num = np.polymul(C.num, G.num)
    den = np.polymul(C.den, G.den)
    return signal.TransferFunction(num, den)

def malha_fechada(G, C=None):
    """T(s) = L/(1+L), L = C·G ou G se C=None."""
    L = series(C, G) if C is not None else G
    num_T = L.num
    den_T = np.polyadd(L.den, L.num)
    return signal.TransferFunction(num_T, den_T)

def step_info(t, y, ref=1.0):
    """Calcula ts (2%), Mp, ess."""
    yss   = y[-1]
    ess   = abs(ref - yss)
    Mp_v  = max(y) - yss
    Mp    = (Mp_v / yss * 100) if yss > 0 else 0
    band  = 0.02 * yss
    ts    = t[np.where(np.abs(y - yss) > band)[0][-1]] if np.any(np.abs(y - yss) > band) else 0
    return ts, Mp, ess

def salvar(fig, nome):
    caminho = os.path.join(OUTPUT_DIR, nome)
    fig.savefig(caminho)
    plt.close(fig)
    print(f"  [✓] {caminho}")


# ─────────────────────────────────────────────────────────────────────────────
# SIM 01 – Malha aberta vs. malha fechada (proporcional unitário)
# ─────────────────────────────────────────────────────────────────────────────
def simulacao_01():
    print("\n[Sim 01] Malha aberta vs. malha fechada")
    G = planta()
    T_mf = malha_fechada(G)          # realimentação unitária, sem controlador extra

    t = np.linspace(0, 5, 2000)
    _, y_ma = signal.step(G, T=t)
    _, y_mf = signal.step(T_mf, T=t)

    ref = np.ones_like(t)

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))

    # Painel esquerdo: curvas
    axes[0].plot(t, y_ma, color=CORES[0], label="Malha aberta $G(s)$")
    axes[0].plot(t, y_mf, color=CORES[1], label="Malha fechada $T(s) = G/(1+G)$")
    axes[0].axhline(1.0, color="gray", linestyle=":", linewidth=1.2, label="Referência $r=1$")
    axes[0].set_xlabel("Tempo [s]")
    axes[0].set_ylabel(r"$\omega(t)$ [rad/s]")
    axes[0].set_title("Resposta ao degrau unitário")
    axes[0].legend(fontsize=9)
    axes[0].set_xlim(0, 5)

    # Painel direito: diagrama de blocos simplificado (texto)
    axes[1].axis("off")
    blocos = (
        "Malha Aberta:\n\n"
        "  r(t) ──► [ G(s) ] ──► y(t)\n\n\n"
        "Malha Fechada (realimentação unitária):\n\n"
        "         e(t)          \n"
        "  r(t) ──►(+)──► [ G(s) ] ──► y(t)\n"
        "           ↑                    │\n"
        "           └────────────────────┘\n\n\n"
        f"G(0) malha aberta  ≈ {G.num[-1]/G.den[-1]:.4f} rad/s/V\n"
        f"T(0) malha fechada ≈ {signal.step(T_mf, T=t)[1][-1]:.4f} rad/s/V\n\n"
        "Erro em regime permanente:\n"
        f"  Malha aberta:  ess = {abs(1 - y_ma[-1]):.4f}\n"
        f"  Malha fechada: ess = {abs(1 - y_mf[-1]):.4f}"
    )
    axes[1].text(0.05, 0.95, blocos, transform=axes[1].transAxes,
                 fontsize=9.5, verticalalignment="top", fontfamily="monospace",
                 bbox=dict(boxstyle="round", facecolor="#F5F5F5", alpha=0.8))
    axes[1].set_title("Estrutura de controle")

    fig.suptitle("Simulação 01 – Malha Aberta vs. Malha Fechada: Motor CC",
                 fontsize=13, y=1.01)
    fig.tight_layout()
    salvar(fig, "sim01_malha_aberta_vs_fechada.png")


# ─────────────────────────────────────────────────────────────────────────────
# SIM 02 – Efeito do ganho proporcional Kp
# ─────────────────────────────────────────────────────────────────────────────
def simulacao_02():
    print("\n[Sim 02] Efeito do ganho proporcional Kp")
    G = planta()
    t = np.linspace(0, 6, 3000)
    valores_Kp = [1, 5, 20, 50, 100]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    rows = []
    for Kp, cor in zip(valores_Kp, CORES):
        C  = signal.TransferFunction([Kp], [1])
        T  = malha_fechada(G, C)
        _, y = signal.step(T, T=t)
        ts, Mp, ess = step_info(t, y)
        ax1.plot(t, y, color=cor, label=f"Kp = {Kp}")
        rows.append([str(Kp), f"{y[-1]:.4f}", f"{ess:.4f}",
                     f"{Mp:.1f} %", f"{ts:.2f} s"])

    ax1.axhline(1.0, color="gray", linestyle=":", linewidth=1.2, label="r = 1")
    ax1.set_xlabel("Tempo [s]")
    ax1.set_ylabel(r"$\omega(t)$ [rad/s]")
    ax1.set_title("Resposta ao degrau – variação de Kp")
    ax1.legend(fontsize=9)
    ax1.set_xlim(0, 6)
    ax1.set_ylim(bottom=0)

    ax2.axis("off")
    col_labels = ["Kp", "y(∞)", "ess", "Mp", "ts"]
    tbl = ax2.table(cellText=rows, colLabels=col_labels,
                    loc="center", cellLoc="center")
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(9.5)
    tbl.scale(1.1, 1.7)
    for (r, c), cell in tbl.get_celld().items():
        cell.set_edgecolor("#AAAAAA")
        if r == 0:
            cell.set_facecolor("#D9D9D9")
            cell.set_text_props(fontweight="bold")
        elif r % 2 == 0:
            cell.set_facecolor("#F5F5F5")
    ax2.set_title("Indicadores de desempenho × Kp", fontsize=11, pad=12)

    fig.suptitle("Simulação 02 – Efeito do Ganho Proporcional Kp (Controlador P)",
                 fontsize=13, y=1.01)
    fig.tight_layout()
    salvar(fig, "sim02_efeito_Kp.png")


# ─────────────────────────────────────────────────────────────────────────────
# SIM 03 – Efeito da ação integral Ki
# ─────────────────────────────────────────────────────────────────────────────
def simulacao_03():
    print("\n[Sim 03] Efeito da ação integral Ki")
    G  = planta()
    t  = np.linspace(0, 10, 4000)
    Kp = 20   # fixo (referência da Sim 02)
    valores_Ki = [0, 1, 5, 15, 30]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    for Ki, cor in zip(valores_Ki, CORES):
        C  = pid_tf(Kp=Kp, Ki=Ki, Kd=0)
        T  = malha_fechada(G, C)
        _, y = signal.step(T, T=t)
        lbl = f"Ki = {Ki}" + (" (P puro)" if Ki == 0 else "")
        ax1.plot(t, y, color=cor, label=lbl)

    ax1.axhline(1.0, color="gray", linestyle=":", linewidth=1.2, label="r = 1")
    ax1.set_xlabel("Tempo [s]")
    ax1.set_ylabel(r"$\omega(t)$ [rad/s]")
    ax1.set_title(f"Resposta ao degrau – variação de Ki  (Kp = {Kp} fixo)")
    ax1.legend(fontsize=9)
    ax1.set_xlim(0, 10)

    # Painel direito: erro em regime permanente × Ki
    Kis  = np.linspace(0.1, 40, 300)
    ess_list = []
    for Ki in Kis:
        C  = pid_tf(Kp=Kp, Ki=Ki, Kd=0)
        T  = malha_fechada(G, C)
        t2, y2 = signal.step(T, T=np.linspace(0, 15, 3000))
        ess_list.append(abs(1 - y2[-1]))

    ax2.plot(Kis, ess_list, color=CORES[0])
    ax2.axhline(0, color="gray", linestyle="--", linewidth=1)
    ax2.set_xlabel("Ki")
    ax2.set_ylabel("Erro em regime permanente $e_{ss}$")
    ax2.set_title("Erro em regime permanente vs. Ki")

    fig.suptitle("Simulação 03 – Efeito da Ação Integral Ki (Controlador PI)",
                 fontsize=13, y=1.01)
    fig.tight_layout()
    salvar(fig, "sim03_efeito_Ki.png")


# ─────────────────────────────────────────────────────────────────────────────
# SIM 04 – Efeito da ação derivativa Kd
# ─────────────────────────────────────────────────────────────────────────────
def simulacao_04():
    print("\n[Sim 04] Efeito da ação derivativa Kd")
    G  = planta()
    t  = np.linspace(0, 5, 2000)
    Kp, Ki = 20, 10   # fixos
    valores_Kd = [0, 0.05, 0.1, 0.3, 0.6]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    rows = []
    for Kd, cor in zip(valores_Kd, CORES):
        C  = pid_tf(Kp=Kp, Ki=Ki, Kd=Kd)
        T  = malha_fechada(G, C)
        _, y = signal.step(T, T=t)
        ts, Mp, ess = step_info(t, y)
        lbl = f"Kd = {Kd}" + (" (PI puro)" if Kd == 0 else "")
        ax1.plot(t, y, color=cor, label=lbl)
        rows.append([str(Kd), f"{Mp:.1f} %", f"{ts:.2f} s", f"{ess:.4f}"])

    ax1.axhline(1.0, color="gray", linestyle=":", linewidth=1.2, label="r = 1")
    ax1.set_xlabel("Tempo [s]")
    ax1.set_ylabel(r"$\omega(t)$ [rad/s]")
    ax1.set_title(f"Resposta ao degrau – variação de Kd  (Kp={Kp}, Ki={Ki})")
    ax1.legend(fontsize=9)
    ax1.set_xlim(0, 5)

    ax2.axis("off")
    tbl = ax2.table(cellText=rows,
                    colLabels=["Kd", "Mp", "ts", "ess"],
                    loc="center", cellLoc="center")
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(9.5)
    tbl.scale(1.1, 1.8)
    for (r, c), cell in tbl.get_celld().items():
        cell.set_edgecolor("#AAAAAA")
        if r == 0:
            cell.set_facecolor("#D9D9D9")
            cell.set_text_props(fontweight="bold")
        elif r % 2 == 0:
            cell.set_facecolor("#F5F5F5")
    ax2.set_title("Indicadores de desempenho × Kd", fontsize=11, pad=12)

    fig.suptitle("Simulação 04 – Efeito da Ação Derivativa Kd (Controlador PID)",
                 fontsize=13, y=1.01)
    fig.tight_layout()
    salvar(fig, "sim04_efeito_Kd.png")


# ─────────────────────────────────────────────────────────────────────────────
# SIM 05 – PID completo: sintonia manual vs. Ziegler-Nichols
# ─────────────────────────────────────────────────────────────────────────────
def simulacao_05():
    """
    Ziegler-Nichols malha fechada:
      1. Aumentar Kp até oscilação sustentada → Ku (ganho último)
      2. Medir Tu (período de oscilação)
      3. Kp = 0.6·Ku, Ki = 2·Kp/Tu, Kd = Kp·Tu/8
    """
    print("\n[Sim 05] PID completo – sintonia manual vs. Ziegler-Nichols")
    G = planta()
    t = np.linspace(0, 8, 4000)

    # ── Estimar Ku e Tu por varredura de Kp ──
    Ku, Tu = None, None
    for Kp_test in np.arange(1, 500, 0.5):
        C_test = signal.TransferFunction([Kp_test], [1])
        T_test = malha_fechada(G, C_test)
        _, y_test = signal.step(T_test, T=np.linspace(0, 30, 8000))
        # Detecta oscilação sustentada: desvio padrão da última metade alto
        # e valor médio próximo de 1
        tail = y_test[len(y_test)//2:]
        if np.std(tail) > 0.01 and np.mean(tail) > 0.5:
            Ku = Kp_test
            # Estima Tu pela autocorrelação
            tail_c = tail - tail.mean()
            acorr  = np.correlate(tail_c, tail_c, mode='full')
            acorr  = acorr[len(acorr)//2:]
            peaks  = np.where((acorr[1:-1] > acorr[:-2]) &
                               (acorr[1:-1] > acorr[2:]))[0] + 1
            if len(peaks) > 1:
                dt = 30 / 8000
                Tu = (peaks[1] - peaks[0]) * dt
            break

    if Ku is None or Tu is None:
        Ku, Tu = 200, 0.8   # fallback

    # Ganhos Ziegler-Nichols
    Kp_zn = 0.6 * Ku
    Ki_zn = 2.0 * Kp_zn / Tu
    Kd_zn = Kp_zn * Tu / 8.0

    # Ganhos manuais (ajuste fino para bom desempenho)
    Kp_m, Ki_m, Kd_m = 30, 15, 0.2

    configs = [
        ("Sem controle (MF)",    0,     0,     0,     CORES[0], "--"),
        ("Manual",               Kp_m,  Ki_m,  Kd_m,  CORES[1], "-"),
        ("Ziegler-Nichols",      Kp_zn, Ki_zn, Kd_zn, CORES[2], "-"),
    ]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    rows_zn = [
        ["Ku (ganho último)",       f"{Ku:.1f}"],
        ["Tu (período último) [s]", f"{Tu:.3f}"],
        ["Kp = 0,6·Ku",            f"{Kp_zn:.2f}"],
        ["Ki = 2·Kp/Tu",           f"{Ki_zn:.2f}"],
        ["Kd = Kp·Tu/8",           f"{Kd_zn:.3f}"],
    ]

    for label, Kp, Ki, Kd, cor, ls in configs:
        if Kp == 0 and Ki == 0 and Kd == 0:
            T = malha_fechada(G)
        else:
            C = pid_tf(Kp=Kp, Ki=Ki, Kd=Kd)
            T = malha_fechada(G, C)
        _, y = signal.step(T, T=t)
        ax1.plot(t, y, color=cor, linestyle=ls, label=label)

    ax1.axhline(1.0, color="gray", linestyle=":", linewidth=1.2, label="r = 1")
    ax1.set_xlabel("Tempo [s]")
    ax1.set_ylabel(r"$\omega(t)$ [rad/s]")
    ax1.set_title("PID completo: sintonia manual vs. Ziegler-Nichols")
    ax1.legend(fontsize=9)
    ax1.set_xlim(0, 8)
    ax1.set_ylim(bottom=0)

    ax2.axis("off")
    tbl = ax2.table(cellText=rows_zn,
                    colLabels=["Parâmetro Z-N", "Valor"],
                    loc="center", cellLoc="center")
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(9.5)
    tbl.scale(1.3, 1.8)
    for (r, c), cell in tbl.get_celld().items():
        cell.set_edgecolor("#AAAAAA")
        if r == 0:
            cell.set_facecolor("#D9D9D9")
            cell.set_text_props(fontweight="bold")
        elif r % 2 == 0:
            cell.set_facecolor("#F5F5F5")
    ax2.set_title("Parâmetros Ziegler-Nichols", fontsize=11, pad=12)

    fig.suptitle("Simulação 05 – PID Completo: Sintonia Manual vs. Ziegler-Nichols",
                 fontsize=13, y=1.01)
    fig.tight_layout()
    salvar(fig, "sim05_PID_completo.png")


# ─────────────────────────────────────────────────────────────────────────────
# SIM 06 – PID Digital (discretizado por Euler progressivo)
# ─────────────────────────────────────────────────────────────────────────────
def simulacao_06():
    """
    Implementação discreta do PID (método de Euler progressivo):
      u[k] = u[k-1] + q0·e[k] + q1·e[k-1] + q2·e[k-2]
      q0 = Kp + Ki·Ts + Kd/Ts
      q1 = -Kp - 2·Kd/Ts
      q2 = Kd/Ts
    Compara PID contínuo vs. digital para diferentes períodos de amostragem Ts.
    """
    print("\n[Sim 06] PID digital (discretização por Euler)")
    G  = planta()
    Kp, Ki, Kd = 30, 15, 0.2
    T_sim = 8.0
    t_cont = np.linspace(0, T_sim, 5000)

    # Referência contínua para comparação
    C_cont = pid_tf(Kp=Kp, Ki=Ki, Kd=Kd)
    T_cont = malha_fechada(G, C_cont)
    _, y_cont = signal.step(T_cont, T=t_cont)

    def pid_digital(Ts, T_sim=T_sim):
        """Simula o PID digital por integração numérica (Euler)."""
        q0 = Kp + Ki*Ts + Kd/Ts
        q1 = -Kp - 2*Kd/Ts
        q2 = Kd/Ts

        # Converte planta para espaço de estados e discretiza
        Ac, Bc, Cc, Dc = signal.tf2ss(G.num, G.den)
        sys_d = signal.cont2discrete((Ac, Bc, Cc, Dc), Ts, method='zoh')
        Ad, Bd, Cd, Dd = sys_d[0], sys_d[1], sys_d[2], sys_d[3]
        n_steps = int(T_sim / Ts)
        t_d = np.arange(n_steps) * Ts

        x  = np.zeros((Ad.shape[0], 1))
        u  = 0.0
        e  = [0.0, 0.0, 0.0]
        y_d = np.zeros(n_steps)

        for k in range(n_steps):
            r = 1.0
            y_k = float((Cd @ x).item() + float(Dd.flat[0]) * u)
            y_d[k] = y_k
            e[2] = e[1]; e[1] = e[0]; e[0] = r - y_k
            delta_u = q0*e[0] + q1*e[1] + q2*e[2]
            u = u + delta_u
            x = Ad @ x + Bd * u

        return t_d, y_d

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    ax1, ax2  = axes

    ax1.plot(t_cont, y_cont, color=CORES[0], linewidth=2.5,
             label="PID contínuo (referência)")

    amostras = [0.05, 0.1, 0.2, 0.5]
    for Ts, cor in zip(amostras, CORES[1:]):
        t_d, y_d = pid_digital(Ts)
        ax1.step(t_d, y_d, color=cor, linewidth=1.4, where="post",
                 label=f"PID digital  Ts = {Ts} s")

    ax1.axhline(1.0, color="gray", linestyle=":", linewidth=1.2, label="r = 1")
    ax1.set_xlabel("Tempo [s]")
    ax1.set_ylabel(r"$\omega(t)$ [rad/s]")
    ax1.set_title("PID contínuo vs. PID digital (vários Ts)")
    ax1.legend(fontsize=8.5)
    ax1.set_xlim(0, T_sim)
    ax1.set_ylim(bottom=0)

    # Painel direito: pseudocódigo da implementação embarcada
    ax2.axis("off")
    codigo = (
        "// Pseudocódigo – PID digital (C/Arduino)\n\n"
        "float Kp=30, Ki=15, Kd=0.2, Ts=0.05;\n"
        "float q0 = Kp + Ki*Ts + Kd/Ts;\n"
        "float q1 = -Kp - 2*Kd/Ts;\n"
        "float q2 = Kd/Ts;\n\n"
        "float u=0, e0=0, e1=0, e2=0;\n\n"
        "void loop() {\n"
        "  float y  = leSensor();     // ω medida\n"
        "  e2 = e1; e1 = e0;\n"
        "  e0 = referencia - y;       // erro atual\n"
        "  u += q0*e0 + q1*e1 + q2*e2;\n"
        "  atuador(u);                // aplica tensão\n"
        "  delay(Ts * 1000);          // aguarda Ts ms\n"
        "}"
    )
    ax2.text(0.04, 0.95, codigo, transform=ax2.transAxes,
             fontsize=9, verticalalignment="top", fontfamily="monospace",
             bbox=dict(boxstyle="round", facecolor="#F0F0F0", alpha=0.9))
    ax2.set_title("Implementação embarcada (Arduino/CLP)", fontsize=11)

    fig.suptitle("Simulação 06 – PID Digital: Discretização por Euler Progressivo",
                 fontsize=13, y=1.01)
    fig.tight_layout()
    salvar(fig, "sim06_PID_digital.png")


# ─────────────────────────────────────────────────────────────────────────────
# Execução
# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("=" * 65)
    print("  Controle e Automação I – Etapa 03: Controle e PID")
    print(f"  Imagens salvas em: ./{OUTPUT_DIR}/")
    print("=" * 65)

    simulacao_01()
    simulacao_02()
    simulacao_03()
    simulacao_04()
    simulacao_05()
    simulacao_06()

    print("\n" + "=" * 65)
    print("  Todas as simulações concluídas com sucesso.")
    print("=" * 65)