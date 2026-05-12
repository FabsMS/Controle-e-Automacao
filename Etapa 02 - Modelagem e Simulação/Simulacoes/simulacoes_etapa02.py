"""

Simulações
----------
  Sim 01 – Resposta ao degrau (parâmetros nominais)
  Sim 02 – Variação do momento de inércia J
  Sim 03 – Variação do atrito viscoso b
  Sim 04 – Variação da indutância L
  Sim 05 – Variação da constante de torque Km
  Sim 06 – Diagrama de polos no plano s (consolidado)

"""

import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from scipy import signal

# ─────────────────────────────────────────────────────────────────────────────
# 0. CONFIGURAÇÃO GLOBAL
# ─────────────────────────────────────────────────────────────────────────────

OUTPUT_DIR = ".\\Etapa 02 - Modelagem e Simulação\\Resultados"
os.makedirs(OUTPUT_DIR, exist_ok=True)

# Estilo global dos gráficos
plt.rcParams.update({
    "font.family":      "serif",
    "font.size":        11,
    "axes.titlesize":   12,
    "axes.labelsize":   11,
    "axes.grid":        True,
    "grid.alpha":       0.35,
    "grid.linestyle":   "--",
    "lines.linewidth":  2.0,
    "figure.dpi":       150,
    "savefig.dpi":      200,
    "savefig.bbox":     "tight",
})

# Paleta de cores consistente em todo o arquivo
CORES = ["#1f77b4", "#e07b00", "#2ca02c", "#d62728", "#9467bd", "#8c564b"]


# ─────────────────────────────────────────────────────────────────────────────
# 1. PARÂMETROS NOMINAIS DO MOTOR CC
# ─────────────────────────────────────────────────────────────────────────────

class MotorCC:
    """Parâmetros físicos do motor CC de laboratório (Franklin; Ogata)."""
    R  = 1.0    # Resistência de armadura      [Ω]
    L  = 0.5    # Indutância de armadura        [H]
    J  = 0.01   # Momento de inércia            [kg·m²]
    b  = 0.1    # Atrito viscoso                [N·m·s]
    Km = 0.01   # Constante de torque           [N·m/A]
    Kb = 0.01   # Constante de fcem             [V·s/rad]


def funcao_transferencia(R, L, J, b, Km, Kb):
    """
    Retorna a função de transferência Ω(s)/V(s) do motor CC.

    G(s) = Km / [LJ·s² + (Lb + RJ)·s + (Rb + Km·Kb)]

    Parâmetros
    ----------
    R, L, J, b, Km, Kb : float
        Parâmetros físicos do motor.

    Retorna
    -------
    sys : scipy.signal.TransferFunction
    """
    num = [Km]
    den = [
        L * J,
        L * b + R * J,
        R * b + Km * Kb,
    ]
    return signal.TransferFunction(num, den)


def info_polos(sys_tf):
    """Calcula e retorna polos, ganho DC e constante de tempo dominante."""
    polos  = np.roots(sys_tf.den)
    ganho_dc = sys_tf.num[-1] / sys_tf.den[-1]
    polo_dom = polos[np.argmax(np.real(polos))]   # polo mais próximo da origem
    tau_dom  = -1.0 / np.real(polo_dom)
    return polos, ganho_dc, polo_dom, tau_dom


def salvar(fig, nome):
    """Salva a figura em OUTPUT_DIR e fecha."""
    caminho = os.path.join(OUTPUT_DIR, nome)
    fig.savefig(caminho)
    plt.close(fig)
    print(f"  [✓] Salvo em: {caminho}")


# ─────────────────────────────────────────────────────────────────────────────
# 2. SIMULAÇÃO 01 – Resposta ao degrau (parâmetros nominais)
# ─────────────────────────────────────────────────────────────────────────────

def simulacao_01():
    """
    Resposta ao degrau unitário de tensão com os parâmetros nominais.
    Exibe: curva de ω(t), valor em regime permanente, ts e τ₁.
    """
    print("\n[Sim 01] Resposta ao degrau – parâmetros nominais")

    m   = MotorCC()
    sys = funcao_transferencia(m.R, m.L, m.J, m.b, m.Km, m.Kb)
    polos, g0, p_dom, tau_dom = info_polos(sys)

    t = np.linspace(0, 5, 2000)
    t, y = signal.step(sys, T=t)

    ts_2pct = 4 * tau_dom   # tempo de acomodação (critério 2 %)

    fig, ax = plt.subplots(figsize=(8, 4.5))

    ax.plot(t, y, color=CORES[0], label=r"$\omega(t)$ – resposta ao degrau")
    ax.axhline(g0, color="red", linestyle="--", linewidth=1.4,
               label=rf"$y(\infty) \approx {g0:.4f}$ rad/s/V")
    ax.axvline(tau_dom, color="green", linestyle=":", linewidth=1.4,
               label=rf"$\tau_1 \approx {tau_dom:.3f}$ s  →  63,2 %")
    ax.axvline(ts_2pct, color="orange", linestyle=":", linewidth=1.4,
               label=rf"$t_s \approx {ts_2pct:.2f}$ s  →  critério 2 %")

    # Anotação do ponto 63,2 %
    y_tau = np.interp(tau_dom, t, y)
    ax.annotate(f"63,2 %\n({y_tau:.4f})",
                xy=(tau_dom, y_tau), xytext=(tau_dom + 0.4, y_tau - 0.012),
                arrowprops=dict(arrowstyle="->", color="green"),
                fontsize=9, color="green")

    ax.set_xlabel("Tempo [s]")
    ax.set_ylabel(r"$\omega(t)$ [rad/s/V]")
    ax.set_title(
        "Simulação 01 – Resposta ao degrau unitário: Motor CC (parâmetros nominais)\n"
        rf"$G(s) = {m.Km} \;/\; ({m.L*m.J}s^2 + {m.L*m.b + m.R*m.J}s + {m.R*m.b + m.Km*m.Kb:.4f})$  "
        rf"| Polos: $p_1 \approx {np.real(polos[1]):.3f}$,  $p_2 \approx {np.real(polos[0]):.3f}$"
    )
    ax.legend(fontsize=9)
    ax.set_xlim(0, 5)
    ax.set_ylim(bottom=0)

    salvar(fig, "sim01_degrau_nominal.png")


# ─────────────────────────────────────────────────────────────────────────────
# 3. SIMULAÇÃO 02 – Variação do momento de inércia J
# ─────────────────────────────────────────────────────────────────────────────

def simulacao_02():
    """
    Compara respostas ao degrau para diferentes valores de J.
    Painel esquerdo: curvas de ω(t).
    Painel direito:  tabela de polos × J.
    """
    print("\n[Sim 02] Variação do momento de inércia J")

    m  = MotorCC()
    valores_J = [0.005, 0.01, 0.05, 0.1]
    labels    = [f"J = {v} kg·m²{'  (nominal)' if v == m.J else ''}"
                 for v in valores_J]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5),
                                    gridspec_kw={"width_ratios": [3, 2]})

    t_max = 20
    t = np.linspace(0, t_max, 4000)

    rows = []
    for v, lbl, cor in zip(valores_J, labels, CORES):
        sys = funcao_transferencia(m.R, m.L, v, m.b, m.Km, m.Kb)
        _, y = signal.step(sys, T=t)
        polos, g0, p_dom, tau_dom = info_polos(sys)
        ax1.plot(t, y, color=cor, label=lbl)
        rows.append([
            f"{v}",
            f"{np.real(polos[1]):.3f}",
            f"{np.real(polos[0]):.3f}",
            f"{g0:.4f}",
            f"{4*tau_dom:.1f} s",
        ])

    ax1.set_xlabel("Tempo [s]")
    ax1.set_ylabel(r"$\omega(t)$ [rad/s/V]")
    ax1.set_title("Resposta ao degrau – variação de J")
    ax1.legend(fontsize=9)
    ax1.set_xlim(0, t_max)
    ax1.set_ylim(bottom=0)

    # Tabela no painel direito
    ax2.axis("off")
    col_labels = ["J (kg·m²)", "p₁", "p₂", "G(0)", "ts ≈ 4τ₁"]
    tbl = ax2.table(
        cellText=rows, colLabels=col_labels,
        loc="center", cellLoc="center"
    )
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(9.5)
    tbl.scale(1.15, 1.6)
    for (r, c), cell in tbl.get_celld().items():
        if r == 0:
            cell.set_facecolor("#D9D9D9")
            cell.set_text_props(fontweight="bold")
        elif r % 2 == 0:
            cell.set_facecolor("#F5F5F5")
        cell.set_edgecolor("#AAAAAA")
    ax2.set_title("Polos × J", fontsize=11, pad=12)

    fig.suptitle("Simulação 02 – Variação do Momento de Inércia J", fontsize=13, y=1.01)
    fig.tight_layout()
    salvar(fig, "sim02_variacao_J.png")


# ─────────────────────────────────────────────────────────────────────────────
# 4. SIMULAÇÃO 03 – Variação do atrito viscoso b
# ─────────────────────────────────────────────────────────────────────────────

def simulacao_03():
    """
    Compara respostas ao degrau para diferentes valores de b.
    Evidencia o compromisso ganho DC × velocidade de resposta.
    """
    print("\n[Sim 03] Variação do atrito viscoso b")

    m  = MotorCC()
    valores_b = [0.02, 0.1, 0.5, 1.0]
    labels    = [f"b = {v} N·m·s{'  (nominal)' if v == m.b else ''}"
                 for v in valores_b]

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    ax1, ax2  = axes

    t = np.linspace(0, 8, 3000)

    rows = []
    for v, lbl, cor in zip(valores_b, labels, CORES):
        sys = funcao_transferencia(m.R, m.L, m.J, v, m.Km, m.Kb)
        _, y = signal.step(sys, T=t)
        polos, g0, p_dom, tau_dom = info_polos(sys)
        ax1.plot(t, y, color=cor, label=lbl)
        rows.append([
            f"{v}",
            f"{np.real(polos[1]):.3f}",
            f"{np.real(polos[0]):.3f}",
            f"{g0:.4f}",
        ])

    ax1.set_xlabel("Tempo [s]")
    ax1.set_ylabel(r"$\omega(t)$ [rad/s/V]")
    ax1.set_title("Resposta ao degrau – variação de b")
    ax1.legend(fontsize=9)
    ax1.set_xlim(0, 8)
    ax1.set_ylim(bottom=0)

    # Ganho DC × b no painel direito
    bs   = np.linspace(0.01, 2.0, 500)
    gdc  = m.Km / (m.R * bs + m.Km * m.Kb)
    ax2.plot(bs, gdc, color=CORES[0], linewidth=2)
    ax2.axvline(m.b, color="red", linestyle="--", linewidth=1.3,
                label=f"b nominal = {m.b}")
    ax2.set_xlabel("b [N·m·s]")
    ax2.set_ylabel(r"$G(0)$ [rad/s/V]")
    ax2.set_title(r"Ganho DC  $G(0) = K_m\,/\,(Rb + K_mK_b)$  vs.  b")
    ax2.legend(fontsize=9)

    fig.suptitle("Simulação 03 – Variação do Atrito Viscoso b", fontsize=13, y=1.01)
    fig.tight_layout()
    salvar(fig, "sim03_variacao_b.png")


# ─────────────────────────────────────────────────────────────────────────────
# 5. SIMULAÇÃO 04 – Variação da indutância L
# ─────────────────────────────────────────────────────────────────────────────

def simulacao_04():
    """
    Compara respostas ao degrau para diferentes valores de L.
    Mostra como a dinâmica elétrica (τe = L/R) se separa da mecânica.
    """
    print("\n[Sim 04] Variação da indutância L")

    m  = MotorCC()
    valores_L = [0.05, 0.1, 0.5, 2.0]
    labels    = [
        f"L = {v} H  (τe = {v/m.R:.2f} s){'  (nominal)' if v == m.L else ''}"
        for v in valores_L
    ]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    t = np.linspace(0, 6, 3000)

    for v, lbl, cor in zip(valores_L, labels, CORES):
        sys = funcao_transferencia(m.R, v, m.J, m.b, m.Km, m.Kb)
        _, y = signal.step(sys, T=t)
        ax1.plot(t, y, color=cor, label=lbl)

    ax1.set_xlabel("Tempo [s]")
    ax1.set_ylabel(r"$\omega(t)$ [rad/s/V]")
    ax1.set_title("Resposta ao degrau – variação de L")
    ax1.legend(fontsize=9)
    ax1.set_xlim(0, 6)
    ax1.set_ylim(bottom=0)

    # Posição dos polos × L no painel direito
    Ls = np.logspace(-2, 0.5, 200)
    p1_list, p2_list = [], []
    for v in Ls:
        den = [v * m.J, v * m.b + m.R * m.J, m.R * m.b + m.Km * m.Kb]
        ps  = np.sort(np.real(np.roots(den)))
        p1_list.append(ps[1])   # polo dominante
        p2_list.append(ps[0])   # polo rápido

    ax2.plot(Ls, p1_list, color=CORES[0], label=r"$p_1$ (polo dominante)")
    ax2.plot(Ls, p2_list, color=CORES[1], label=r"$p_2$ (polo rápido)")
    ax2.axvline(m.L, color="red", linestyle="--", linewidth=1.2, label=f"L nominal = {m.L} H")
    ax2.set_xlabel("L [H]  (escala log)")
    ax2.set_ylabel("Re(polo)")
    ax2.set_title("Posição dos polos vs. L")
    ax2.set_xscale("log")
    ax2.legend(fontsize=9)

    fig.suptitle("Simulação 04 – Variação da Indutância L", fontsize=13, y=1.01)
    fig.tight_layout()
    salvar(fig, "sim04_variacao_L.png")


# ─────────────────────────────────────────────────────────────────────────────
# 6. SIMULAÇÃO 05 – Variação da constante de torque Km
# ─────────────────────────────────────────────────────────────────────────────

def simulacao_05():
    """
    Compara respostas ao degrau para diferentes valores de Km.
    Mostra a transição de superamortecido para subamortecido.
    """
    print("\n[Sim 05] Variação da constante de torque Km")

    m  = MotorCC()
    valores_Km = [0.001, 0.01, 0.1, 1.0]
    labels = [
        f"Km = {v} N·m/A{'  (nominal)' if v == m.Km else ''}"
        for v in valores_Km
    ]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    t = np.linspace(0, 6, 3000)

    for v, lbl, cor in zip(valores_Km, labels, CORES):
        sys = funcao_transferencia(m.R, m.L, m.J, m.b, v, m.Kb)
        _, y = signal.step(sys, T=t)
        ax1.plot(t, y, color=cor, label=lbl)

    ax1.set_xlabel("Tempo [s]")
    ax1.set_ylabel(r"$\omega(t)$ [rad/s/V]")
    ax1.set_title("Resposta ao degrau – variação de Km")
    ax1.legend(fontsize=9)
    ax1.set_xlim(0, 6)
    ax1.set_ylim(bottom=0)

    # Trajetória dos polos no plano s × Km (painel direito)
    Kms = np.linspace(0.001, 2.0, 800)
    re1, im1 = [], []
    re2, im2 = [], []
    for v in Kms:
        den = [m.L * m.J, m.L * m.b + m.R * m.J, m.R * m.b + v * m.Kb]
        ps  = np.roots(den)
        ps  = sorted(ps, key=lambda x: np.real(x))
        re1.append(np.real(ps[0]));  im1.append(np.imag(ps[0]))
        re2.append(np.real(ps[1]));  im2.append(np.imag(ps[1]))

    ax2.plot(re1, im1, color=CORES[0], linewidth=1.8, label="Trajeto p₁")
    ax2.plot(re2, im2, color=CORES[1], linewidth=1.8, label="Trajeto p₂")

    # Marca os pontos nominais
    sys_nom = funcao_transferencia(m.R, m.L, m.J, m.b, m.Km, m.Kb)
    p_nom   = np.roots(sys_nom.den)
    ax2.scatter(np.real(p_nom), np.imag(p_nom),
                color="red", zorder=5, s=60, label="Km nominal")

    ax2.axvline(0, color="black", linewidth=0.8)
    ax2.axhline(0, color="black", linewidth=0.8)
    ax2.fill_betweenx([-20, 20], -30, 0, alpha=0.06, color="green")
    ax2.set_xlabel("Re(s)")
    ax2.set_ylabel("Im(s)")
    ax2.set_title("Trajetória dos polos vs. Km (lugar das raízes simplificado)")
    ax2.legend(fontsize=9)
    ax2.set_xlim(-20, 2)
    ax2.set_ylim(-15, 15)

    fig.suptitle("Simulação 05 – Variação da Constante de Torque Km", fontsize=13, y=1.01)
    fig.tight_layout()
    salvar(fig, "sim05_variacao_Km.png")


# ─────────────────────────────────────────────────────────────────────────────
# 7. SIMULAÇÃO 06 – Diagrama de polos consolidado no plano s
# ─────────────────────────────────────────────────────────────────────────────

def simulacao_06():
    """
    Diagrama de polos no plano s para variações de J, b, L e Km.
    Quatro subgráficos — um por parâmetro — em uma única figura.
    """
    print("\n[Sim 06] Diagrama de polos consolidado no plano s")

    m = MotorCC()

    variações = [
        {
            "param":  "J",
            "vals":   [0.005, 0.01, 0.05, 0.1],
            "label":  lambda v: f"J={v}",
            "sys_fn": lambda v: funcao_transferencia(m.R, m.L, v,   m.b, m.Km, m.Kb),
            "xlim":   (-25, 1), "ylim": (-5, 5),
            "titulo": "Variação de J (kg·m²)",
        },
        {
            "param":  "b",
            "vals":   [0.02, 0.1, 0.5, 1.0],
            "label":  lambda v: f"b={v}",
            "sys_fn": lambda v: funcao_transferencia(m.R, m.L, m.J, v,   m.Km, m.Kb),
            "xlim":   (-20, 1), "ylim": (-5, 5),
            "titulo": "Variação de b (N·m·s)",
        },
        {
            "param":  "L",
            "vals":   [0.05, 0.1, 0.5, 2.0],
            "label":  lambda v: f"L={v}",
            "sys_fn": lambda v: funcao_transferencia(m.R, v,   m.J, m.b, m.Km, m.Kb),
            "xlim":   (-110, 1), "ylim": (-5, 5),
            "titulo": "Variação de L (H)",
        },
        {
            "param":  "Km",
            "vals":   [0.001, 0.01, 0.1, 1.0],
            "label":  lambda v: f"Km={v}",
            "sys_fn": lambda v: funcao_transferencia(m.R, m.L, m.J, m.b, v,   m.Kb),
            "xlim":   (-15, 1), "ylim": (-12, 12),
            "titulo": "Variação de Km (N·m/A)",
        },
    ]

    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    axes = axes.flatten()

    for ax, cfg in zip(axes, variações):
        for v, cor in zip(cfg["vals"], CORES):
            sys  = cfg["sys_fn"](v)
            pols = np.roots(sys.den)
            mrk  = "x" if np.all(np.isreal(pols)) else "x"
            ax.scatter(np.real(pols), np.imag(pols),
                       marker="x", s=90, linewidths=2.2,
                       color=cor, label=cfg["label"](v), zorder=5)

        ax.axvline(0, color="black", linewidth=0.9)
        ax.axhline(0, color="black", linewidth=0.9)
        ax.fill_betweenx(
            [cfg["ylim"][0], cfg["ylim"][1]], cfg["xlim"][0], 0,
            alpha=0.07, color="green", label="SPE (estável)"
        )
        ax.set_xlim(cfg["xlim"])
        ax.set_ylim(cfg["ylim"])
        ax.set_xlabel("Re(s)")
        ax.set_ylabel("Im(s)")
        ax.set_title(cfg["titulo"], fontsize=11)
        ax.legend(fontsize=8.5, loc="upper right")

    fig.suptitle(
        "Simulação 06 – Diagrama de Polos no Plano s  (×  =  polo)\n"
        "Região verde = SPE (semiplano esquerdo, região de estabilidade)",
        fontsize=12, y=1.01
    )
    fig.tight_layout()
    salvar(fig, "sim06_diagrama_polos.png")


# ─────────────────────────────────────────────────────────────────────────────
# 8. EXECUÇÃO PRINCIPAL
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":

    simulacao_01()
    simulacao_02()
    simulacao_03()
    simulacao_04()
    simulacao_05()
    simulacao_06()


