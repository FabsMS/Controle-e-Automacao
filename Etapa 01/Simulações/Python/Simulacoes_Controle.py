import os
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
 
# -----------------------------------------------------------------------------
# Configuracao global dos graficos
# -----------------------------------------------------------------------------
plt.rcParams.update({
    "figure.figsize": (11, 7),
    "figure.dpi": 110,
    "axes.grid": True,
    "grid.alpha": 0.35,
    "axes.titlesize": 12,
    "axes.labelsize": 10,
    "legend.fontsize": 9,
    "lines.linewidth": 1.8,
})
 
# Caminho de saida (relativo ao arquivo do script)
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.abspath(os.path.join(BASE_DIR, "..", "..", "resultados"))
os.makedirs(OUT_DIR, exist_ok=True)
 
 
def save_fig(fig, nome):
    """Salva a figura no diretorio de resultados com nome padronizado."""
    caminho = os.path.join(OUT_DIR, nome)
    fig.savefig(caminho, bbox_inches="tight", dpi=120)
    print(f"[OK] Figura salva: {caminho}")
 
 
# =============================================================================
# SIMULACAO 1 - Funcao de Transferencia: resposta ao impulso e ao degrau
# =============================================================================
# Objetivo: apresentar o conceito de funcao de transferencia G(s) = Y(s)/U(s)
# atraves de dois sistemas de referencia e suas respostas no tempo.
#
#   G1(s) = 1 / (s + 2)              -> 1a ordem, polo real em s = -2
#   G2(s) = 4 / (s^2 + 2s + 4)       -> 2a ordem, wn = 2, zeta = 0.5
# =============================================================================
def simulacao_1_funcao_transferencia():
    print("\n>>> Simulacao 1 - Funcao de Transferencia")
 
    # Sistema de 1a ordem: G1(s) = 1 / (s + 2)
    num1, den1 = [1], [1, 2]
    G1 = signal.TransferFunction(num1, den1)
 
    # Sistema de 2a ordem: G2(s) = 4 / (s^2 + 2s + 4)
    num2, den2 = [4], [1, 2, 4]
    G2 = signal.TransferFunction(num2, den2)
 
    # Vetor de tempo
    t = np.linspace(0, 8, 1000)
 
    # Resposta ao impulso
    t1_imp, y1_imp = signal.impulse(G1, T=t)
    t2_imp, y2_imp = signal.impulse(G2, T=t)
 
    # Resposta ao degrau
    t1_step, y1_step = signal.step(G1, T=t)
    t2_step, y2_step = signal.step(G2, T=t)
 
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
 
    axes[0, 0].plot(t1_imp, y1_imp, color="#1f77b4")
    axes[0, 0].set_title(r"Resposta ao Impulso - $G_1(s)=\dfrac{1}{s+2}$")
    axes[0, 0].set_xlabel("Tempo [s]")
    axes[0, 0].set_ylabel("y(t)")
 
    axes[0, 1].plot(t2_imp, y2_imp, color="#d62728")
    axes[0, 1].set_title(r"Resposta ao Impulso - $G_2(s)=\dfrac{4}{s^2+2s+4}$")
    axes[0, 1].set_xlabel("Tempo [s]")
    axes[0, 1].set_ylabel("y(t)")
 
    axes[1, 0].plot(t1_step, y1_step, color="#1f77b4")
    axes[1, 0].axhline(1.0, color="gray", linestyle="--", linewidth=1)
    axes[1, 0].set_title(r"Resposta ao Degrau - $G_1(s)$")
    axes[1, 0].set_xlabel("Tempo [s]")
    axes[1, 0].set_ylabel("y(t)")
 
    axes[1, 1].plot(t2_step, y2_step, color="#d62728")
    axes[1, 1].axhline(1.0, color="gray", linestyle="--", linewidth=1)
    axes[1, 1].set_title(r"Resposta ao Degrau - $G_2(s)$")
    axes[1, 1].set_xlabel("Tempo [s]")
    axes[1, 1].set_ylabel("y(t)")
 
    fig.suptitle("Simulacao 1 - Funcao de Transferencia: respostas ao impulso e ao degrau",
                 fontsize=13, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    save_fig(fig, "sim1_funcao_transferencia.png")
    plt.close(fig)
 
 
# =============================================================================
# SIMULACAO 2 - Polos e Zeros no Plano s
# =============================================================================
# Objetivo: mostrar como a localizacao dos polos no plano complexo
# influencia o comportamento temporal. Comparamos 4 configuracoes:
#   (a) polo real negativo     -> decaimento exponencial
#   (b) polos complexos (SPE)  -> oscilacao amortecida
#   (c) polos no eixo imag.    -> oscilacao sustentada (marginal)
#   (d) polo real positivo     -> divergencia exponencial
# =============================================================================
def simulacao_2_polos_zeros():
    print("\n>>> Simulacao 2 - Polos e Zeros no Plano s")
 
    sistemas = {
        "(a) Polo real negativo\nG(s) = 2/(s+2)":
            ([2], [1, 2]),
        "(b) Polos complexos no SPE\nG(s) = 4/(s^2+s+4)":
            ([4], [1, 1, 4]),
        "(c) Polos no eixo imaginario\nG(s) = 4/(s^2+4)":
            ([4], [1, 0, 4]),
        "(d) Polo real positivo\nG(s) = 2/(s-1)":
            ([2], [1, -1]),
    }
 
    fig, axes = plt.subplots(2, 2, figsize=(12, 9))
    t = np.linspace(0, 10, 1500)
 
    for ax, (titulo, (num, den)) in zip(axes.flatten(), sistemas.items()):
        sys = signal.TransferFunction(num, den)
        zeros = np.roots(num) if len(num) > 1 else np.array([])
        polos = np.roots(den)
 
        # Plano s
        ax.axhline(0, color="black", linewidth=0.7)
        ax.axvline(0, color="black", linewidth=0.7)
        if len(polos) > 0:
            ax.scatter(polos.real, polos.imag, marker="x", s=140,
                       color="#d62728", linewidths=2.5, label="Polos")
        if len(zeros) > 0:
            ax.scatter(zeros.real, zeros.imag, marker="o", s=100,
                       facecolors="none", edgecolors="#1f77b4",
                       linewidths=2, label="Zeros")
 
        # Regiao de estabilidade sombreada (SPE)
        ax.axvspan(-10, 0, alpha=0.08, color="green")
 
        ax.set_xlim(-4, 4)
        ax.set_ylim(-4, 4)
        ax.set_aspect("equal")
        ax.set_xlabel(r"Re($s$)")
        ax.set_ylabel(r"Im($s$)")
        ax.set_title(titulo, fontsize=10)
        ax.legend(loc="upper right", fontsize=8)
 
    fig.suptitle("Simulacao 2 - Localizacao de polos e zeros no plano s",
                 fontsize=13, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    save_fig(fig, "sim2_polos_zeros_plano_s.png")
    plt.close(fig)
 
 
# =============================================================================
# SIMULACAO 3 - Analise de Estabilidade
# =============================================================================
# Objetivo: relacionar a posicao dos polos com a resposta temporal ao degrau,
# evidenciando os tres cenarios de estabilidade:
#   - Estavel:    todos os polos no SPE (Re < 0)
#   - Marginal:   polos sobre o eixo imaginario
#   - Instavel:   ao menos um polo no SPD (Re > 0)
# =============================================================================
def simulacao_3_estabilidade():
    print("\n>>> Simulacao 3 - Analise de Estabilidade")
 
    sistemas = [
        ("Estavel:   polos em -1 +/- j2",   [5],    [1, 2, 5],  "#2ca02c"),
        ("Marginal: polos em 0 +/- j2",     [4],    [1, 0, 4],  "#ff7f0e"),
        ("Instavel: polos em +1 +/- j2",    [5],    [1, -2, 5], "#d62728"),
    ]
 
    fig, axes = plt.subplots(1, 2, figsize=(13, 6))
 
    # Plano s com os polos dos 3 sistemas
    ax1 = axes[0]
    ax1.axhline(0, color="black", linewidth=0.7)
    ax1.axvline(0, color="black", linewidth=0.7)
    ax1.axvspan(-5, 0, alpha=0.08, color="green", label="Regiao estavel (SPE)")
 
    for titulo, num, den, cor in sistemas:
        polos = np.roots(den)
        ax1.scatter(polos.real, polos.imag, marker="x", s=140,
                    color=cor, linewidths=2.5, label=titulo)
 
    ax1.set_xlim(-3, 3)
    ax1.set_ylim(-3.5, 3.5)
    ax1.set_xlabel(r"Re($s$)")
    ax1.set_ylabel(r"Im($s$)")
    ax1.set_title("Polos no plano s")
    ax1.legend(loc="upper right", fontsize=8)
    ax1.set_aspect("equal")
 
    # Respostas ao degrau
    # Usamos tempos diferentes para que o sistema instavel nao domine
    # visualmente o grafico com oscilacoes de amplitude gigante.
    ax2 = axes[1]
    tempos = {
        "Estavel":  np.linspace(0, 10, 2000),
        "Marginal": np.linspace(0, 10, 2000),
        "Instavel": np.linspace(0, 3.5, 1500),  # truncado
    }
 
    for titulo, num, den, cor in sistemas:
        sys = signal.TransferFunction(num, den)
        nome = titulo.split(":")[0].strip()
        ti, yi = signal.step(sys, T=tempos[nome])
        ax2.plot(ti, yi, label=nome, color=cor)
 
    ax2.axhline(0, color="gray", linewidth=0.6)
    ax2.set_xlabel("Tempo [s]")
    ax2.set_ylabel("y(t)")
    ax2.set_title("Resposta ao degrau dos tres sistemas")
    ax2.set_xlim(0, 10)
    ax2.set_ylim(-3, 8)
    ax2.legend(loc="upper left", fontsize=9)
 
    fig.suptitle("Simulacao 3 - Estabilidade: relacao entre polos e resposta temporal",
                 fontsize=13, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    save_fig(fig, "sim3_estabilidade.png")
    plt.close(fig)
 
 
# =============================================================================
# SIMULACAO 4 - Resposta Temporal: 1a ordem e 2a ordem
# =============================================================================
# Objetivo: quantificar parametros classicos da resposta temporal:
#
#   - 1a ordem: G(s) = K/(tau*s + 1)
#         tempo de subida (10%->90%): tr ~ 2.2*tau
#         tempo de acomodacao (2%):   ts ~ 4*tau
#
#   - 2a ordem: G(s) = wn^2 / (s^2 + 2*zeta*wn*s + wn^2)
#         sub-amortecido (0<zeta<1), criticamente amortecido (zeta=1),
#         super-amortecido (zeta>1)
# =============================================================================
def simulacao_4_resposta_temporal():
    print("\n>>> Simulacao 4 - Resposta Temporal")
 
    fig, axes = plt.subplots(1, 2, figsize=(13, 6))
 
    # ----- 1a ordem: efeito da constante de tempo tau -----
    t1 = np.linspace(0, 6, 1000)
    taus = [0.5, 1.0, 2.0]
    ax1 = axes[0]
    for tau in taus:
        sys = signal.TransferFunction([1], [tau, 1])
        ti, yi = signal.step(sys, T=t1)
        ax1.plot(ti, yi, label=fr"$\tau$ = {tau:.1f} s")
 
    ax1.axhline(1.0, color="gray", linestyle="--", linewidth=1, label="Valor final")
    ax1.axhline(0.632, color="red", linestyle=":", linewidth=1, label="63.2% (t = $\\tau$)")
    ax1.set_title(r"1a ordem: $G(s) = \dfrac{1}{\tau s + 1}$")
    ax1.set_xlabel("Tempo [s]")
    ax1.set_ylabel("y(t)")
    ax1.legend(loc="lower right", fontsize=9)
 
    # ----- 2a ordem: efeito do fator de amortecimento zeta -----
    t2 = np.linspace(0, 12, 1500)
    wn = 1.0
    zetas = [0.2, 0.5, 0.707, 1.0, 2.0]
    ax2 = axes[1]
    for z in zetas:
        num = [wn**2]
        den = [1, 2*z*wn, wn**2]
        sys = signal.TransferFunction(num, den)
        ti, yi = signal.step(sys, T=t2)
        ax2.plot(ti, yi, label=fr"$\zeta$ = {z:.3f}")
 
    ax2.axhline(1.0, color="gray", linestyle="--", linewidth=1)
    ax2.set_title(r"2a ordem: $G(s) = \dfrac{\omega_n^2}{s^2 + 2\zeta\omega_n s + \omega_n^2}$ ($\omega_n=1$)")
    ax2.set_xlabel("Tempo [s]")
    ax2.set_ylabel("y(t)")
    ax2.legend(loc="lower right", fontsize=9)
 
    fig.suptitle("Simulacao 4 - Resposta temporal: 1a e 2a ordem",
                 fontsize=13, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    save_fig(fig, "sim4_resposta_temporal.png")
    plt.close(fig)
 
 
# =============================================================================
# SIMULACAO 5 - Motor CC (PBL - preview para a Etapa 02)
# =============================================================================
# Objetivo: introduzir o sistema fio-condutor do estudo dirigido. Aqui usamos
# parametros tipicos para gerar a funcao de transferencia velocidade/tensao:
#
#   omega(s)/V(s) = Km / [ (L*s + R)*(J*s + b) + Km*Kb ]
#
# Parametros adotados (Franklin, motor CC de laboratorio):
#   R = 1.0 ohm       L = 0.5 H
#   J = 0.01 kg.m^2   b = 0.1 N.m.s
#   Km = Kb = 0.01
# =============================================================================
def simulacao_5_motor_cc_preview():
    print("\n>>> Simulacao 5 - Motor CC (preview)")
 
    # Parametros
    R, L = 1.0, 0.5
    J, b = 0.01, 0.1
    Km = Kb = 0.01
 
    # Funcao de transferencia omega(s)/V(s)
    # Denominador: (L*s + R)*(J*s + b) + Km*Kb
    # = L*J*s^2 + (L*b + R*J)*s + (R*b + Km*Kb)
    num = [Km]
    den = [L*J, L*b + R*J, R*b + Km*Kb]
    motor = signal.TransferFunction(num, den)
 
    polos = np.roots(den)
    print(f"     Polos do motor CC: {polos}")
 
    t = np.linspace(0, 3, 1500)
    t_step, y_step = signal.step(motor, T=t)
    t_imp, y_imp = signal.impulse(motor, T=t)
 
    fig = plt.figure(figsize=(13, 9))
    gs = fig.add_gridspec(2, 2)
 
    # Resposta ao degrau
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.plot(t_step, y_step, color="#1f77b4")
    y_ss = num[0] / den[-1]  # ganho DC
    ax1.axhline(y_ss, color="gray", linestyle="--",
                label=f"Valor final = {y_ss:.4f}")
    ax1.set_xlabel("Tempo [s]")
    ax1.set_ylabel(r"$\omega(t)$ [rad/s]")
    ax1.set_title("Resposta ao degrau unitario de tensao (1 V)")
    ax1.legend()
 
    # Resposta ao impulso
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.plot(t_imp, y_imp, color="#d62728")
    ax2.set_xlabel("Tempo [s]")
    ax2.set_ylabel(r"$\omega(t)$ [rad/s]")
    ax2.set_title("Resposta ao impulso")
 
    # Mapa de polos
    ax3 = fig.add_subplot(gs[1, 0])
    ax3.axhline(0, color="black", linewidth=0.7)
    ax3.axvline(0, color="black", linewidth=0.7)
    ax3.axvspan(-15, 0, alpha=0.08, color="green", label="Regiao estavel")
    ax3.scatter(polos.real, polos.imag, marker="x", s=160,
                color="#d62728", linewidths=2.5, label="Polos")
    for p in polos:
        ax3.annotate(f"  s = {p.real:.2f}", (p.real, p.imag),
                     fontsize=9, va="bottom", ha="left")
    ax3.set_xlim(-12, 1)
    ax3.set_ylim(-3, 3)
    ax3.set_xlabel(r"Re($s$)")
    ax3.set_ylabel(r"Im($s$)")
    ax3.set_title("Polos do motor CC no plano s (ambos reais, no SPE)")
    ax3.legend(loc="upper right", fontsize=9)
 
    # Caixa de texto com parametros e FT
    ax4 = fig.add_subplot(gs[1, 1])
    ax4.axis("off")
    texto = (
        r"$\bf{Modelo\ do\ Motor\ CC}$" + "\n\n"
        r"$\dfrac{\omega(s)}{V(s)} = \dfrac{K_m}{(Ls+R)(Js+b) + K_m K_b}$"
        "\n\n"
        r"$\bf{Parametros:}$" + "\n"
        f"  R  = {R}  ohm\n"
        f"  L  = {L}  H\n"
        f"  J  = {J}  kg.m^2\n"
        f"  b  = {b}  N.m.s\n"
        f"  Km = Kb = {Km}\n\n"
        r"$\bf{Funcao\ de\ transferencia:}$" + "\n"
        f"  num = {num}\n"
        f"  den = {den}\n\n"
        r"$\bf{Polos:}$" + "\n"
        + "\n".join([f"  s = {p.real:.3f} + {p.imag:.3f}j" for p in polos])
    )
    ax4.text(0.02, 0.95, texto, transform=ax4.transAxes,
             fontsize=10, family="monospace", verticalalignment="top",
             bbox=dict(boxstyle="round,pad=0.7", facecolor="#f5f5f5",
                       edgecolor="gray"))
 
    fig.suptitle("Simulacao 5 - Motor CC (PBL): preview do sistema fio-condutor",
                 fontsize=13, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    save_fig(fig, "sim5_motor_cc_preview.png")
    plt.close(fig)
 
 
# =============================================================================
# Execucao principal
# =============================================================================
if __name__ == "__main__":
    print("=" * 75)
    print(" Estudo Dirigido - Controle e Automacao I - Etapa 01")
    print(" Fabricio Moreno da Silva / Ynnayron Juan Lopes da Silva")
    print("=" * 75)
 
    simulacao_1_funcao_transferencia()
    simulacao_2_polos_zeros()
    simulacao_3_estabilidade()
    simulacao_4_resposta_temporal()
    simulacao_5_motor_cc_preview()
 
    print("\n" + "=" * 75)
    print(f" Concluido. Figuras salvas em: {OUT_DIR}")
    print("=" * 75)