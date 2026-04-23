// =============================================================================
//  Estudo Dirigido - Controle e Automacao I
//  Etapa 01 - Fundamentos Teoricos
//  Simulacoes em Scilab
//
//  Autores: Fabricio Moreno da Silva
//           Ynnayron Juan Lopes da Silva
//
//  Instituicao: IFPB - Campus Campina Grande
//  Curso: Engenharia da Computacao
//  Professor: Dr. Moacy Pereira da Silva
//
//  Execucao no Scilab:
//    -> cd('caminho/para/simulacoes/scilab')
//    -> exec('simulacoes_controle.sce', -1)
//
//  Observacao: este script reproduz em Scilab as mesmas cinco simulacoes
//  desenvolvidas em Python. Os resultados numericos devem coincidir.
// =============================================================================

clc;
clear;
close;

mprintf("\n==========================================================\n");
mprintf(" Etapa 01 - Fundamentos Teoricos - Simulacoes em Scilab\n");
mprintf("==========================================================\n");

// Variavel de Laplace
s = poly(0, 's');


// =============================================================================
// SIMULACAO 1 - Funcao de Transferencia
// =============================================================================
mprintf("\n>>> Simulacao 1 - Funcao de Transferencia\n");

G1 = syslin('c', 1 / (s + 2));
G2 = syslin('c', 4 / (s^2 + 2*s + 4));

t  = 0:0.01:8;

// Resposta ao impulso (csim com "impul")
y1_imp = csim('impul', t, G1);
y2_imp = csim('impul', t, G2);

// Resposta ao degrau (csim com "step")
y1_step = csim('step', t, G1);
y2_step = csim('step', t, G2);

scf(1); clf();
subplot(2,2,1);
plot(t, y1_imp, 'b-', 'LineWidth', 2);
xtitle('Resposta ao Impulso - G1(s) = 1/(s+2)', 'Tempo [s]', 'y(t)');
xgrid();

subplot(2,2,2);
plot(t, y2_imp, 'r-', 'LineWidth', 2);
xtitle('Resposta ao Impulso - G2(s) = 4/(s^2+2s+4)', 'Tempo [s]', 'y(t)');
xgrid();

subplot(2,2,3);
plot(t, y1_step, 'b-', 'LineWidth', 2);
xtitle('Resposta ao Degrau - G1(s)', 'Tempo [s]', 'y(t)');
xgrid();

subplot(2,2,4);
plot(t, y2_step, 'r-', 'LineWidth', 2);
xtitle('Resposta ao Degrau - G2(s)', 'Tempo [s]', 'y(t)');
xgrid();

mprintf("    [OK] Graficos gerados.\n");


// =============================================================================
// SIMULACAO 2 - Polos e Zeros no Plano s
// =============================================================================
mprintf("\n>>> Simulacao 2 - Polos e Zeros no Plano s\n");

// Quatro configuracoes
Ga = syslin('c', 2 / (s + 2));         // polo real negativo
Gb = syslin('c', 4 / (s^2 + s + 4));   // polos complexos no SPE
Gc = syslin('c', 4 / (s^2 + 4));       // polos no eixo imaginario
Gd = syslin('c', 2 / (s - 1));         // polo real positivo

sistemas = list(Ga, Gb, Gc, Gd);
titulos  = ['(a) Polo real negativo';
           '(b) Polos complexos no SPE';
           '(c) Polos no eixo imaginario';
           '(d) Polo real positivo'];

scf(2); clf();
for k = 1:4
    subplot(2, 2, k);
    polos = roots(denom(sistemas(k)));

    // Grade e eixos
    plot([-4 4], [0 0], 'k-');
    plot([0 0], [-4 4], 'k-');

    // Polos
    plot(real(polos), imag(polos), 'rx', 'MarkerSize', 12);

    xtitle(titulos(k), 'Re(s)', 'Im(s)');
    set(gca(), 'data_bounds', [-4 -4; 4 4]);
    xgrid();
end

mprintf("    [OK] Mapas de polos gerados.\n");


// =============================================================================
// SIMULACAO 3 - Analise de Estabilidade
// =============================================================================
mprintf("\n>>> Simulacao 3 - Analise de Estabilidade\n");

G_estavel  = syslin('c', 5 / (s^2 + 2*s + 5));   // polos em -1 +/- j2
G_marginal = syslin('c', 4 / (s^2 + 4));          // polos em 0 +/- j2
G_instavel = syslin('c', 5 / (s^2 - 2*s + 5));   // polos em +1 +/- j2

t_est = 0:0.01:10;
t_ins = 0:0.01:3.5;   // truncado para nao dominar o grafico

y_est = csim('step', t_est, G_estavel);
y_mar = csim('step', t_est, G_marginal);
y_ins = csim('step', t_ins, G_instavel);

scf(3); clf();
subplot(1, 2, 1);
p1 = roots(denom(G_estavel));
p2 = roots(denom(G_marginal));
p3 = roots(denom(G_instavel));
plot([-5 5], [0 0], 'k-');
plot([0 0], [-4 4], 'k-');
plot(real(p1), imag(p1), 'gx', 'MarkerSize', 12);
plot(real(p2), imag(p2), 'yx', 'MarkerSize', 12);  // Scilab nao tem 'orange' nativo
plot(real(p3), imag(p3), 'rx', 'MarkerSize', 12);
xtitle('Polos no plano s', 'Re(s)', 'Im(s)');
set(gca(), 'data_bounds', [-3 -3.5; 3 3.5]);
xgrid();

subplot(1, 2, 2);
plot(t_est, y_est, 'g-', 'LineWidth', 2);
plot(t_est, y_mar, 'y-', 'LineWidth', 2);
plot(t_ins, y_ins, 'r-', 'LineWidth', 2);
xtitle('Resposta ao degrau', 'Tempo [s]', 'y(t)');
legend(['Estavel'; 'Marginal'; 'Instavel']);
set(gca(), 'data_bounds', [0 -3; 10 8]);
xgrid();

mprintf("    [OK] Analise de estabilidade concluida.\n");


// =============================================================================
// SIMULACAO 4 - Resposta Temporal
// =============================================================================
mprintf("\n>>> Simulacao 4 - Resposta Temporal\n");

// 1a ordem: variacao de tau
t1 = 0:0.01:6;
taus = [0.5, 1.0, 2.0];

scf(4); clf();
subplot(1, 2, 1);
cores1 = ['b-', 'r-', 'g-'];
for k = 1:length(taus)
    G = syslin('c', 1 / (taus(k)*s + 1));
    y = csim('step', t1, G);
    plot(t1, y, cores1(k), 'LineWidth', 2);
end
plot([0 6], [1 1], 'k--');      // valor final
plot([0 6], [0.632 0.632], 'r:'); // 63.2%
xtitle('1a ordem: G(s) = 1/(tau*s + 1)', 'Tempo [s]', 'y(t)');
legend(['tau = 0.5'; 'tau = 1.0'; 'tau = 2.0']);
xgrid();

// 2a ordem: variacao de zeta (wn = 1)
t2 = 0:0.01:12;
wn = 1;
zetas = [0.2, 0.5, 0.707, 1.0, 2.0];
cores2 = ['b-', 'r-', 'g-', 'm-', 'c-'];

subplot(1, 2, 2);
for k = 1:length(zetas)
    z = zetas(k);
    G = syslin('c', wn^2 / (s^2 + 2*z*wn*s + wn^2));
    y = csim('step', t2, G);
    plot(t2, y, cores2(k), 'LineWidth', 2);
end
plot([0 12], [1 1], 'k--');
xtitle('2a ordem: efeito de zeta (wn = 1)', 'Tempo [s]', 'y(t)');
legend(['zeta = 0.2'; 'zeta = 0.5'; 'zeta = 0.707'; 'zeta = 1.0'; 'zeta = 2.0']);
xgrid();

mprintf("    [OK] Respostas temporais comparadas.\n");


// =============================================================================
// SIMULACAO 5 - Motor CC (PBL)
// =============================================================================
mprintf("\n>>> Simulacao 5 - Motor CC (preview)\n");

// Parametros do motor
R_mot = 1.0;
L_mot = 0.5;
J_mot = 0.01;
b_mot = 0.1;
Km = 0.01;
Kb = 0.01;

// FT: omega(s)/V(s) = Km / [(L*s + R)(J*s + b) + Km*Kb]
num_motor = Km;
den_motor = (L_mot*s + R_mot) * (J_mot*s + b_mot) + Km*Kb;
G_motor   = syslin('c', num_motor / den_motor);

polos_motor = roots(den_motor);
mprintf("     Polos do motor CC:\n");
for k = 1:length(polos_motor)
    mprintf("        p%d = %.4f + %.4fj\n", k, real(polos_motor(k)), imag(polos_motor(k)));
end

ganho_dc = horner(G_motor, 0);
mprintf("     Ganho DC = %.6f\n", ganho_dc);

t_mot = 0:0.005:3;
y_step_mot = csim('step', t_mot, G_motor);
y_imp_mot  = csim('impul', t_mot, G_motor);

scf(5); clf();
subplot(2, 2, 1);
plot(t_mot, y_step_mot, 'b-', 'LineWidth', 2);
plot([0 3], [ganho_dc ganho_dc], 'k--');
xtitle('Resposta ao degrau (1 V)', 'Tempo [s]', 'omega(t) [rad/s]');
xgrid();

subplot(2, 2, 2);
plot(t_mot, y_imp_mot, 'r-', 'LineWidth', 2);
xtitle('Resposta ao impulso', 'Tempo [s]', 'omega(t) [rad/s]');
xgrid();

subplot(2, 2, 3);
plot([-12 1], [0 0], 'k-');
plot([0 0], [-3 3], 'k-');
plot(real(polos_motor), imag(polos_motor), 'rx', 'MarkerSize', 14);
xtitle('Polos do motor CC', 'Re(s)', 'Im(s)');
set(gca(), 'data_bounds', [-12 -3; 1 3]);
xgrid();

mprintf("    [OK] Analise do motor CC concluida.\n");


// =============================================================================
mprintf("\n==========================================================\n");
mprintf(" Execucao finalizada. Verifique as cinco janelas geradas.\n");
mprintf("==========================================================\n");
