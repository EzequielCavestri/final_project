"""
test_visual.py  —  Cocotb: evolución de coeficientes + constelaciones
=======================================================================
Captura 3 cosas en paralelo:
  1. Pesos LMS  w[k]  (k=0..15) frame a frame → curvas de convergencia
  2. Constelación a la salida del CANAL  (ch_I_dbg / ch_Q_dbg)
  3. Constelación a la salida del ECUALIZADOR  (dn_out_I / dn_out_Q)

Señales usadas del top_global_all:
  - lms_w_valid / lms_w_start / lms_w_I / lms_w_Q
  - lms_frame_cnt
  - ch_I_dbg / ch_Q_dbg     (muestra cada flanco de clk_low)
  - dn_out_valid / dn_out_I / dn_out_Q   (salida del discard_n)
  - os_valid                 (marca muestras válidas en clk_fast)

Parámetro principal:
  SIGMA_SCALE  — nivel de ruido del canal (0 = sin ruido)
                 Modificar directamente en la sección de parámetros abajo.
"""

import os
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np
from pathlib import Path

# ------------------------------------------------------------------ #
#  Parámetros de captura  ← MODIFICAR AQUÍ                            #
# ------------------------------------------------------------------ #
SIGMA_SCALE     = 4         # Nivel de ruido del canal
                            #   0  = sin ruido (solo ISI)
                            #   2  = Eb/N0 ~ 33 dB
                            #   8  = Eb/N0 ~ 21 dB
                            #   16 = Eb/N0 ~ 15 dB
                            #   32 = Eb/N0 ~  9 dB

N_TAPS          = 16        # Número de coeficientes del filtro
SNAP_PERIOD     = 5         # Cada cuántos frames guardar snapshot de pesos
SIM_US          = 2500      # Duración de la simulación en µs
CONST_MAX_PTS   = 4000      # Máx puntos a capturar por constelación
Q_FACTOR        = 7         # Fraccionario de señal  Q(9,7)
Q_FACTOR_W      = 10        # Fraccionario de pesos  Q(17,10)

def to_float(val, qf):
    """Convierte entero con signo a flotante Q(n,qf)."""
    return val / (2**qf)


# ------------------------------------------------------------------ #
#  Buffers compartidos entre corrutinas                                #
# ------------------------------------------------------------------ #
weight_snapshots = []       # [ (frame_num, w_re[16], w_im[16]), ... ]
_w_re_buf = [0] * N_TAPS   # buffer temporal de un frame de pesos
_w_im_buf = [0] * N_TAPS

ch_pts_re  = []
ch_pts_im  = []
eq_pts_re  = []
eq_pts_im  = []


# ================================================================== #
#  CORRUTINA 1: captura de pesos LMS  (dominio clk_fast)              #
#                                                                      #
#  Optimización: triggerea en lms_w_start (1 vez/frame) y luego       #
#  lee los N_TAPS samples consecutivos. Evita polling cada ciclo.     #
# ================================================================== #
async def captura_pesos(dut):
    frame_cnt_local = 0

    while True:
        # Esperar el inicio de un nuevo frame de pesos
        await RisingEdge(dut.lms_w_start)

        # Leer los N_TAPS coeficientes en serie (ya estamos en k=0)
        for k in range(N_TAPS):
            try:
                _w_re_buf[k] = int(dut.lms_w_I.value.signed_integer)
                _w_im_buf[k] = int(dut.lms_w_Q.value.signed_integer)
            except Exception:
                pass
            if k < N_TAPS - 1:
                await RisingEdge(dut.clk_fast)

        frame_cnt_local += 1

        # Snapshot cada SNAP_PERIOD frames
        if frame_cnt_local % SNAP_PERIOD == 0:
            try:
                frame_cnt = int(dut.lms_frame_cnt.value)
            except Exception:
                frame_cnt = frame_cnt_local

            w_re = [to_float(v, Q_FACTOR_W) for v in _w_re_buf]
            w_im = [to_float(v, Q_FACTOR_W) for v in _w_im_buf]
            weight_snapshots.append((frame_cnt, w_re[:], w_im[:]))


# ================================================================== #
#  CORRUTINA 2: constelación del canal  (ch_I_dbg / ch_Q_dbg)        #
#                                                                      #
#  Optimización: triggerea en os_start (1 vez/frame = cada N muestras)#
#  en vez de revisar os_valid cada ciclo de clk_fast.                 #
# ================================================================== #
async def captura_canal(dut):
    while len(ch_pts_re) < CONST_MAX_PTS:
        # os_start pulsa 1 ciclo por cada bloque de N muestras
        await RisingEdge(dut.os_start)
        try:
            re = int(dut.ch_I_dbg.value.signed_integer)
            im = int(dut.ch_Q_dbg.value.signed_integer)
            ch_pts_re.append(to_float(re, Q_FACTOR))
            ch_pts_im.append(to_float(im, Q_FACTOR))
        except Exception:
            pass


# ================================================================== #
#  CORRUTINA 3: constelación del ecualizador (dn_out)                 #
#                                                                      #
#  Optimización: triggerea en dn_out_start y luego lee N_TAPS/2       #
#  samples consecutivos (los N útiles del frame).                     #
# ================================================================== #
async def captura_ecualizador(dut):
    while len(eq_pts_re) < CONST_MAX_PTS:
        # dn_out_start pulsa 1 ciclo al inicio de los N samples útiles
        await RisingEdge(dut.dn_out_start)

        # Leer todos los samples del frame (N = N_TAPS)
        for _ in range(N_TAPS):
            try:
                re = int(dut.dn_out_I.value.signed_integer)
                im = int(dut.dn_out_Q.value.signed_integer)
                eq_pts_re.append(to_float(re, Q_FACTOR))
                eq_pts_im.append(to_float(im, Q_FACTOR))
            except Exception:
                pass
            await RisingEdge(dut.clk_fast)


# ================================================================== #
#  CORRUTINA 4: heartbeat para logging                                 #
# ================================================================== #
async def heartbeat(dut):
    t = 0
    while True:
        await Timer(200, unit="us")
        t += 200
        n_snaps  = len(weight_snapshots)
        last_frm = weight_snapshots[-1][0] if n_snaps else 0
        dut._log.info(
            f"[HB] {t} µs | frames={last_frm} | snaps={n_snaps} | "
            f"CH={len(ch_pts_re)} pts | EQ={len(eq_pts_re)} pts"
        )


# ================================================================== #
#  FUNCIÓN DE PLOTS                                                    #
# ================================================================== #
def make_plots(sigma, out_dir):
    """Genera y guarda todos los gráficos."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    # ---------------------------------------------------------- #
    #  Fig 1: Evolución de coeficientes Re e Im (|w[k]| vs frame) #
    # ---------------------------------------------------------- #
    if weight_snapshots:
        frames = np.array([s[0] for s in weight_snapshots])
        W_re   = np.array([s[1] for s in weight_snapshots])  # (T, 16)
        W_im   = np.array([s[2] for s in weight_snapshots])  # (T, 16)
        W_mag  = np.sqrt(W_re**2 + W_im**2)                  # (T, 16)

        cmap = plt.cm.tab20
        colors = [cmap(i/N_TAPS) for i in range(N_TAPS)]

        # — Subplot: Re, Im, |w| por tap —
        fig, axes = plt.subplots(3, 1, figsize=(14, 12), sharex=True)

        for k in range(N_TAPS):
            axes[0].plot(frames, W_re[:, k], color=colors[k],
                         alpha=0.8, linewidth=1.2, label=f'w[{k}]')
            axes[1].plot(frames, W_im[:, k], color=colors[k],
                         alpha=0.8, linewidth=1.2)
            axes[2].plot(frames, W_mag[:, k], color=colors[k],
                         alpha=0.8, linewidth=1.2)

        axes[0].set_ylabel('Re{w[k]}', fontsize=11)
        axes[1].set_ylabel('Im{w[k]}', fontsize=11)
        axes[2].set_ylabel('|w[k]|',   fontsize=11)
        axes[2].set_xlabel('Frame', fontsize=11)

        axes[0].set_title(
            f'Evolución de Coeficientes LMS — σ={sigma}  '
            f'(N={N_TAPS} taps, muestreo cada {SNAP_PERIOD} frames)',
            fontsize=13)
        axes[0].legend(ncol=4, fontsize=7, loc='upper right')

        for ax in axes:
            ax.grid(True, alpha=0.4)
            ax.axhline(0, color='k', linewidth=0.5)

        plt.tight_layout()
        p = out_dir / f"coef_evolution_sigma{sigma}.png"
        plt.savefig(p, dpi=150)
        plt.close()
        print(f"  Guardado: {p}")

        # — Heatmap de |w| final —
        fig2, ax2 = plt.subplots(figsize=(10, 4))
        im2 = ax2.imshow(W_mag.T, aspect='auto', origin='lower',
                         cmap='viridis',
                         extent=[frames[0], frames[-1], 0, N_TAPS])
        fig2.colorbar(im2, ax=ax2, label='|w[k]|')
        ax2.set_xlabel('Frame')
        ax2.set_ylabel('Tap k')
        ax2.set_title(f'Heatmap |w[k]| a lo largo del tiempo — σ={sigma}')
        plt.tight_layout()
        p2 = out_dir / f"coef_heatmap_sigma{sigma}.png"
        plt.savefig(p2, dpi=150)
        plt.close()
        print(f"  Guardado: {p2}")

    # ---------------------------------------------------------- #
    #  Fig 2: Constelaciones comparadas                           #
    # ---------------------------------------------------------- #
    fig3, axes3 = plt.subplots(1, 2, figsize=(12, 6))

    # ---- Canal (ISI + ruido) ----
    ax = axes3[0]
    if ch_pts_re:
        ax.scatter(ch_pts_re, ch_pts_im, s=3, alpha=0.35, color='steelblue')
    ax.set_title(f'Constelación — Salida CANAL\n(ISI h=[45,110,45]/128, σ={sigma})',
                 fontsize=11)
    ax.set_xlabel('Re', fontsize=10)
    ax.set_ylabel('Im', fontsize=10)
    ax.axhline(0, color='k', lw=0.5); ax.axvline(0, color='k', lw=0.5)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)

    # QPSK ideal markers
    qpsk_pts = [(1, 1), (1, -1), (-1, 1), (-1, -1)]
    for (x, y) in qpsk_pts:
        ax.plot(x * 0.707, y * 0.707, 'r+', markersize=12, markeredgewidth=2)

    # ---- Ecualizador (antes del slicer) ----
    ax = axes3[1]
    if eq_pts_re:
        # Color por densidad para ver la dispersión
        from matplotlib.colors import LogNorm
        try:
            h, xedges, yedges = np.histogram2d(
                eq_pts_re, eq_pts_im,
                bins=80, range=[[-1.5, 1.5], [-1.5, 1.5]])
            ax.imshow(h.T, origin='lower',
                      extent=[-1.5, 1.5, -1.5, 1.5],
                      cmap='hot', norm=LogNorm(vmin=0.5),
                      aspect='auto')
        except Exception:
            ax.scatter(eq_pts_re, eq_pts_im, s=3, alpha=0.35, color='darkorange')
    ax.set_title(f'Constelación — Salida ECUALIZADOR (dn_out)\n'
                 f'PBFDAF-LMS  σ={sigma}',
                 fontsize=11)
    ax.set_xlabel('Re', fontsize=10)
    ax.set_ylabel('Im', fontsize=10)
    ax.axhline(0, color='cyan', lw=0.5, alpha=0.7)
    ax.axvline(0, color='cyan', lw=0.5, alpha=0.7)
    ax.set_aspect('equal')
    for (x, y) in qpsk_pts:
        ax.plot(x * 0.707, y * 0.707, 'g+', markersize=14, markeredgewidth=2)

    plt.tight_layout()
    p3 = out_dir / f"constelaciones_sigma{sigma}.png"
    plt.savefig(p3, dpi=150)
    plt.close()
    print(f"  Guardado: {p3}")

    # ---------------------------------------------------------- #
    #  Fig 3: Coeficientes finales (stem plot, Re e Im)          #
    # ---------------------------------------------------------- #
    if weight_snapshots:
        last_snap = weight_snapshots[-1]
        taps = np.arange(N_TAPS)

        fig4, (axr, axi) = plt.subplots(2, 1, figsize=(10, 7))

        axr.stem(taps, last_snap[1], linefmt='C0-', markerfmt='C0o',
                 basefmt='k-', label='Re{w[k]} final')
        axi.stem(taps, last_snap[2], linefmt='C1-', markerfmt='C1o',
                 basefmt='k-', label='Im{w[k]} final')

        axr.set_ylabel('Re{w[k]}'); axr.grid(True, alpha=0.4)
        axi.set_ylabel('Im{w[k]}'); axi.grid(True, alpha=0.4)
        axi.set_xlabel('Tap k')

        axr.set_title(
            f'Respuesta impulsional del filtro LMS (frame={last_snap[0]})  σ={sigma}')
        axr.legend(); axi.legend()
        plt.tight_layout()
        p4 = out_dir / f"coef_final_sigma{sigma}.png"
        plt.savefig(p4, dpi=150)
        plt.close()
        print(f"  Guardado: {p4}")

    # ---------------------------------------------------------- #
    #  Fig 4: Panel resumen (4 subplots en una figura)            #
    # ---------------------------------------------------------- #
    if weight_snapshots:
        fig5 = plt.figure(figsize=(16, 12))
        gs   = gridspec.GridSpec(2, 3, figure=fig5, hspace=0.4, wspace=0.35)

        # Top-left: evolución |w[k]|
        ax_evo = fig5.add_subplot(gs[0, :2])
        for k in range(N_TAPS):
            ax_evo.plot(frames, W_mag[:, k], color=colors[k],
                        alpha=0.8, linewidth=1.1, label=f'w[{k}]')
        ax_evo.set_title(f'Convergencia |w[k]| vs frame  (σ={sigma})', fontsize=11)
        ax_evo.set_xlabel('Frame'); ax_evo.set_ylabel('|w[k]|')
        ax_evo.legend(ncol=4, fontsize=7, loc='upper right')
        ax_evo.grid(True, alpha=0.4)

        # Top-right: coef finales
        ax_stem_r = fig5.add_subplot(gs[0, 2])
        ax_stem_r.stem(taps, last_snap[1], linefmt='C0-', markerfmt='C0o',
                       basefmt='k-')
        ax_stem_r.stem(taps, last_snap[2], linefmt='C1--', markerfmt='C1s',
                       basefmt='k-')
        ax_stem_r.set_title('Coef. finales\nRe (azul) / Im (naranja)', fontsize=10)
        ax_stem_r.set_xlabel('Tap k'); ax_stem_r.grid(True, alpha=0.4)

        # Bottom-left: constelación canal
        ax_ch = fig5.add_subplot(gs[1, 0])
        if ch_pts_re:
            ax_ch.scatter(ch_pts_re, ch_pts_im, s=2, alpha=0.3, color='steelblue')
        for (x, y) in qpsk_pts:
            ax_ch.plot(x * 0.707, y * 0.707, 'r+', markersize=10, markeredgewidth=2)
        ax_ch.set_title(f'Canal (σ={sigma})', fontsize=10)
        ax_ch.set_xlabel('Re'); ax_ch.set_ylabel('Im')
        ax_ch.set_aspect('equal'); ax_ch.grid(True, alpha=0.3)
        ax_ch.axhline(0, color='k', lw=0.5); ax_ch.axvline(0, color='k', lw=0.5)

        # Bottom-center: constelación ecualizador (todo el tiempo)
        ax_eq_all = fig5.add_subplot(gs[1, 1])
        if eq_pts_re:
            ax_eq_all.scatter(eq_pts_re, eq_pts_im, s=2, alpha=0.25,
                              color='darkorange')
        for (x, y) in qpsk_pts:
            ax_eq_all.plot(x * 0.707, y * 0.707, 'g+', markersize=10,
                           markeredgewidth=2)
        ax_eq_all.set_title(f'EQ — todos los puntos', fontsize=10)
        ax_eq_all.set_xlabel('Re'); ax_eq_all.set_ylabel('Im')
        ax_eq_all.set_aspect('equal'); ax_eq_all.grid(True, alpha=0.3)
        ax_eq_all.axhline(0, color='k', lw=0.5); ax_eq_all.axvline(0, color='k', lw=0.5)

        # Bottom-right: constelación ecualizador (segunda mitad = convergido)
        ax_eq_cvg = fig5.add_subplot(gs[1, 2])
        mid = len(eq_pts_re) // 2
        if mid > 0:
            ax_eq_cvg.scatter(eq_pts_re[mid:], eq_pts_im[mid:], s=2,
                              alpha=0.3, color='forestgreen')
        for (x, y) in qpsk_pts:
            ax_eq_cvg.plot(x * 0.707, y * 0.707, 'r+', markersize=10,
                           markeredgewidth=2)
        ax_eq_cvg.set_title(f'EQ — 2ª mitad (convergido)', fontsize=10)
        ax_eq_cvg.set_xlabel('Re'); ax_eq_cvg.set_ylabel('Im')
        ax_eq_cvg.set_aspect('equal'); ax_eq_cvg.grid(True, alpha=0.3)
        ax_eq_cvg.axhline(0, color='k', lw=0.5); ax_eq_cvg.axvline(0, color='k', lw=0.5)

        fig5.suptitle(
            f'PBFDAF-LMS RTL — Panel de Diagnóstico  |  σ={sigma}  |  '
            f'Canal ISI h=[45,110,45]/128  |  QPSK',
            fontsize=13, fontweight='bold')

        p5 = out_dir / f"panel_diagnostico_sigma{sigma}.png"
        plt.savefig(p5, dpi=150)
        plt.close()
        print(f"  Guardado: {p5}")


# ================================================================== #
#  TEST PRINCIPAL                                                      #
# ================================================================== #
@cocotb.test()
async def test_visual(dut):
    """
    Simulación de convergencia con captura de:
      - Evolución de coeficientes LMS  (lms_w_*)
      - Constelación del canal          (ch_I/Q_dbg)
      - Constelación del ecualizador    (dn_out_I/Q)
    """
    sigma = SIGMA_SCALE
    dut._log.info(f"=== test_visual  sigma_scale={sigma} ===")

    # ---- Clock y reset ----
    cocotb.start_soon(Clock(dut.clk_fast, 10, unit="ns").start())

    dut.rst.value        = 1
    dut.enable_div.value = 1
    dut.sigma_scale.value = sigma
    await Timer(300, unit="ns")
    dut.rst.value = 0
    await Timer(100, unit="ns")

    # ---- Lanzar corrutinas de captura ----
    cocotb.start_soon(captura_pesos(dut))
    cocotb.start_soon(captura_canal(dut))
    cocotb.start_soon(captura_ecualizador(dut))
    cocotb.start_soon(heartbeat(dut))

    # ---- Correr simulación ----
    await Timer(SIM_US, unit="us")

    # ---- Resumen ----
    n_snaps  = len(weight_snapshots)
    last_frm = weight_snapshots[-1][0] if n_snaps else 0
    dut._log.info(f"--- Captura finalizada ---")
    dut._log.info(f"  Snapshots de pesos : {n_snaps} (último frame={last_frm})")
    dut._log.info(f"  Pts. constelación canal    : {len(ch_pts_re)}")
    dut._log.info(f"  Pts. constelación ecualiz. : {len(eq_pts_re)}")

    if n_snaps < 3:
        dut._log.warning("Pocos snapshots — aumentar SIM_US o reducir SNAP_PERIOD")
        return

    # ---- Generar gráficos ----
    sim_dir = os.path.dirname(os.path.abspath(__file__))
    out_dir = os.path.join(sim_dir, f"visual_sigma{sigma}")
    dut._log.info(f"Generando gráficos en: {out_dir}")
    make_plots(sigma, out_dir)
    dut._log.info("=== Gráficos generados ===")