"""
test_top.py  —  Testbench cocotb para top_global_all (v15, K=2 particiones)

Tests disponibles:
  test_convergencia        →  sigma=0, N_FRAMES frames, gráficos completos
  ber_convergencia         →  sweep BER completo
  test_medir_delay         →  Mide el delay real entre XHD y FFTE
  test_debug_pipeline      →  Captura logs de todas las etapas del DSP
  test_medir_tap_central   →  Busca el pico del impulso en el IFFT/Slicer
  test_trace_lms_alignment →  Trace ciclo a ciclo de XHD, FFTE, GRAD y LMS

v15:
  - Captura AMBAS particiones del filtro (P0=lms_w_*, P1=lms1_w_*).
  - w_history  → taps  0..15  (partición 0, X_curr)
  - w1_history → taps 16..31  (partición 1, X_old)
  - Gráficos integrados de Constelación (3 paneles) y Curva de Aprendizaje (MSE).
"""

import os
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

# ============================================================
# Helpers y Formateo
# ============================================================
def rs(sig):
    try: return int(sig.value.to_signed())
    except: return 0

def rb(sig):
    try: return int(sig.value)
    except: return 0

def fmt_complex(re, im, scale):
    return f"{re/scale:+8.4f} {im/scale:+8.4f}j"

def read_signed(sig):
    return int(sig.value.to_signed())

# ============================================================
# PRBS-9
# ============================================================
def prbs9(seed, n_bits):
    reg  = seed & 0x1FF
    bits = []
    for _ in range(n_bits):
        bit = (reg >> 8) & 1
        bits.append(bit)
        feedback = ((reg >> 8) ^ (reg >> 4)) & 1
        reg = ((reg << 1) | feedback) & 0x1FF
    return bits

# ============================================================
# Estado global
# ============================================================
rx_bits_I  = []
rx_bits_Q  = []
w_history  = [[] for _ in range(16)]   # partición 0: taps  0..15 (X_curr)
w1_history = [[] for _ in range(16)]   # partición 1: taps 16..31 (X_old)
lms_frames = 0

# Constelaciones y Errores
const_canal   = []    # salida del canal:  ch_I_dbg / ch_Q_dbg  (Q9.7 → /128)
const_eq      = []    # salida del eq:     dn_out_I / dn_out_Q  (Q9.7 → /128)
const_slicer  = []    # salida del slicer: sl_out_yhat_I/Q      (Q9.7 → /128)
error_history = []    # errores del slicer: sl_out_e_I/Q        (Q9.7 → /128)

QPSK_SCALE = 128.0  # Q(9,7): 1 LSB = 1/128
W_SCALE    = 1024.0 # Q(17,10): 1.0 = 1024

# ============================================================
# Monitores para Trace
# ============================================================
async def monitor_eventos(dut):
    ciclo = 0
    while True:
        await RisingEdge(dut.clk_fast)
        ciclo += 1

        if rb(dut.hb_out_valid) and rb(dut.hb_out_start):
            print()
            print(f"[{ciclo:8d}] HB START")
            print("  curr0 =", fmt_complex(rs(dut.hb_out_curr_I), rs(dut.hb_out_curr_Q), 1024))
            print("  old0  =", fmt_complex(rs(dut.hb_out_old_I), rs(dut.hb_out_old_Q), 1024))

        if rb(dut.ffte_out_valid) and rb(dut.ffte_out_start):
            print()
            print(f"[{ciclo:8d}] FFTE START")
            print("  E0 =", fmt_complex(rs(dut.ffte_out_I), rs(dut.ffte_out_Q), 1024))

        if rb(dut.xhd_out_valid) and rb(dut.xhd_out_start):
            print()
            print(f"[{ciclo:8d}] XHD START")
            print("  X0 =", fmt_complex(rs(dut.xhd_out_re), rs(dut.xhd_out_im), 1024))

        if rb(dut.grad_out_valid) and rb(dut.grad_out_start):
            print()
            print(f"[{ciclo:8d}] GRAD START")
            print("  phi0 =", fmt_complex(rs(dut.grad_out_re), rs(dut.grad_out_im), 1024))

        if rb(dut.ifft_grad_valid) and rb(dut.ifft_grad_start):
            print()
            print(f"[{ciclo:8d}] IFFT_GRAD START")
            print("  g0 =", fmt_complex(rs(dut.ifft_grad_I), rs(dut.ifft_grad_Q), 1024))

        if rb(dut.grad_t_valid) and rb(dut.grad_t_start):
            print()
            print(f"[{ciclo:8d}] GRAD_T START")
            print("  g0 =", fmt_complex(rs(dut.grad_t_I), rs(dut.grad_t_Q), 1024))

        if rb(dut.lms_w_valid) and rb(dut.lms_w_start):
            print()
            print(f"[{ciclo:8d}] LMS START")
            print("  w0 =", fmt_complex(rs(dut.lms_w_I), rs(dut.lms_w_Q), 1024))

async def monitor_delay(dut):
    ciclo = 0
    hb_cycle = None
    while True:
        await RisingEdge(dut.clk_fast)
        ciclo += 1

        if rb(dut.hb_out_valid) and rb(dut.hb_out_start):
            hb_cycle = ciclo

        if rb(dut.xhd_out_valid) and rb(dut.xhd_out_start):
            if hb_cycle is not None:
                print()
                print("====================================")
                print("HB start :", hb_cycle)
                print("XHD start:", ciclo)
                print("delay =", ciclo - hb_cycle)
                print("====================================")

# ============================================================
# Coroutine: captura constelación canal y ecualizador
# ============================================================
async def captura_constelacion(dut):
    global const_canal, const_eq
    prev_chI = None
    while True:
        await RisingEdge(dut.clk_fast)
        try:
            chI = read_signed(dut.ch_I_dbg)
            chQ = read_signed(dut.ch_Q_dbg)
            if chI != prev_chI and chI != 0:
                const_canal.append((chI / QPSK_SCALE, chQ / QPSK_SCALE))
                prev_chI = chI

            if int(dut.dn_out_valid.value) == 1:
                dnI = read_signed(dut.dn_out_I)
                dnQ = read_signed(dut.dn_out_Q)
                const_eq.append((dnI / QPSK_SCALE, dnQ / QPSK_SCALE))

        except Exception as e:
            dut._log.error(f"[captura_constelacion] {e}")
            raise

# ============================================================
# Coroutine: captura bits del slicer, constelación y errores
# ============================================================
async def captura_slicer(dut):
    global rx_bits_I, rx_bits_Q, const_slicer, error_history
    while True:
        await RisingEdge(dut.clk_fast)
        try:
            if int(dut.sl_out_valid.value) == 1:
                yI = read_signed(dut.sl_out_yhat_I)
                yQ = read_signed(dut.sl_out_yhat_Q)
                eI = read_signed(dut.sl_out_e_I)
                eQ = read_signed(dut.sl_out_e_Q)
                
                rx_bits_I.append(0 if yI > 0 else 1)
                rx_bits_Q.append(0 if yQ > 0 else 1)
                
                const_slicer.append((yI / QPSK_SCALE, yQ / QPSK_SCALE))
                error_history.append((eI / QPSK_SCALE, eQ / QPSK_SCALE))
        except Exception as e:
            dut._log.error(f"[captura_slicer] {e}")
            raise

# ============================================================
# Coroutine: captura pesos LMS — AMBAS particiones
# ============================================================
async def captura_pesos(dut):
    global w_history, w1_history, lms_frames
    while True:
        await RisingEdge(dut.clk_fast)
        try:
            if int(dut.lms_w_valid.value) == 1 and int(dut.lms_w_start.value) == 1:
                frame_w0 = []
                frame_w1 = []
                for k in range(16):
                    frame_w0.append(read_signed(dut.lms_w_I)  / W_SCALE)
                    frame_w1.append(read_signed(dut.lms1_w_I) / W_SCALE)
                    if k < 15:
                        await RisingEdge(dut.clk_fast)
                        while int(dut.lms_w_valid.value) == 0:
                            await RisingEdge(dut.clk_fast)
                lms_frames += 1
                for k in range(16):
                    w_history[k].append(frame_w0[k])
                    w1_history[k].append(frame_w1[k])
        except Exception as e:
            dut._log.error(f"[captura_pesos] {e}")
            raise

# ============================================================
# Coroutine: heartbeat
# ============================================================
async def heartbeat(dut):
    t = 0
    while True:
        await Timer(500, unit="us")
        t += 500
        dut._log.info(
            f"[HB] {t}us  rx={len(rx_bits_I)}  "
            f"lms={lms_frames}  canal={len(const_canal)}  "
            f"eq={len(const_eq)}  switched={int(dut.lms_switched.value)}"
        )

# ============================================================
# Setup: Reset y clock
# ============================================================
async def setup(dut, sigma):
    cocotb.start_soon(Clock(dut.clk_fast, 10, unit="ns").start())
    dut.rst.value         = 1
    dut.enable_div.value  = 1
    dut.sigma_scale.value = sigma
    await Timer(500, unit="ns")
    dut.rst.value = 0
    await Timer(100, unit="ns")

# ============================================================
# Test MEDICIÓN DE DELAY
# ============================================================
@cocotb.test()
async def test_medir_delay(dut):
    dut._log.info("=== TEST MEDICIÓN DE DELAY xhist ===")
    await setup(dut, sigma=0)

    dut._log.info("Esperando primer hb_out_start...")
    timeout = 0
    while True:
        await RisingEdge(dut.clk_fast)
        timeout += 1
        if timeout > 100000:
            dut._log.error("Timeout esperando hb_out_start")
            return
        if rb(dut.hb_out_start) and rb(dut.hb_out_valid):
            break

    t0 = timeout
    dut._log.info(f"hb_out_start detectado en ciclo {t0}")

    dut._log.info("Esperando primer ffte_out_start...")
    while True:
        await RisingEdge(dut.clk_fast)
        timeout += 1
        if timeout > t0 + 500:
            dut._log.error("Timeout esperando ffte_out_start — delay > 500 ciclos")
            return
        if rb(dut.ffte_out_start) and rb(dut.ffte_out_valid):
            break

    t1     = timeout
    delay  = t1 - t0

    dut._log.info(f"ffte_out_start detectado en ciclo {t1}")
    dut._log.info(f"")
    dut._log.info(f"════════════════════════════════════════")
    dut._log.info(f"  DELAY MEDIDO  = {delay} ciclos")
    dut._log.info(f"  DELAY ACTUAL  = 118 ciclos (en xhist_delay.v)")
    dut._log.info(f"  DIFERENCIA    = {delay - 118} ciclos")
    dut._log.info(f"════════════════════════════════════════")

    if delay != 118:
        dut._log.warning(
            f"El delay real ({delay}) difiere del configurado (118). "
            f"Cambiar DELAY={delay} en xhist_delay.v y recompilar."
        )
    else:
        dut._log.info("DELAY correcto — xhist_delay.v está bien configurado.")
    dut._log.info("=== FIN TEST DELAY ===")

# ============================================================
# Test CONVERGENCIA + CONSTELACIÓN
# ============================================================
@cocotb.test()
async def test_convergencia(dut):
    global rx_bits_I, rx_bits_Q, w_history, w1_history, lms_frames 
    global const_canal, const_eq, const_slicer, error_history
    
    rx_bits_I = []; rx_bits_Q = []
    w_history = [[] for _ in range(16)]
    w1_history = [[] for _ in range(16)]
    lms_frames = 0
    const_canal = []; const_eq = []; const_slicer = []; error_history = []

    N_FRAMES = int(os.environ.get("N_FRAMES", "400"))
    sigma    = int(os.environ.get("SIGMA_SCALE", "0"))
    dut._log.info(f"=== MODO CONVERGENCIA — sigma={sigma}, {N_FRAMES} frames ===")

    await setup(dut, sigma=sigma)
    
    cocotb.start_soon(captura_slicer(dut))
    cocotb.start_soon(captura_pesos(dut))
    cocotb.start_soon(captura_constelacion(dut))

    timeout_us = N_FRAMES * 8
    elapsed    = 0
    step_us    = 50
    while lms_frames < N_FRAMES and elapsed < timeout_us:
        await Timer(step_us, unit="us")
        elapsed += step_us
        if elapsed % 500 == 0:
            dut._log.info(
                f"  t={elapsed}us  frames={lms_frames}/{N_FRAMES}"
                f"  canal={len(const_canal)}  eq={len(const_eq)}"
                f"  switched={int(dut.lms_switched.value)}"
            )

    n = lms_frames
    dut._log.info(f"Capturados {n} frames en {elapsed} us")
    dut._log.info(f"Muestras canal: {len(const_canal)}  eq: {len(const_eq)}")

    if n < 10:
        dut._log.error("Muy pocos frames — verificar lms_w_valid/start")
        return

    w0 = w_history[0]; w1 = w_history[1]
    w2 = w_history[2]; w3 = w_history[3]
    p1_0 = w1_history[0]; p1_1 = w1_history[1]
    dut._log.info(f"{'frame':>6}  {'w[0]':>8}  {'w[1]':>8}  {'w[2]':>8}  {'w[3]':>8}"
                  f"  {'w[16]':>8}  {'w[17]':>8}")
    step = max(1, n // 20)
    for i in range(0, n, step):
        dut._log.info(
            f"{i:>6}  {w0[i]:>+8.4f}  {w1[i]:>+8.4f}  "
            f"{w2[i]:>+8.4f}  {w3[i]:>+8.4f}  "
            f"{p1_0[i]:>+8.4f}  {p1_1[i]:>+8.4f}"
        )

    dut._log.info("── Pesos finales — filtro completo (32 taps) ──")
    dut._log.info("   P0 = partición X_curr (taps 0..15)")
    for k in range(16):
        v = w_history[k][-1] if w_history[k] else 0.0
        dut._log.info(f"  w[{k:2d}] = {v:+.4f}  (raw={int(v*W_SCALE)})")
    dut._log.info("   P1 = partición X_old  (taps 16..31)")
    for k in range(16):
        v = w1_history[k][-1] if w1_history[k] else 0.0
        dut._log.info(f"  w[{k+16:2d}] = {v:+.4f}  (raw={int(v*W_SCALE)})")
    dut._log.info("   (tap 31 queda libre: el Python usa L_EQ=31)")

    sim_dir = os.path.dirname(os.path.abspath(__file__))
    _plot_convergencia_completa(n, sim_dir)
    _plot_filtro_completo(n, sim_dir)
    _plot_constelaciones(sim_dir, sigma, n)
    _plot_flujo_completo(sim_dir, sigma, n)
    _plot_evolucion_error(sim_dir, sigma)
    dut._log.info("Guardados gráficos de convergencia, filtro, constelación y evolución del error.")

# ============================================================
# Test BER
# ============================================================
@cocotb.test()
async def ber_convergencia(dut):
    global rx_bits_I, rx_bits_Q, w_history, w1_history, lms_frames
    global const_canal, const_eq, const_slicer, error_history
    
    rx_bits_I = []; rx_bits_Q = []
    w_history = [[] for _ in range(16)]
    w1_history = [[] for _ in range(16)]
    lms_frames = 0
    const_canal = []; const_eq = []; const_slicer = []; error_history = []

    sigma       = int(os.environ.get("SIGMA_SCALE", "0"))
    SIM_TIME_US = int(os.environ.get("SIM_TIME_US", "8000"))
    dut._log.info(f"=== MODO BER — sigma={sigma}, {SIM_TIME_US} us ===")

    await setup(dut, sigma)
    cocotb.start_soon(captura_slicer(dut))
    cocotb.start_soon(captura_pesos(dut))
    cocotb.start_soon(captura_constelacion(dut))
    cocotb.start_soon(heartbeat(dut))

    await Timer(SIM_TIME_US, unit="us")

    dut._log.info(f"RX símbolos: {len(rx_bits_I)}")
    dut._log.info(f"LMS frames:  {lms_frames}")
    dut._log.info(f"switched:    {int(dut.lms_switched.value)}")

    if len(rx_bits_I) < 500:
        dut._log.error("Muy pocos símbolos")
        return

    MAX_OFFSET = 1200
    REF_LEN    = len(rx_bits_I) + MAX_OFFSET + 100
    ref_I      = np.array(prbs9(0x17F, REF_LEN))
    ref_Q      = np.array(prbs9(0x11D, REF_LEN))
    rx_arr     = np.array(rx_bits_I)
    search_len = min(300, len(rx_arr))

    best_offset, best_corr = 0, -1
    for offset in range(MAX_OFFSET):
        if offset + search_len > len(ref_I):
            break
        c = np.sum(rx_arr[:search_len] == ref_I[offset:offset + search_len])
        if c > best_corr:
            best_corr, best_offset = c, offset

    match_pct = 100.0 * best_corr / search_len
    dut._log.info(f"Delay TX→RX: {best_offset} sym  (match {match_pct:.1f}%)")
    if match_pct < 55.0:
        dut._log.warning(f"Correlación baja ({match_pct:.1f}%)")

    ref_aI = ref_I[best_offset:]
    ref_aQ = ref_Q[best_offset:]
    rx_I   = np.array(rx_bits_I)
    rx_Q   = np.array(rx_bits_Q)
    N      = min(len(rx_I), len(ref_aI), len(ref_aQ))
    mitad  = N // 2
    n_ss   = N - mitad

    errores   = (np.sum(rx_I[mitad:N] != ref_aI[mitad:N]) +
                 np.sum(rx_Q[mitad:N] != ref_aQ[mitad:N]))
    ber_final = errores / (2 * n_ss) if n_ss > 0 else 1.0

    dut._log.info(f"BER (sigma={sigma}): {ber_final:.6f}  ({errores}/{2*n_ss} bits)")

    sim_dir      = os.path.dirname(os.path.abspath(__file__))
    results_file = os.path.join(sim_dir, "snr_sweep_results.txt")
    with open(results_file, "a") as f:
        f.write(f"{sigma},{ber_final:.8f}\n")

    _plot_constelaciones(sim_dir, sigma, lms_frames)
    _plot_convergencia_sigma(sigma, sim_dir)
    _plot_filtro_completo(lms_frames, sim_dir, suffix=f"_sigma{sigma}")
    _plot_flujo_completo(sim_dir, sigma, lms_frames)
    _plot_evolucion_error(sim_dir, sigma)

# ============================================================
# Test DEBUG — verificación sistemática de bloques
# ============================================================
@cocotb.test()
async def test_debug_pipeline(dut):
    dut._log.info("=== TEST DEBUG PIPELINE ===")
    dut._log.info("Capturando 5 frames completos de cada etapa")
    await setup(dut, sigma=0)
    await Timer(5000, unit="ns")

    N_FRAMES_DEBUG = 5
    frames_capturados = {
        'fft_out':    [], 'hb_curr':    [], 'hb_old':     [],
        'cmul_out':   [], 'ifft_out':   [], 'dn_out':     [],
        'sl_yhat':    [], 'sl_error':   [], 'zpe_out':    [],
        'ffte_out':   [], 'xhd_out':    [], 'grad_out':   [],
        'ifft_grad':  [], 'grad_t':     [], 'lms_w':      [],
    }
    frame_count = {k: 0 for k in frames_capturados}
    MAX_CICLOS = 200000
    ciclo = 0

    while ciclo < MAX_CICLOS and any(v < N_FRAMES_DEBUG for v in frame_count.values()):
        await RisingEdge(dut.clk_fast)
        ciclo += 1

        if rb(dut.fft_out_valid) and rb(dut.fft_out_start):
            if frame_count['fft_out'] < N_FRAMES_DEBUG:
                samples = []
                for _ in range(32):
                    if rb(dut.fft_out_valid):
                        samples.append((rs(dut.fft_out_I), rs(dut.fft_out_Q)))
                    await RisingEdge(dut.clk_fast)
                    ciclo += 1
                frames_capturados['fft_out'].append(samples)
                frame_count['fft_out'] += 1
                continue

        if rb(dut.hb_out_valid) and rb(dut.hb_out_start):
            if frame_count['hb_curr'] < N_FRAMES_DEBUG:
                curr_s, old_s = [], []
                for _ in range(32):
                    if rb(dut.hb_out_valid):
                        curr_s.append((rs(dut.hb_out_curr_I), rs(dut.hb_out_curr_Q)))
                        old_s.append((rs(dut.hb_out_old_I),  rs(dut.hb_out_old_Q)))
                    await RisingEdge(dut.clk_fast)
                    ciclo += 1
                frames_capturados['hb_curr'].append(curr_s)
                frames_capturados['hb_old'].append(old_s)
                frame_count['hb_curr'] += 1
                frame_count['hb_old']  += 1
                continue

        if rb(dut.cmul_out_valid) and rb(dut.cmul_out_start):
            if frame_count['cmul_out'] < N_FRAMES_DEBUG:
                samples = []
                for _ in range(32):
                    if rb(dut.cmul_out_valid):
                        samples.append((rs(dut.cmul_out_I), rs(dut.cmul_out_Q)))
                    await RisingEdge(dut.clk_fast)
                    ciclo += 1
                frames_capturados['cmul_out'].append(samples)
                frame_count['cmul_out'] += 1
                continue

        if rb(dut.ifft_out_valid) and rb(dut.ifft_out_start):
            if frame_count['ifft_out'] < N_FRAMES_DEBUG:
                samples = []
                for _ in range(32):
                    if rb(dut.ifft_out_valid):
                        samples.append((rs(dut.ifft_out_I), rs(dut.ifft_out_Q)))
                    await RisingEdge(dut.clk_fast)
                    ciclo += 1
                frames_capturados['ifft_out'].append(samples)
                frame_count['ifft_out'] += 1
                continue

        if rb(dut.dn_out_valid) and rb(dut.dn_out_start):
            if frame_count['dn_out'] < N_FRAMES_DEBUG:
                samples = []
                for _ in range(16):
                    if rb(dut.dn_out_valid):
                        samples.append((rs(dut.dn_out_I), rs(dut.dn_out_Q)))
                    await RisingEdge(dut.clk_fast)
                    ciclo += 1
                frames_capturados['dn_out'].append(samples)
                frame_count['dn_out'] += 1
                continue

        if rb(dut.sl_out_valid) and rb(dut.sl_out_start):
            if frame_count['sl_yhat'] < N_FRAMES_DEBUG:
                yhat_s, err_s = [], []
                for _ in range(16):
                    if rb(dut.sl_out_valid):
                        yhat_s.append((rs(dut.sl_out_yhat_I), rs(dut.sl_out_yhat_Q)))
                        err_s.append((rs(dut.sl_out_e_I),     rs(dut.sl_out_e_Q)))
                    await RisingEdge(dut.clk_fast)
                    ciclo += 1
                frames_capturados['sl_yhat'].append(yhat_s)
                frames_capturados['sl_error'].append(err_s)
                frame_count['sl_yhat']  += 1
                frame_count['sl_error'] += 1
                continue

        if rb(dut.zpe_out_valid) and rb(dut.zpe_out_start):
            if frame_count['zpe_out'] < N_FRAMES_DEBUG:
                samples = []
                for _ in range(32):
                    if rb(dut.zpe_out_valid):
                        samples.append((rs(dut.zpe_out_eI), rs(dut.zpe_out_eQ)))
                    await RisingEdge(dut.clk_fast)
                    ciclo += 1
                frames_capturados['zpe_out'].append(samples)
                frame_count['zpe_out'] += 1
                continue

        if rb(dut.ffte_out_valid) and rb(dut.ffte_out_start):
            if frame_count['ffte_out'] < N_FRAMES_DEBUG:
                samples = []
                for _ in range(32):
                    if rb(dut.ffte_out_valid):
                        samples.append((rs(dut.ffte_out_I), rs(dut.ffte_out_Q)))
                    await RisingEdge(dut.clk_fast)
                    ciclo += 1
                frames_capturados['ffte_out'].append(samples)
                frame_count['ffte_out'] += 1
                continue

        if rb(dut.xhd_out_valid) and rb(dut.xhd_out_start):
            if frame_count['xhd_out'] < N_FRAMES_DEBUG:
                samples = []
                for _ in range(32):
                    if rb(dut.xhd_out_valid):
                        samples.append((rs(dut.xhd_out_re), rs(dut.xhd_out_im)))
                    await RisingEdge(dut.clk_fast)
                    ciclo += 1
                frames_capturados['xhd_out'].append(samples)
                frame_count['xhd_out'] += 1
                continue

        if rb(dut.grad_out_valid) and rb(dut.grad_out_start):
            if frame_count['grad_out'] < N_FRAMES_DEBUG:
                samples = []
                for _ in range(32):
                    if rb(dut.grad_out_valid):
                        samples.append((rs(dut.grad_out_re), rs(dut.grad_out_im)))
                    await RisingEdge(dut.clk_fast)
                    ciclo += 1
                frames_capturados['grad_out'].append(samples)
                frame_count['grad_out'] += 1
                continue

        if rb(dut.ifft_grad_valid) and rb(dut.ifft_grad_start):
            if frame_count['ifft_grad'] < N_FRAMES_DEBUG:
                samples = []
                for _ in range(32):
                    if rb(dut.ifft_grad_valid):
                        samples.append((rs(dut.ifft_grad_I), rs(dut.ifft_grad_Q)))
                    await RisingEdge(dut.clk_fast)
                    ciclo += 1
                frames_capturados['ifft_grad'].append(samples)
                frame_count['ifft_grad'] += 1
                continue

        if rb(dut.grad_t_valid) and rb(dut.grad_t_start):
            if frame_count['grad_t'] < N_FRAMES_DEBUG:
                samples = []
                for _ in range(16):
                    if rb(dut.grad_t_valid):
                        samples.append((rs(dut.grad_t_I), rs(dut.grad_t_Q)))
                    await RisingEdge(dut.clk_fast)
                    ciclo += 1
                frames_capturados['grad_t'].append(samples)
                frame_count['grad_t'] += 1
                continue

        if rb(dut.lms_w_valid) and rb(dut.lms_w_start):
            if frame_count['lms_w'] < N_FRAMES_DEBUG:
                samples = []
                for _ in range(16):
                    if rb(dut.lms_w_valid):
                        samples.append((rs(dut.lms_w_I), rs(dut.lms_w_Q)))
                    await RisingEdge(dut.clk_fast)
                    ciclo += 1
                frames_capturados['lms_w'].append(samples)
                frame_count['lms_w'] += 1
                continue

    SEP = "=" * 60
    Q17 = 1024.0
    Q9  = 128.0

    def fmt_frame(samples, scale, n_show=8):
        if not samples:
            return "  (sin datos)"
        lines = []
        for i, (re, im) in enumerate(samples[:n_show]):
            lines.append(f"  [{i:2d}] re={re/scale:+.4f}  im={im/scale:+.4f}  (raw {re:6d},{im:6d})")
        if len(samples) > n_show:
            lines.append(f"  ... ({len(samples)} muestras total)")
        return "\n".join(lines)

    dut._log.info(SEP)
    dut._log.info("RESUMEN DEBUG PIPELINE — canal impulso, sin ruido")
    dut._log.info(SEP)

    for etapa, escala, nombre in [
        ('fft_out',   Q17, "1. FFT salida (X_k)"),
        ('hb_curr',   Q17, "2. HB curr (frame actual X_curr)"),
        ('hb_old',    Q17, "3. HB old  (frame anterior X_old)"),
        ('cmul_out',  Q17, "4. CMUL salida (W*X)"),
        ('ifft_out',  Q9,  "5. IFFT salida (y_blk bruto, 32 muestras)"),
        ('dn_out',    Q9,  "6. discard_n salida (y_blk, 16 muestras útiles)"),
        ('sl_yhat',   Q9,  "7. Slicer yhat (±0.711)"),
        ('sl_error',  Q9,  "8. Slicer error e=yhat-y"),
        ('zpe_out',   Q9,  "9. ZPE salida [0..0|e]"),
        ('ffte_out',  Q17, "10. FFT_ERROR salida E_k"),
        ('xhd_out',   Q17, "11. XHD salida (X retrasado 118 ciclos)"),
        ('grad_out',  Q17, "12. GRADIENTE salida PHI_k=conj(X)*E"),
        ('ifft_grad', Q17, "13. IFFT_GRAD phi(t)"),
        ('grad_t',    Q17, "14. PROYECCION grad_t[0..15]"),
        ('lms_w',     Q17, "15. LMS pesos w (P0)"),
    ]:
        datos = frames_capturados[etapa]
        dut._log.info(f"\n{nombre}")
        dut._log.info(f"  Frames capturados: {len(datos)}")
        if datos:
            dut._log.info(f"  Frame 0:")
            dut._log.info(fmt_frame(datos[0], escala))
            if len(datos) > 1:
                dut._log.info(f"  Frame 1:")
                dut._log.info(fmt_frame(datos[1], escala))

    dut._log.info(f"\n{SEP}")
    dut._log.info("VERIFICACIONES AUTOMÁTICAS")
    dut._log.info(SEP)

    if frames_capturados['sl_error']:
        errs = frames_capturados['sl_error'][0]
        max_err = max(abs(e[0]) + abs(e[1]) for e in errs) if errs else 999
        status = "✓ OK" if max_err <= 4 else "✗ FALLO — error grande"
        dut._log.info(f"  Error slicer max (frame 0): {max_err/Q9:.4f} → {status}")

    if frames_capturados['grad_t']:
        grads = frames_capturados['grad_t'][0]
        max_g = max(abs(g[0]) + abs(g[1]) for g in grads) if grads else 999
        status = "✓ OK" if max_g < 5000 else "✗ FALLO — gradiente grande inesperado"
        dut._log.info(f"  Gradiente max (frame 0): {max_g/Q17:.4f} → {status}")

    if len(frames_capturados['lms_w']) >= 2:
        w0 = frames_capturados['lms_w'][0]
        w1 = frames_capturados['lms_w'][1]
        cambio = sum(abs(a[0]-b[0]) + abs(a[1]-b[1]) for a,b in zip(w0,w1))
        status = "✓ OK" if cambio < 50 else f"✗ FALLO — pesos cambian {cambio} LSB entre frames"
        dut._log.info(f"  Cambio pesos frame0→frame1: {cambio} LSB → {status}")

    if frames_capturados['zpe_out']:
        zpe = frames_capturados['zpe_out'][0]
        primera_mitad = zpe[:16]
        no_zeros = [(i, v) for i, v in enumerate(primera_mitad) if v[0] != 0 or v[1] != 0]
        if no_zeros:
            dut._log.info(f"  ✗ ZPE primera mitad NO es cero: {no_zeros[:3]}")
        else:
            dut._log.info(f"  ✓ ZPE primera mitad = ceros OK")

    if frames_capturados['fft_out'] and frames_capturados['cmul_out']:
        fft = frames_capturados['fft_out'][0]
        cmul = frames_capturados['cmul_out'][0]
        if fft and cmul:
            diff = sum(abs(a[0]-b[0]) + abs(a[1]-b[1]) for a,b in zip(fft[:16], cmul[:16]))
            status = "✓ OK" if diff < 1000 else f"✗ FALLO — CMUL≠FFT_out, diff={diff}"
            dut._log.info(f"  CMUL vs FFT_out diferencia: {diff} → {status}")

    dut._log.info(SEP)
    dut._log.info("=== FIN TEST DEBUG ===")

# ============================================================
# Test MEDICIÓN DE TAP CENTRAL
# ============================================================
@cocotb.test()
async def test_medir_tap_central(dut):
    dut._log.info("=== TEST MEDICIÓN TAP CENTRAL ===")
    await setup(dut, sigma=0)
    await Timer(10000, unit="ns")

    N_FRAMES  = 200
    N_TAPS_DN = 16
    N_TAPS_IF = 32

    energy_dn   = np.zeros(N_TAPS_DN)
    energy_if   = np.zeros(N_TAPS_IF)
    frames_dn   = 0
    frames_if   = 0

    MAX_CICLOS = 2000000
    ciclo = 0

    while ciclo < MAX_CICLOS and (frames_dn < N_FRAMES or frames_if < N_FRAMES):
        await RisingEdge(dut.clk_fast)
        ciclo += 1

        if frames_dn < N_FRAMES and rb(dut.dn_out_valid) and rb(dut.dn_out_start):
            for k in range(N_TAPS_DN):
                if rb(dut.dn_out_valid):
                    v = rs(dut.dn_out_I)**2 + rs(dut.dn_out_Q)**2
                    energy_dn[k] += v
                await RisingEdge(dut.clk_fast)
                ciclo += 1
            frames_dn += 1
            continue

        if frames_if < N_FRAMES and rb(dut.ifft_out_valid) and rb(dut.ifft_out_start):
            for k in range(N_TAPS_IF):
                if rb(dut.ifft_out_valid):
                    v = rs(dut.ifft_out_I)**2 + rs(dut.ifft_out_Q)**2
                    energy_if[k] += v
                await RisingEdge(dut.clk_fast)
                ciclo += 1
            frames_if += 1
            continue

    dut._log.info(f"Frames capturados — dn_out: {frames_dn}  ifft_out: {frames_if}")

    SEP = "=" * 60
    dut._log.info(SEP)
    dut._log.info("ENERGÍA POR TAP — ifft_out (32 muestras, antes de discard_n)")
    dut._log.info(SEP)
    max_if  = np.max(energy_if) if np.max(energy_if) > 0 else 1
    for k in range(N_TAPS_IF):
        bar = int(30 * energy_if[k] / max_if)
        marker = " ← PICO" if energy_if[k] == np.max(energy_if) else ""
        dut._log.info(f"  ifft[{k:2d}] = {energy_if[k]:12.0f}  {'█'*bar}{marker}")

    peak_if = int(np.argmax(energy_if))

    dut._log.info(SEP)
    dut._log.info("ENERGÍA POR TAP — dn_out (16 muestras útiles, post discard_n)")
    dut._log.info(SEP)
    max_dn  = np.max(energy_dn) if np.max(energy_dn) > 0 else 1
    for k in range(N_TAPS_DN):
        bar = int(30 * energy_dn[k] / max_dn)
        marker = " ← PICO" if energy_dn[k] == np.max(energy_dn) else ""
        dut._log.info(f"  dn[{k:2d}]   = {energy_dn[k]:12.0f}  {'█'*bar}{marker}")

    peak_dn = int(np.argmax(energy_dn))

    dut._log.info(SEP)
    dut._log.info("RESULTADO")
    dut._log.info(SEP)
    dut._log.info(f"  Pico en ifft_out: tap {peak_if}")
    dut._log.info(f"  Pico en dn_out:   tap {peak_dn}")
    dut._log.info(f"  El LMS debe inicializarse en tap {peak_dn}")
    dut._log.info(SEP)
    dut._log.info("=== FIN TEST TAP CENTRAL ===")

# ============================================================
# Test TRACE ALINEACIÓN (LMS, XHD, GRAD)
# ============================================================
@cocotb.test()
async def test_trace_lms_alignment(dut):
    dut._log.info("")
    dut._log.info("==========================================")
    dut._log.info(" TRACE LMS ALIGNMENT ")
    dut._log.info("==========================================")

    await setup(dut, sigma=0)
    cocotb.start_soon(monitor_eventos(dut))
    cocotb.start_soon(monitor_delay(dut))

    await Timer(1000, unit="us")

    dut._log.info("")
    dut._log.info("FIN TRACE")

# ============================================================
# Funciones de Ploteo (Matplotlib)
# ============================================================
def _plot_convergencia_completa(n_frames, sim_dir):
    n = min(n_frames, len(w_history[0]))
    if n == 0: return
    frames = np.arange(n)
    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)

    ax = axes[0]
    ax.plot(frames, w_history[0][:n], 'b-', lw=1.5, label='w[0] Re')
    ax.axhline(1.0, color='k', ls='--', alpha=0.4, label='1.0 (identidad)')
    ax.axhline(0.0, color='gray', ls=':', alpha=0.3)
    ax.set_ylabel('Valor (float)')
    ax.set_title('Convergencia LMS — tap principal w[0]')
    ax.legend(fontsize=8); ax.grid(True)

    ax = axes[1]
    colors = plt.cm.tab10(np.linspace(0, 1, 7))
    for k in range(1, 8):
        if w_history[k]:
            ax.plot(frames, w_history[k][:n], lw=1, color=colors[k-1], label=f'w[{k}]')
    ax.axhline(0.0, color='gray', ls=':', alpha=0.3)
    ax.set_xlabel('Frame LMS')
    ax.set_ylabel('Valor (float)')
    ax.set_title('Convergencia LMS — taps w[1..7] (P0)')
    ax.legend(fontsize=7, ncol=4); ax.grid(True)

    plt.tight_layout()
    plt.savefig(os.path.join(sim_dir, 'convergencia_lms.png'), dpi=150)
    plt.close(fig)

def _plot_filtro_completo(n_frames, sim_dir, suffix=""):
    n0 = len(w_history[0])
    n1 = len(w1_history[0])
    if n0 == 0: return

    w_full = [(w_history[k][-1]  if w_history[k]  else 0.0) for k in range(16)] + \
             [(w1_history[k][-1] if w1_history[k] else 0.0) for k in range(16)]
    taps = np.arange(32)

    fig, axes = plt.subplots(2, 1, figsize=(12, 8))

    ax = axes[0]
    ax.vlines(taps[:16], 0, w_full[:16], color='C0', lw=1.2)
    ax.scatter(taps[:16], w_full[:16], color='C0', s=28, zorder=3, label='P0 (X_curr)  taps 0..15')
    ax.vlines(taps[16:], 0, w_full[16:], color='C1', lw=1.2)
    ax.scatter(taps[16:], w_full[16:], color='C1', s=28, zorder=3, label='P1 (X_old)   taps 16..31')
    ax.axhline(0.0, color='k', lw=0.8)
    ax.axvline(15.5, color='gray', ls='--', alpha=0.5)
    ax.set_xlabel('tap'); ax.set_ylabel('valor (float)')
    ax.set_title('Filtro ecualizador completo — 32 taps (2 particiones)')
    ax.legend(fontsize=9); ax.grid(True, alpha=0.3)

    ax = axes[1]
    m = min(n0, n1)
    frames = np.arange(m)
    if m > 0:
        ax.plot(frames, w_history[0][:m], 'C0-', lw=1.5, label='w[0] (P0, tap principal)')
        w1_arr  = np.array([w1_history[k][:m] for k in range(16)])
        max_p1  = np.max(np.abs(w1_arr), axis=0)
        ener_p1 = np.sqrt(np.sum(w1_arr**2, axis=0))
        ax.plot(frames, max_p1,  'C1-',  lw=1.2, label='max|w| de P1')
        ax.plot(frames, ener_p1, 'C3--', lw=1.0, label='‖w P1‖₂')
    ax.axhline(1.0, color='k', ls='--', alpha=0.4)
    ax.axhline(0.0, color='gray', ls=':', alpha=0.3)
    ax.set_xlabel('Frame LMS'); ax.set_ylabel('valor (float)')
    ax.set_title('Convergencia: tap principal (P0) y actividad de la partición 1')
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

    plt.tight_layout()
    out = os.path.join(sim_dir, f'filtro_completo{suffix}.png')
    plt.savefig(out, dpi=150)
    plt.close(fig)

def _plot_constelaciones(sim_dir, sigma, n_frames):
    if not const_canal and not const_eq: return

    n_eq  = len(const_eq)
    mitad = n_eq // 2
    MAX_PTS = 2000

    def subsample(pts, max_n):
        if len(pts) <= max_n: return pts
        idx = np.linspace(0, len(pts)-1, max_n, dtype=int)
        return [pts[i] for i in idx]

    canal_pts  = subsample(const_canal, MAX_PTS)
    eq_trans   = subsample(const_eq[:mitad], MAX_PTS)
    eq_ss      = subsample(const_eq[mitad:], MAX_PTS)

    ideal = [(+0.711, +0.711), (+0.711, -0.711),
             (-0.711, +0.711), (-0.711, -0.711)]

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    fig.suptitle(f'Constelaciones QPSK — sigma={sigma}  |  {n_frames} frames LMS', fontsize=12)

    titles = [
        f'Entrada al ecualizador\n(salida del canal)\n{len(canal_pts)} puntos',
        f'Salida ecualizador — transitorio\n(frames 0..{mitad})\n{len(eq_trans)} puntos',
        f'Salida ecualizador — estado estable\n(frames {mitad}..{n_eq})\n{len(eq_ss)} puntos',
    ]
    datasets = [canal_pts, eq_trans, eq_ss]
    colors   = ['steelblue', 'coral', 'seagreen']

    for ax, title, pts, color in zip(axes, titles, datasets, colors):
        if pts:
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            ax.scatter(xs, ys, s=3, alpha=0.35, color=color, rasterized=True)
        for ix, iy in ideal:
            ax.plot(ix, iy, 'k+', ms=14, mew=2, zorder=5)
        ax.axhline(0, color='gray', lw=0.7, ls='--', alpha=0.5)
        ax.axvline(0, color='gray', lw=0.7, ls='--', alpha=0.5)
        ax.set_xlim(-1.5, 1.5); ax.set_ylim(-1.5, 1.5)
        ax.set_aspect('equal')
        ax.set_xlabel('I'); ax.set_ylabel('Q')
        ax.set_title(title, fontsize=9)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    out = os.path.join(sim_dir, f'constelacion_sigma{sigma}.png')
    plt.savefig(out, dpi=150)
    plt.close(fig)

def _plot_flujo_completo(sim_dir, sigma, n_frames):
    if not const_canal and not const_eq: return

    MAX_PTS = 3000

    def subsample(pts, max_n):
        if len(pts) <= max_n: return pts
        idx = np.linspace(0, len(pts) - 1, max_n, dtype=int)
        return [pts[i] for i in idx]

    n_eq  = len(const_eq)
    mitad = n_eq // 2
    
    in_pts = subsample(const_canal, MAX_PTS)
    eq_pts = subsample(const_eq[mitad:], MAX_PTS)
    sl_pts = subsample(const_slicer[mitad:] if const_slicer else [], MAX_PTS)

    ideal = [(+0.711, +0.711), (+0.711, -0.711),
             (-0.711, +0.711), (-0.711, -0.711)]

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    fig.suptitle(f'Flujo de Señal: Canal → EQ → Slicer (Estado Estable) | sigma={sigma}',
                 fontsize=14, fontweight='bold')

    panels = [
        (axes[0], in_pts, 'steelblue', f'1. ENTRADA AL EQ\n(Salida del Canal)'),
        (axes[1], eq_pts, 'seagreen',  f'2. SALIDA DEL EQ\n(Entrada al Slicer)'),
        (axes[2], sl_pts, 'darkred',   f'3. SALIDA DEL SLICER\n(Decisiones Duras)'),
    ]

    for ax, pts, color, title in panels:
        if pts:
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            ax.scatter(xs, ys, s=4, alpha=0.35, color=color, rasterized=True)
        for ix, iy in ideal:
            ax.plot(ix, iy, 'k+', ms=16, mew=2, zorder=5)
        ax.axhline(0, color='gray', lw=0.7, ls='--', alpha=0.5)
        ax.axvline(0, color='gray', lw=0.7, ls='--', alpha=0.5)
        ax.set_xlim(-1.6, 1.6); ax.set_ylim(-1.6, 1.6)
        ax.set_aspect('equal')
        ax.set_xlabel('I'); ax.set_ylabel('Q')
        ax.set_title(title, fontsize=11)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    out = os.path.join(sim_dir, f'flujo_canal_eq_slicer_sigma{sigma}.png')
    plt.savefig(out, dpi=150)
    plt.close(fig)

def _plot_convergencia_sigma(sigma, sim_dir):
    n = len(w_history[0])
    if n == 0: return
    frames = np.arange(n)
    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    for k, ax in zip([0, 1], axes):
        vals = w_history[k][:n]
        if vals:
            ax.plot(frames, vals, label=f'w[{k}] Re')
            if k == 0:
                ax.axhline(1.0, color='k', ls='--', alpha=0.4)
            ax.axhline(0.0, color='gray', ls=':', alpha=0.3)
            ax.set_ylabel(f'w[{k}]'); ax.legend(fontsize=8); ax.grid(True)
    axes[0].set_title(f'Convergencia pesos LMS  (sigma={sigma})')
    axes[1].set_xlabel('Frame LMS')
    plt.tight_layout()
    plt.savefig(os.path.join(sim_dir, f'convergencia_sigma{sigma}.png'), dpi=120)
    plt.close(fig)

def _plot_evolucion_error(sim_dir, sigma):
    if not error_history:
        return

    err_sq = [eI**2 + eQ**2 for eI, eQ in error_history]
    
    muestras_por_frame = 16
    n_frames = len(err_sq) // muestras_por_frame
    if n_frames == 0: return
        
    mse_por_frame = []
    for i in range(n_frames):
        chunk = err_sq[i * muestras_por_frame : (i + 1) * muestras_por_frame]
        mse_por_frame.append(sum(chunk) / muestras_por_frame)
        
    fig, ax = plt.subplots(figsize=(10, 5))
    frames = np.arange(n_frames)
    
    ax.plot(frames, mse_por_frame, color='orchid', lw=1, alpha=0.6, label='MSE por frame')
    
    window = min(20, max(2, n_frames // 10))
    if n_frames >= window:
        mse_smooth = np.convolve(mse_por_frame, np.ones(window)/window, mode='valid')
        ax.plot(frames[window-1:], mse_smooth, color='indigo', lw=2, label=f'Media móvil ({window} frames)')

    ax.set_yscale('log')
    ax.set_xlabel('Frame LMS')
    ax.set_ylabel('Mean Squared Error (Escala Log)')
    ax.set_title(f'Curva de Aprendizaje del Ecualizador — sigma={sigma}')
    ax.legend(fontsize=10)
    ax.grid(True, which="both", ls="--", alpha=0.4)
    
    plt.tight_layout()
    out = os.path.join(sim_dir, f'evolucion_error_sigma{sigma}.png')
    plt.savefig(out, dpi=150)
    plt.close(fig)

# ============================================================
# DEBUG PROFUNDO — trazabilidad completa del lazo adaptativo
# Agregar al final de test_top.py. No reemplaza los tests previos.
# ============================================================
try:
    from cocotb.utils import get_sim_time
except Exception:
    def get_sim_time(unit="ns"):
        return 0


def _get_sig_safe(dut, path):
    """Devuelve una señal por nombre jerárquico simple. Si no existe, None."""
    try:
        obj = dut
        for p in path.split('.'):
            obj = getattr(obj, p)
        return obj
    except Exception:
        return None


def _rb_path(dut, path):
    sig = _get_sig_safe(dut, path)
    if sig is None:
        return 0
    return rb(sig)


def _rs_path(dut, path):
    sig = _get_sig_safe(dut, path)
    if sig is None:
        return 0
    return rs(sig)


def _read_mem_signed_safe(dut, inst_path, mem_name, idx):
    """Lee memorias Verilog tipo W0_re[k] si el simulador las expone."""
    try:
        inst = _get_sig_safe(dut, inst_path)
        if inst is None:
            return None
        mem = getattr(inst, mem_name)
        return int(mem[idx].value.to_signed())
    except Exception:
        try:
            inst = _get_sig_safe(dut, inst_path)
            if inst is None:
                return None
            sig = getattr(inst, f"{mem_name}[{idx}]")
            return int(sig.value.to_signed())
        except Exception:
            return None


def _cfloat(pair, scale):
    return complex(pair[0] / scale, pair[1] / scale)


def _frame_energy(samples, scale):
    if not samples:
        return 0.0
    return sum((re / scale) ** 2 + (im / scale) ** 2 for re, im in samples)


def _max_abs_complex(samples, scale):
    if not samples:
        return 0.0
    return max(abs(_cfloat(s, scale)) for s in samples)


def _diff_l1(a, b, scale):
    n = min(len(a), len(b))
    if n == 0:
        return None
    return sum(abs(a[i][0] - b[i][0]) + abs(a[i][1] - b[i][1]) for i in range(n)) / scale


def _complex_mult_q_float(a, b):
    """Producto complejo en float, útil para checks aproximados."""
    ar, ai = a.real, a.imag
    br, bi = b.real, b.imag
    return complex(ar * br - ai * bi, ar * bi + ai * br)


def _fmt_sample(k, re, im, scale):
    return f"[{k:02d}] {re/scale:+9.5f} {im/scale:+9.5f}j  raw=({re:+7d},{im:+7d})"


def _log_frame(dut, title, samples, scale, max_bins=None):
    if max_bins is None:
        max_bins = int(os.environ.get("DEBUG_BINS", "32"))
    dut._log.info(title)
    if not samples:
        dut._log.info("  sin datos")
        return
    n_show = min(max_bins, len(samples))
    for k in range(n_show):
        re, im = samples[k]
        dut._log.info("  " + _fmt_sample(k, re, im, scale))
    if len(samples) > n_show:
        dut._log.info(f"  ... {len(samples)-n_show} muestras no impresas")
    dut._log.info(
        f"  energy={_frame_energy(samples, scale):.6f}  "
        f"max_abs={_max_abs_complex(samples, scale):.6f}"
    )


async def _wait_lms_frame_count(dut, n_frames, timeout_cycles=2000000):
    """Espera n starts de lms_w. Devuelve cantidad observada."""
    cnt = 0
    cyc = 0
    last_print = 0
    while cnt < n_frames and cyc < timeout_cycles:
        await RisingEdge(dut.clk_fast)
        cyc += 1
        if rb(dut.lms_w_valid) and rb(dut.lms_w_start):
            cnt += 1
            if cnt <= 3 or cnt % 50 == 0:
                dut._log.info(f"[WAIT] LMS frame {cnt}/{n_frames} en ciclo {cyc}")
        if cyc - last_print > 100000:
            last_print = cyc
            dut._log.info(f"[WAIT] ciclos={cyc}  lms_frames={cnt}/{n_frames}")
    return cnt


async def _wait_one_full_weight_write(dut, timeout_cycles=50000):
    """Espera un frame completo de fft_w para asegurar escritura W0/W1 en CMUL."""
    cyc = 0
    seen_start = False
    n = 0
    while cyc < timeout_cycles:
        await RisingEdge(dut.clk_fast)
        cyc += 1
        if rb(dut.fft_w_valid):
            if rb(dut.fft_w_start):
                seen_start = True
                n = 0
            if seen_start:
                n += 1
                if n >= 32:
                    dut._log.info(f"[WAIT] escritura completa de W0/W1 observada ({n} bins)")
                    return True
    dut._log.warning("[WAIT] no se observó escritura completa de W0/W1")
    return False


async def _capture_frames_from_stage(dut, name, valid_path, start_path, sig_paths, n_samples, max_frames, store):
    """Captura frames completos de una etapa sin bloquear a las demás."""
    store[name] = []
    while len(store[name]) < max_frames:
        await RisingEdge(dut.clk_fast)
        if _rb_path(dut, valid_path) and _rb_path(dut, start_path):
            frame = {
                "time_ns": get_sim_time("ns"),
                "samples": []
            }
            for k in range(n_samples):
                if _rb_path(dut, valid_path):
                    vals = tuple(_rs_path(dut, p) for p in sig_paths)
                    frame["samples"].append(vals)
                else:
                    frame["samples"].append((None, None))
                if k < n_samples - 1:
                    await RisingEdge(dut.clk_fast)
            store[name].append(frame)


async def _monitor_alignment_and_writes(dut, stop_flag, events, max_print_frames=3):
    """Monitor liviano: starts, lockstep W0/W1 y escritura hacia CMUL."""
    cycle = 0
    wr_k = 0
    wr_frame = -1
    starts_to_track = [
        ("fft_in",       "fft_in_valid",       "fft_in_start"),
        ("fft_out",      "fft_out_valid",      "fft_out_start"),
        ("hb",           "hb_out_valid",       "hb_out_start"),
        ("cmul",         "cmul_out_valid",     "cmul_out_start"),
        ("ifft",         "ifft_out_valid",     "ifft_out_start"),
        ("dn",           "dn_out_valid",       "dn_out_start"),
        ("slicer",       "sl_out_valid",       "sl_out_start"),
        ("zpe",          "zpe_out_valid",      "zpe_out_start"),
        ("ffte",         "ffte_out_valid",     "ffte_out_start"),
        ("xhd0",         "xhd_out_valid",      "xhd_out_start"),
        ("grad0",        "grad_out_valid",     "grad_out_start"),
        ("ifft_grad0",   "ifft_grad_valid",    "ifft_grad_start"),
        ("grad_t0",      "grad_t_valid",       "grad_t_start"),
        ("lms0",         "lms_w_valid",        "lms_w_start"),
        ("zpp0",         "zpp_out_valid",      "zpp_out_start"),
        ("fft_w0",       "fft_w_valid",        "fft_w_start"),
        ("fft_w1",       "fft_w1_valid",       "fft_w1_start"),
    ]

    # Estas son internas; si el simulador no las expone, simplemente no se imprimen.
    optional = [
        ("xhd1",       "xhd_old_valid",     "xhd_old_start"),
        ("grad1",      "grad1_valid",       "grad1_start"),
        ("ifft_grad1", "ifft_grad1_valid",  "ifft_grad1_start"),
        ("grad_t1",    "grad1_t_valid",     "grad1_t_start"),
        ("lms1",       "lms1_w_valid",      "lms1_w_start"),
        ("zpp1",       "zpp1_out_valid",    "zpp1_out_start"),
    ]
    starts_to_track += [(n, v, s) for n, v, s in optional if _get_sig_safe(dut, v) is not None]

    while not stop_flag["done"]:
        await RisingEdge(dut.clk_fast)
        cycle += 1

        line = []
        for name, valid, start in starts_to_track:
            if _rb_path(dut, valid) and _rb_path(dut, start):
                events.setdefault(name, []).append(cycle)
                line.append(name)
        if line:
            if len(events.get("lms0", [])) <= max_print_frames + 2:
                dut._log.info(f"[START cyc={cycle:8d} t={get_sim_time('ns'):10.0f} ns] " + " | ".join(line))

        # Lockstep de las dos FFT de pesos.
        fw0_v = rb(dut.fft_w_valid)
        fw1_v = rb(dut.fft_w1_valid)
        if fw0_v != fw1_v:
            dut._log.error(f"[LOCKSTEP] fft_w_valid={fw0_v} fft_w1_valid={fw1_v} en ciclo {cycle}")
        if fw0_v and (rb(dut.fft_w_start) != rb(dut.fft_w1_start)):
            dut._log.error(f"[LOCKSTEP] fft_w_start != fft_w1_start en ciclo {cycle}")

        # Escrituras de W0/W1 al CMUL.
        if fw0_v:
            if rb(dut.fft_w_start):
                wr_k = 0
                wr_frame += 1
                if wr_frame < max_print_frames:
                    dut._log.info(f"[CMUL_WR] frameW={wr_frame} start ciclo={cycle}")
            k = wr_k
            if wr_frame < max_print_frames:
                dut._log.info(
                    f"[CMUL_WR] frameW={wr_frame:03d} k={k:02d}  "
                    f"W0={fmt_complex(rs(dut.fft_w_I),  rs(dut.fft_w_Q),  1024)}  "
                    f"W1={fmt_complex(rs(dut.fft_w1_I), rs(dut.fft_w1_Q), 1024)}"
                )
            wr_k = (wr_k + 1) & 31


async def _dump_cmul_internal_weights(dut, title="CMUL internal W memory"):
    dut._log.info("=" * 80)
    dut._log.info(title)
    dut._log.info("=" * 80)
    any_ok = False
    for k in range(32):
        w0r = _read_mem_signed_safe(dut, "u_cmul", "W0_re", k)
        w0i = _read_mem_signed_safe(dut, "u_cmul", "W0_im", k)
        w1r = _read_mem_signed_safe(dut, "u_cmul", "W1_re", k)
        w1i = _read_mem_signed_safe(dut, "u_cmul", "W1_im", k)
        if None not in (w0r, w0i, w1r, w1i):
            any_ok = True
            dut._log.info(
                f"  k={k:02d}  "
                f"W0={fmt_complex(w0r, w0i, 1024)} raw=({w0r:+7d},{w0i:+7d})  "
                f"W1={fmt_complex(w1r, w1i, 1024)} raw=({w1r:+7d},{w1i:+7d})"
            )
    if not any_ok:
        dut._log.warning("No pude leer u_cmul.W0/W1. El simulador puede no exponer memorias internas.")


def _summarize_debug_capture(dut, frames, events):
    dut._log.info("=" * 80)
    dut._log.info("RESUMEN NUMERICO DEL DEBUG PROFUNDO")
    dut._log.info("=" * 80)

    scales = {
        "fft_in": 128.0,
        "fft_out": 1024.0,
        "hb_curr": 1024.0,
        "hb_old": 1024.0,
        "cmul_out": 1024.0,
        "ifft_out": 128.0,
        "dn_out": 128.0,
        "slicer_yhat": 128.0,
        "slicer_err": 128.0,
        "zpe": 128.0,
        "ffte": 1024.0,
        "xhd0": 1024.0,
        "xhd1": 1024.0,
        "grad0": 1024.0,
        "grad1": 1024.0,
        "ifft_grad0": 1024.0,
        "ifft_grad1": 1024.0,
        "grad_t0": 1024.0,
        "grad_t1": 1024.0,
        "lms0": 1024.0,
        "lms1": 1024.0,
        "zpp0": 1024.0,
        "zpp1": 1024.0,
        "fft_w0": 1024.0,
        "fft_w1": 1024.0,
    }

    for name, flist in frames.items():
        if not flist:
            dut._log.warning(f"{name:14s}: sin frames")
            continue
        s = flist[-1]["samples"]
        scale = scales.get(name, 1024.0)
        dut._log.info(
            f"{name:14s}: frames={len(flist):2d}  "
            f"E_ult={_frame_energy(s, scale):12.6f}  "
            f"max_abs_ult={_max_abs_complex(s, scale):10.6f}  "
            f"t0={flist[0]['time_ns']}ns"
        )

    def cmp(name_a, name_b, scale):
        if frames.get(name_a) and frames.get(name_b):
            a = frames[name_a][-1]["samples"]
            b = frames[name_b][-1]["samples"]
            d = _diff_l1(a, b, scale)
            dut._log.info(f"DIFF {name_a:12s} vs {name_b:12s}: L1/scale={d:.6f}")

    dut._log.info("-" * 80)
    dut._log.info("Comparaciones directas")
    cmp("fft_out", "hb_curr", 1024.0)
    cmp("hb_curr", "cmul_out", 1024.0)
    cmp("fft_in", "dn_out", 128.0)

    if frames.get("zpe"):
        z = frames["zpe"][-1]["samples"]
        nz_first = [(i, v) for i, v in enumerate(z[:16]) if v[0] != 0 or v[1] != 0]
        dut._log.info(f"ZPE primera mitad no-cero: {nz_first[:8]}  count={len(nz_first)}")

    if frames.get("grad0") and frames.get("xhd0") and frames.get("ffte"):
        g = frames["grad0"][-1]["samples"]
        x = frames["xhd0"][-1]["samples"]
        e = frames["ffte"][-1]["samples"]
        errs = []
        for k in range(min(len(g), len(x), len(e))):
            xf = complex(x[k][0]/1024.0, x[k][1]/1024.0)
            ef = complex(e[k][0]/1024.0, e[k][1]/1024.0)
            exp = complex(xf.real, -xf.imag) * ef
            got = complex(g[k][0]/1024.0, g[k][1]/1024.0)
            errs.append(abs(got - exp))
        dut._log.info(f"CHECK grad0=conj(XHD0)*E: err_max_float={max(errs):.6f} err_mean={np.mean(errs):.6f}")

    if frames.get("grad1") and frames.get("xhd1") and frames.get("ffte"):
        g = frames["grad1"][-1]["samples"]
        x = frames["xhd1"][-1]["samples"]
        e = frames["ffte"][-1]["samples"]
        errs = []
        for k in range(min(len(g), len(x), len(e))):
            xf = complex(x[k][0]/1024.0, x[k][1]/1024.0)
            ef = complex(e[k][0]/1024.0, e[k][1]/1024.0)
            exp = complex(xf.real, -xf.imag) * ef
            got = complex(g[k][0]/1024.0, g[k][1]/1024.0)
            errs.append(abs(got - exp))
        dut._log.info(f"CHECK grad1=conj(XHD1)*E: err_max_float={max(errs):.6f} err_mean={np.mean(errs):.6f}")

    dut._log.info("-" * 80)
    dut._log.info("Deltas de START por frame")
    for a, b in [("ffte", "xhd0"), ("ffte", "xhd1"), ("ffte", "grad0"), ("ffte", "grad1"),
                 ("grad_t0", "lms0"), ("grad_t1", "lms1"), ("fft_w0", "fft_w1")]:
        ea = events.get(a, [])
        eb = events.get(b, [])
        if ea and eb:
            n = min(len(ea), len(eb), 5)
            ds = [eb[i] - ea[i] for i in range(n)]
            dut._log.info(f"  {b:10s} - {a:10s}: {ds}")
        else:
            dut._log.warning(f"  sin datos para delta {b}-{a}")

    if frames.get("lms0"):
        _log_frame(dut, "ULTIMO LMS0 temporal w0[0..15]", frames["lms0"][-1]["samples"], 1024.0, 16)
    if frames.get("lms1"):
        _log_frame(dut, "ULTIMO LMS1 temporal w1[0..15]", frames["lms1"][-1]["samples"], 1024.0, 16)
    if frames.get("fft_w0"):
        _log_frame(dut, "ULTIMA FFT_W0 escrita al CMUL", frames["fft_w0"][-1]["samples"], 1024.0, 32)
    if frames.get("fft_w1"):
        _log_frame(dut, "ULTIMA FFT_W1 escrita al CMUL", frames["fft_w1"][-1]["samples"], 1024.0, 32)


@cocotb.test()
async def test_debug_full_loop(dut):
    """
    Debug completo para el caso: los pesos convergen pero la salida del EQ queda igual.

    Variables útiles:
      DEBUG_SKIP_LMS_FRAMES=300  frames LMS a esperar antes de capturar
      DEBUG_FRAMES=3             frames completos por etapa a guardar
      DEBUG_BINS=32              bins/muestras a imprimir por frame
      SIGMA_SCALE=0              ruido
    """
    dut._log.info("=" * 80)
    dut._log.info("TEST DEBUG FULL LOOP — PBFDAF/LMS completo")
    dut._log.info("=" * 80)

    sigma = int(os.environ.get("SIGMA_SCALE", "0"))
    skip_lms = int(os.environ.get("DEBUG_SKIP_LMS_FRAMES", "300"))
    n_frames = int(os.environ.get("DEBUG_FRAMES", "3"))

    await setup(dut, sigma=sigma)

    dut._log.info(f"Esperando convergencia parcial: {skip_lms} frames LMS")
    got = await _wait_lms_frame_count(dut, skip_lms)
    if got < skip_lms:
        dut._log.error(f"No se llegó a {skip_lms} frames LMS. Se observaron {got}.")
        return

    await _wait_one_full_weight_write(dut)
    await _dump_cmul_internal_weights(dut, "CMUL W justo antes de capturar frames de señal")

    frames = {}
    events = {}
    stop_flag = {"done": False}

    capture_specs = [
        ("fft_in",      "fft_in_valid",    "fft_in_start",    ["fft_in_I", "fft_in_Q"], 32),
        ("fft_out",     "fft_out_valid",   "fft_out_start",   ["fft_out_I", "fft_out_Q"], 32),
        ("hb_curr",     "hb_out_valid",    "hb_out_start",    ["hb_out_curr_I", "hb_out_curr_Q"], 32),
        ("hb_old",      "hb_out_valid",    "hb_out_start",    ["hb_out_old_I", "hb_out_old_Q"], 32),
        ("cmul_out",    "cmul_out_valid",  "cmul_out_start",  ["cmul_out_I", "cmul_out_Q"], 32),
        ("ifft_out",    "ifft_out_valid",  "ifft_out_start",  ["ifft_out_I", "ifft_out_Q"], 32),
        ("dn_out",      "dn_out_valid",    "dn_out_start",    ["dn_out_I", "dn_out_Q"], 16),
        ("slicer_yhat", "sl_out_valid",    "sl_out_start",    ["sl_out_yhat_I", "sl_out_yhat_Q"], 16),
        ("slicer_err",  "sl_out_valid",    "sl_out_start",    ["sl_out_e_I", "sl_out_e_Q"], 16),
        ("zpe",         "zpe_out_valid",   "zpe_out_start",   ["zpe_out_eI", "zpe_out_eQ"], 32),
        ("ffte",        "ffte_out_valid",  "ffte_out_start",  ["ffte_out_I", "ffte_out_Q"], 32),
        ("xhd0",        "xhd_out_valid",   "xhd_out_start",   ["xhd_out_re", "xhd_out_im"], 32),
        ("grad0",       "grad_out_valid",  "grad_out_start",  ["grad_out_re", "grad_out_im"], 32),
        ("ifft_grad0",  "ifft_grad_valid", "ifft_grad_start", ["ifft_grad_I", "ifft_grad_Q"], 32),
        ("grad_t0",     "grad_t_valid",    "grad_t_start",    ["grad_t_I", "grad_t_Q"], 16),
        ("lms0",        "lms_w_valid",     "lms_w_start",     ["lms_w_I", "lms_w_Q"], 16),
        ("zpp0",        "zpp_out_valid",   "zpp_out_start",   ["zpp_out_wI", "zpp_out_wQ"], 32),
        ("fft_w0",      "fft_w_valid",     "fft_w_start",     ["fft_w_I", "fft_w_Q"], 32),
        ("fft_w1",      "fft_w1_valid",    "fft_w1_start",    ["fft_w1_I", "fft_w1_Q"], 32),
    ]

    # Señales internas de la rama P1. Se agregan solo si están visibles.
    optional_specs = [
        ("xhd1",       "xhd_old_valid",    "xhd_old_start",    ["xhd_old_re", "xhd_old_im"], 32),
        ("grad1",      "grad1_valid",      "grad1_start",      ["grad1_re", "grad1_im"], 32),
        ("ifft_grad1", "ifft_grad1_valid", "ifft_grad1_start", ["ifft_grad1_I", "ifft_grad1_Q"], 32),
        ("grad_t1",    "grad1_t_valid",    "grad1_t_start",    ["grad1_t_I", "grad1_t_Q"], 16),
        ("lms1",       "lms1_w_valid",     "lms1_w_start",     ["lms1_w_I", "lms1_w_Q"], 16),
        ("zpp1",       "zpp1_out_valid",   "zpp1_out_start",   ["zpp1_out_wI", "zpp1_out_wQ"], 32),
    ]
    capture_specs += [spec for spec in optional_specs if _get_sig_safe(dut, spec[1]) is not None]

    for name, valid, start, sigs, nsamp in capture_specs:
        cocotb.start_soon(_capture_frames_from_stage(dut, name, valid, start, sigs, nsamp, n_frames, frames))
    cocotb.start_soon(_monitor_alignment_and_writes(dut, stop_flag, events))

    timeout_cycles = int(os.environ.get("DEBUG_TIMEOUT_CYCLES", "500000"))
    cyc = 0
    while cyc < timeout_cycles:
        await RisingEdge(dut.clk_fast)
        cyc += 1
        ready = all(len(frames.get(name, [])) >= n_frames for name, _, _, _, _ in capture_specs)
        if ready:
            break
        if cyc % 100000 == 0:
            status = ", ".join(f"{name}:{len(frames.get(name, []))}/{n_frames}" for name, _, _, _, _ in capture_specs)
            dut._log.info(f"[CAPTURE] ciclo={cyc}  {status}")

    stop_flag["done"] = True

    if cyc >= timeout_cycles:
        dut._log.warning("Timeout de captura. Se imprime lo capturado hasta ahora.")

    _summarize_debug_capture(dut, frames, events)
    await _dump_cmul_internal_weights(dut, "CMUL W al final del debug")

    # Impresión completa de los últimos frames de señal directa.
    for name, scale in [
        ("fft_in", 128.0), ("fft_out", 1024.0), ("hb_curr", 1024.0), ("hb_old", 1024.0),
        ("cmul_out", 1024.0), ("ifft_out", 128.0), ("dn_out", 128.0),
        ("slicer_yhat", 128.0), ("slicer_err", 128.0), ("zpe", 128.0),
        ("ffte", 1024.0), ("xhd0", 1024.0), ("xhd1", 1024.0),
        ("grad0", 1024.0), ("grad1", 1024.0),
        ("ifft_grad0", 1024.0), ("ifft_grad1", 1024.0),
        ("grad_t0", 1024.0), ("grad_t1", 1024.0),
        ("lms0", 1024.0), ("lms1", 1024.0),
        ("zpp0", 1024.0), ("zpp1", 1024.0),
        ("fft_w0", 1024.0), ("fft_w1", 1024.0),
    ]:
        if frames.get(name):
            _log_frame(dut, f"FRAME COMPLETO FINAL — {name}", frames[name][-1]["samples"], scale)

    dut._log.info("=" * 80)
    dut._log.info("FIN TEST DEBUG FULL LOOP")
    dut._log.info("=" * 80)
