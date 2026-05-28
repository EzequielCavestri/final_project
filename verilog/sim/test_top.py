import os
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

# ============================================================
# MODO DE OPERACION — cambiá estas variables
# ============================================================
MODO_BER          = True   # True = corre sweep BER (tarda ~90 min)
MODO_CONSTELACION = True    # True = genera constelacion
MODO_COEFICIENTES = True    # True = genera convergencia de taps
SIGMA_FIJO        = 4       # sigma usado cuando MODO_BER=False
# ============================================================

def prbs9(seed, n_bits):
    reg = seed & 0x1FF
    bits = []
    for _ in range(n_bits):
        bit = (reg >> 8) & 1
        bits.append(bit)
        feedback = ((reg >> 8) ^ (reg >> 4)) & 1
        reg = ((reg << 1) | feedback) & 0x1FF
    return bits

rx_bits_I = []
rx_bits_Q = []
w_history = [[] for _ in range(16)]
const_I   = []
const_Q   = []

async def captura_slicer(dut):
    while True:
        await RisingEdge(dut.clk_fast)
        if int(dut.sl_out_valid.value) == 1:
            try:
                yI = int(dut.sl_out_yhat_I.value.signed_integer)
                yQ = int(dut.sl_out_yhat_Q.value.signed_integer)
                rx_bits_I.append(0 if yI > 0 else 1)
                rx_bits_Q.append(0 if yQ > 0 else 1)
            except:
                pass

async def captura_constelacion(dut):
    while True:
        await RisingEdge(dut.clk_fast)
        if int(dut.dn_out_valid.value) == 1:
            try:
                yI = int(dut.dn_out_I.value.signed_integer)
                yQ = int(dut.dn_out_Q.value.signed_integer)
                const_I.append(yI)
                const_Q.append(yQ)
            except:
                pass

async def captura_pesos(dut):
    while True:
        await RisingEdge(dut.lms_w_start)
        try:
            for k in range(16):
                w = int(dut.u_lms.w_re[k].value.signed_integer)
                w_history[k].append(w / 1024.0)
        except:
            pass

async def heartbeat(dut):
    t = 0
    while True:
        await Timer(100, unit="us")
        t += 100
        dut._log.info(f"[HEARTBEAT] {t} us — "
                      f"rx={len(rx_bits_I)} simbolos, "
                      f"frames={len(w_history[0])}, "
                      f"const={len(const_I)}")

@cocotb.test()
async def ber_convergencia(dut):
    # Lee sigma desde variable de entorno (sweep) o usa SIGMA_FIJO
    sigma = int(os.environ.get("SIGMA_SCALE", str(SIGMA_FIJO)))
    dut._log.info(f"=== sigma_scale={sigma} | BER={MODO_BER} | "
                  f"CONST={MODO_CONSTELACION} | COEF={MODO_COEFICIENTES} ===")

    sim_dir = os.path.dirname(os.path.abspath(__file__))

    cocotb.start_soon(Clock(dut.clk_fast, 10, unit="ns").start())

    dut.rst.value = 1
    dut.enable_div.value = 1
    dut.sigma_scale.value = sigma
    await Timer(2000, unit="us")
    dut.rst.value = 0

    # Lanza coroutines según modo
    if MODO_BER:
        cocotb.start_soon(captura_slicer(dut))
    if MODO_CONSTELACION:
        cocotb.start_soon(captura_constelacion(dut))
    if MODO_COEFICIENTES:
        cocotb.start_soon(captura_pesos(dut))
    cocotb.start_soon(heartbeat(dut))

    await Timer(500, unit="us")

    dut._log.info(f"Simbolos RX: {len(rx_bits_I)}")
    dut._log.info(f"Frames LMS:  {len(w_history[0])}")
    dut._log.info(f"Muestras constelacion: {len(const_I)}")

    # ---- BER ----
    if MODO_BER and len(rx_bits_I) >= 200:
        REF_LEN = len(rx_bits_I) + 600
        ref_I   = np.array(prbs9(0x17F, REF_LEN))
        ref_Q   = np.array(prbs9(0x11D, REF_LEN))
        rx_arr  = np.array(rx_bits_I)

        best_offset, best_corr = 0, -1
        search_len = min(200, len(rx_arr))
        for offset in range(600):
            if offset + search_len > len(ref_I):
                break
            c = np.sum(rx_arr[:search_len] == ref_I[offset:offset+search_len])
            if c > best_corr:
                best_corr, best_offset = c, offset

        dut._log.info(f"Delay TX->RX: {best_offset} (corr {best_corr}/{search_len})")

        ref_aI = ref_I[best_offset:]
        ref_aQ = ref_Q[best_offset:]
        rx_I   = np.array(rx_bits_I)
        rx_Q   = np.array(rx_bits_Q)
        N      = min(len(rx_I), len(ref_aI))
        mitad  = N // 2
        e = (np.sum(rx_I[mitad:N] != ref_aI[mitad:N]) +
             np.sum(rx_Q[mitad:N] != ref_aQ[mitad:N]))
        ber_final = e / (2 * (N - mitad))

        dut._log.info(f"BER estacionaria (sigma={sigma}): {ber_final:.6f}")

        results_file = os.path.join(sim_dir, "snr_sweep_results.txt")
        with open(results_file, "a") as f:
            f.write(f"{sigma},{ber_final:.8f}\n")
        dut._log.info(f"Resultado guardado en: {results_file}")

    elif MODO_BER:
        dut._log.error("Muy pocos simbolos — verificar sl_out_valid y loop cerrado")

    # ---- Constelacion ----
    if MODO_CONSTELACION and len(const_I) > 0:
        fig, ax = plt.subplots(figsize=(6, 6))
        N_plot = min(3000, len(const_I))
        # primeros simbolos (antes de convergencia)
        N_ini = min(500, len(const_I) // 4)
        ax.scatter(const_I[:N_ini], const_Q[:N_ini],
                   alpha=0.3, s=2, color='red', label='Inicio (no convergido)')
        # ultimos simbolos (convergido)
        ax.scatter(const_I[-N_plot:], const_Q[-N_plot:],
                   alpha=0.15, s=1, color='steelblue', label='Estacionario')
        ax.axhline(0, color='k', linewidth=0.5)
        ax.axvline(0, color='k', linewidth=0.5)
        ax.set_title(f'Constelacion salida ecualizador\n'
                     f'sigma={sigma}  |  {N_plot} simbolos estacionarios')
        ax.set_xlabel('I')
        ax.set_ylabel('Q')
        ax.set_xlim(-200, 200)
        ax.set_ylim(-200, 200)
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
        plt.tight_layout()
        path = os.path.join(sim_dir, f'constelacion_sigma{sigma}.png')
        plt.savefig(path, dpi=150)
        plt.close()
        dut._log.info(f"Constelacion guardada: {path}")

    # ---- Coeficientes ----
    if MODO_COEFICIENTES and len(w_history[0]) > 0:
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))

        # Todos los taps
        for k in range(16):
            if len(w_history[k]) > 0:
                ax1.plot(w_history[k], label=f'w[{k}]', alpha=0.7)
        ax1.set_title(f'Convergencia taps LMS w[0..15]  |  sigma={sigma}')
        ax1.set_xlabel('Frame')
        ax1.set_ylabel('w[k]')
        ax1.legend(ncol=4, fontsize=7)
        ax1.axhline(0, color='k', linestyle=':', linewidth=0.8)
        ax1.grid(True)

        # Solo w[0] y w[1] con mas detalle
        ax2.plot(w_history[0], label='w[0] (tap principal)', linewidth=2)
        ax2.plot(w_history[1], label='w[1]', linewidth=1.5)
        ax2.plot(w_history[2], label='w[2]', linewidth=1.5)
        ax2.set_title('Detalle taps principales')
        ax2.set_xlabel('Frame')
        ax2.set_ylabel('w[k]')
        ax2.legend()
        ax2.axhline(0, color='k', linestyle=':', linewidth=0.8)
        ax2.grid(True)

        plt.tight_layout()
        path = os.path.join(sim_dir, f'coeficientes_sigma{sigma}.png')
        plt.savefig(path, dpi=150)
        plt.close()
        dut._log.info(f"Coeficientes guardados: {path}")
