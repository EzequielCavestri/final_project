import os
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

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
                      f"frames={len(w_history[0])}")

@cocotb.test()
async def ber_convergencia(dut):
    sigma = int(os.environ.get("SIGMA_SCALE", "0"))
    dut._log.info(f"=== Corriendo con sigma_scale={sigma} ===")

    cocotb.start_soon(Clock(dut.clk_fast, 10, unit="ns").start())

    dut.rst.value = 1
    dut.enable_div.value = 1
    dut.sigma_scale.value = sigma
    await Timer(200, unit="ns")
    dut.rst.value = 0

    cocotb.start_soon(captura_slicer(dut))
    cocotb.start_soon(captura_pesos(dut))
    cocotb.start_soon(heartbeat(dut))

    await Timer(2000, unit="us")

    dut._log.info(f"Simbolos RX: {len(rx_bits_I)}")
    dut._log.info(f"Frames LMS:  {len(w_history[0])}")

    if len(rx_bits_I) < 200:
        dut._log.error("Muy pocos simbolos")
        return

    # Alineacion
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

    dut._log.info(f"Delay TX->RX: {best_offset} simbolos (corr {best_corr}/{search_len})")

    # BER final (ultimos 1000 frames = estado estacionario)
    ref_aI = ref_I[best_offset:]
    ref_aQ = ref_Q[best_offset:]
    rx_I   = np.array(rx_bits_I)
    rx_Q   = np.array(rx_bits_Q)
    N      = min(len(rx_I), len(ref_aI))

    # Usar solo la segunda mitad (ya convergido)
    mitad = N // 2
    e = (np.sum(rx_I[mitad:N] != ref_aI[mitad:N]) +
         np.sum(rx_Q[mitad:N] != ref_aQ[mitad:N]))
    ber_final = e / (2 * (N - mitad))

    dut._log.info(f"BER estacionaria (sigma={sigma}): {ber_final:.6f}")

    # Guardar resultado para el sweep
    sim_dir = os.path.dirname(os.path.abspath(__file__))
    results_file = os.path.join(sim_dir, "snr_sweep_results.txt")
    with open(results_file, "a") as f:
        f.write(f"{sigma},{ber_final:.8f}\n")
    dut._log.info(f"Resultado guardado en: {results_file}")