import os
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

# ============================================================
# Parámetros del sistema
# ============================================================
NBF_INT = 10
N       = 16

# ============================================================
# PRBS-9
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

# ============================================================
# Buffers globales
# ============================================================
rx_bits_I = []
rx_bits_Q = []
w_history = [[] for _ in range(N)]
const_I   = []
const_Q   = []
# Monitor de alineamiento X vs E
align_xhd  = []   # ciclos donde arranca el frame de X (xhist_delay)
align_ffte = []   # ciclos donde arranca el frame de E (fft_error)
align_ovf  = [0]  # se pone en 1 si el os_buffer desborda
mu_zero  = [0]   # veces que mu_g dio 0
mu_total = [0]   # veces totales que el LMS estuvo activo
grad_busy = [0]
old_ok    = [0]; old_bad = [0]; curr_hist = []

# ============================================================
# Corrutinas
# ============================================================
async def monitor_alineamiento(dut):
    cyc = 0
    while True:
        await RisingEdge(dut.clk_fast)
        try:
            if int(dut.u_xhd.o_start.value) == 1:        # start del camino X
                align_xhd.append(cyc)
            if int(dut.u_fft_error.o_start.value) == 1:   # start del camino E
                align_ffte.append(cyc)
            if int(dut.u_os.o_overflow.value) == 1:       # se cayo un frame
                align_ovf[0] = 1
        except Exception:
            pass
        cyc += 1
async def captura_slicer(dut):
    while True:
        await RisingEdge(dut.clk_fast)
        try:
            if int(dut.sl_out_valid.value) == 1:
                yI = int(dut.sl_out_yhat_I.value.to_signed())
                yQ = int(dut.sl_out_yhat_Q.value.to_signed())
                rx_bits_I.append(0 if yI > 0 else 1)
                rx_bits_Q.append(0 if yQ > 0 else 1)
        except Exception:
            pass

async def captura_constelacion(dut):
    while True:
        await RisingEdge(dut.clk_fast)
        try:
            if int(dut.dn_out_valid.value) == 1:
                yI = int(dut.dn_out_I.value.to_signed())
                yQ = int(dut.dn_out_Q.value.to_signed())
                const_I.append(yI)
                const_Q.append(yQ)
        except Exception:
            pass

async def captura_pesos(dut):
    while True:
        await RisingEdge(dut.clk_fast)
        try:
            if (int(dut.u_lms.o_valid.value) == 1 and
                    int(dut.u_lms.o_start.value) == 1):
                for k in range(N):
                    w = int(dut.u_lms.w_re[k].value.to_signed())
                    w_history[k].append(w / (2**NBF_INT))
        except Exception:
            pass

async def heartbeat(dut):
    t = 0
    while True:
        await Timer(500, unit="us")
        t += 500
        dut._log.info(
            f"[HB] {t} us | "
            f"rx={len(rx_bits_I)} sym | "
            f"frames={len(w_history[0])} | "
            f"const={len(const_I)}"
        )

async def cuenta_mu(dut):
    while True:
        await RisingEdge(dut.clk_fast)
        try:
            if int(dut.u_lms.i_valid.value) == 1:
                mu_total[0] += 1
                if int(dut.u_lms.mu_gI.value.to_signed()) == 0:
                    mu_zero[0] += 1
        except Exception:
            pass

w1_moved = [False]
async def chequea_w1(dut):
    while True:
        await RisingEdge(dut.clk_fast)
        try:
            for k in range(8):  # miro algunos bins
                if int(dut.u_cmul.W1_re[k].value.to_signed()) != 0:
                    w1_moved[0] = True
                    break
        except Exception:
            pass
async def mide_para_w1(dut):
    cyc = 0
    while True:
        await RisingEdge(dut.clk_fast)
        try:
            # ocupacion de la cadena del gradiente
            if int(dut.u_grad.o_valid.value) == 1:
                grad_busy[0] += 1
            # chequeo hb_out_old == frame anterior
            if int(dut.u_hb.o_valid.value) == 1 and int(dut.u_hb.o_start.value) == 1:
                curr = int(dut.u_hb.o_X_curr_re.value.to_signed())
                old  = int(dut.u_hb.o_X_old_re.value.to_signed())
                curr_hist.append(curr)
                if len(curr_hist) >= 2:
                    if old == curr_hist[-2]: old_ok[0] += 1
                    else:                    old_bad[0] += 1
        except Exception:
            pass
        cyc += 1
        if cyc == 50000:  # mido una ventana
            ocup = 100 * grad_busy[0] / 50000
            dut._log.info(f"OCUPACION cadena gradiente: {ocup:.0f}% (libre={100-ocup:.0f}%)")
            dut._log.info(f"hb_old == curr[n-1]: ok={old_ok[0]} bad={old_bad[0]}")
# ============================================================
# TEST PRINCIPAL
# ============================================================
@cocotb.test()
async def ber_convergencia(dut):
    # ── Leer parámetros desde env ────────────────────────────
    sigma        = int(os.environ.get("SIGMA_SCALE",      "0"))
    mu_sh_init   = int(os.environ.get("MU_SH_INIT",       "7"))
    mu_sh_final  = int(os.environ.get("MU_SH_FINAL",      "9"))
    n_switch     = int(os.environ.get("N_SWITCH",         "200"))
    sim_time_us  = int(os.environ.get("SIM_TIME_US",      "200"))
    modo_ber     = bool(int(os.environ.get("MODO_BER",           "0")))
    modo_const   = bool(int(os.environ.get("MODO_CONSTELACION",  "1")))
    modo_coef    = bool(int(os.environ.get("MODO_COEFICIENTES",  "1")))

    sim_dir = os.path.dirname(os.path.abspath(__file__))

    dut._log.info(
        f"=== sigma={sigma} | mu_init=1/2^{mu_sh_init} | "
        f"mu_final=1/2^{mu_sh_final} | n_switch={n_switch} | "
        f"sim={sim_time_us} us ==="
    )

    # ── Clocks ──────────────────────────────────────────────
    cocotb.start_soon(Clock(dut.clk_fast, 10, unit="ns").start())  # 100 MHz
    cocotb.start_soon(Clock(dut.clk_low,  20, unit="ns").start())  # 50 MHz

    # ── Reset ────────────────────────────────────────────────
    dut.rst.value         = 1
    cocotb.start_soon(monitor_alineamiento(dut))
    cocotb.start_soon(cuenta_mu(dut))
    dut.sigma_scale.value = sigma
    dut.mu_sh_init.value  = mu_sh_init
    dut.mu_sh_final.value = mu_sh_final
    dut.n_switch.value    = n_switch
    await Timer(200, unit="ns")
    dut.rst.value = 0

    # ── Corrutinas ───────────────────────────────────────────
    if modo_ber:
        cocotb.start_soon(captura_slicer(dut))
    if modo_const:
        cocotb.start_soon(captura_constelacion(dut))
    if modo_coef:
        cocotb.start_soon(captura_pesos(dut))
    cocotb.start_soon(heartbeat(dut))
    cocotb.start_soon(chequea_w1(dut))
    await Timer(sim_time_us, unit="us")
    # ── Chequeo de alineamiento X vs E ───────────────────────
    dut._log.info("=" * 50)
    dut._log.info("ALINEAMIENTO X vs E (gradiente)")
    dut._log.info(f"  starts X (xhd):  {len(align_xhd)}")
    dut._log.info(f"  starts E (ffte): {len(align_ffte)}")
    dut._log.info(f"  os_overflow:     {align_ovf[0]}  (0=OK, 1=se cayo un frame)")
    dut._log.info(f"W1 se movio de cero: {w1_moved[0]}")

    if len(align_xhd) > 5 and len(align_ffte) > 5:
        # para cada arranque de E, busco el arranque de X mas cercano
        deltas = []
        for cE in align_ffte[:30]:
            cX = min(align_xhd, key=lambda c: abs(c - cE))
            deltas.append(cE - cX)   # >0 => E llega despues => falta delay en X
        dut._log.info(f"  deltas (E - X) primeros frames: {deltas}")

        if all(d == 0 for d in deltas):
            dut._log.info("  -> ALINEADO. DELAY=118 esta perfecto.")
        elif len(set(deltas)) == 1:
            d = deltas[0]
            dut._log.info(f"  -> CORRIDO {d} ciclos constante. Pone DELAY = 118 + ({d}) = {118 + d}")
        else:
            dut._log.info("  -> DELTA VARIABLE (jitter). El delay fijo no alcanza.")
    else:
        dut._log.info("  -> Pocos starts capturados, revisar nombres de instancia.")

    dut._log.info(f"Símbolos RX:  {len(rx_bits_I)}")
    dut._log.info(f"Frames LMS:   {len(w_history[0])}")
    dut._log.info(f"Muestras con: {len(const_I)}")

    pct = 100 * mu_zero[0] / max(mu_total[0], 1)
    dut._log.info("=" * 50)
    dut._log.info("TRUNCADO DEL GRADIENTE (mu_g)")
    dut._log.info(f"  mu_g == 0 en {mu_zero[0]}/{mu_total[0]} ciclos = {pct:.0f}%")
    if pct > 50:
        dut._log.info("  -> El gradiente se muere en el shift. Bajar MU_SH o subir resolucion del gradiente.")
    else:
        dut._log.info("  -> El gradiente sobrevive la mayoria de los ciclos. El problema es otro.")
    # ── BER ──────────────────────────────────────────────────
    if modo_ber and len(rx_bits_I) >= 500:
        REF_LEN = len(rx_bits_I) + 2000
        ref_I   = np.array(prbs9(0x17F, REF_LEN))
        ref_Q   = np.array(prbs9(0x11D, REF_LEN))
        rx_arr  = np.array(rx_bits_I)

        best_offset, best_corr = 0, -1
        search_len = min(300, len(rx_arr))
        for offset in range(2000):
            if offset + search_len > len(ref_I):
                break
            c = np.sum(rx_arr[:search_len] == ref_I[offset:offset+search_len])
            if c > best_corr:
                best_corr, best_offset = c, offset

        confianza = best_corr / search_len
        dut._log.info(f"Offset TX→RX: {best_offset} | confianza: {confianza:.1%}")
        if confianza < 0.80:
            dut._log.error(f"Offset poco confiable ({confianza:.1%})")

        rx_I  = np.array(rx_bits_I)
        rx_Q  = np.array(rx_bits_Q)
        ref_aI = ref_I[best_offset:]
        ref_aQ = ref_Q[best_offset:]
        N_cmp  = min(len(rx_I), len(ref_aI))
        mitad  = N_cmp // 2

        errores  = (np.sum(rx_I[mitad:N_cmp] != ref_aI[mitad:N_cmp]) +
                    np.sum(rx_Q[mitad:N_cmp] != ref_aQ[mitad:N_cmp]))
        ber_final = errores / (2 * (N_cmp - mitad))
        dut._log.info(f"BER estacionaria (sigma={sigma}): {ber_final:.6f}")

        results_file = os.path.join(sim_dir, "snr_sweep_results.txt")
        with open(results_file, "a") as f:
            f.write(f"{sigma},{ber_final:.8f}\n")

    elif modo_ber:
        dut._log.error("Muy pocos símbolos — verificar sl_out_valid")

    # ── Constelación — estilo Python ─────────────────────────
    # ── Constelación y_n (salida del ecualizador) ────────────
    if modo_const and len(const_I) > 0:
        NBF_DN  = 7          # dn_out es Q9.7  -> valor real = raw / 2^7
        QPSK_A  = 90 / 128   # simbolo QPSK ideal del slicer = 0.703

        # Pasar a valor real en punto fijo (NO normalizar por el maximo)
        cI = [v / (2**NBF_DN) for v in const_I]
        cQ = [v / (2**NBF_DN) for v in const_Q]

        fig, axes = plt.subplots(1, 2, figsize=(12, 5))

        # Puntos QPSK ideales para referencia
        ideal_x = [ QPSK_A,  QPSK_A, -QPSK_A, -QPSK_A]
        ideal_y = [ QPSK_A, -QPSK_A,  QPSK_A, -QPSK_A]

        # --- Antes de convergencia (primeras muestras) ---
        N_ini = min(2000, len(cI) // 4)
        axes[0].scatter(cI[:N_ini], cQ[:N_ini], alpha=0.4, s=5, color='steelblue')
        axes[0].scatter(ideal_x, ideal_y, color='red', marker='x', s=120,
                        linewidths=2, zorder=5, label='QPSK ideal')
        axes[0].set_title('Constelacion y_n\n(antes de convergencia)')
        axes[0].legend(loc='upper right', fontsize=8)

        # --- Estacionario (ultimas muestras) ---
        N_fin = min(2000, len(cI))
        axes[1].scatter(cI[-N_fin:], cQ[-N_fin:], alpha=0.3, s=5, color='steelblue')
        axes[1].scatter(ideal_x, ideal_y, color='red', marker='x', s=120,
                        linewidths=2, zorder=5, label='QPSK ideal')
        axes[1].set_title('Constelacion y_n\n(estacionario)')
        axes[1].legend(loc='upper right', fontsize=8)

        for ax in axes:
            ax.set_xlim(-1.2, 1.2); ax.set_ylim(-1.2, 1.2)
            ax.axhline(0, color='k', lw=0.8)
            ax.axvline(0, color='k', lw=0.8)
            ax.set_xlabel('Re'); ax.set_ylabel('Im')
            ax.set_aspect('equal')
            ax.grid(True, alpha=0.3)

        fig.suptitle(f'Ecualización PBFDAF-LMS — sigma={sigma}', fontsize=12)
        plt.tight_layout()
        path = os.path.join(sim_dir, f'constelacion_sigma{sigma}.png')
        plt.savefig(path, dpi=150)
        plt.close()
        dut._log.info(f"Constelación guardada: {path}")

    # ── Coeficientes — estilo Python ─────────────────────────
    if modo_coef and len(w_history[0]) > 0:
        n_frames = len(w_history[0])
        fig, ax = plt.subplots(figsize=(13, 5))

        for k in range(N):
            if len(w_history[k]) > 0:
                ax.plot(w_history[k], linewidth=0.8, alpha=0.8,
                        label=f'Re w[{k}]')

        ax.set_title(f'Evolución de coeficientes del FFE\nsigma={sigma} | {n_frames} bloques')
        ax.set_xlabel('bloque')
        ax.set_ylabel('valor del tap')
        ax.axhline(0, color='k', lw=0.6, ls=':')
        ax.legend(ncol=4, fontsize=7, bbox_to_anchor=(1.01, 1), loc='upper left')
        ax.grid(True, alpha=0.3)
        plt.tight_layout()
        path = os.path.join(sim_dir, f'coeficientes_sigma{sigma}.png')
        plt.savefig(path, dpi=150, bbox_inches='tight')
        plt.close()
        dut._log.info(f"Coeficientes guardados: {path}")

        dut._log.info("Taps finales:")
        for k in range(N):
            if len(w_history[k]) > 0:
                dut._log.info(f"  w[{k:2d}] = {w_history[k][-1]:+.4f}")

# ── Convolución canal ⊗ ecualizador (test de impulso) ────
    if modo_coef and len(w_history[0]) > 0:
        # Canal H = [45, 110, 45] en Q(9,7) -> valor real = raw / 2^7
        H = np.array([45, 110, 45]) / (2**7)

        # Taps finales del ecualizador (ya vienen en valor real)
        w_final = np.array([w_history[k][-1] if len(w_history[k]) > 0 else 0.0
                            for k in range(N)])

        # Convolución completa: respuesta combinada canal + FFE
        c = np.convolve(H, w_final)

        # Normalizar al pico para ver el impulso claro
        pico_idx = int(np.argmax(np.abs(c)))
        c_norm   = c / c[pico_idx]

        # Métricas de calidad
        pico   = abs(c[pico_idx])
        resto  = np.sum(np.abs(c)) - pico
        isi_db = 20*np.log10(resto / pico) if pico > 0 else 0.0

        dut._log.info("=" * 50)
        dut._log.info("CONVOLUCION CANAL ⊗ ECUALIZADOR (test impulso)")
        dut._log.info(f"  pico en posicion: {pico_idx}  (valor {c[pico_idx]:+.4f})")
        dut._log.info(f"  suma resto (ISI residual): {resto:.4f}")
        dut._log.info(f"  ISI / pico: {isi_db:.1f} dB  (mas negativo = mejor)")
        dut._log.info("  respuesta combinada (normalizada al pico):")
        for i, v in enumerate(c_norm):
            barra = "#" * int(abs(v) * 40)
            dut._log.info(f"    n={i:2d}: {v:+.4f} {barra}")

        # Gráfico stem
        fig, ax = plt.subplots(figsize=(10, 5))
        ax.stem(range(len(c_norm)), c_norm, basefmt=" ")
        ax.axhline(0, color='k', lw=0.6)
        ax.set_title(f'Respuesta combinada canal ⊗ ecualizador\n'
                     f'sigma={sigma} | ISI={isi_db:.1f} dB '
                     f'(ideal = un solo pico en 1.0)')
        ax.set_xlabel('muestra (n)')
        ax.set_ylabel('amplitud (normalizada al pico)')
        ax.grid(True, alpha=0.3)
        plt.tight_layout()
        path = os.path.join(sim_dir, f'impulso_sigma{sigma}.png')
        plt.savefig(path, dpi=150)
        plt.close()
        dut._log.info(f"Impulso guardado: {path}")
# ── Taps optimos teoricos (que DEBERIAN salir) ───────
        H_full = np.array([45, 110, 45]) / (2**7)
        # Inverso por minimos cuadrados: w que minimiza ||H*w - delta||
        L = N
        # matriz de convolucion de H (Toeplitz) de tamano (L+len(H)-1) x L
        conv_len = L + len(H_full) - 1
        Hmat = np.zeros((conv_len, L))
        for col in range(L):
            Hmat[col:col+len(H_full), col] = H_full
        # probamos cada posicion de delta como objetivo y elegimos la mejor
        best_err = 1e9; best_w = None; best_d = 0
        for d in range(conv_len):
            target = np.zeros(conv_len); target[d] = 1.0
            w_opt, _, _, _ = np.linalg.lstsq(Hmat, target, rcond=None)
            err = np.sum((Hmat @ w_opt - target)**2)
            if err < best_err:
                best_err, best_w, best_d = err, w_opt, d

        dut._log.info("=" * 50)
        dut._log.info("COMPARACION: taps que diste vs optimo teorico")
        dut._log.info(f"  (delta optima en posicion {best_d})")
        dut._log.info(f"  {'k':>3} {'tu w[k]':>10} {'w optimo':>10} {'dif':>10}")
        for k in range(N):
            diff = w_final[k] - best_w[k]
            dut._log.info(f"  {k:>3} {w_final[k]:>+10.4f} {best_w[k]:>+10.4f} {diff:>+10.4f}")
        # cuanto se parecen (normalizado)
        cos_sim = np.dot(w_final, best_w) / (np.linalg.norm(w_final)*np.linalg.norm(best_w) + 1e-12)
        dut._log.info(f"  similitud (coseno) tu_w vs optimo: {cos_sim:+.3f}  (1.0 = identicos)")