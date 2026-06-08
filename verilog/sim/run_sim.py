import os
from pathlib import Path
from cocotb_tools.runner import get_runner
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from math import erfc as _erfc

# ============================================================
# CONFIGURACION CENTRAL — solo tocar esto
# ============================================================
MODO_BER          = False  # True = sweep BER | False = sim rápida
MODO_CONSTELACION = True
MODO_COEFICIENTES = True

SIGMA             = 3      # sigma_scale (0 = sin ruido)
MU_SH_INIT        = 8     # mu arranque  = 1/2^7
MU_SH_FINAL       = 12      # mu estable   = 1/2^9
N_SWITCH          = 500    # frames hasta switchear mu
SIM_TIME_US       = 5000   # tiempo simulado en microsegundos
# ============================================================

def Q(x):
    return 0.5 * np.vectorize(_erfc)(x / np.sqrt(2))

def build_once():
    sim          = "icarus"
    hdl_toplevel = "top_global"
    curr_dir     = Path(__file__).resolve().parent
    rtl_dir      = curr_dir.parent / "rtl"

    sources = [
        rtl_dir / "top_global.v",
        rtl_dir / "tx/prbs9.v",
        rtl_dir / "tx/qpsk_mapper.v",
        rtl_dir / "tx/top_tx.v",
        rtl_dir / "ch/top_ch.v",
        rtl_dir / "ch/filtro_fir.v",
        rtl_dir / "ch/gng.v",
        rtl_dir / "ch/gng_coef.v",
        rtl_dir / "ch/gng_ctg.v",
        rtl_dir / "ch/gng_interp.v",
        rtl_dir / "ch/gng_lzd.v",
        rtl_dir / "ch/gng_smul_16_18.v",
        rtl_dir / "ch/gng_smul_16_18_sadd_37.v",
        rtl_dir / "rx/butterfly.v",
        rtl_dir / "rx/complex_mult.v",
        rtl_dir / "rx/discard_n.v",
        rtl_dir / "rx/fft_ifft_stream.v",
        rtl_dir / "rx/fft_stage_dit.v",
        rtl_dir / "rx/fifo.v",
        rtl_dir / "rx/gradiente.v",
        rtl_dir / "rx/history_buffer.v",
        rtl_dir / "rx/os_buffer.v",
        rtl_dir / "rx/keep_first_n.v",
        rtl_dir / "rx/sat_trunc.v",
        rtl_dir / "rx/slicer_qpsk.v",
        rtl_dir / "rx/tabla_w.v",
        rtl_dir / "rx/twiddle_rom.v",
        rtl_dir / "rx/update_lms.v",
        rtl_dir / "rx/cmul_pbfdaf.v",
        rtl_dir / "rx/xhist_delay.v",
        rtl_dir / "rx/zero_pad_error.v",
        rtl_dir / "rx/zero_pad_pesos.v",
    ]

    runner = get_runner(sim)
    runner.build(
        sources=sources,
        hdl_toplevel=hdl_toplevel,
        includes=[rtl_dir, rtl_dir/"tx", rtl_dir/"ch", rtl_dir/"rx"],
        always=True
    )
    return runner, hdl_toplevel

def run():
    sim_dir              = Path(__file__).resolve().parent
    runner, hdl_toplevel = build_once()

    env = {
        "SIGMA_SCALE"     : str(SIGMA),
        "MU_SH_INIT"      : str(MU_SH_INIT),
        "MU_SH_FINAL"     : str(MU_SH_FINAL),
        "N_SWITCH"        : str(N_SWITCH),
        "SIM_TIME_US"     : str(SIM_TIME_US),
        "MODO_BER"        : str(int(MODO_BER)),
        "MODO_CONSTELACION": str(int(MODO_CONSTELACION)),
        "MODO_COEFICIENTES": str(int(MODO_COEFICIENTES)),
    }

    if MODO_BER:
        results_file = sim_dir / "snr_sweep_results.txt"
        if results_file.exists():
            results_file.unlink()

        sigma_values = [3, 4, 6, 8, 12, 16, 24]
        print(f"\n=== SWEEP BER: sigma={sigma_values} ===\n")

        for sigma in sigma_values:
            print(f"\n--- sigma_scale={sigma} ---")
            env["SIGMA_SCALE"] = str(sigma)
            runner.test(
                hdl_toplevel=hdl_toplevel,
                test_module="test_top",
                extra_env=env
            )

        if not results_file.exists():
            print("ERROR: no se genero snr_sweep_results.txt")
            return

        data    = np.loadtxt(results_file, delimiter=",")
        if data.ndim == 1:
            data = data.reshape(1, -1)
        sigmas  = data[:, 0]
        bers    = data[:, 1]

        mask    = sigmas > 0
        ebno_db = 10 * np.log10(8100 / (8 * sigmas[mask]**2))
        ebno_teo = np.linspace(0, 20, 300)
        ber_teo  = Q(np.sqrt(2 * 10**(ebno_teo / 10)))

        fig, ax = plt.subplots(figsize=(10, 6))
        ax.semilogy(ebno_teo, ber_teo, color='steelblue', linewidth=2,
                    label='QPSK teórica (Es/N0)')
        ax.semilogy(ebno_db, bers[mask], 'o-', color='darkorange',
                    linewidth=2, markersize=7, label='Simulación RTL')
        ax.set_title('QPSK: BER teórica vs simulada (RTL)\nPBFDAF-LMS  |  N=16')
        ax.set_xlabel('Es/N0 [dB]')
        ax.set_ylabel('BER')
        ax.set_xlim(0, 20)
        ax.set_ylim(1e-5, 1)
        ax.legend(fontsize=10)
        ax.grid(True, which='both', linestyle='--', alpha=0.6)
        plt.tight_layout()
        out = sim_dir / "ber_vs_snr.png"
        plt.savefig(out, dpi=150)
        print(f"\nGráfico BER guardado: {out}")

    else:
        print(f"\n=== MODO RAPIDO: sigma={SIGMA} | mu_init=1/2^{MU_SH_INIT} | sim={SIM_TIME_US} us ===\n")
        runner.test(
            hdl_toplevel=hdl_toplevel,
            test_module="test_top",
            extra_env=env
        )
        print(f"\nListo. PNGs en: {sim_dir}")

if __name__ == "__main__":
    run()