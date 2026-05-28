import os
from pathlib import Path
from cocotb_tools.runner import get_runner
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from math import erfc as _erfc

# ============================================================
# MODO DE OPERACION — cambiá esto según lo que necesites
# ============================================================
MODO_BER = True   # True = sweep completo (~90 min) | False = una sim rapida
SIGMA_FIJO = 4     # sigma usado cuando MODO_BER=False
# ============================================================

def Q(x):
    return 0.5 * np.vectorize(_erfc)(x / np.sqrt(2))

def build_once():
    sim = "icarus"
    hdl_toplevel = "top_global_all"
    curr_dir = Path(__file__).resolve().parent
    rtl_dir  = curr_dir.parent / "rtl"

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
        rtl_dir / "rx/clock_div.v",
        rtl_dir / "rx/cmul_stream.v",
        rtl_dir / "rx/complex_mult.v",
        rtl_dir / "rx/discard_n.v",
        rtl_dir / "rx/fft_ifft_stream.v",
        rtl_dir / "rx/fft_stage_dit.v",
        rtl_dir / "rx/fifo.v",
        rtl_dir / "rx/gradiente.v",
        rtl_dir / "rx/history_buffer.v",
        rtl_dir / "rx/os_buffer.v",
        rtl_dir / "rx/Proyeccion_n.v",
        rtl_dir / "rx/sat_trunc.v",
        rtl_dir / "rx/slicer_qpsk.v",
        rtl_dir / "rx/tabla_w.v",
        rtl_dir / "rx/twiddle_rom.v",
        rtl_dir / "rx/update_lms.v",
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
    sim_dir = Path(__file__).resolve().parent
    runner, hdl_toplevel = build_once()

    if MODO_BER:
        # Sweep completo
        results_file = sim_dir / "snr_sweep_results.txt"
        if results_file.exists():
            results_file.unlink()

        #sigma_values = [0, 2, 4, 6, 8, 12, 16, 24, 32]
        sigma_values = [8]
        print(f"\n=== SWEEP BER: sigma={sigma_values} ===\n")

        for sigma in sigma_values:
            print(f"\n--- sigma_scale={sigma} ---")
            runner.test(
                hdl_toplevel=hdl_toplevel,
                test_module="test_top",
                extra_env={"SIGMA_SCALE": str(sigma)}
            )

        # Grafico BER
        if not results_file.exists():
            print("ERROR: no se genero snr_sweep_results.txt")
            return

        data = np.loadtxt(results_file, delimiter=",")
        if data.ndim == 1:
            data = data.reshape(1, -1)

        sigmas = data[:, 0]
        bers   = data[:, 1]

        print("\n=== RESULTADOS ===")
        for s, b in zip(sigmas, bers):
            print(f"  sigma={int(s):3d}  ->  Eb/N0={10*np.log10(max(8100/(8*s**2), 1e-10)):.1f} dB  BER={b:.6f}"
                  if s > 0 else f"  sigma=  0  ->  sin ruido  BER={b:.6f}")

        mask    = sigmas > 0
        sig_rtl = sigmas[mask]
        ber_rtl = bers[mask]
        ebno_db = 10 * np.log10(8100 / (8 * sig_rtl**2))

        ebno_teo = np.linspace(0, 30, 300)
        ber_teo  = Q(np.sqrt(2 * 10**(ebno_teo / 10)))

        fig, ax = plt.subplots(figsize=(10, 6))
        ax.semilogy(ebno_teo, ber_teo, 'k--', linewidth=2,
                    label='QPSK teorica (AWGN sin ISI)')
        ax.semilogy(ebno_db, ber_rtl + 1e-7, 'o-', linewidth=2,
                    markersize=8, color='steelblue',
                    label='PBFDAF-LMS RTL')

        ber_sigma0 = bers[sigmas == 0]
        if len(ber_sigma0) > 0:
            ax.axhline(ber_sigma0[0] + 1e-7, color='gray', linestyle=':',
                       label=f'BER sin ruido = {ber_sigma0[0]:.2e}')

        ax.set_title('BER vs Eb/N0 — PBFDAF-LMS RTL\nQPSK  |  N=16')
        ax.set_xlabel('Eb/N0 [dB]')
        ax.set_ylabel('BER')
        ax.set_xlim(0, 30)
        ax.set_ylim(1e-5, 1)
        ax.axhline(0.01,  color='r', linestyle=':', alpha=0.7, label='BER = 1%')
        ax.axhline(0.001, color='g', linestyle=':', alpha=0.7, label='BER = 0.1%')
        ax.legend(fontsize=9)
        ax.grid(True, which='both')
        plt.tight_layout()

        out = sim_dir / "ber_vs_snr.png"
        plt.savefig(out, dpi=150)
        print(f"\nGrafico BER guardado: {out}")

    else:
        # Una sola simulacion rapida
        print(f"\n=== MODO RAPIDO: sigma_scale={SIGMA_FIJO} ===\n")
        runner.test(
            hdl_toplevel=hdl_toplevel,
            test_module="test_top",
            extra_env={"SIGMA_SCALE": str(SIGMA_FIJO)}
        )
        print(f"\nListo. Busca los PNG en: {sim_dir}")

if __name__ == "__main__":
    run()