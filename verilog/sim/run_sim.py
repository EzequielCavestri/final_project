"""
run_sim.py  —  Runner principal para top_global_all

Modos:
  python run_sim.py                →  sweep BER completo (9 sigmas, lento)
  python run_sim.py --modo conv      →  debug convergencia sigma=0 (rápido, ~20s)
  python run_sim.py --modo debugfull →  debug completo del lazo LMS→CMUL→salida
  python run_sim.py --frames 200   →  convergencia con 200 frames
  python run_sim.py --sigma 8      →  un solo punto BER
  python run_sim.py --tiempo 2000  →  sweep con 2000 us por punto (más rápido)
"""

import argparse
import os
import subprocess
import sys
from pathlib import Path
from math import erfc as _erfc

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from cocotb_tools.runner import get_runner


# ── Curva teórica QPSK ──────────────────────────────────────
def Q(x):
    return 0.5 * np.vectorize(_erfc)(x / np.sqrt(2))


# ── Test module cocotb ───────────────────────────────────────
# Por defecto usa test_top.py. Si querés correr el archivo separado
# test_top_debug_full.py, seteá: $env:TEST_MODULE = "test_top_debug_full"
def test_module_name():
    return os.environ.get("TEST_MODULE", "test_top")


# ── Lista de fuentes RTL ─────────────────────────────────────
def get_sources(rtl_dir: Path):
    return [
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
        rtl_dir / "rx/cmul_stream.v",        # módulo: cmul_pbfdaf
        rtl_dir / "rx/complex_mult.v",
        rtl_dir / "rx/discard_n.v",
        rtl_dir / "rx/fft_ifft_stream.v",
        rtl_dir / "rx/fft_stage_dit.v",
        rtl_dir / "rx/fifo.v",
        rtl_dir / "rx/gradiente.v",
        rtl_dir / "rx/history_buffer.v",
        rtl_dir / "rx/os_buffer.v",
        rtl_dir / "rx/Proyeccion_n.v",       # módulo: keep_first_n
        rtl_dir / "rx/sat_trunc.v",
        rtl_dir / "rx/slicer_qpsk.v",
        rtl_dir / "rx/tabla_w.v",
        rtl_dir / "rx/twiddle_rom.v",
        rtl_dir / "rx/update_lms.v",
        rtl_dir / "rx/xhist_delay.v",
        rtl_dir / "rx/zero_pad_error.v",
        rtl_dir / "rx/zero_pad_pesos.v",
        rtl_dir / "rx/block_power.v",      # NLMS: potencia de bloque
    ]


# ── Build (con diagnóstico de errores) ───────────────────────
def build_once(rtl_dir: Path):
    sources = get_sources(rtl_dir)

    missing = [s for s in sources if not s.exists()]
    if missing:
        print("\n[ERROR] Archivos RTL no encontrados:")
        for m in missing:
            print(f"  FALTA: {m}")
        sys.exit(1)
    print(f"[OK] {len(sources)} archivos RTL encontrados.")

    try:
        r = subprocess.run(["iverilog", "-V"], capture_output=True, text=True)
        print(f"[OK] {r.stdout.splitlines()[0]}")
    except FileNotFoundError:
        print("[ERROR] iverilog no está en el PATH.")
        print("  Windows: agrega C:\\iverilog\\bin a las variables de entorno del sistema.")
        sys.exit(1)

    hdl_toplevel = "top_global_all"
    runner = get_runner("icarus")

    try:
        runner.build(
            sources=sources,
            hdl_toplevel=hdl_toplevel,
            includes=[rtl_dir, rtl_dir/"tx", rtl_dir/"ch", rtl_dir/"rx"],
            always=True
        )
    except Exception as e:
        # Mostrar el error real de iverilog
        build_dir = Path("sim_build")
        for log_name in ["iverilog.stderr", "build.log", "compile.log"]:
            lp = build_dir / log_name
            if lp.exists():
                print(f"\n── {lp} ──\n{lp.read_text(errors='replace')}")
                break
        else:
            print("\n── iverilog directo ──")
            inc = []
            for d in [rtl_dir, rtl_dir/"tx", rtl_dir/"ch", rtl_dir/"rx"]:
                inc += ["-I", str(d)]
            cmd = (["iverilog", "-g2012", "-o", "sim_build/check.vvp",
                    "-s", hdl_toplevel] + inc + [str(s) for s in sources])
            res = subprocess.run(cmd, capture_output=True, text=True)
            if res.stderr:
                print(res.stderr)
        raise

    return runner, hdl_toplevel


# ── Modo CONVERGENCIA ─────────────────────────────────────────
def run_convergencia(runner, hdl_toplevel, sim_dir: Path, n_frames: int):
    print(f"\n=== MODO CONVERGENCIA — sigma=0, {n_frames} frames ===\n")
    runner.test(
        hdl_toplevel=hdl_toplevel,
        test_module=test_module_name(),
        extra_env={
            "COCOTB_TESTCASE": "test_convergencia",
            "N_FRAMES":        str(n_frames),
        }
    )
    out = sim_dir / "convergencia_lms.png"
    if out.exists():
        print(f"\n[OK] Gráfico guardado: {out}")
    else:
        print("\n[AVISO] convergencia_lms.png no generado — revisar log")
    out = sim_dir / "convergencia_lms.png"
    if out.exists():
        print(f"\n[OK] Gráfico guardado: {out}")
    else:
        print("\n[AVISO] convergencia_lms.png no generado — revisar log")


# ── Modo BER (sweep o punto único) ──────────────────────────
def run_ber(runner, hdl_toplevel, sim_dir: Path,
            sigmas: list, tiempo_us: int):
    results_file = sim_dir / "snr_sweep_results.txt"
    if results_file.exists():
        results_file.unlink()

    print(f"\n=== BARRIDO BER — sigmas={sigmas}, {tiempo_us} us/punto ===\n")

    for sigma in sigmas:
        print(f"\n--- sigma={sigma} ---")
        runner.test(
            hdl_toplevel=hdl_toplevel,
            test_module=test_module_name(),
            extra_env={
                "COCOTB_TESTCASE": "ber_convergencia",
                "SIGMA_SCALE":     str(sigma),
                "SIM_TIME_US":     str(tiempo_us),
            }
        )

    if not results_file.exists():
        print("ERROR: snr_sweep_results.txt no generado")
        return

    data = np.loadtxt(results_file, delimiter=",")
    if data.ndim == 1:
        data = data.reshape(1, -1)

    sigmas_r = data[:, 0]
    bers     = data[:, 1]

    print("\n=== RESULTADOS BER ===")
    for s, b in zip(sigmas_r, bers):
        print(f"  sigma={int(s):3d}  BER={b:.6f}")

    # Convertir a Eb/N0 y graficar
    mask    = sigmas_r > 0
    sig_rtl = sigmas_r[mask]
    ber_rtl = bers[mask]
    if len(sig_rtl) == 0:
        return

    ebno_db  = 10 * np.log10(8100 / (4 * sig_rtl**2))
    ebno_teo = np.linspace(0, 30, 300)
    ber_teo  = Q(np.sqrt(2 * 10**(ebno_teo / 10)))

    fig, ax = plt.subplots(figsize=(10, 6))
    ax.semilogy(ebno_teo, ber_teo, 'k--', lw=2, label='QPSK teórica (AWGN sin ISI)')
    ax.semilogy(ebno_db, ber_rtl + 1e-7, 'o-', lw=2, ms=8,
                color='steelblue', label='PBFDAF-LMS RTL')

    ber0 = bers[sigmas_r == 0]
    if len(ber0) > 0:
        ax.axhline(ber0[0] + 1e-7, color='gray', ls=':',
                   label=f'BER sin ruido = {ber0[0]:.2e}')

    ax.set_title('BER vs Eb/N0 — PBFDAF-LMS RTL\nCanal ISI | QPSK | N=16')
    ax.set_xlabel('Eb/N0 [dB]')
    ax.set_ylabel('BER')
    ax.set_xlim(0, 30); ax.set_ylim(1e-5, 1)
    ax.axhline(0.01,  color='r', ls=':', alpha=0.7, label='BER=1%')
    ax.axhline(0.001, color='g', ls=':', alpha=0.7, label='BER=0.1%')
    ax.legend(fontsize=9); ax.grid(True, which='both')
    plt.tight_layout()

    out = sim_dir / "ber_vs_snr.png"
    plt.savefig(out, dpi=150)
    print(f"\n[OK] Gráfico guardado: {out}")


# ── Main ─────────────────────────────────────────────────────
def main():

    parser = argparse.ArgumentParser(
        description="Runner cocotb para top_global_all"
    )

    parser.add_argument(
        "--modo",
        choices=["tap", "debug", "debugfull", "delay", "conv", "ber", "trace"],
        default="ber",
        help="tap | debug | debugfull | delay | conv | ber | trace"
    )

    parser.add_argument(
        "--frames",
        type=int,
        default=400,
        help="Frames LMS a capturar en modo conv"
    )

    parser.add_argument(
        "--sigma",
        type=int,
        default=None,
        help="Correr un solo sigma"
    )

    parser.add_argument(
        "--tiempo",
        type=int,
        default=8000,
        help="Microsegundos por punto BER"
    )

    args = parser.parse_args()

    curr_dir = Path(__file__).resolve().parent
    rtl_dir = curr_dir.parent / "rtl"

    runner, hdl_toplevel = build_once(rtl_dir)

    sim_dir = curr_dir

    if args.modo == "debug":

        print("\n=== MODO DEBUG PIPELINE ===\n")

        runner.test(
            hdl_toplevel=hdl_toplevel,
            test_module=test_module_name(),
            extra_env={
                "COCOTB_TEST_FILTER": "test_debug_pipeline",
                "N_FRAMES": "5",
                "SIGMA_SCALE": "0",
            }
        )

    elif args.modo == "debugfull":

        print("\n=== MODO DEBUG FULL LOOP ===\n")

        runner.test(
            hdl_toplevel=hdl_toplevel,
            test_module=test_module_name(),
            extra_env={
                "COCOTB_TESTCASE": "test_debug_full_loop",
                "SIGMA_SCALE": "0",
                "N_FRAMES": str(args.frames),
                # Compatibilidad con tu forma actual de correrlo:
                # $env:SKIP_FRAMES = "150"
                "DEBUG_SKIP_LMS_FRAMES": os.environ.get("SKIP_FRAMES", os.environ.get("DEBUG_SKIP_LMS_FRAMES", "150")),
                "DEBUG_FRAMES": os.environ.get("DEBUG_FRAMES", "1"),
                "DEBUG_BINS": os.environ.get("DEBUG_BINS", "32"),
            }
        )

    elif args.modo == "tap":

        print("\n=== MODO MEDICION TAP CENTRAL ===\n")

        runner.test(
            hdl_toplevel=hdl_toplevel,
            test_module=test_module_name(),
            extra_env={
                "COCOTB_TEST_FILTER": "test_medir_tap_central",
                "SIGMA_SCALE": "0",
            }
        )

    elif args.modo == "delay":

        print("\n=== MODO MEDICION DELAY ===\n")

        runner.test(
            hdl_toplevel=hdl_toplevel,
            test_module=test_module_name(),
            extra_env={
                "COCOTB_TESTCASE": "test_medir_delay",
            }
        )

    elif args.modo == "conv":

        run_convergencia(
            runner,
            hdl_toplevel,
            sim_dir,
            args.frames
        )

    elif args.modo == "trace":

        print("\n=== TRACE LMS ALIGNMENT ===\n")

        runner.test(
            hdl_toplevel=hdl_toplevel,
            test_module=test_module_name(),
            extra_env={
                "COCOTB_TEST_FILTER": "test_trace_lms_alignment",
            }
        )

    else:

        if args.sigma is not None:
            sigmas = [args.sigma]
        else:
            sigmas = [0, 2, 4, 6, 8, 12, 16, 24, 32]

        run_ber(
            runner,
            hdl_toplevel,
            sim_dir,
            sigmas,
            args.tiempo
        )


if __name__ == "__main__":
    main()

