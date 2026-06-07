# ==============================================================================
# Makefile para Simulación PBFDAF-LMS con Cocotb
# ==============================================================================
PYTHON_BIN ?= python
export PYTHON_BIN

SIM ?= icarus
TOPLEVEL_LANG ?= verilog

# Rutas a tus subcarpetas
TX_DIR = $(PWD)/tx
CH_DIR = $(PWD)/ch
RX_DIR = $(PWD)/rx

# 1. Archivos en la carpeta raíz (RTL)
VERILOG_SOURCES += $(PWD)/top_global.v

# 2. Archivos de la carpeta TX (Transmisor)
VERILOG_SOURCES += $(TX_DIR)/prbs9.v
VERILOG_SOURCES += $(TX_DIR)/qpsk_mapper.v
VERILOG_SOURCES += $(TX_DIR)/top_tx.v

# 3. Archivos de la carpeta CH (Canal)
VERILOG_SOURCES += $(CH_DIR)/filtro_fir.v
VERILOG_SOURCES += $(CH_DIR)/gng.v
VERILOG_SOURCES += $(CH_DIR)/gng_coef.v
VERILOG_SOURCES += $(CH_DIR)/gng_ctg.v
VERILOG_SOURCES += $(CH_DIR)/gng_interp.v
VERILOG_SOURCES += $(CH_DIR)/gng_lzd.v
VERILOG_SOURCES += $(CH_DIR)/gng_smul_16_18.v
VERILOG_SOURCES += $(CH_DIR)/gng_smul_16_18_sadd_37.v
VERILOG_SOURCES += $(CH_DIR)/top_ch.v

# 4. Archivos de la carpeta RX (Receptor / Ecualizador)
VERILOG_SOURCES += $(RX_DIR)/butterfly.v
VERILOG_SOURCES += $(RX_DIR)/clock_div.v
VERILOG_SOURCES += $(RX_DIR)/cmul_stream.v
VERILOG_SOURCES += $(RX_DIR)/complex_mult.v
VERILOG_SOURCES += $(RX_DIR)/discard_n.v
VERILOG_SOURCES += $(RX_DIR)/fft_ifft_stream.v
VERILOG_SOURCES += $(RX_DIR)/fft_stage_dit.v
VERILOG_SOURCES += $(RX_DIR)/fifo.v
VERILOG_SOURCES += $(RX_DIR)/gradiente.v
VERILOG_SOURCES += $(RX_DIR)/history_buffer.v
VERILOG_SOURCES += $(RX_DIR)/os_buffer.v
VERILOG_SOURCES += $(RX_DIR)/Proyeccion_n.v
VERILOG_SOURCES += $(RX_DIR)/sat_trunc.v
VERILOG_SOURCES += $(RX_DIR)/slicer_qpsk.v
VERILOG_SOURCES += $(RX_DIR)/tabla_w.v
VERILOG_SOURCES += $(RX_DIR)/twiddle_rom.v
VERILOG_SOURCES += $(RX_DIR)/update_lms.v
VERILOG_SOURCES += $(RX_DIR)/xhist_delay.v
VERILOG_SOURCES += $(RX_DIR)/zero_pad_error.v
VERILOG_SOURCES += $(RX_DIR)/zero_pad_pesos.v

# Módulo principal y Testbench
TOPLEVEL = top_global_all
MODULE   = test_top

# Argumentos para incluir las carpetas en la búsqueda de Icarus
COMPILE_ARGS += -I$(PWD) -I$(TX_DIR) -I$(CH_DIR) -I$(RX_DIR)

include $(shell cocotb-config --makefiles)/Makefile.sim