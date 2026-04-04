`timescale 1ns/1ps
`default_nettype none

// ============================================================
// zero_pad_pesos.v
//
// Prepara los pesos temporales para la FFT de coeficientes
// añadiendo N ceros al final (overlap-save constraint):
//
//   Entrada  (N=16 muestras):    w[0],  w[1], ..., w[N-1]
//   Salida  (NFFT=32 muestras):  w[0], ..., w[N-1], 0, ..., 0
//                                 ←── N ───→  ←──── N ────→
//
// FSM de 3 estados:
//
//   S_IDLE  — espera i_valid && i_start
//             En la transición: registra w[0], emite o_start=1, va a S_PESOS
//             cnt arranca en 1 (w[0] ya se procesó)
//
//   S_PESOS — emite w[1..N-1] sample a sample (solo avanza con i_valid)
//             Cuando cnt==N-1: va a S_ZEROS con cnt=0
//
//   S_ZEROS — emite N ceros de forma autónoma (no necesita i_valid)
//             Cuando cnt==N-1: vuelve a S_IDLE
//
// Latencia: 1 ciclo (registro de salida, consistente con la cadena).
//
// Secuencia de salida por frame:
//   ciclo 0         : o_start=1, o_valid=1, o_wI=w[0]
//   ciclos 1..N-1   : o_start=0, o_valid=1, o_wI=w[k]
//   ciclos N..2N-1  : o_start=0, o_valid=1, o_wI=0
//   Total: 2N = NFFT muestras válidas con 1 start
//
// Si i_valid se interrumpe durante S_PESOS, el módulo espera
// (o_valid=0) hasta que i_valid vuelva.
//
// Parámetros:
//   NB_W  = 17   (Q17.10)
//   NBF_W = 10   (documentación)
//   NFFT  = 32   (N = 16)
// ============================================================

module zero_pad_pesos #(
    parameter integer NB_W  = 17,
    parameter integer NBF_W = 10,
    parameter integer NFFT  = 32
)(
    input  wire                    clk,
    input  wire                    rst,

    // Entrada: w_new de update_lms (N muestras/frame)
    input  wire                    i_valid,
    input  wire                    i_start,   // 1 en k=0
    input  wire signed [NB_W-1:0]  i_wI,
    input  wire signed [NB_W-1:0]  i_wQ,

    // Salida: [w|0..0] para fft_pesos (NFFT muestras/frame)
    output reg                     o_valid,
    output reg                     o_start,   // 1 en primera muestra
    output reg  signed [NB_W-1:0]  o_wI,
    output reg  signed [NB_W-1:0]  o_wQ
);

    localparam integer N   = NFFT / 2;          // 16
    localparam integer KW  = $clog2(NFFT);      // 5 bits para 0..31
    localparam [KW-1:0] N1 = N - 1;             // 15

    localparam [1:0] S_IDLE  = 2'd0;
    localparam [1:0] S_PESOS = 2'd1;
    localparam [1:0] S_ZEROS = 2'd2;

    reg [1:0]    state;
    reg [KW-1:0] cnt;

    always @(posedge clk) begin
        if (rst) begin
            state   <= S_IDLE;
            cnt     <= {KW{1'b0}};
            o_valid <= 1'b0;
            o_start <= 1'b0;
            o_wI    <= {NB_W{1'b0}};
            o_wQ    <= {NB_W{1'b0}};
        end else begin
            case (state)

                // ================================================
                // IDLE — esperando primer dato del frame
                // ================================================
                S_IDLE: begin
                    o_valid <= 1'b0;
                    o_start <= 1'b0;
                    o_wI    <= {NB_W{1'b0}};
                    o_wQ    <= {NB_W{1'b0}};
                    if (i_valid && i_start) begin
                        // Registrar w[0] y emitir start
                        o_valid <= 1'b1;
                        o_start <= 1'b1;
                        o_wI    <= i_wI;
                        o_wQ    <= i_wQ;
                        // Pasar a PESOS, ya procesamos k=0
                        // → siguiente ciclo necesita k=1
                        state <= S_PESOS;
                        cnt   <= {{(KW-1){1'b0}}, 1'b1};  // cnt=1
                    end
                end

                // ================================================
                // PESOS — emitir w[1..N-1] (esperando i_valid)
                // ================================================
                S_PESOS: begin
                    o_start <= 1'b0;
                    if (i_valid) begin
                        o_valid <= 1'b1;
                        o_wI    <= i_wI;
                        o_wQ    <= i_wQ;
                        if (cnt == N1) begin
                            // Último peso (k=N-1) → ir a ZEROS
                            state <= S_ZEROS;
                            cnt   <= {KW{1'b0}};
                        end else begin
                            cnt <= cnt + 1'b1;
                        end
                    end else begin
                        // Esperando → no emitir
                        o_valid <= 1'b0;
                        o_wI    <= {NB_W{1'b0}};
                        o_wQ    <= {NB_W{1'b0}};
                    end
                end

                // ================================================
                // ZEROS — emitir N ceros de forma autónoma
                // ================================================
                S_ZEROS: begin
                    o_valid <= 1'b1;
                    o_start <= 1'b0;
                    o_wI    <= {NB_W{1'b0}};
                    o_wQ    <= {NB_W{1'b0}};
                    if (cnt == N1) begin
                        // Último cero → volver a IDLE
                        state <= S_IDLE;
                        cnt   <= {KW{1'b0}};
                    end else begin
                        cnt <= cnt + 1'b1;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire
