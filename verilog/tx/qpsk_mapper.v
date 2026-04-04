`timescale 1ns/1ps

module qpsk_mapper (
    // Entrada: Bits crudos del PRBS
    input  wire bit_I,
    input  wire bit_Q,
    
    // Salida: Símbolos QPSK en 9 bits signed
    // Rango: -256 a +255
    // +90 (Decimal) -> 9'h05A
    // -90 (Decimal) -> 9'h1A6 (Complemento a 2)
    output wire signed [8:0] sym_I, // <--- CAMBIO A 9 BITS [8:0]
    output wire signed [8:0] sym_Q  // <--- CAMBIO A 9 BITS [8:0]
);

    // Definición de la amplitud
    // Valor 90 decimal en 9 bits con signo
    localparam signed [8:0] QPSK_VAL_POS = 9'sd90; // <--- CAMBIO A 9'sd...
    
    // Valor -90 decimal en 9 bits con signo
    localparam signed [8:0] QPSK_VAL_NEG = -9'sd90; // <--- CAMBIO A -9'sd...

    // Lógica de Mapeo:
    assign sym_I = (bit_I == 1'b0) ? QPSK_VAL_POS : QPSK_VAL_NEG;
    assign sym_Q = (bit_Q == 1'b0) ? QPSK_VAL_POS : QPSK_VAL_NEG;

endmodule