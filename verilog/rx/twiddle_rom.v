// Twiddle ROM NFFT=32
// Q format: signed, NB_W=17, NBF_W=10  (scale = 2^10 = 1024)
// o_im_fft corresponds to FFT sign: -sin(2*pi*k/32)

module twiddle_rom #(
    parameter integer NB_W  = 17,
    parameter integer NBF_W = 10
)(
    input  wire [4:0]             i_k,       // 0..15 used
    output reg  signed [NB_W-1:0] o_re,
    output reg  signed [NB_W-1:0] o_im_fft
);

    always @(*) begin
        case (i_k)
            5'd0:  begin o_re =  17'sd1024; o_im_fft =   17'sd0;    end
            5'd1:  begin o_re =  17'sd1004; o_im_fft =  -17'sd200;  end
            5'd2:  begin o_re =  17'sd946;  o_im_fft =  -17'sd392;  end
            5'd3:  begin o_re =  17'sd851;  o_im_fft =  -17'sd569;  end
            5'd4:  begin o_re =  17'sd724;  o_im_fft =  -17'sd724;  end
            5'd5:  begin o_re =  17'sd569;  o_im_fft =  -17'sd851;  end
            5'd6:  begin o_re =  17'sd392;  o_im_fft =  -17'sd946;  end
            5'd7:  begin o_re =  17'sd200;  o_im_fft =  -17'sd1004; end
            5'd8:  begin o_re =  17'sd0;    o_im_fft =  -17'sd1024; end
            5'd9:  begin o_re = -17'sd200;  o_im_fft =  -17'sd1004; end
            5'd10: begin o_re = -17'sd392;  o_im_fft =  -17'sd946;  end
            5'd11: begin o_re = -17'sd569;  o_im_fft =  -17'sd851;  end
            5'd12: begin o_re = -17'sd724;  o_im_fft =  -17'sd724;  end
            5'd13: begin o_re = -17'sd851;  o_im_fft =  -17'sd569;  end
            5'd14: begin o_re = -17'sd946;  o_im_fft =  -17'sd392;  end
            5'd15: begin o_re = -17'sd1004; o_im_fft =  -17'sd200;  end
            default: begin o_re = 17'sd0; o_im_fft = 17'sd0; end
        endcase
    end

endmodule
