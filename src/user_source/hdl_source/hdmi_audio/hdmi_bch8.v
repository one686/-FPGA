`timescale 1ns/1ps

// HDMI 数据岛 BCH 校验字节（纯组合）：包头用 DATA_BYTES=3 即 BCH(32,24)，子包用 DATA_BYTES=7 即 BCH(64,56)。
module hdmi_bch8 #(
    parameter integer DATA_BYTES = 3
) (
    input  wire [DATA_BYTES*8-1:0] data,
    output reg  [7:0]              ecc
);
    integer byte_index;
    integer bit_index;
    reg [7:0] remainder;
    reg feedback;

    // HDMI packet BCH uses G(x)=1+x^6+x^7+x^8. Bits must be processed in
    // transmission order: data[7:0] first, LSB first within each byte
    // (hdmi_data_island_mapper sends bit index 0 first). The register is kept
    // bit-reversed (poly 0x83, right shift) so ecc[0] is the first ECC bit sent.
    // Feeding bits MSB first (poly 0xC1) gives codewords not divisible by G(x);
    // sinks then drop every audio/ACR/InfoFrame packet, so the order matters.
    always @* begin
        remainder = 8'h00;
        feedback = 1'b0;
        for (byte_index = 0; byte_index < DATA_BYTES;
             byte_index = byte_index + 1) begin
            for (bit_index = 0; bit_index < 8;
                 bit_index = bit_index + 1) begin
                feedback = data[byte_index*8 + bit_index] ^ remainder[0];
                remainder = {1'b0, remainder[7:1]};
                if (feedback)
                    remainder = remainder ^ 8'h83;
            end
        end
        ecc = remainder;
    end
endmodule
