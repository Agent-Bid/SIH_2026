`timescale 1ns/1ps
// capture.v on its own: the sample is a counter, so capture i must be base + i x 2^shift.
// Checks the header, and shifts 0, 2 and 10 (one sample in 1, 4 and 1024); then stream mode:
// armed once, it must capture two pulses in a row. Self-checking.
module capture_tb;

    localparam N_BITS = 11, N = 1 << N_BITS;

    reg                clk = 0;
    reg                rst = 1;
    reg                arm = 0, trigger = 0, stream = 0;
    reg         [3:0]  shift = 0;
    reg  signed [11:0] cnt = 0;
    wire        [7:0]  tx_data;
    wire               tx_valid, armed, recording, sending;

    capture #(.N_BITS(N_BITS), .HDR_BYTES(3)) dut (
        .clk(clk), .rst(rst), .arm(arm), .stream(stream), .trigger(trigger), .sample(cnt), .shift(shift),
        .header(24'h332211), .tx_data(tx_data), .tx_valid(tx_valid), .tx_ready(1'b1),
        .armed(armed), .recording(recording), .sending(sending));

    always #10 clk = ~clk;
    always @(posedge clk) cnt <= cnt + 1;

    reg [7:0] got [0:2 + 2 * N];
    integer n_got = 0;
    always @(posedge clk)
        if (tx_valid) begin
            got[n_got] = tx_data;
            n_got = n_got + 1;
        end

    integer errors = 0;

    task run(input [3:0] s, input do_arm);
        reg signed [11:0] base, want, have;
        integer i;
        begin
            shift = s;
            n_got = 0;
            if (do_arm) begin
                @(negedge clk) arm = 1;
                @(negedge clk) arm = 0;
            end
            repeat (37) @(negedge clk);
            base = cnt;                          // the sample at the trigger's clock edge
            trigger = 1;
            @(negedge clk) trigger = 0;
            wait (n_got == 3 + 2 * N);
            @(negedge clk);
            if (got[0] != 8'h11 || got[1] != 8'h22 || got[2] != 8'h33) begin
                $display("FAIL: shift %0d: header %h %h %h", s, got[0], got[1], got[2]);
                errors = errors + 1;
            end
            for (i = 0; i < N; i = i + 1) begin
                want = base + i * (1 << s);
                have = {got[4 + 2 * i][3:0], got[3 + 2 * i]};
                if (have !== want || got[4 + 2 * i][7:4] !== {4{want[11]}}) begin
                    if (errors < 10)
                        $display("FAIL: shift %0d, sample %0d: got %0d, expected %0d", s, i, have, want);
                    errors = errors + 1;
                end
            end
        end
    endtask

    initial begin
        repeat (3) @(negedge clk);
        rst = 0;
        run(0, 1);
        run(2, 1);
        run(10, 1);
        if (armed) begin
            $display("FAIL: armed again without stream mode");
            errors = errors + 1;
        end
        stream = 1;
        run(3, 1);
        run(1, 0);                                   // no arm: stream mode re-armed it
        if (errors == 0) $display("PASS: header and %0d samples each at shifts 0, 2 and 10, and two streamed captures", N);
        else             $display("FAIL: %0d errors", errors);
        $finish;
    end

endmodule
