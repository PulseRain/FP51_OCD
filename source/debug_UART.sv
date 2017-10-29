/*
###############################################################################
# Copyright (c) 2017, PulseRain Technology LLC 
#
# This program is distributed under a dual license: an open source license, 
# and a commercial license. 
# 
# The open source license under which this program is distributed is the 
# GNU Public License version 3 (GPLv3).
#
# And for those who want to use this program in ways that are incompatible
# with the GPLv3, PulseRain Technology LLC offers commercial license instead.
# Please contact PulseRain Technology LLC (www.pulserain.com) for more detail.
#
###############################################################################
 */


//=============================================================================
// Remarks:
//     UART for OCD
//=============================================================================

`include "common.svh"
`include "Serial_8051.svh"
`include "debug_coprocessor.svh"

`default_nettype none

module debug_UART #(parameter BAUD_PERIOD) (
    input   wire                                clk,
    input   wire                                reset_n,
    
    input   wire                                sync_reset,
    input   wire                                UART_enable,
    
    input   wire                                TX_enable_in,
    
    input   wire                                RXD,
    input   wire unsigned [DEBUG_DATA_WIDTH - 1 : 0]    SBUF_in,
    input   wire                                REN,
    output  wire                                TXD,
    output  wire unsigned [DEBUG_DATA_WIDTH - 1 : 0]    SBUF_out,
    output  logic                               TX_done_pulse,
    output  wire                                TI,
    output  wire                                RI
);
    
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Signals
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        logic unsigned [$clog2(BAUD_PERIOD + 1) - 1 : 0]        baud_counter;
        wire                                                    baud_pulse;
        logic                                                   ctl_uart_rx_enable;
        logic                                                   ctl_uart_tx_start;
        logic                                                   ctl_tx_done;
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // baud counter
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        always_ff @(posedge clk, negedge reset_n) begin : baud_counter_proc
            if (!reset_n) begin
                baud_counter <= 0;
            end else if (sync_reset) begin
                baud_counter <= 0;
            end else if (UART_enable) begin
                if (baud_counter == (BAUD_PERIOD - 1)) begin
                    baud_counter <= 0;
                end else begin
                    baud_counter <= baud_counter + ($size(baud_counter))'(1);
                end
            end
        end : baud_counter_proc
    
        assign baud_pulse = (baud_counter == (BAUD_PERIOD - 1)) ? 1'b1 : 1'b0;
        
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // UART 
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
            
        Serial_8051 #(.STABLE_TIME (UART_STABLE_COUNT), 
                      .MAX_BAUD_PERIOD (MAX_UART_BAID_PERIOD)) 
                UART_i (.*,
                    .start_TX (ctl_uart_tx_start),
                    .start_RX (ctl_uart_rx_enable),
                    .class_8051_unit_pulse (1'b0),
                    .timer_trigger (baud_pulse),
                    .RXD (RXD),
                    .SBUF_in (SBUF_in),
                    .SM (3'b011),
                    .REN (REN & ctl_uart_rx_enable),
                    .TXD (TXD),
                    .TI (TI),
                    .RI (RI),
                    .SBUF_out (SBUF_out));
            
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // TX Done
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
        always_ff @(posedge clk, negedge reset_n) begin : TX_done_pulse_proc
            if (!reset_n) begin
                TX_done_pulse <= 0;
            end else begin
                TX_done_pulse <= ctl_tx_done;
            end
        end : TX_done_pulse_proc
        
            
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // FSM
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
                
        enum {S_RX, S_TX} states = 0;
                
        localparam FSM_NUM_OF_STATES = states.num();
        logic [FSM_NUM_OF_STATES - 1:0] current_state = 0, next_state;
            
        // Declare states
        always_ff @(posedge clk, negedge reset_n) begin : state_machine_reg
            if (!reset_n) begin
                current_state <= 0;
            end else begin
                current_state <= next_state;
            end
        end : state_machine_reg
            
        // state cast for debug, one-hot translation, enum value can be shown in the simulation in this way
        // Hopefully, synthesizer will optimize out the "states" variable
            
        // synthesis translate_off
        ///////////////////////////////////////////////////////////////////////
            always_comb begin : state_cast_for_debug
                for (int i = 0; i < FSM_NUM_OF_STATES; ++i) begin
                    if (current_state[i]) begin
                        $cast(states, i);
                    end
                end
            end : state_cast_for_debug
        ///////////////////////////////////////////////////////////////////////
        // synthesis translate_on   
        
        // FSM main body
        always_comb begin : state_machine_comb

            next_state = 0;
            
            ctl_uart_rx_enable = 0;
            ctl_uart_tx_start = 0;
    
            ctl_tx_done = 0;
            
            case (1'b1) // synthesis parallel_case 
                
                current_state[S_RX]: begin
                    
                    if (TX_enable_in) begin
                        ctl_uart_tx_start = 1'b1;
                        next_state [S_TX] = 1;
                    end else begin
                        ctl_uart_rx_enable = 1'b1;
                        next_state [S_RX] = 1;
                    end
                end
                
                current_state [S_TX] : begin
                    if (TI) begin
                        ctl_tx_done = 1'b1;
                        next_state [S_RX] = 1;
                    end else begin
                        next_state [S_TX] = 1;
                    end
                end
                                
                default: begin
                    next_state[S_RX] = 1'b1;
                end
                
            endcase
            
        end : state_machine_comb
endmodule : debug_UART

`default_nettype wire
