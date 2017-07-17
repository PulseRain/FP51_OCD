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
//     Top level wrapper for FP51 Onchip Debugger 
//=============================================================================

`include "debug_coprocessor.svh"

module debug_coprocessor_wrapper #(parameter BAUD_PERIOD) (
        input   wire                                clk,
        input   wire                                reset_n,
    
        
        input   wire                                RXD,
        output  wire                                TXD,
        
        input wire                                  debug_stall_flag,
        input wire  unsigned [PC_BITWIDTH - 1 : 0]  PC,
        
        input wire                                  debug_counter_pulse,
        input wire                                  timer_pulse,
        
        input  wire                                                                 pram_read_enable_in,
        input  wire unsigned [DEBUG_DATA_WIDTH * DEBUG_FRAME_DATA_LEN - 1 : 0]      pram_read_data_in,
    
        output wire                                                                 pram_read_enable_out,
        output wire unsigned [DEBUG_PRAM_ADDR_WIDTH - 1 : 0]                        pram_read_addr_out,
    
        
        output wire                                                                 pram_write_enable_out,
        output wire unsigned [DEBUG_PRAM_ADDR_WIDTH - 3 : 0]                        pram_write_addr_out,
        output wire unsigned [DEBUG_DATA_WIDTH * DEBUG_FRAME_DATA_LEN  - 1 : 0]     pram_write_data_out,
        
        output logic                                                                cpu_reset,
        output logic                                                                pause,
        output logic                                                                break_on,
        
        output wire unsigned [PC_BITWIDTH - 1 : 0]                                  break_addr_A,
        output wire unsigned [PC_BITWIDTH - 1 : 0]                                  break_addr_B,
        
        output logic                                                                run_pulse,
        
        input  wire                                                                 debug_read_data_enable_in,
        input  wire unsigned [DEBUG_DATA_WIDTH - 1 : 0]                             debug_read_data_in,
        
        output wire                                                                 debug_data_read,
        output wire unsigned [DEBUG_PRAM_ADDR_WIDTH - 1 : 0]                        debug_data_read_addr,
        output wire                                                                 debug_rd_indirect1_direct0,
        output wire                                                                 debug_data_read_restore,
        
        output wire                                                                 debug_data_write,
        output wire unsigned [DEBUG_PRAM_ADDR_WIDTH - 1 : 0]                        debug_data_write_addr,
        output wire unsigned                                                        debug_wr_indirect1_direct0, 
        output wire unsigned [DEBUG_DATA_WIDTH - 1 : 0]                             debug_data_write_data,
        output wire                                                                 debug_uart_tx_sel_ocd1_cpu0
    
);

    wire  unsigned [7 : 0]                    SBUF_out;
    wire                                      TI, RI;
    
    wire                                                        reply_enable;
    wire  unsigned [DBG_NUM_OF_OPERATIONS - 1 : 0]              reply_debug_cmd;
    wire  unsigned [DEBUG_ACK_PAYLOAD_BITS - 1 : 0]             reply_payload;
    
    wire                                                        TX_done_pulse;
    
    wire                                                        ctl_start_uart_tx;
    wire unsigned [7 : 0]                                       SBUF_in;
    wire                                                        reply_done;
    
    
   

    debug_UART #(.BAUD_PERIOD (BAUD_PERIOD)) debug_UART_i (.*,
                .sync_reset (1'b0),
                .UART_enable (1'b1),
                .TX_enable_in (ctl_start_uart_tx),
                .RXD (RXD),
                .SBUF_in (SBUF_in),
                .REN (1'b1),
                .TXD (TXD),
                .SBUF_out (SBUF_out),
                .TX_done_pulse (TX_done_pulse), 
                .TI (TI),
                .RI (RI));
    
    debug_reply debug_reply_i (.*,
            .reply_enable_in (reply_enable),
            .reply_debug_cmd (reply_debug_cmd),
            .reply_payload (reply_payload),
            
            .uart_tx_done (TX_done_pulse),
            
            .ctl_start_uart_tx (ctl_start_uart_tx),
            .uart_data_out (SBUF_in),
            .reply_done (reply_done));
    
    
    debug_coprocessor debug_coprocessor_i (.*,
            .enable_in (RI),
            .debug_data_in (SBUF_out),
                        
            .debug_stall_flag (debug_stall_flag),
            .PC (PC),
            
            .debug_counter_pulse (debug_counter_pulse),
            .timer_pulse (timer_pulse),
            
            .reply_enable_out (reply_enable),
            .reply_debug_cmd (reply_debug_cmd),
            .reply_payload (reply_payload),
            
            .reply_done (reply_done),
            
            .pram_read_enable_in (pram_read_enable_in),
            .pram_read_data_in (pram_read_data_in),
            
            .pram_read_enable_out (pram_read_enable_out),
            .pram_read_addr_out (pram_read_addr_out),
            
            .pram_write_enable_out (pram_write_enable_out),
            .pram_write_addr_out (pram_write_addr_out),
            .pram_write_data_out (pram_write_data_out),
            
            .cpu_reset (cpu_reset),
            .pause (pause),
            .break_on (break_on),
            .break_addr_A (break_addr_A),
            .break_addr_B (break_addr_B),
            .run_pulse (run_pulse),
            
            .debug_read_data_enable_in (debug_read_data_enable_in),
            .debug_read_data_in (debug_read_data_in),
            
            .debug_data_read (debug_data_read),
            .debug_data_read_addr (debug_data_read_addr),
            .debug_rd_indirect1_direct0 (debug_rd_indirect1_direct0),
            .debug_data_read_restore (debug_data_read_restore),
            
            .debug_data_write (debug_data_write),
            .debug_data_write_addr (debug_data_write_addr),
            .debug_wr_indirect1_direct0 (debug_wr_indirect1_direct0),
            .debug_data_write_data (debug_data_write_data),
            .debug_uart_tx_sel_ocd1_cpu0 (debug_uart_tx_sel_ocd1_cpu0)
    
            );
        
    
endmodule : debug_coprocessor_wrapper

