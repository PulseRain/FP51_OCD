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
//     reply module for OCD
//=============================================================================

`include "debug_coprocessor.svh"

module debug_reply (
    input wire                                              clk,                             // clock input
    input wire                                              reset_n,                         // reset, active low
    
    input wire                                              reply_enable_in,
    input wire  unsigned [DBG_NUM_OF_OPERATIONS - 1 : 0]    reply_debug_cmd,
    input wire  unsigned [DEBUG_ACK_PAYLOAD_BITS - 1 : 0]   reply_payload,
    
    input wire                                              uart_tx_done,
    output logic                                            ctl_start_uart_tx,
    output wire unsigned [DEBUG_DATA_WIDTH - 1 : 0]         uart_data_out,
    output logic                                            reply_done
);
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // signals
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        logic unsigned [(DEBUG_FRAME_LENGTH - DEBUG_CRC_LEN) * DEBUG_DATA_WIDTH - 1 : 0]    data_out_sr;
        logic unsigned [$clog2(DEBUG_FRAME_LENGTH + 1) - 1 : 0]                             data_counter;
        logic                                                                               ctl_reset_data_counter;
        logic                                                                               ctl_inc_data_counter;
        logic                                                                               ctl_crc_sync_reset;
        wire unsigned [DEBUG_DATA_WIDTH * 2 - 1 : 0]                                        crc_out;
        logic                                                                               ctl_data_out_sr_shift;
        logic                                                                               ctl_crc_enable_in;
        logic                                                                               ctl_load_crc;
        logic                                                                               ctl_reply_done;
        
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // data_out_sr
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        always_ff @(posedge clk, negedge reset_n) begin : data_out_sr_proc
            if (!reset_n) begin
                data_out_sr <= 0;
            end else begin
                if (reply_enable_in) begin
                    data_out_sr <= {DEBUG_REPLY_SYNC, reply_payload};
                end else if (ctl_load_crc) begin
                    data_out_sr [$high(data_out_sr) : $high(data_out_sr) - 15] = crc_out; 
                end else if (ctl_data_out_sr_shift) begin
                    data_out_sr <= {data_out_sr [$high(data_out_sr) - DEBUG_DATA_WIDTH : 0], (DEBUG_DATA_WIDTH)'(170)};
                end
            end
        end : data_out_sr_proc
        
        assign uart_data_out = data_out_sr [$high(data_out_sr) : $high(data_out_sr) - DEBUG_DATA_WIDTH + 1]; 
            
        always_ff @(posedge clk, negedge reset_n) begin : data_counter_proc
            if (!reset_n) begin
                data_counter <= 0;
            end else if (ctl_reset_data_counter) begin
                data_counter <= 0;
            end else if (ctl_inc_data_counter) begin
                data_counter <= data_counter + ($size(data_counter))'(1);
            end
        end : data_counter_proc
            
        always_ff @(posedge clk, negedge reset_n) begin : reply_done_proc
            if (!reset_n) begin
                reply_done <= 0;
            end else begin
                reply_done <= ctl_reply_done;
            end
        end : reply_done_proc
        
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // CRC
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        crc16_CCITT crc16_CCITT_i (.*,
            .sync_reset (ctl_crc_sync_reset),
            .crc_en (ctl_crc_enable_in),
            .data_in (uart_data_out),
            .crc_out (crc_out)
        );
    
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // FSM
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
                
        enum {S_IDLE, S_TX_WITHOUT_CRC, S_TX_WITHOUT_CRC_WAIT_DONE, S_TX_CRC, S_TX_CRC_DONE} states = S_IDLE;
                
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
            
            ctl_reset_data_counter = 0;
            ctl_inc_data_counter = 0;
            
            ctl_start_uart_tx = 0;
            
            ctl_data_out_sr_shift = 0;
            ctl_crc_enable_in = 0;
            
            ctl_crc_sync_reset = 0;
            
            ctl_load_crc = 0;
            ctl_reply_done = 0;
            
            case (1'b1) // synthesis parallel_case 
                
                current_state [S_IDLE]: begin
                    
                    if (reply_enable_in) begin
                        ctl_reset_data_counter = 1'b1;
                        next_state [S_TX_WITHOUT_CRC] = 1;
                    end else begin
                        ctl_crc_sync_reset = 1'b1;
                        next_state [S_IDLE] = 1;                        
                    end
                    
                end
                
                current_state [S_TX_WITHOUT_CRC] : begin
                    if (data_counter == (DEBUG_FRAME_LENGTH - DEBUG_CRC_LEN)) begin
                        ctl_reset_data_counter = 1'b1;
                        ctl_load_crc = 1'b1;
                        next_state [S_TX_CRC] = 1;
                    end else begin
                        ctl_start_uart_tx = 1'b1;
                        ctl_crc_enable_in = 1'b1;
                        next_state [S_TX_WITHOUT_CRC_WAIT_DONE] = 1;
                    end
                end
                
                
                current_state [S_TX_WITHOUT_CRC_WAIT_DONE] : begin
                    if (uart_tx_done) begin
                        ctl_inc_data_counter = 1'b1; 
                        ctl_data_out_sr_shift = 1'b1;
                        next_state [S_TX_WITHOUT_CRC] = 1;
                    end else begin
                        next_state [S_TX_WITHOUT_CRC_WAIT_DONE] = 1;
                    end
                end
                
                
                current_state [S_TX_CRC] : begin
                    if (data_counter == 2) begin
                        ctl_reply_done = 1'b1;
                        next_state [S_IDLE] = 1;
                    end else begin
                        ctl_start_uart_tx = 1'b1;
                        next_state [S_TX_CRC_DONE] = 1;
                    end
                end
                
                current_state [S_TX_CRC_DONE] : begin
                    if (uart_tx_done) begin
                        ctl_inc_data_counter = 1'b1; 
                        ctl_data_out_sr_shift = 1'b1;
                        next_state [S_TX_CRC] = 1;
                    end else begin
                        next_state [S_TX_CRC_DONE] = 1;
                    end
                end
                
                default: begin
                    next_state[S_IDLE] = 1'b1;
                end
                
            endcase
            
        end : state_machine_comb
    
    
    
endmodule : debug_reply
