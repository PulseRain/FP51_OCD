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
//     Onchip Debugger for FP51
//=============================================================================

`include "debug_coprocessor.svh"
`include "CRC.svh"

`default_nettype none

module debug_coprocessor (
    input wire                                      clk,                             // clock input
    input wire                                      reset_n,                         // reset, active low
    
    input wire                                      enable_in,
    input wire unsigned [DEBUG_DATA_WIDTH - 1 : 0]  debug_data_in,
    
    input wire                                      debug_stall_flag,
    input wire  unsigned [PC_BITWIDTH - 1 : 0]      PC,
    
    input wire                                      debug_counter_pulse,
    input wire                                      timer_pulse,
    
    output logic                                                                reply_enable_out,
    output logic unsigned [DBG_NUM_OF_OPERATIONS - 1 : 0]                       reply_debug_cmd,
    output logic unsigned [DEBUG_ACK_PAYLOAD_BITS - 1 : 0]                      reply_payload,
    
    input  wire                                                                 reply_done,
    
    input  wire                                                                 pram_read_enable_in,
    input  wire unsigned [DEBUG_DATA_WIDTH * DEBUG_FRAME_DATA_LEN - 1 : 0]      pram_read_data_in,
    
    output logic                                                                pram_read_enable_out,
    output logic unsigned [DEBUG_PRAM_ADDR_WIDTH - 1 : 0]                       pram_read_addr_out,
        
    output logic                                                                pram_write_enable_out,
    output logic unsigned [DEBUG_PRAM_ADDR_WIDTH - 3 : 0]                       pram_write_addr_out,
    output logic unsigned [DEBUG_DATA_WIDTH * DEBUG_FRAME_DATA_LEN - 1 : 0]     pram_write_data_out,
    
    output logic                                                                cpu_reset,
    output logic                                                                pause,
    output logic                                                                break_on,
    output logic unsigned [PC_BITWIDTH - 1 : 0]                                 break_addr_A,
    output logic unsigned [PC_BITWIDTH - 1 : 0]                                 break_addr_B,
    
    output logic                                                                run_pulse,
    
    input  wire                                                                 debug_read_data_enable_in,
    input  wire unsigned [DEBUG_DATA_WIDTH - 1 : 0]                             debug_read_data_in,
    
    output logic                                                                debug_data_read,
    output logic unsigned [DEBUG_PRAM_ADDR_WIDTH - 1 : 0]                       debug_data_read_addr,
    output wire                                                                 debug_rd_indirect1_direct0,
    output logic                                                                debug_data_read_restore,
    
    output logic                                                                debug_data_write,
    output logic unsigned [DEBUG_PRAM_ADDR_WIDTH - 1 : 0]                       debug_data_write_addr,
    output logic unsigned                                                       debug_wr_indirect1_direct0, 
    output logic unsigned [DEBUG_DATA_WIDTH - 1 : 0]                            debug_data_write_data,
    output logic                                                                debug_uart_tx_sel_ocd1_cpu0
    
                
);
    
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Signals
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        logic unsigned [1 : 0]                                          enable_in_sr;
        logic unsigned [DEBUG_FRAME_LENGTH * DEBUG_DATA_WIDTH - 1 : 0]  data_in_sr;
        wire  unsigned [DEBUG_DATA_WIDTH -  1 : 0]                      new_data_in;
        logic unsigned [$clog2(DEBUG_EXT_FRAME_LENGTH + 1) - 1 : 0]     input_counter;
        logic                                                           ctl_reset_input_counter;
        
        logic                                                           ctl_crc_sync_reset;
        
        wire  unsigned [15 : 0]                                         crc_out;
                    
        logic                                                             ctl_pram_write_enable;
        logic                                                             ctl_pram_read_enable;
        
        wire unsigned [DEBUG_PRAM_ADDR_WIDTH - 1 : 0]                     pram_addr;
        wire unsigned [DEBUG_PRAM_ADDR_WIDTH - 1 : 0]                     data_mem_addr;
        wire unsigned [DEBUG_FRAME_DATA_LEN * DEBUG_DATA_WIDTH -  1 : 0]  pram_data;
        
        wire  unsigned [DEBUG_DATA_WIDTH -  2 : 0]                        frame_type;
        wire                                                              toggle_bit;
                        
        wire                                                              uart_sel_ocd1_cpu0;
        
        logic unsigned [6 : 0]                                            debug_counter;
        logic unsigned [15 : 0]                                           timer_counter;
        logic                                                             enable_debug_counter;
        logic                                                             enable_timer_counter;
        
                    
        logic                                                             ctl_reply_wr_ack;
        logic                                                             ctl_reply_pram_read_back;
        logic                                                             ctl_reply_data_mem_read_back;
        logic                                                             ctl_set_pause_on;
        logic                                                             ctl_set_pause_off;    
        logic                                                             ctl_break_on;
        logic                                                             ctl_break_off;
        logic                                                             ctl_cpu_reset;
        logic unsigned [DEBUG_CPU_RESET_LENGTH - 1 : 0]                   cpu_reset_sr;
        logic                                                             ctl_run_pulse;
        logic                                                             ctl_cpu_status_ack;
        logic                                                             ctl_debug_data_read;
        logic                                                             ctl_debug_data_write;
        logic                                                             ctl_reset_debug_counter;
        logic                                                             ctl_enable_debug_counter;
        logic                                                             ctl_reset_timer_counter;
        logic                                                             ctl_enable_timer_counter;
        logic                                                             ctl_disable_debug_counter;
        logic                                                             ctl_disable_timer_counter;
        logic                                                             ctl_uart_sel;
        
        logic unsigned [DEBUG_PRAM_ADDR_WIDTH - 1 : 0]                    pram_addr_ext;
        logic                                                             ctl_load_pram_addr_ext;
        logic                                                             ctl_inc_pram_addr_ext;
        
        logic                                                             wr_ext_enable;
        logic                                                             ctl_wr_ext_disable;
        logic                                                             ctl_wr_ext_enable;
        
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // shift registers
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        always_ff @(posedge clk, negedge reset_n) begin : enable_in_sr_proc
            if (!reset_n) begin
                enable_in_sr <= 0;
                data_in_sr <= 0;
            end else if (enable_in) begin
                enable_in_sr <= {enable_in_sr [$high(enable_in_sr) - 1 : 0] , 1'b1};
                data_in_sr <= {data_in_sr[$high(data_in_sr) - DEBUG_DATA_WIDTH : 0], debug_data_in};
            end else begin
                enable_in_sr <= {enable_in_sr [$high(enable_in_sr) - 1 : 0] , 1'b0};
            end
        end : enable_in_sr_proc
        
        assign new_data_in = data_in_sr [DEBUG_DATA_WIDTH - 1 : 0];
    
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // CRC
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        crc16_CCITT crc16_CCITT_i (.*,
            .sync_reset (ctl_crc_sync_reset),
            .crc_en (enable_in_sr[0]),
            .data_in (new_data_in),
            .crc_out (crc_out)
        );
            
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // input_counter
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        
        always_ff @(posedge clk, negedge reset_n) begin : input_counter_proc
            if (!reset_n) begin
                input_counter <= 0;
            end else if (ctl_reset_input_counter) begin
                input_counter <= 0;
            end else if (enable_in_sr[0]) begin
                input_counter <= input_counter + ($size(input_counter))'(1);
            end
            
        end : input_counter_proc
        
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // pause
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        
        always_ff @(posedge clk, negedge reset_n) begin : pause_proc
            if (!reset_n) begin
                pause <= 0;
            end else if (ctl_set_pause_on) begin
                pause <= 1'b1;
            end else if (ctl_set_pause_off) begin
                pause <= 1'b0;
            end
        end : pause_proc
        
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // break
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        
        always_ff @(posedge clk, negedge reset_n) begin : break_proc
            if (!reset_n) begin
                break_on <= 0;
                break_addr_A <= 0;
                break_addr_B <= 0;
            end else if (ctl_break_on) begin
                break_on <= 1'b1;
                break_addr_A <= pram_addr;
                break_addr_B <= pram_data [$high(break_addr_B) : 0];
            end else if (ctl_break_off) begin
                break_on <= 0;
            end
        end : break_proc
        
            
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // CPU reset
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        
        always_ff @(posedge clk, negedge reset_n) begin : cpu_reset_proc
            if (!reset_n) begin
                cpu_reset <= 0;
                cpu_reset_sr <= 0;
            end else begin
                cpu_reset_sr <= {cpu_reset_sr [$high(cpu_reset_sr) - 1 : 0], ctl_cpu_reset};
                cpu_reset <= |cpu_reset_sr;
            end
            
        end : cpu_reset_proc
    
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // run_pulse
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        
        always_ff @(posedge clk, negedge reset_n) begin : run_pulse_proc
            if (!reset_n) begin
                run_pulse <= 0;
            end else begin
                run_pulse <= ctl_run_pulse;
            end
        end : run_pulse_proc
        
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // debug_pram_write
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        
        assign pram_addr = data_in_sr [(DEBUG_FRAME_DATA_LEN + DEBUG_CRC_LEN + DEBUG_FRAME_ADDR_LEN) * DEBUG_DATA_WIDTH - 1 : 
                                       (DEBUG_FRAME_DATA_LEN + DEBUG_CRC_LEN) * DEBUG_DATA_WIDTH];
        
        assign data_mem_addr = pram_addr;
        
        assign pram_data = data_in_sr [(DEBUG_FRAME_DATA_LEN + DEBUG_CRC_LEN) * DEBUG_DATA_WIDTH - 1 : 
                                       DEBUG_CRC_LEN * DEBUG_DATA_WIDTH];
        
        assign debug_rd_indirect1_direct0 = pram_data [0];
        assign uart_sel_ocd1_cpu0         = pram_data [1];
        
        assign frame_type = data_in_sr [(DEBUG_FRAME_DATA_LEN + DEBUG_CRC_LEN + DEBUG_FRAME_ADDR_LEN + DEBUG_FRAME_TYPE_LEN) * DEBUG_DATA_WIDTH - 1 : 
                                        (DEBUG_FRAME_DATA_LEN + DEBUG_CRC_LEN + DEBUG_FRAME_ADDR_LEN ) * DEBUG_DATA_WIDTH + 1];
        
        assign toggle_bit = data_in_sr [(DEBUG_FRAME_DATA_LEN + DEBUG_CRC_LEN + DEBUG_FRAME_ADDR_LEN ) * DEBUG_DATA_WIDTH];
        
        /*debug_pram_write debug_pram_write_i (.*,
            .we    (ctl_pram_write_enable),
            .addr  (pram_addr),
            .data  (pram_data),
            .wr_ext_flag (1'b0),
            
            .write_enable_out (pram_write_enable_out),
            .write_addr (pram_write_addr_out),
            .write_data (pram_write_data_out),
            .write_done (write_done));
        */
        
        always_ff @(posedge clk, negedge reset_n) begin : pram_write_proc
            if (!reset_n) begin
                pram_write_enable_out <= 0;
                pram_write_addr_out   <= 0;
                pram_write_data_out   <= 0;
            end else begin
                pram_write_enable_out <= ctl_pram_write_enable;
                
                if (wr_ext_enable) begin
                    pram_write_addr_out   <= pram_addr_ext [DEBUG_PRAM_ADDR_WIDTH - 1 : 2];
                end else begin  
                    pram_write_addr_out   <= pram_addr [DEBUG_PRAM_ADDR_WIDTH - 1 : 2];
                end
                
                pram_write_data_out   <= pram_data;
                
            end
        end : pram_write_proc
            
        always_ff @(posedge clk, negedge reset_n) begin : pram_read_proc
            if (!reset_n) begin
                pram_read_enable_out <= 0;
                pram_read_addr_out   <= 0;
            end else begin
                pram_read_enable_out <= ctl_pram_read_enable;
                pram_read_addr_out   <= pram_addr;
            end
            
        end : pram_read_proc
        
    
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // uart selection
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        
        always_ff @(posedge clk, negedge reset_n) begin
            if (!reset_n) begin
                debug_uart_tx_sel_ocd1_cpu0 <= 0;
            end else if (ctl_uart_sel) begin
                debug_uart_tx_sel_ocd1_cpu0 <= uart_sel_ocd1_cpu0;
            end
            
        end
        
            
    
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // debug counter, timer counter
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        always_ff @(posedge clk, negedge reset_n) begin
            if (!reset_n) begin
                debug_counter  <= 0;
                timer_counter <= 0;
            end else begin
                if (ctl_reset_debug_counter) begin
                    debug_counter <= 0;                 
                end else if (enable_debug_counter & debug_counter_pulse) begin
                    debug_counter <= debug_counter + ($size(debug_counter))'(1);
                end
                
                if (ctl_reset_timer_counter) begin
                    timer_counter <= 0;
                end else if (enable_timer_counter & timer_pulse) begin
                    timer_counter <= timer_counter + ($size(timer_counter))'(1);
                end
                
            end
        end 
        
        always_ff @(posedge clk, negedge reset_n) begin
            if (!reset_n) begin
                enable_debug_counter <= 0;
                enable_timer_counter <= 0;
            end else begin
                
                if (ctl_disable_debug_counter) begin
                    enable_debug_counter <= 0;
                end else if (ctl_enable_debug_counter) begin
                    enable_debug_counter <= 1'b1;
                end
                
                if (ctl_disable_timer_counter) begin
                    enable_timer_counter <= 0;
                end else if (ctl_enable_timer_counter) begin
                    enable_timer_counter <= 1'b1;
                end
                
            end
        end
        
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // data_mem_read
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        
        always_ff @(posedge clk, negedge reset_n) begin : data_mem_read_proc
            if (!reset_n) begin
                debug_data_read <= 0;
                debug_data_read_restore <= 0;
                debug_data_read_addr <= 0;
            end else begin
                debug_data_read          <= ctl_debug_data_read;
                debug_data_read_restore  <= debug_data_read;
                debug_data_read_addr     <= data_mem_addr;
            end
        end : data_mem_read_proc
        
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // data_mem_write
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        
        always_ff @(posedge clk, negedge reset_n) begin : data_mem_write_proc
            if (!reset_n) begin
                debug_data_write           <= 0;
                debug_wr_indirect1_direct0 <= 0;
                debug_data_write_data      <= 0;
            end else begin
                debug_data_write           <= ctl_debug_data_write;
                debug_wr_indirect1_direct0 <= debug_rd_indirect1_direct0;
                debug_data_write_data      <= pram_data [DEBUG_WRITE_DATA_INDEX_HIGH : DEBUG_WRITE_DATA_INDEX_LOW];
            end
        end : data_mem_write_proc
                                      
        assign debug_data_write_addr = debug_data_read_addr;
        
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Reply Acknowledge 
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        always_ff @(posedge clk, negedge reset_n) begin : reply_ack_proc
            if (!reset_n) begin
                reply_enable_out <= 0;
                reply_debug_cmd <= 0;
                reply_payload <= 0;
            end else begin
                reply_enable_out <= ctl_reply_wr_ack | ctl_reply_pram_read_back | ctl_cpu_status_ack | ctl_reply_data_mem_read_back;
                
                case (1'b1) // synthesis parallel_case 
                    ctl_reply_wr_ack : begin
                        reply_debug_cmd <= ($size(reply_debug_cmd))'(1 << OP_DBG_ACK); 
                        reply_payload <= {frame_type, toggle_bit, pram_addr, 32'hAAABACAD};
                    end
                    
                    ctl_reply_pram_read_back : begin
                        reply_debug_cmd <= ($size(reply_debug_cmd))'(1 << OP_READ_BACK_4_BYTES);   
                        reply_payload <= {frame_type, toggle_bit, pram_addr, pram_read_data_in};
                    end
                    
                    ctl_cpu_status_ack : begin
                        reply_debug_cmd <= ($size(reply_debug_cmd))'(1 << OP_CPU_STATUS_ACK);
                        reply_payload <= {frame_type, toggle_bit, 8'h0, debug_counter, debug_stall_flag, timer_counter, PC};
                    end
                    
                    ctl_reply_data_mem_read_back : begin
                        reply_debug_cmd <= ($size(reply_debug_cmd))'(1 << OP_DATA_MEM_READ);
                        reply_payload <= {frame_type, toggle_bit, data_mem_addr, 24'hAABBCC, debug_read_data_in};
                    end
                    
                    default : begin
                        reply_payload <= 0;
                    end
                endcase
            end
        end : reply_ack_proc
    
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // debug RAM
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        
        always_ff @(posedge clk, negedge reset_n) begin
            if (!reset_n) begin
                pram_addr_ext <= 0;
            end else if (ctl_load_pram_addr_ext) begin
                pram_addr_ext <= pram_addr + ($size(pram_addr))'(4);
            end else if (ctl_inc_pram_addr_ext) begin
                pram_addr_ext <= pram_addr_ext + ($size(pram_addr_ext))'(4);
            end
        end
        
        always_ff @(posedge clk, negedge reset_n) begin
            if (!reset_n) begin
                wr_ext_enable <= 0;
            end else if (ctl_wr_ext_disable) begin
                wr_ext_enable <= 0;
            end else if (ctl_wr_ext_enable) begin
                wr_ext_enable <= 1'b1;
            end
        end
                 
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // FSM
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
                
        enum {S_IDLE, S_SYNC_1, S_SYNC_0, S_INPUT_WAIT, S_FRAME_TYPE, S_CRC, S_WR_ACK, S_PRAM_READ_WAIT, 
              S_CPU_STATUS_ACK, S_DATA_MEM_READ_WAIT, S_WAIT_DONE, S_WR_EXT, S_WR_EXT_128, S_EXT_CRC} states = 0;
                
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
            
            ctl_reset_input_counter = 0;
            
            ctl_crc_sync_reset = 0;
            
            ctl_pram_write_enable = 0;
            ctl_pram_read_enable = 0;
            
            ctl_reply_wr_ack = 0;
            
            ctl_reply_pram_read_back = 0;
            
            ctl_set_pause_on = 0;
            ctl_set_pause_off = 0;
            
            ctl_break_on = 0;
            ctl_break_off = 0;
            
            ctl_cpu_reset = 0;
            
            ctl_run_pulse = 0;
            
            ctl_cpu_status_ack = 0;
            
            ctl_debug_data_read = 0;
            ctl_debug_data_write = 0;
            
            ctl_reply_data_mem_read_back = 0;
                    
            ctl_reset_debug_counter = 0;
            ctl_enable_debug_counter = 0;
            ctl_reset_timer_counter = 0;
            ctl_enable_timer_counter = 0;
            
            ctl_disable_debug_counter  = 0;
            ctl_disable_timer_counter = 0;
            
            ctl_uart_sel= 0;
            
            ctl_load_pram_addr_ext = 0;
            ctl_inc_pram_addr_ext = 0;
            
            ctl_wr_ext_disable = 0;
            ctl_wr_ext_enable = 0;
        
            case (1'b1) // synthesis parallel_case 
                
                current_state[S_IDLE]: begin
                    ctl_wr_ext_disable = 1'b1;
                    
                    if (enable_in_sr[0] && (new_data_in == DEBUG_SYNC[DEBUG_DATA_WIDTH * 3 - 1 : DEBUG_DATA_WIDTH * 2])) begin
                        next_state [S_SYNC_1] = 1;
                    end else begin
                        ctl_crc_sync_reset = 1'b1;
                        next_state [S_IDLE] = 1;                        
                    end
                end
                
                current_state [S_SYNC_1] : begin
                    if (enable_in_sr[0]) begin
                        if (new_data_in == DEBUG_SYNC[DEBUG_DATA_WIDTH * 2 - 1 : DEBUG_DATA_WIDTH * 1]) begin
                            next_state [S_SYNC_0] = 1;
                        end else begin
                            next_state [S_IDLE] = 1;
                        end
                    end else begin
                        next_state [S_SYNC_1] = 1;
                    end
                end
                
                current_state [S_SYNC_0] : begin
                    ctl_reset_input_counter = 1;
                    
                    if (enable_in_sr[0]) begin
                        if (new_data_in == DEBUG_SYNC[DEBUG_DATA_WIDTH - 1 : 0]) begin
                            next_state [S_INPUT_WAIT] = 1;
                        end else begin
                            next_state [S_IDLE] = 1;
                        end
                    end else begin
                        next_state [S_SYNC_0] = 1;
                    end
                end
                
                current_state [S_INPUT_WAIT] : begin
                    if (input_counter == (DEBUG_FRAME_LENGTH - DEBUG_SYNC_LENGTH - 1)) begin
                        next_state [S_FRAME_TYPE] = 1;          
                    end else begin
                        next_state [S_INPUT_WAIT] = 1;
                    end
                end
                
                current_state [S_FRAME_TYPE] : begin
                    ctl_reset_input_counter = 1'b1;
                    
                    if (enable_in_sr[0]) begin
                        next_state [S_CRC] = 1;
                    end else begin
                        next_state [S_FRAME_TYPE] = 1;
                    end
                    
                end
                
                current_state [S_CRC] : begin
                    ctl_load_pram_addr_ext = 1'b1;
                    
                    if (!crc_out) begin
                        case (frame_type) // synthesis parallel_case
                                          
                            DEBUG_TYPE_PRAM_WRITE_128_BYTES_WITH_ACK  : begin
                                ctl_pram_write_enable = 1'b1;
                                ctl_crc_sync_reset = 1'b1;
                                
                                next_state [S_WR_EXT_128] = 1;
                                
                            end
                            
                            DEBUG_TYPE_PRAM_WRITE_EXT_BYTES_WITH_ACK  : begin
                                ctl_pram_write_enable = 1'b1;
                                ctl_crc_sync_reset = 1'b1;
                                
                                next_state [S_WR_EXT] = 1;
                            end
                            
                            DEBUG_TYPE_PRAM_WRITE_4_BYTES_WITHOUT_ACK : begin
                                ctl_pram_write_enable = 1'b1;
                                next_state [S_IDLE] = 1;
                            end
                            
                            DEBUG_TYPE_PRAM_WRITE_4_BYTES_WITH_ACK : begin
                                
                                ctl_pram_write_enable = 1'b1;
                                next_state [S_WR_ACK] = 1;
                            end
                            
                            DEBUG_TYPE_PRAM_READ_4_BYTES : begin
                                ctl_pram_read_enable = 1'b1;
                                next_state [S_PRAM_READ_WAIT] = 1;
                            end
                            
                            DEBUG_TYPE_PAUSE_ON_WITH_ACK : begin
                                ctl_set_pause_on = 1'b1;
                                next_state [S_WR_ACK] = 1;
                            end
                                                
                            DEBUG_TYPE_PAUSE_OFF_WITH_ACK : begin
                                ctl_set_pause_off = 1'b1;
                                next_state [S_WR_ACK] = 1;
                            end
                            
                            DEBUG_TYPE_BREAK_ON_WITH_ACK : begin
                                ctl_break_on = 1'b1;
                                next_state [S_WR_ACK] = 1;
                            end
                            
                            DEBUG_TYPE_BREAK_OFF_WITH_ACK : begin
                                ctl_break_off = 1'b1;
                                next_state [S_WR_ACK] = 1;
                            end
                            
                            DEBUG_TYPE_CPU_RESET_WITH_ACK : begin
                                ctl_cpu_reset = 1'b1;
                                next_state [S_WR_ACK] = 1;
                            end
                            
                            DEBUG_TYPE_RUN_PULSE_WITH_ACK : begin
                                ctl_run_pulse = 1'b1;
                                next_state [S_WR_ACK] = 1;
                            end
                            
                            DEBUG_TYPE_READ_CPU_STATUS : begin
                                next_state [S_CPU_STATUS_ACK] = 1;
                            end
                            
                            DEBUG_TYPE_READ_DATA_MEM : begin
                                ctl_debug_data_read = 1'b1;
                                next_state [S_DATA_MEM_READ_WAIT] = 1;
                            end
                            
                            DEBUG_TYPE_WRITE_DATA_MEM : begin
                                ctl_debug_data_write = 1'b1;
                                next_state [S_WR_ACK] = 1;
                            end
                                                        
                            
                            DEBUG_TYPE_COUNTER_CONFIG : begin
                                
                                ctl_reset_debug_counter   = pram_data[DEBUG_COUNTER_INDEX_RESET];
                                ctl_enable_debug_counter  = pram_data[DEBUG_COUNTER_INDEX_SET];
                                ctl_disable_debug_counter = ~pram_data[DEBUG_COUNTER_INDEX_SET];
                                
                                ctl_reset_timer_counter   = pram_data[TIME_COUNTER_INDEX_RESET];
                                ctl_enable_timer_counter  = pram_data[TIME_COUNTER_INDEX_SET];
                                ctl_disable_timer_counter = ~pram_data[TIME_COUNTER_INDEX_SET];
        
                                next_state [S_WR_ACK] = 1;
                            end
                            
                            DEBUG_TYPE_UART_SEL : begin
                                ctl_uart_sel = 1'b1;
                                next_state [S_IDLE] = 1;
                            end
                            
                            default : begin
                                next_state [S_IDLE] = 1;
                            end
                                
                        endcase
                        
                        
                    end else begin
                        next_state [S_IDLE] = 1;
                    end
                end
                                
                current_state [S_WR_ACK] : begin
                    ctl_crc_sync_reset = 1'b1;
                    
                    ctl_reply_wr_ack = 1'b1;
                    next_state [S_WAIT_DONE] = 1;
                    
                end
                
                current_state [S_CPU_STATUS_ACK] : begin
                    ctl_crc_sync_reset = 1'b1;
                    
                    ctl_cpu_status_ack = 1'b1;
                    next_state [S_WAIT_DONE] = 1;
                end
                
                
                current_state [S_PRAM_READ_WAIT] : begin
                    ctl_crc_sync_reset = 1'b1;
                    
                    if (!pram_read_enable_in) begin
                        next_state [S_PRAM_READ_WAIT] = 1;
                    end else begin
                        ctl_reply_pram_read_back = 1'b1;
                        next_state [S_WAIT_DONE] = 1;
                    end
                end
                
                current_state [S_DATA_MEM_READ_WAIT] : begin
                    ctl_crc_sync_reset = 1'b1;
                    
                    if (!debug_read_data_enable_in) begin
                        next_state [S_DATA_MEM_READ_WAIT] = 1;
                    end else begin
                        ctl_reply_data_mem_read_back = 1'b1;
                        next_state [S_WAIT_DONE] = 1;
                    end
                    
                end
                                
                current_state [S_WAIT_DONE] :begin
                    if (reply_done) begin
                        next_state [S_IDLE] = 1;
                    end else begin
                        next_state [S_WAIT_DONE] = 1;
                    end
                end
                
                current_state [S_WR_EXT] : begin
                    
                    ctl_wr_ext_enable = 1'b1;
                    
                    if (input_counter == (DEBUG_EXT_FRAME_LENGTH - DEBUG_FRAME_LENGTH)) begin
                        next_state [S_EXT_CRC] = 1;         
                    end else begin
                        next_state [S_WR_EXT] = 1;
                    end
                    
                    if (enable_in_sr[0] && (|input_counter[$high(input_counter) : 2]) && (input_counter [1:0] == 2'b01)) begin
                        ctl_pram_write_enable = 1'b1;   
                        ctl_inc_pram_addr_ext = 1'b1;
                    end
                    
                end
                
                current_state [S_WR_EXT_128] : begin
                    
                    ctl_wr_ext_enable = 1'b1;
                    
                    if (input_counter == (DEBUG_128_FRAME_LENGTH - DEBUG_FRAME_LENGTH)) begin
                        next_state [S_EXT_CRC] = 1;         
                    end else begin
                        next_state [S_WR_EXT_128] = 1;
                    end
                    
                    if (enable_in_sr[0] && (|input_counter[$high(input_counter) : 2]) && (input_counter [1:0] == 2'b01)) begin
                        ctl_pram_write_enable = 1'b1;   
                        ctl_inc_pram_addr_ext = 1'b1;
                    end
                    
                end
                
                
                
                current_state [S_EXT_CRC] : begin
                    if (!crc_out) begin
                        next_state [S_WR_ACK] = 1;
                    end else begin
                        next_state [S_IDLE] = 1;    
                    end
                end
                
                default: begin
                    next_state[S_IDLE] = 1'b1;
                end
                
            endcase
            
        end : state_machine_comb
    
endmodule : debug_coprocessor


`default_nettype wire
