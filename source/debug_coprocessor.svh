`ifndef DEBUG_COPROCESSOR_SVH
`define DEBUG_COPROCESSOR_SVH

`include "common.svh"

parameter int              DEBUG_DATA_WIDTH = 8;
parameter int              DEBUG_FRAME_LENGTH = 12;
parameter int              DEBUG_EXT_FRAME_LENGTH = DEBUG_FRAME_LENGTH + 1024 - 4 + 2; // 1024 bytes data, minus 4 in the front, with 16 bit CRC

parameter int              DEBUG_128_FRAME_LENGTH = DEBUG_FRAME_LENGTH + 128 - 4 + 2; // 128 bytes data, minus 4 in the front, with 16 bit CRC

parameter int              DEBUG_SYNC_LENGTH    = 3;
parameter int              DEBUG_CRC_LEN        = 2;
parameter int              DEBUG_FRAME_TYPE_LEN = 1;
parameter int              DEBUG_FRAME_ADDR_LEN = 2;
parameter int              DEBUG_FRAME_DATA_LEN = DEBUG_FRAME_LENGTH - DEBUG_SYNC_LENGTH 
                                     - DEBUG_CRC_LEN - DEBUG_FRAME_TYPE_LEN - DEBUG_FRAME_ADDR_LEN;
                                
parameter unsigned [DEBUG_DATA_WIDTH * DEBUG_SYNC_LENGTH - 1 : 0] DEBUG_SYNC = 24'h5AA501;
parameter unsigned [DEBUG_DATA_WIDTH * DEBUG_SYNC_LENGTH - 1 : 0] DEBUG_REPLY_SYNC = 24'h80A55A;



parameter unsigned [DEBUG_DATA_WIDTH - 2 : 0]   DEBUG_TYPE_PRAM_WRITE_4_BYTES_WITHOUT_ACK  = 7'b101_1100;
parameter unsigned [DEBUG_DATA_WIDTH - 2 : 0]   DEBUG_TYPE_PRAM_WRITE_4_BYTES_WITH_ACK     = DEBUG_TYPE_PRAM_WRITE_4_BYTES_WITHOUT_ACK | 7'h1;

parameter unsigned [DEBUG_DATA_WIDTH - 2 : 0]   DEBUG_TYPE_PRAM_WRITE_128_BYTES_WITH_ACK   = 7'b101_1011;

parameter unsigned [DEBUG_DATA_WIDTH - 2 : 0]   DEBUG_TYPE_PRAM_WRITE_EXT_BYTES_WITH_ACK   = 7'b101_0111;

parameter unsigned [DEBUG_DATA_WIDTH - 2 : 0]   DEBUG_TYPE_PRAM_READ_4_BYTES   = 7'b110_1101;

parameter unsigned [DEBUG_DATA_WIDTH - 2 : 0]   DEBUG_TYPE_PAUSE_ON_WITH_ACK   = 7'b010_1101;
parameter unsigned [DEBUG_DATA_WIDTH - 2 : 0]   DEBUG_TYPE_PAUSE_OFF_WITH_ACK  = 7'b011_1101;

parameter unsigned [DEBUG_DATA_WIDTH - 2 : 0]   DEBUG_TYPE_BREAK_ON_WITH_ACK   = 7'b111_1101;
parameter unsigned [DEBUG_DATA_WIDTH - 2 : 0]   DEBUG_TYPE_BREAK_OFF_WITH_ACK  = 7'b001_1101;

parameter unsigned [DEBUG_DATA_WIDTH - 2 : 0]   DEBUG_TYPE_CPU_RESET_WITH_ACK  = 7'b100_1011;
parameter unsigned [DEBUG_DATA_WIDTH - 2 : 0]   DEBUG_TYPE_RUN_PULSE_WITH_ACK  = 7'b100_1001;

parameter unsigned [DEBUG_DATA_WIDTH - 2 : 0]   DEBUG_TYPE_READ_CPU_STATUS     = 7'b010_1111;
parameter unsigned [DEBUG_DATA_WIDTH - 2 : 0]   DEBUG_TYPE_READ_DATA_MEM       = 7'b110_1111;
parameter unsigned [DEBUG_DATA_WIDTH - 2 : 0]   DEBUG_TYPE_WRITE_DATA_MEM      = 7'b010_1011;
parameter unsigned [DEBUG_DATA_WIDTH - 2 : 0]   DEBUG_TYPE_UART_SEL            = 7'b010_1010;



parameter unsigned [DEBUG_DATA_WIDTH - 2 : 0]   DEBUG_TYPE_COUNTER_CONFIG      = 7'b110_1011;

parameter int                                       DEBUG_COUNTER_INDEX_RESET       = 1;
parameter int                                       DEBUG_COUNTER_INDEX_SET         = 2;
parameter int                                       TIME_COUNTER_INDEX_RESET        = 3;
parameter int                                       TIME_COUNTER_INDEX_SET          = 4;

parameter int                                       DEBUG_WRITE_DATA_INDEX_HIGH     = 31;
parameter int                                       DEBUG_WRITE_DATA_INDEX_LOW      = 24;

parameter unsigned [DEBUG_DATA_WIDTH - 2 : 0]   DEBUG_TYPE_ACK                 = 7'b110_0101; 


parameter int                                   DEBUG_PRAM_ADDR_WIDTH   =   16;

parameter int                                   DEBUG_ACK_PAYLOAD_BITS  = 7 * 8; // 7 bytes


parameter int                                   DEBUG_CPU_RESET_LENGTH = 6;

// DEBUG Reply command
typedef enum {OP_DBG_ACK, OP_READ_BACK_4_BYTES, OP_CPU_STATUS_ACK, OP_DATA_MEM_READ} op_DBG_t;

op_DBG_t op_DBG;
parameter DBG_NUM_OF_OPERATIONS = op_DBG.num();





extern module debug_UART #(parameter BAUD_PERIOD) (
    input   wire                                clk,
    input   wire                                reset_n,
    
    input   wire                                sync_reset,
    input   wire                                UART_enable,
    
    input   wire                                TX_enable_in,
    
    input   wire                                RXD,
    input   wire unsigned [7 : 0]               SBUF_in,
    input   wire                                REN,
    output  wire                                TXD,
    output  wire unsigned [7 : 0]               SBUF_out,
    output  logic                               TX_done_pulse,
    output  wire                                TI,
    output  wire                                RI
);

extern module debug_coprocessor (
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
    
extern module debug_coprocessor_wrapper #(parameter BAUD_PERIOD) (
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
    
extern module debug_reply (
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



    
`endif


