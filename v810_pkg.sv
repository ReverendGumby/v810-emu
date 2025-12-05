// V810 common definitions
//
// Copyright (c) 2025 David Hunter
//
// This program is GPL licensed. See COPYING for the full license.

package v810_pkg;

typedef struct packed {
    logic Carry;
    logic Over;
    logic Sign;
    logic Zero;
} aluflags_t;

typedef struct packed {
    logic [11:0] rfu20;
    logic [3:0] i;              // Interrupt level
    logic       np;             // NMI Pending
    logic       ep;             // Exception Pending
    logic       ae;             // Address Trap Enable
    logic       id;             // Interrupt Disable
    logic [1:0] rfu10;
    logic [5:0] float_fl;       // TODO
    aluflags_t  alu_fl;
} psw_t;

typedef struct packed {
    logic [15:0] fecc;
    logic [15:0] eicc;
} ecr_t;

typedef enum bit [4:0] {
    SRSEL_EIPC = 5'd0,
    SRSEL_EIPSW = 5'd1,
    SRSEL_FEPC = 5'd2,
    SRSEL_FEPSW = 5'd3,
    SRSEL_ECR = 5'd4,
    SRSEL_PSW = 5'd5,
    SRSEL_PIR = 5'd6,
    SRSEL_TKCW = 5'd7,
    SRSEL_CHCW = 5'd24,
    SRSEL_ADTRE = 5'd25
} sr_sel_t;

endpackage
