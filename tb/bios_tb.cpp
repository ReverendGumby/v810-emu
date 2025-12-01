// Verilated -*- C++ -*-
// DESCRIPTION: main() calling loop, created with Verilator --main

#include "verilated.h"
#include "Vbios_tb.h"

//======================

int main(int argc, char** argv, char**) {
    // Setup context, defaults, and parse command line
    Verilated::debug(0);
    const std::unique_ptr<VerilatedContext> contextp{new VerilatedContext};
    contextp->traceEverOn(true);
    contextp->commandArgs(argc, argv);

    // Construct the Verilated model, from Vtop.h generated from Verilating
    const std::unique_ptr<Vbios_tb> topp{new Vbios_tb{contextp.get(), ""}};

    // Simulate until $finish
    while (VL_LIKELY(!contextp->gotFinish())) {
        // Evaluate model
        topp->eval();
        // Advance time
        if (!topp->eventsPending()) break;
        contextp->time(topp->nextTimeSlot());
    }

    if (VL_LIKELY(!contextp->gotFinish())) {
        VL_DEBUG_IF(VL_PRINTF("+ Exiting without $finish; no events left\n"););
    }

    // Execute 'final' processes
    topp->final();

    // Print statistical summary report
    contextp->statsPrintSummary();

    return 0;
}

// Local Variables:
// compile-command: "verilator --cc --exe bios_tb.cpp --build -j 0 --timing --assert --trace-fst --trace-threads 1 -Wno-TIMESCALEMOD --top-module bios_tb --relative-includes -F v810.files ram.sv data_bus_resizer.sv bios_tb.sv"
// End:
