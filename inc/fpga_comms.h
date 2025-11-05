#ifndef FPGA_COMMS_H
#define FPGA_COMMS_H

#include "pico/stdlib.h"

// Protocolo de Comando
#define CMD_SAVE_POS 0xA0

void fpga_send_sequence_step(int step_index, float base, float braco, float angulo, float garra);

#endif // FPGA_COMMS_H