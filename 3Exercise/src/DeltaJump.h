#ifndef DELTAJUMP_H
#define DELTAJUMP_H

#include "PID.h"
#include <vector>

class DeltaJump {
//control PID
PID& pid_;
//deltas for updating filter as car drives around the track
 std::vector<double>  ganancia_deltas;
 //initial deltas 
 std::vector<double>  deltas_iniciales;
 //gain index
 int indice_de_ganancia;
 //best error detected
 double best_error;
 //current_situation
 int current_sit;
 //JumpTo the next error
 void SaltaSiguienteEscalon();
 
public:
//Constructor: PID, and gain indexes, d_Kpd,d_Ki, d_Kd
DeltaJump(PID& pid, double d_Kp, double d_Ki, double d_Kd);
//Destructor
virtual ~DeltaJump();
//Update PID between updates
bool UpdateGainsPID();
//ganancia_deltas
std::vector<double> DeltaJump::GananciaDeltas();
//Reset Method
void Reset();


}
#endif
