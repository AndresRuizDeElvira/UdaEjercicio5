#include "DeltaJump.h"
#include <math.h>
#include <numeric>

//Constructor initialization
DeltaJump::DeltaJump(PID& pid, double d_Kp, double d_Ki, double d_Kd): pid_(pid) {

	//We initialize deltas with values 
    deltas_iniciales= std::vector<double> {d_Kp, d_Ki, d_Kd};
    //we diminish size of vector to 0,1,2.....
    deltas_iniciales.resize(3);
    this->Reset();
}

//Destructor
DeltaJump::~DeltaJump();


//Reset- 1- gain deltas become initial deltas 
//we set best_error to -1.0 , current situation to 0
void DeltaJump::Reset()
{
for (int i = 0; i < deltas_iniciales.size(); i++) {
    ganancia_deltas[i] = deltas_iniciales[i];
  }
  //current gain index=0, best error =-1.0, current_sit=0;
   	indice_de_ganancia= 0;
  	best_error = -1.0;
	current_sit= 0;

}

void DeltaJump::SaltaSiguienteEscalon() {
  indice_de_ganancia++;
  //if growth index of PID is more than the size of the deltas it must be set to zero.
  if (indice_de_ganancia >= ganancia_deltas_.size()) {
   indice_de_ganancia = 0;
  }
}

bool DeltaJump::UpdateGainsPID() {
//if error is less than 1, our error is initial error of PID
  if (best_error < 0.0) {
    best_error= pid_.AccumulatedSquaredError();
  }
//however i
switch (current_sit) {
    case 0: {
      pid_.UpdateGain(ganancia_deltas[indice_de_ganancia], indice_de_ganancia);
      current_sit = 1;
      break;
}
//
case 1: {
      double errorAcumulado = pid_.AccumulatedSquaredError();
      //if error is less than the best error, we then choose to minimize it, by 
      //saying its minimum
      if (errorAcumulado < best_error) {
        best_error= errorAcumulado;
        //from trial and error
        ganancia_deltas[indice_de_ganancia] *= 1.1;
        current_sit = 0;
        this->SaltaSiguienteEscalon();
      } else {
        pid_.UpdateGain(-2.0 * ganancia_deltas[indice_de_ganancia], indice_de_ganancia);
        current_sit = 2;
      }
break;
}
 case 2: {
      double errorAcumulado = pid_.AccumulatedSquaredError();

      if (errorAcumulado < best_error) {
        best_error= error;
        ganancia_deltas[indice_de_ganancia] *= 1.1;
      } else {
        pid_.UpdateGain(ganancia_deltas[indice_de_ganancia], indice_de_ganancia);
        ganancia_deltas[indice_de_ganancia] *= 0.9;
      }

      this->SaltaSiguienteEscalon();
      current_sit = 0;
      break;
    }
}

 const double tolerance = 0.001;
  double sum = accumulate(gain_deltas_.begin(), gain_deltas_.end(), 0.0);

  return sum > tolerance;
}

std::vector<double> Twiddle::GainDeltas() {
  return gain_deltas_;
}












