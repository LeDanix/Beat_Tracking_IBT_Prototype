/*
** Copyright (C) 2000-2010 George Tzanetakis <gtzan@cs.uvic.ca>
**
** This program is free software; you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published by
** the Free Software Foundation; either version 2 of the License, or
** (at your option) any later version.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with this program; if not, write to the Free Software
** Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
*/

/*
** IBT - standing for INESC-Porto Beat Tracker - is a real-time/off-line
** tempo induction and beat tracking system based on a competing multi-agent
** strategy, which considers parallel hypotheses regarding tempo and beats.
**
** Published in:
** Oliveira J. L., Gouyon F., Martins L., Reis L. P. IBT: A Real-Time Tempo and Beat Tracking System.
** International Conference on Music Information Retrieval, pp. 291- 296, Utrecht, 2010. ISBN: 978-90-393-53813.
**
** Modified by:
** Daniel Saiz Azor in April 2021.
** Comparison and Implementation of Beat-Tracking Techniques to aid musical learning
*/

#include <cstdio>
#include <cstdlib>
#include <string>
#include <iomanip>
#include <windows.h>
#include <stdio.h>

#include <marsyas/common_source.h>
#include <marsyas/CommandLineOptions.h>
#include <marsyas/Collection.h>
#include <marsyas/FileName.h>
#include <marsyas/NumericLib.h>
#include <marsyas/system/MarSystemManager.h>
#include <marsyas/sched/EvValUpd.h>
#include <marsyas/marsystems/MarSystemTemplateBasic.h>
#include <marsyas/marsystems/MarSystemTemplateAdvanced.h>

//#include <string.h>
#include <iostream>
#include <string>

#ifdef MARSYAS_MIDIIO
#include "RtMidi.h"
#endif

//#ifdef MARSYAS_AUDIOIO
//#include "RtAudio3.h"
//#endif

using namespace std;
using namespace Marsyas;

#ifdef MARSYAS_WIN32
#pragma warning(disable: 4244)  //disable double to float warning
#pragma warning(disable: 4100) //disable argc warning
#endif

//mrs_bool sonicOutFlux = 0;
//mrs_bool sonicOutFluxFilter = 0;

//============================== IBT FUNCTIONAL PARAMETERS ==============================
#define ERROR_DEBUG_MODE 1
#define DEBUG_MODE 1
#define BPM_HYPOTHESES 6 //Nr. of initial BPM hypotheses (must be <= than the nr. of agents) (6)
#define PHASE_HYPOTHESES 30//Nr. of phases per BPM hypothesis (30)
#define MIN_BPM 81 //minimum tempo considered, in BPMs (50) [81 -> to prevent octave error]
#define MAX_BPM 160 //maximum tempo considered, in BPMs (250) [160 -> to prevent octave error]
#define NR_AGENTS 30 //Nr. of agents in the pool (30)
#define LFT_OUTTER_MARGIN 0.20 //(Inertia1.1) The size of the outer half-window (in % of the IBI) before the predicted beat time (0.20)
#define RGT_OUTTER_MARGIN 0.40 //(Inertia1.2) The size of the outer half-window (in % of the IBI) after the predicted beat time (0.30)
#define INNER_MARGIN 4.0 //(Inertia1.3) Inner tolerance window margin size (= half inner window size -> in ticks) (4.0)
#define OBSOLETE_FACTOR 0.8 //An agent is killed if, at any time (after the initial Xsecs-defined in BeatReferee), the difference between its score and the bestScore is below OBSOLETE_FACTOR * bestScore (0.8)
#define LOST_FACTOR 8 //An agent is killed if it become lost, i.e. if it found LOST_FACTOR consecutive beat predictions outside its inner tolerance window (8)
#define CHILDREN_SCORE_FACTOR 0.9 //(Inertia2) Each created agent imports its father score multiplied (or divided if negative) by this factor (0.9)
#define BEST_FACTOR 1.0 //(Inertia3) Mutiple of the bestScore an agent's score must have for replacing the current best agent (1.0)
#define CORRECTION_FACTOR 0.25 //(Inertia4) correction factor for compensating each agents' own {phase, period} hypothesis errors (0.25)
#define EQ_PERIOD 1 //Period threshold which identifies two agents as predicting the same period (IBI, in ticks) (1)
#define EQ_PHASE 2 //Phase threshold which identifies two agents as predicting the same phase (phase, in ticks) (2)
#define CHILD1_FACTOR 1.0 //Correction factor (error proportion-[0.0-1.0]) for compensating its father's {phase, period} hypothesis - used by child1 (2.0 - only full phase adjustment; -1 - no child considered) (1.0)
#define CHILD2_FACTOR 2.0 //Correction factor (error proportion-[0.0-1.0]) for compensating its father's {phase, period} hypothesis - used by child2 (2.0 - only full phase adjustment; -1 - no child considered) (2.0)
#define CHILD3_FACTOR 0.5 //Correction factor (error proportion-[0.0-1.0]) for compensating its father's {phase, period} hypothesis - used by child3 (2.0 - only full phase adjustment; -1 - no child considered) (0.5)
#define TRIGGER_GT_TOL 5 //Number of miss computed beats, in comparison to ground-truth beat-times, tolerated before triggering new induction (used in trigger "groundtruth" mode) -> can be defined via -tigt_tol ¡¡¡¡¡¡No se usa!!!!! 
#define TRIGGER_BEST_FACTOR 1.0 //Proportion of the current best agent score which is inherited by the agents created at each triggered induction [shouldn't be much higher than 1, for not inflating scores two much] (1.0)
#define SUPERVISED_TRIGGER_THRES 0.03 //Degree (in percentage) of mean bestScore decrease to trigger a new induction in supervised induction mode (0.03)
#define BEAT_TRANSITION_TOL 0.6 //Tolerance for handling beats at transitions between agents [-1 for unconsider it]: (0.6)
//In causal mode, if between two consecutive beats there is over a BEAT_TRANSITION_TOL decrease in current IBI the second beat is unconsidered;
//In non-causal mode, if between a son's first beat and its father's last there is over a BEAT_TRANSITION_TOL descrease on the father last IBI the son's first beat is unconsidered;
//In non-causal mode, if between a son's first beat and its father's last there is over a BEAT_TRANSITION_TOL increase on the father last IBI the son's first beat shall be its father's next beat, and the second beat shall be its assigned first.

#define WINSIZE 1024 //(2048?)
#define HOPSIZE 512 //(512)   Es la mitad de la ventana, habra un HopSize del 50% y un overlap de 50%

//=======================================================================================
//IBT Variables

CommandLineOptions cmd_options;

mrs_string PORT_COM; //Puerto Serie COM
mrs_string score_function;  //Forma de sacar datos de tempos y beats
mrs_string induction_mode;  //Nos dice si se hara uso del metodo simple de induccion, el que sabemos, o el basado en un fichero con la verdad real.
mrs_real induction_time;  //Se usa para definir un tiempo de computo inicial en el modo causal. 
mrs_bool avoid_metrical_changes;  //Evita los problema de metrica haciendo que los bpm esten limitados a una octava [81-160]BPM
mrs_string execPath;  //Direccion donde esta el archivo de audio. Si hacemos lo del microfono, esto nos sobrara.
mrs_natural helpopt;  //Aunque ponga que es un mrs_natural, es un booleano que se usa a modo de printear help o no.
mrs_natural usageopt;  //Aunque ponga que es un mrs_natural, se usa como booleano para printear usage o no.
mrs_bool backtraceopt;  //Hace una especia de analisis solo en offline mode.
mrs_bool noncausalopt;  //Hace que funcione el modo offline, con mejores prestaciones. Aun asi, hay que ahorrar espacio y demas. HAY QUE QUITAR LA OPCION
mrs_bool micinputopt;  //Para usar el microfono como input
mrs_real phase_;
mrs_natural minBPM_;
mrs_natural maxBPM_;
mrs_real sup_thres;
mrs_real beatDetec = 0;

//=======================================================================================
//COM Variables
HANDLE handlerSerialComm;
DCB dcbSerialParams = {0};
COMMTIMEOUTS timeouts = {0};


int
printUsage(string progName)
{
  MRSDIAG("ibt.cpp - printUsage");
  cerr << "Usage : " << progName <<
       " [-p : PORT_COM] [-off : offline (non-causal)] [-mic : microphone_input] [-t time(secs) : inductionLength] [-i : induction_operation] [-s \"heuristics\" : scoreFunction ] [-io : induction_out] [-b : backtrace] [-m : avoid_metrical_changes] fileName" << endl; //[-send_udp send_udp]
  cerr << "where fileName is a sound file in a MARSYAS supported format and COM, the serial PORT where the microcontroler is connected." << endl;
  cerr << endl;
  return(1);
}


int
printHelp(string progName)
{
  MRSDIAG("ibt.cpp - printHelp");
  cerr << "ibt, MARSYAS, Copyright George Tzanetakis" << endl;
  cerr << "*****************************************" << endl;
  cerr << "IBT implemented by João Lobato Oliveira (contact: jmldso@gmail.com)" << endl;
  cerr << "*****************************************" << endl;
  cerr << "IBT based on: Oliveira J. L., Davies M. E. P., Gouyon F., and Reis L. P. Beat tracking for multiple applications: A multi-agent system architecture with state recovery. In IEEE Transactions on Audio Speech and Language Processing, 2012" << endl;
  cerr << "--------------------------------------------------------" << endl;
  cerr << "Modified by Daniel Saiz Azor to allow communication with physical hardware" << endl;
  cerr << "--------------------------------------------------------" << endl;
  cerr << "IBT plugins for Max/MSP, Pure Data, and Sonic Visualiser available @ http://smc.inescporto.pt/research/demo_software/" << endl;
  cerr << "--------------------------------------------------------" << endl;
  cerr << "Detect the beat times and tempo (median IBI) in real-time (or off-line), for the sound file (or microphone input) provided as argument" << endl;
  cerr << endl;
  cerr << "Usage : " << progName << " [-options] fileName" << endl;
  cerr << "[fileName is a sound file in a Marsyas supported format]" << endl;
  cerr << endl;
  cerr << "Help Options [default]:" << endl;
  cerr << "-u --usage          		  : display short usage info." << endl;
  cerr << "-h --help           		  : display this information." << endl;
  cerr << "-off --offline    		  : for running in offline (non-causal) mode -> best performance! [deactivated]" << endl;
  cerr << "-mic --microphone		  : input sound via microphone interface. [deactivated]" << endl;
  cerr << "-t --induction_length 		  : length (in secs) of the induction window [5.0]" << endl;
  cerr << "-i --induction_operation 	  : induction mode of operation, from the following: [\"single\"]" << endl;
  cerr <<	"  \"single\"		 	  : only one induction at the beginning of the analysis." << endl;
  cerr <<	"  \"auto-reset\" 			  : reset induction mode of operation, requesting re-inductions of the system automatically." << endl;
  cerr <<	"  \"auto-regen\" 			  : regen induction mode of operation, requesting re-inductions of the system automatically." << endl;
  cerr <<	"  \"repeated-reset\" 		  : reset induction mode of operation, requesting re-inductions of the system at fixed time-points, spaced by 100frames." << endl;
  cerr <<	"  \"repeated-regen\" 		  : regen induction mode of operation, requesting re-inductions of the system at fixed time-points, spaced by 100frames." << endl;
  cerr <<	"  \"random-reset\" 		  : reset induction mode of operation, requesting re-inductions of the system at random time-points spaced in the range of [1.2-15]s." << endl;
  cerr <<	"  \"random-regen\" 		  : regen induction mode of operation, requesting re-inductions of the system at random time-points spaced in the range of [1.2-15]s." << endl;
  //cerr << "  \"groundtruth\"" << endl;
  //cerr << "-tigt_tol --triggergtmode_tol : Number of miss computed beats, in comparison to ground-truth beat-times, tolerated before triggering new induction (to be used in trigger \"groundtruth\" mode)" << endl;
  cerr << "-s --score_function 		  : heuristics which conducts the beat tracking (\"regular\" (default); \"correlation\"; \"squareCorr\")" << endl;
  cerr << "-b --backtrace      		  : after induction backtrace the analysis to the beginning. [deactivated - used by default in offline mode]" << endl;
  cerr << "-m --avoid_metrical_changes	  :  avoid metrical changes by setting the tempo range within one octave at [81-160]BPM. [default in causal operation and induction_operation different than \"single\"]" << endl;
  //cerr << "-send_udp --send_udp : [!!WINDOWS_ONLY!!] send beats - \"beat_flag(tempo)\" - via udp sockets at defined port (in localhost) - for causal mode" << endl;
  return(1);
}

void    //Da valores a variables locales, el valor predefinido o el valor metido por pantalla.  CREO QUE PUEDO DEJARLO ASI O INCLUSO DEBERIA MIRAR QUE HACE cmd_options.add___ y ver si se puede cambiar con una inicializacion basica, por ejemplo, induction_lentgh = 5.0;
initOptions()
{
  cmd_options.addStringOption("PORT_COM", "p", "10");
  cmd_options.addBoolOption("help", "h", false);
  cmd_options.addBoolOption("usage", "u", false);
  cmd_options.addBoolOption("backtrace", "b", false);
  cmd_options.addBoolOption("offline", "off", false);
  cmd_options.addBoolOption("induction_out", "io", false);
  cmd_options.addBoolOption("microphone", "mic", false);
  //cmd_options.addNaturalOption("sendudp", "send_udp", -1);
  cmd_options.addStringOption("induction_operation", "i", "-1");
  //the score function (heuristics) which conducts the beat tracking ("regular" by default)
  cmd_options.addStringOption("score_function", "s", "regular");
  //Time (in seconds) of induction before tracking. Has to be > 60/MIN_BPM (5.0 by default)
  cmd_options.addRealOption("induction_length", "t", 5.0);
  //initial time (in secs) allowed for eventual tracking metrical changes (0 not allowing at all; -1 for the whole music)" (5.0 by default)
  cmd_options.addBoolOption("avoid_metrical_changes", "m", false);
  cmd_options.addRealOption("sup_thres", "st", SUPERVISED_TRIGGER_THRES);
}

void  //Carga lo que se ha especificado en consola a las variables correspondientes.  CREO QUE SE PUEDE QUITAR Y PONER LO QUE HE COMENTADO EN LA FUNCION ANTERIOR
loadOptions()
{
  PORT_COM = cmd_options.getStringOption("PORT_COM"); 
  helpopt = cmd_options.getBoolOption("help");
  usageopt = cmd_options.getBoolOption("usage");
  backtraceopt = cmd_options.getBoolOption("backtrace");
  noncausalopt = cmd_options.getBoolOption("offline");
  micinputopt = cmd_options.getBoolOption("microphone");
  induction_mode = cmd_options.getStringOption("induction_operation");
  score_function = cmd_options.getStringOption("score_function");
  induction_time = cmd_options.getRealOption("induction_length");
  avoid_metrical_changes = cmd_options.getBoolOption("avoid_metrical_changes");
  sup_thres = cmd_options.getRealOption("sup_thres");
}

mrs_bool  //Comprueba que exista el archivo de audio.  CREO QUE SE PUEDE QUITAR
existsFile(mrs_string fileName)
{
  if (FILE * file = fopen(fileName.c_str(), "r"))
  {
    fclose(file);
    return true;
  }
  else
  {
    cerr << "Bad or nonexistent file! Please specify a supported one." << endl;
    return false;
  }
}


void  //El puto main, se le pasan el nombre del archivo de audio y el nombre del archivo de salida de texto
ibt(mrs_string sfName)
{
  MarSystemManager mng;  //Se creara un objeto marsistem que servira para manejar todo el sistema compuesto posterior
  MarSystem* audioflow = mng.create("Series", "audioflow"); //Creamos una serie donde se encapsulara todo el proceso

  if(micinputopt) //si se conecta un microfono, se usara de entrada un AudioSource
    audioflow->addMarSystem(mng.create("AudioSource", "micsrc"));

  else //Sino, se utilizara el fichero de audio
    audioflow->addMarSystem(mng.create("SoundFileSource", "src"));

  //En el caso de que se obtenga una señal con 2 señales, se modificara a una sola señal (creo que esta funcion no existe)
  audioflow->addMarSystem(mng.create("Stereo2Mono", "src")); //replace by a "Monofier" MarSystem (to be created) [!]

  //Se crea una instacia beattracker
  MarSystem* beattracker = mng.create("FlowThru","beattracker");
  //beattracker->addMarSystem(mng.create("Stereo2Mono", "src")); //replace by a "Monofier" MarSystem (to be created) [!]

  //Se va a crear otra "Serie" que se encargara de la deteccion de comienzos compuesta por:
  MarSystem* onsetdetectionfunction = mng.create("Series", "onsetdetectionfunction");
  //ShiftInput que se usara para el overlap y el hop size, evitando asi, cambios bruscos en la señal (no lo tengo nada claro)
  onsetdetectionfunction->addMarSystem(mng.create("ShiftInput", "si"));
  //Windowing donde se enventanara la señal porque es la unica forma de tratarla con fourier y distintas tecnicas
  onsetdetectionfunction->addMarSystem(mng.create("Windowing", "win"));
  //Spectrum consiste en hallar el desarrollo en frecuencias a lo largo del tiempo
  onsetdetectionfunction->addMarSystem(mng.create("Spectrum","spk"));
  //PowerSpectrum muestra las frecuencias con mas prominencia en el instante de muestreo
  onsetdetectionfunction->addMarSystem(mng.create("PowerSpectrum", "pspk"));
  //Flux es una medicion de la variacion del PowerSpectrum y nos permite conocer una variacion brusca de dichas potencias, que sera un indicativo claro de onset
  onsetdetectionfunction->addMarSystem(mng.create("Flux", "flux"));

  //if(sonicOutFlux)
  //	onsetdetectionfunction->addMarSystem(mng.create("SonicVisualiserSink", "sonicsink"));

  //Metemos la serie onsetdetection en el conjunto beattracker
  beattracker->addMarSystem(onsetdetectionfunction);
  //Metemos otro shiftImput despues de la deteccion de onset
  beattracker->addMarSystem(mng.create("ShiftInput", "acc"));

  //Creamos otra serie dentro de beattracker para filtrar, con 2 filtros y dos reverses
  MarSystem* normfiltering = mng.create("Series", "normfiltering");
  //Filtramos con unos coeficientes que se explicaran mas adelante
  normfiltering->addMarSystem(mng.create("Filter","filt1"));

  //if(sonicOutFluxFilter)
  //	normfiltering->addMarSystem(mng.create("SonicVisualiserSink", "sonicsinkfilt"));

  //Reverse lo que hace es girar el array, asi volviendo a filtrar y volviendo a girar se consigue filtrar en magnitud pero deja la fase igual, o al menos practicamente
  normfiltering->addMarSystem(mng.create("Reverse","rev1"));
  normfiltering->addMarSystem(mng.create("Filter","filt2"));
  normfiltering->addMarSystem(mng.create("Reverse","rev2"));

  //if(sonicOutFluxFilter)
  //	normfiltering->addMarSystem(mng.create("SonicVisualiserSink", "sonicsinkfilt"));
  
  //Añadimos normfiltering a beattracking
  beattracker->addMarSystem(normfiltering);

  //Creamos un conjunto para la induccion del tempo (se trataran los procesos de señal de tempo)
  MarSystem* tempoinduction = mng.create("FlowThru", "tempoinduction");

  //Creamos un conjunto para manejar la hipotesis de tempo. Es un Fanout, por lo que con una entrada 
  //de la señal proveniente del filtrado, se generaran tantas otras señales como se requieran
  MarSystem* tempohypotheses = mng.create("Fanout", "tempohypotheses");

  //Dentro de la creaccion de una hipotesis de tempo, haremos una autocorrelaccion para encontrar patrones repetitivos, 
  //un peaker para recojer los picos que mostraran esa repetitividad y recogerlos calculando sus posiciones con MaxArgMax
  //Asi, con la diferencia de tiempos entre estos, obtendremos un valor  de tmepo inicial (no lo tengo del todo claro, pero creo que si)
  MarSystem* tempo = mng.create("Series", "tempo");
  tempo->addMarSystem(mng.create("AutoCorrelation","acf"));
  tempo->addMarSystem(mng.create("Peaker", "pkr"));
  tempo->addMarSystem(mng.create("MaxArgMax", "mxr"));

  tempohypotheses->addMarSystem(tempo);

  //Crearemos una serie Phase que se pondran en consonancia con la hipotesis inicial de tempo. Se obtendran en base a los onsets
  MarSystem* phase = mng.create("Series", "phase");
  //Se encuentran los onsets y posteriormente, OnsetTimes halla la phase dados estos onsets dentro de la ventana
  phase->addMarSystem(mng.create("PeakerOnset","pkronset"));
  phase->addMarSystem(mng.create("OnsetTimes","onsettimes"));

  tempohypotheses->addMarSystem(phase);

  tempoinduction->addMarSystem(tempohypotheses);

  //Esto agrupa todas las posibles hipotesis que se tienen con una matriz Nx3, donde N es el numero 
  //de periodos * fases que halla, ya que sabemos que por cada periodo, habra varias fases. 
  //
  //    [[fase0 periodo0 periodSalience0]
  //     [fase1 periodo1 periodSalience1]
  //     [fase2 periodo2 periodSalience2]}
  //
  tempoinduction->addMarSystem(mng.create("TempoHypotheses", "tempohyp"));

  beattracker->addMarSystem(tempoinduction);

  //El puto seleccionador (PhaseLock), es jesucristo y yasta
  //Permite recojer las N mejores hipotesis que se le han dado en la matriz del tempoHypotheses dentro de la ventana de tratamiento. 
  //Todo esto gracias a la correlaccion entre estas hipotesis y la deteccion de comienzos. Aunque tambien se puede correlaccionar con la ground truth, que no vamos a usar
  //Se le meten un huevo de parametros
  MarSystem* initialhypotheses = mng.create("FlowThru", "initialhypotheses");
  initialhypotheses->addMarSystem(mng.create("PhaseLock", "phaselock"));

  beattracker->addMarSystem(initialhypotheses);

  //Se creara la piscina de agentes, como un fanout, de entrada, las N mejores hipotesis
  MarSystem* agentpool = mng.create("Fanout", "agentpool");
  //Se genera un bloque de agente, por agente, con un numero concreto
  for(int i = 0; i < NR_AGENTS; ++i)
  {
    ostringstream oss;
    oss << "agent" << i;
    agentpool->addMarSystem(mng.create("BeatAgent", oss.str()));
  }
  beattracker->addMarSystem(agentpool);
  //BeatReferee es otra de las tochas, se encarga de evaluar los agentes, crer nuevos y desechar a los que 
  //no se necesitan en funcion de las especificaciones dadas en los papers
  beattracker->addMarSystem(mng.create("BeatReferee", "br"));

  //Si la salida se habia definido como None, es decir, que el programa no saque ningun resultado, cosa queme parece estupida, pues sacamos un resultado


  audioflow->addMarSystem(beattracker);


  minBPM_ = MIN_BPM; //Aqui definiremos los BPM
  maxBPM_ = MAX_BPM;

  ///////////////////////////////////////////////////////////////////////////////////////
  //link controls
  ///////////////////////////////////////////////////////////////////////////////////////
  //if(!micinputopt)
  //	IBTsystem->linkControl("mrs_bool/hasData",
  //		"Fanout/beatmix/Series/audioflow/SoundFileSource/src/mrs_bool/hasData");

  //lOS LINKERS SIRVEN PARA ENTRELAZAR LAS VARIABLES QUE SE USAN Y DEVUELVEN LOS MARSYSTEMS (ENTRADAS, SALIDAS, VARIBALES UTILES ...)


  //Link LookAheadSamples used in PeakerOnset para compensar el tiempo que podriamos perder al computar creo, y que todas tengan el mismo valor de variable
  tempoinduction->linkControl("Fanout/tempohypotheses/Series/phase/PeakerOnset/pkronset/mrs_natural/lookAheadSamples",
                              "Fanout/tempohypotheses/Series/phase/OnsetTimes/onsettimes/mrs_natural/lookAheadSamples");

  //Linkeamos shiftinput con el tamaño de acumulador de onsettimes para compensar posibles desincronismos
  beattracker->linkControl("FlowThru/tempoinduction/Fanout/tempohypotheses/Series/phase/OnsetTimes/onsettimes/mrs_natural/accSize",
                           "ShiftInput/acc/mrs_natural/winSize");

  //Linkeamos La matriz de tempo hypotheses con el tamaño del acumulador de onset times que sera la que le pase la info                          
  beattracker->linkControl("FlowThru/tempoinduction/TempoHypotheses/tempohyp/mrs_natural/accSize",
                           "FlowThru/tempoinduction/Fanout/tempohypotheses/Series/phase/OnsetTimes/onsettimes/mrs_natural/accSize");

  //Pasa la matrix de hipotesis al bloque PhaseLock, que es el que elige las buenas buenas
  beattracker->linkControl("FlowThru/initialhypotheses/PhaseLock/phaselock/mrs_realvec/beatHypotheses",
                           "FlowThru/tempoinduction/mrs_realvec/innerOut");

  //Linkeamos la opcion de backtrace 
  beattracker->linkControl("FlowThru/initialhypotheses/PhaseLock/phaselock/mrs_bool/backtrace",
                           "BeatReferee/br/mrs_bool/backtrace");
  //Mas opciones de linkado entre PhaseLock y BeatReferee, para controlar agentes
  beattracker->linkControl("FlowThru/initialhypotheses/PhaseLock/phaselock/mrs_real/curBestScore",
                           "BeatReferee/br/mrs_real/curBestScore");

  //Linkeamos el factor de correccion que compensa el error de hipotesis de cada agente, 0.25
  beattracker->linkControl("FlowThru/initialhypotheses/PhaseLock/phaselock/mrs_real/corFactor",
                           "BeatReferee/br/mrs_real/corFactor");

  //Linkeamos el triggerinduction que supongo que es una especia de trigger que activa los bloques en ciertos puntos
  //Se linkean onsetTimes - TempoHypothesis
  //           tempoHypothesis - PhaseLock
  //           PhaseLock - BeatReferee
  beattracker->linkControl("FlowThru/initialhypotheses/PhaseLock/phaselock/mrs_bool/triggerInduction",
                           "BeatReferee/br/mrs_bool/triggerInduction");
  beattracker->linkControl("FlowThru/tempoinduction/TempoHypotheses/tempohyp/mrs_bool/triggerInduction",
                           "FlowThru/initialhypotheses/PhaseLock/phaselock/mrs_bool/triggerInduction");
  beattracker->linkControl("FlowThru/tempoinduction/Fanout/tempohypotheses/Series/phase/OnsetTimes/onsettimes/mrs_bool/triggerInduction",
                           "FlowThru/tempoinduction/TempoHypotheses/tempohyp/mrs_bool/triggerInduction");

  //Linkeamos la salida del FlowThru de Hipotesis inicial, que contiene lo que saca PhaseLock al beatHypothesis de BeatReferee, para que controle a los agentes
  beattracker->linkControl("BeatReferee/br/mrs_realvec/beatHypotheses",
                           "FlowThru/initialhypotheses/mrs_realvec/innerOut");

  //Los N maximos que encuentra la funcion MaxArgMax para estimar el tempo, seran las N hypothesys de la funcion que engloba todas en una matriz
  beattracker->linkControl("FlowThru/tempoinduction/Fanout/tempohypotheses/Series/tempo/MaxArgMax/mxr/mrs_natural/nMaximums",
                           "FlowThru/tempoinduction/TempoHypotheses/tempohyp/mrs_natural/nPeriods");

  //Logicamente, las N hipotesis de tempo de la matriz de TempoHypothesis seran las N hipothesis de PhaseLock
  beattracker->linkControl("FlowThru/tempoinduction/TempoHypotheses/tempohyp/mrs_natural/nPeriods",
                           "FlowThru/initialhypotheses/PhaseLock/phaselock/mrs_natural/nrPeriodHyps");

  //Las N hipotesis de tempo de PhaseLock, seran el mismo que el numero de hipotesis de periodo que tuviese OnsetTimes
  beattracker->linkControl("FlowThru/initialhypotheses/PhaseLock/phaselock/mrs_natural/nrPeriodHyps",
                           "FlowThru/tempoinduction/Fanout/tempohypotheses/Series/phase/OnsetTimes/onsettimes/mrs_natural/nPeriods");

  //Numero de fases generadas por OnsetTimes sera el mismo que el numero de fases de tempoHypothesis
  beattracker->linkControl("FlowThru/tempoinduction/Fanout/tempohypotheses/Series/phase/OnsetTimes/onsettimes/mrs_natural/n1stOnsets",
                           "FlowThru/tempoinduction/TempoHypotheses/tempohyp/mrs_natural/nPhases");

  //Numero de fases que hay en la matriz tempoHypothesis seran las mismas que las que habra en el phaseLock
  beattracker->linkControl("FlowThru/tempoinduction/TempoHypotheses/tempohyp/mrs_natural/nPhases",
                           "FlowThru/initialhypotheses/PhaseLock/phaselock/mrs_natural/nrPhasesPerPeriod");
                           
  //Como al Fanout le llega una entrada y salen multiples, la salida debera coincidir con la entrada siguiente, en este caso, el numero de fases del phaseLock
  //que es la entrada de un bloque, debera coincidir con la salida del bloque Fanout, que coincide con el numero de tempo hipotesis del MaxArgMax (fanoutLength)
  beattracker->linkControl("FlowThru/initialhypotheses/PhaseLock/phaselock/mrs_natural/nrPhasesPerPeriod",
                           "FlowThru/tempoinduction/Fanout/tempohypotheses/Series/tempo/MaxArgMax/mxr/mrs_natural/fanoutLength");

  //Los agentes que el BeatReferee mutea se pasaran en forma de realvec al Fanout de agentpool para evitar que sigan funcionando
  beattracker->linkControl("Fanout/agentpool/mrs_realvec/muted", "BeatReferee/br/mrs_realvec/mutedAgents");
  
  //Linkeamos las hipotesis muteadas al BeatReferee para desabilitar el periodo de induccion despues de el tiempo de induccion
  beattracker->linkControl("FlowThru/tempoinduction/Fanout/tempohypotheses/mrs_realvec/muted",
                           "BeatReferee/br/mrs_realvec/inductionEnabler");


  for(int i = 0; i < NR_AGENTS; ++i)
  {
    ostringstream oss;
    oss << "agent" << i;
    //Le pasamos a cada agente una hipotesis de tempo y fase que viene retroaliemtnada por BeatAgent
    //Lo de retroalimentada es porque BeatAgent se encarga de modificar estos valores cada periodo de iteracion ajustando los valores y dando scores, y a su vez,
    //BeatReferee modifica estos agentes 
    beattracker->linkControl("Fanout/agentpool/BeatAgent/"+oss.str()+"/mrs_realvec/agentControl",
                             "BeatReferee/br/mrs_realvec/agentControl");
    //Las hipotesis factibles iniciales provenientes de PhaseLock, con minimo y maximo periodo posible se les dan a cada agente
    beattracker->linkControl("Fanout/agentpool/BeatAgent/"+oss.str()+"/mrs_natural/minPeriod",
                             "FlowThru/initialhypotheses/PhaseLock/phaselock/mrs_natural/minPeriod");
    beattracker->linkControl("Fanout/agentpool/BeatAgent/"+oss.str()+"/mrs_natural/maxPeriod",
                             "FlowThru/initialhypotheses/PhaseLock/phaselock/mrs_natural/maxPeriod");
  }

  //Defines tempo induction time after which the BeatAgents' hypotheses are populated:
  //El tiempo de induccion de phaseLock sera el mismo que el tiempo de induccion en el bloque de tempoHypothesis
  beattracker->linkControl("FlowThru/initialhypotheses/PhaseLock/phaselock/mrs_natural/inductionTime",
                           "FlowThru/tempoinduction/TempoHypotheses/tempohyp/mrs_natural/inductionTime");

  //E igual que el de BeatReferee
  beattracker->linkControl("BeatReferee/br/mrs_natural/inductionTime",
                           "FlowThru/initialhypotheses/PhaseLock/phaselock/mrs_natural/inductionTime");

  //Y los minimos y maximos peridos de tempo seran iguales para el beatReferee como para el phaseLock, y para el beatReferee como para la hipotesis de tempo (matrix)
  beattracker->linkControl("BeatReferee/br/mrs_natural/minPeriod",
                           "FlowThru/initialhypotheses/PhaseLock/phaselock/mrs_natural/minPeriod");
  beattracker->linkControl("BeatReferee/br/mrs_natural/maxPeriod",
                           "FlowThru/initialhypotheses/PhaseLock/phaselock/mrs_natural/maxPeriod");
  beattracker->linkControl("FlowThru/tempoinduction/TempoHypotheses/tempohyp/mrs_natural/minPeriod",
                           "BeatReferee/br/mrs_natural/minPeriod");
  beattracker->linkControl("FlowThru/tempoinduction/TempoHypotheses/tempohyp/mrs_natural/maxPeriod",
                           "BeatReferee/br/mrs_natural/maxPeriod");

  //El tiempo de induccion sera igual para la deteccionde fase de onsetTimes como para el beatReferee
  beattracker->linkControl("FlowThru/tempoinduction/Fanout/tempohypotheses/Series/phase/OnsetTimes/onsettimes/mrs_natural/inductionTime",
                           "BeatReferee/br/mrs_natural/inductionTime");

  //Link score function from a BeatAgent (all have the same) to PhaseLock
  //Linkeamos la funcion de puntuacion (es un metodo de deduccion) de phaseLock a todos los agentes (Con iniciarlizarla en 1, agente0, ya vale). Pensar que esta funcion le da un valor inicial de puntuacion
  //a los agentes, cada uno una distinta ya que tienen distintos tempos y fases, pero luego los agentes ya van evolucionando dicha puntuacion.
  //Tambien les daremos los margenes tanto internos como externos(derecha e izquierda, 0.4*IBI y 0.2*IBI)
  beattracker->linkControl("Fanout/agentpool/BeatAgent/agent0/mrs_string/scoreFunc",
                           "FlowThru/initialhypotheses/PhaseLock/phaselock/mrs_string/scoreFunc");
  beattracker->linkControl("Fanout/agentpool/BeatAgent/agent0/mrs_real/innerMargin",
                           "FlowThru/initialhypotheses/PhaseLock/phaselock/mrs_real/innerMargin");
  beattracker->linkControl("Fanout/agentpool/BeatAgent/agent0/mrs_real/lftOutterMargin",
                           "FlowThru/initialhypotheses/PhaseLock/phaselock/mrs_real/lftOutterMargin");
  beattracker->linkControl("Fanout/agentpool/BeatAgent/agent0/mrs_real/rgtOutterMargin",
                           "FlowThru/initialhypotheses/PhaseLock/phaselock/mrs_real/rgtOutterMargin");

  //Linkeamos el hopSize, aunque ponga inSamples, esto llevara dentro el valor del hopSize
  beattracker->linkControl("BeatReferee/br/mrs_natural/hopSize", "mrs_natural/inSamples");

  //Logicamente, linkeamos el hopsize y la Fs de tempo Hypothesis y el de beatReferee
  beattracker->linkControl("FlowThru/tempoinduction/TempoHypotheses/tempohyp/mrs_natural/hopSize",
                           "BeatReferee/br/mrs_natural/hopSize");
  beattracker->linkControl("FlowThru/tempoinduction/TempoHypotheses/tempohyp/mrs_real/srcFs",
                           "BeatReferee/br/mrs_real/srcFs");

  //Todos los tick counter debe ir linkeados
  beattracker->linkControl("FlowThru/tempoinduction/Fanout/tempohypotheses/Series/phase/OnsetTimes/onsettimes/mrs_natural/tickCount",
                           "BeatReferee/br/mrs_natural/tickCount");
  beattracker->linkControl("FlowThru/tempoinduction/TempoHypotheses/tempohyp/mrs_natural/tickCount",
                           "FlowThru/tempoinduction/Fanout/tempohypotheses/Series/phase/OnsetTimes/onsettimes/mrs_natural/tickCount");
  beattracker->linkControl("FlowThru/initialhypotheses/PhaseLock/phaselock/mrs_natural/tickCount",
                           "FlowThru/tempoinduction/TempoHypotheses/tempohyp/mrs_natural/tickCount");

  //Hacemos un ajuste entre los frames (unidad que trabajan estos MarSystems) a segundos, para poder llevarlo al sink
  beattracker->linkControl("BeatReferee/br/mrs_natural/adjustment",
                           "FlowThru/initialhypotheses/PhaseLock/phaselock/mrs_natural/adjustment");
  
  
  //Link SonicVisualiserSink parameters with the used ones:
  /*
  if(sonicOutFlux)
  {
  	beattracker->linkControl("Series/onsetdetectionfunction/SonicVisualiserSink/sonicsink/mrs_natural/hopSize",
  		"BeatReferee/br/mrs_natural/hopSize");
  	beattracker->linkControl("Series/onsetdetectionfunction/SonicVisualiserSink/sonicsink/mrs_real/srcFs",
  		"FlowThru/tempoinduction/TempoHypotheses/tempohyp/mrs_real/srcFs");
  }
  if(sonicOutFluxFilter)
  {
  	beattracker->linkControl("Series/normfiltering/SonicVisualiserSink/sonicsinkfilt/mrs_natural/hopSize",
  		"BeatReferee/br/mrs_natural/hopSize");
  	beattracker->linkControl("Series/normfiltering/SonicVisualiserSink/sonicsinkfilt/mrs_real/srcFs",
  		"FlowThru/tempoinduction/TempoHypotheses/tempohyp/mrs_real/srcFs");
  }
  */



  // OJO, PARA MANDAR LA SEÑAL AL MICRO DE QUE PARPADEE, HAY UNA VARIABLE BOOLEANA EN BEATREFEREE QUE ES beatDetected que creo que se pone a 1 cuando hay un beat detectado.
  //MIRAR A VER SI PUEDO USARLO


  //Linkeamos frecuencia de sampleo, y el hop size desde phaseLock
  beattracker->linkControl("FlowThru/initialhypotheses/PhaseLock/phaselock/mrs_natural/hopSize",
                           "BeatReferee/br/mrs_natural/hopSize");
  beattracker->linkControl("FlowThru/initialhypotheses/PhaseLock/phaselock/mrs_real/srcFs",
                           "FlowThru/tempoinduction/TempoHypotheses/tempohyp/mrs_real/srcFs");

  //Si se ha definido un induction mode. 
  if(!strcmp(induction_mode.c_str(), "-1") == 0)
  {
    //Si no es ninguna de las siguientes opciones
    if(!strcmp(induction_mode.c_str(), "single") == 0
        && !strcmp(induction_mode.c_str(), "repeated-reset") == 0 && !strcmp(induction_mode.c_str(), "repeated-regen") == 0
        && !strcmp(induction_mode.c_str(), "random-reset") == 0 && !strcmp(induction_mode.c_str(), "random-regen") == 0
        && !strcmp(induction_mode.c_str(), "auto-reset") == 0 && !strcmp(induction_mode.c_str(), "auto-regen") == 0)
    {
      cerr << "Induction Mode: re-define induction_mode value as one of the following: \"single\", \"auto-reset\", \"auto-regen\", \"repeated-reset\", \"repeated-regen\", \"random-reset\", \"random-regen\" -> \"single\" assumed." << endl;
      //Ponemos el single mode
      induction_mode = "single";
    }
    //Decimos por pantalla que modo se esta utilizando
    cout << "Requested induction in \"" << induction_mode << "\" operation" << endl;
  }
  //Linkeamos unos valores cuya activacion borra los datos de calculo cada enventanado
  beattracker->linkControl("BeatReferee/br/mrs_bool/resetFeatWindow", "ShiftInput/acc/mrs_bool/clean");

    //Se linkean los coeficientes de los filtros, ya que ambos 2 son el mismo filtro
  beattracker->linkControl("Series/normfiltering/Filter/filt2/mrs_realvec/ncoeffs",
                           "Series/normfiltering/Filter/filt1/mrs_realvec/ncoeffs");
  beattracker->linkControl("Series/normfiltering/Filter/filt2/mrs_realvec/dcoeffs",
                           "Series/normfiltering/Filter/filt1/mrs_realvec/dcoeffs");

  //En el caso NO CAUSAL, proponemos valores entre 50-250 bpm, porque no hay problemas de octvas como existian en el modo causal. PREGUNTAR 
  //Si se usa el modo no causal y no se ha definido induction_mode o se ha definido como single y los BPM estaban definidos entre 81 y 160, los ponemos a 50 y 250
  //Es decir, si nosotros habiamos definido otros valores iniciales para el modo causal que no fuesen [81-160], no se cambiaran a estos valores, seguiran esos
  if(noncausalopt && ((strcmp(induction_mode.c_str(), "-1") == 0) || (strcmp(induction_mode.c_str(), "single") == 0)) && minBPM_ == 81 && maxBPM_ == 160)
  {
    minBPM_ = 50;
    maxBPM_ = 250;
  }
  //Si permites "evitar cambios metricos", se pondrn los valores por defecto para evitar problemas con las octavas
  if(avoid_metrical_changes)
  {
    minBPM_ = 81;
    maxBPM_ = 160;
    cerr << "Avoid metrical changes mode activated (default in causal operation)." << endl;
  }
  //Lo decimos por pantalla
  cerr << "Considered tempo in the range [" << minBPM_ << "-" << maxBPM_ << "]BPM." << endl;


  ///////////////////////////////////////////////////////////////////////////////////////
  // update controls
  ///////////////////////////////////////////////////////////////////////////////////////

  //Si se pone el modo no causal, se activara siempre el modo backtrace. CADA VEZ ME PARECE MAS FACTIBLE LA OPCION DEL COMPUTO NO CAUSAL
  if(noncausalopt)
    backtraceopt = true;

  //Creamos la variable tamaño de entrada
  mrs_natural inputSize;
  if(micinputopt) //En el caso de que se use el micro
  {
    mrs_real micRate = 44100.0; //Fs da 44100Hz
    mrs_real length = 1000.0; //Longitud de captura del microfono en segundos (big number for "endless" capturing)
    audioflow->updControl("mrs_real/israte", micRate); //Damos valor de Fs al MarSystem global
    audioflow->updControl("AudioSource/micsrc/mrs_natural/nChannels", 2); //Definimos 2 canales
    audioflow->updControl("AudioSource/micsrc/mrs_bool/initAudio", true); //Inicializamos el sistema inicializando el AudioSource

    inputSize = (mrs_natural) ceil(length * micRate); //El tamaño del input seran los segundos totales por el numero de muestras por segundo. 
    //OJO, SI DEFINIMOS UN ARRAY DE ESTE VALOR DEAREMOS AL MICRO SIN RAM, EN EL CASO DE METERLO A UNO
  }
  else //Si se usa el modo normal, no micro
  {
    audioflow->updControl("mrs_natural/inSamples", HOPSIZE); //Damos valor de HopSize a todo el sistema
    audioflow->updControl("SoundFileSource/src/mrs_string/filename", sfName); //Damos el archivo de salida al marsystem de entrada
    inputSize = audioflow->getctrl("SoundFileSource/src/mrs_natural/size")->to<mrs_natural>(); //Cojemos el tamaño del archivo en samples y lo metemos en inputSize
  }

  //best result till now are using dB power Spectrum!
  //Utilizamos la tecnica de powerSpectrum basandonos en la magnitud
  beattracker->updControl("Series/onsetdetectionfunction/PowerSpectrum/pspk/mrs_string/spectrumType", "magnitude");

  //Utilizamos el flujo espectral de la forma definida por Dixon 
  beattracker->updControl("Series/onsetdetectionfunction/Flux/flux/mrs_string/mode", "DixonDAFX06");

  beattracker->updControl("mrs_natural/inSamples", HOPSIZE); //Pasamos HOPSIZE de nuevo al sistema de beattracker
  beattracker->updControl("Series/onsetdetectionfunction/ShiftInput/si/mrs_natural/winSize", WINSIZE); //Pasamos WINSIZE al sistema 

  mrs_real fsSrc = beattracker->getctrl("Series/onsetdetectionfunction/ShiftInput/si/mrs_real/israte")->to<mrs_real>(); //Hallamos la Fs del sistema de trackeo

  //induction time (in nr. of ticks) -> -1 because starting index on accumulator is 0 and it finnishes at accSize-1
  //So IBT's tick time notion starts also on 0 and finnishes on sound_file_size(in_frames)-1.

  //Se utilizara un ajuste del tamaño de la mitad del HOPSIZE
  mrs_natural adjustment = HOPSIZE / 2; //linked to BeatTimesSink
  //adjustment = (winSize_ - hopSize_) + floor((mrs_real) winSize_/2);

  //el numero de ticks de la etapa de induccion se basara en el numero de hopsize que caben en los samples totales de la cancion mas el pequeño ajuste
  //Los tick se ejecutan en el induction time
  mrs_natural inductionTickCount = ((mrs_natural) ceil((induction_time * fsSrc + adjustment) / HOPSIZE)) -1;
  //Para evitar que el tiempo de induccion sea mas grande que el propio track
  if((inputSize / HOPSIZE) < inductionTickCount)
    //Definiremos el tickCount de la etapa de induccion al tamaño en ticks del track total
    inductionTickCount = (inputSize / HOPSIZE) -1;

  beattracker->updControl("FlowThru/tempoinduction/TempoHypotheses/tempohyp/mrs_natural/inductionTime", inductionTickCount); //Damos al sistema de beattracking el valor en ticks de la etapa de induccion

  //Tamaño del acumulador igual al tiempo de induccion + 1 [0, tiempor_de_induccion]
  //beattracker->updControl("ShiftInput/acc/mrs_natural/winSize", inductionTickCount+1);


  //Tamaño del acumulador igual a 2 veces el tiempo de induccion, para mejorar el filtrado (Mayor que el tiempo de induccion)
  //(and to avoid strange slow memory behaviour with inductionTime=5)
  mrs_natural accSize = 2*inductionTickCount;
  if(accSize > (inputSize / HOPSIZE)) //Para evitar que el acumulador sea mas grande en tick que el propio track en ticks
     accSize = (inputSize / HOPSIZE) +1; //Lo ponemos del tamaño del track + 1, en ticks

  beattracker->updControl("ShiftInput/acc/mrs_natural/winSize", accSize);//Damos el valor del tamaño del acumulador de salida que se generara en el shiftInput. MIRAR BIEN COMO FUNCIONA EL SHIFTINPUT POR DENTRO

  //Hallamos el numero de picos encontrados en la ventana
  mrs_natural pkinS = tempoinduction->getctrl("Fanout/tempohypotheses/Series/tempo/Peaker/pkr/mrs_natural/onSamples")->to<mrs_natural>(); 
  //PREGUNTAR SOBRE LAS ECUACIONES
  mrs_natural peakEnd = (mrs_natural)((60.0 * fsSrc)/(minBPM_ * HOPSIZE)); //Nos da la posicion del ultimo pico encontrado, basandonos en los BPM minimos que hemos definido
  mrs_natural peakStart = (mrs_natural) ((60.0 * fsSrc)/(maxBPM_ * HOPSIZE));  //Nos da la posicion del primer pico encontrado, basandonos en los BPM maximos que hemos definido

  //mrs_real peakSpacing = ceil(((peakEnd-peakStart) * 4.0) / ((mrs_real)(MAX_BPM-MIN_BPM))) / pkinS;

  mrs_real peakSpacing = ((mrs_natural) (fsSrc/HOPSIZE) * (1.0-(60.0/64.0))) / (pkinS * 1.0); //Hallamos el espaciado maximo entre picos (4BPMs at 60BPM resolution) se expresa en porcentaje
  //mrs_real peakSpacing = ((mrs_natural) (peakEnd-peakStart) / (2*BPM_HYPOTHESES)) / (pkinS * 1.0);  //nrPeaks <= 2*nrBPMs

  //cout << "PkinS: " << pkinS << "; peakEnd: " << peakEnd << "; peakStart: " << peakStart << "; peakSpacing: " << peakSpacing << endl;

  //Definimos los controles para el espaciado entre picos, el humbral que existe entre la misma informacion en RMS, el lugar donde empezar a mirar picos y el lugar donde dejar de buscarlos (dentro del array que esta enventanado)
  tempoinduction->updControl("Fanout/tempohypotheses/Series/tempo/Peaker/pkr/mrs_real/peakSpacing", peakSpacing);
  tempoinduction->updControl("Fanout/tempohypotheses/Series/tempo/Peaker/pkr/mrs_real/peakStrength", 0.75); //0.75
  tempoinduction->updControl("Fanout/tempohypotheses/Series/tempo/Peaker/pkr/mrs_natural/peakStart", peakStart);
  tempoinduction->updControl("Fanout/tempohypotheses/Series/tempo/Peaker/pkr/mrs_natural/peakEnd", peakEnd);
  // invalid control
  //tempoinduction->updControl("Fanout/tempohypotheses/Series/tempo/Peaker/pkr/mrs_real/peakGain", 2.0);

  //En la autocorrelaccion solo miramos la mitad de la ventana, de ahi, estos valores
  tempoinduction->updControl("Fanout/tempohypotheses/Series/tempo/AutoCorrelation/acf/mrs_real/lowCutoff", 0.5);
  tempoinduction->updControl("Fanout/tempohypotheses/Series/tempo/AutoCorrelation/acf/mrs_real/highCutoff", 1.0);
  //Lo que no es util, lo borramos
  beattracker->updControl("ShiftInput/acc/mrs_real/lowCleanLimit", 0.0);
  beattracker->updControl("ShiftInput/acc/mrs_real/highCleanLimit", 0.5);

  //A los nMaximos que va a encontrar MaxArgMax para generas a posteriori hipotesis de tempo, le damos los N BPM_HYPOTHESES que queriamos
  tempoinduction->updControl("Fanout/tempohypotheses/Series/tempo/MaxArgMax/mxr/mrs_natural/nMaximums", BPM_HYPOTHESES);

  //
  mrs_natural minPeriod = (mrs_natural) floor(60.0 / (maxBPM_ * HOPSIZE) * fsSrc); //Nos devuelve el numero de ticks para el periodo minimo  NOSE PORQUE, ES LA MISMA OPERACION QUE EL peakEnd y el peakStart
  mrs_natural maxPeriod = (mrs_natural) ceil(60.0 / (minBPM_ * HOPSIZE) * fsSrc); //Nos deveulve el numero maximo de ticks para el periodo maximo

  mrs_natural lookAheadSamples = 20; //Numero minimo para mirar hacia atrar; //Debe ser mayor que 9 por incosistencias que hay al principio de la ventana filtrada, 
  //ya sabemos que se dan los problemas del efecto bordes, por la convolucion.
  mrs_real thres = 1.1;//Este umbral le sirve al peaker saber que picos cojer, si estan por encima de threshold*media señal
  //mrs_natural lookAheadSamples = 6;
  //mrs_real thres = 1.2;

  //Se dan valores al PeakerOnsets
  tempoinduction->updControl("Fanout/tempohypotheses/Series/phase/PeakerOnset/pkronset/mrs_natural/lookAheadSamples", lookAheadSamples);
  tempoinduction->updControl("Fanout/tempohypotheses/Series/phase/PeakerOnset/pkronset/mrs_real/threshold", thres);

  //Se da el valor de hipotesis de fase posibles por periodo estimado
  tempoinduction->updControl("Fanout/tempohypotheses/Series/phase/OnsetTimes/onsettimes/mrs_natural/n1stOnsets", PHASE_HYPOTHESES);

  
  for(int i = 0; i < NR_AGENTS; ++i)
  {
    ostringstream oss, oss2;
    oss << "agent" << i;
    oss2 << "Agent" << i;
    //Se da la funcion de score a cada agente
    beattracker->updControl("Fanout/agentpool/BeatAgent/"+oss.str()+"/mrs_string/scoreFunc", score_function);

    //Se dan los margenes internos, y externos a cada agente
    beattracker->updControl("Fanout/agentpool/BeatAgent/"+oss.str()+"/mrs_real/lftOutterMargin", LFT_OUTTER_MARGIN);
    beattracker->updControl("Fanout/agentpool/BeatAgent/"+oss.str()+"/mrs_real/rgtOutterMargin", RGT_OUTTER_MARGIN);
    beattracker->updControl("Fanout/agentpool/BeatAgent/"+oss.str()+"/mrs_real/innerMargin", INNER_MARGIN);

    //Parece que solo se le cambia el nombre o algo asi, no es muy relevante
    beattracker->updControl("Fanout/agentpool/BeatAgent/"+oss.str()+"/mrs_string/identity", oss2.str());
  }

  beattracker->updControl("FlowThru/initialhypotheses/PhaseLock/phaselock/mrs_real/triggerBestScoreFactor", TRIGGER_BEST_FACTOR); //Valor proporcional que se da a los agentes creados respecto de la puntuacion padre

  //Si tenemos el modo induccion en autoreset o en repeated-reset o  en random-reset o en givetransi... el beatReferee se reseteara en cada nueva induccion
  if(strcmp(induction_mode.c_str(), "auto-reset") == 0 || strcmp(induction_mode.c_str(), "repeated-reset") == 0
      || strcmp(induction_mode.c_str(), "random-reset") == 0)
    beattracker->updControl("BeatReferee/br/mrs_bool/resetAfterNewInduction", true);
  //si esta puesto en auto-regen o repeated-regen o en random-regen o en givetransitions-regen, no se reseteara
  else if(strcmp(induction_mode.c_str(), "auto-regen") == 0 || strcmp(induction_mode.c_str(), "repeated-regen") == 0
          || strcmp(induction_mode.c_str(), "random-regen") == 0)
    beattracker->updControl("BeatReferee/br/mrs_bool/resetAfterNewInduction", false);
  
  //Vamos a dar todos los parametros que hemos definido al inicio que se utilizaran para tratar, crear y eliminar a los agentes
  beattracker->updControl("BeatReferee/br/mrs_real/srcFs", fsSrc);
  beattracker->updControl("BeatReferee/br/mrs_natural/minPeriod", minPeriod);
  beattracker->updControl("BeatReferee/br/mrs_natural/maxPeriod", maxPeriod);
  beattracker->updControl("BeatReferee/br/mrs_real/obsoleteFactor", OBSOLETE_FACTOR);
  beattracker->updControl("BeatReferee/br/mrs_natural/lostFactor", LOST_FACTOR);
  beattracker->updControl("BeatReferee/br/mrs_real/childrenScoreFactor", CHILDREN_SCORE_FACTOR);
  beattracker->updControl("BeatReferee/br/mrs_real/bestFactor", BEST_FACTOR);
  beattracker->updControl("BeatReferee/br/mrs_real/corFactor", CORRECTION_FACTOR);
  beattracker->updControl("BeatReferee/br/mrs_real/child1Factor", (mrs_real) CHILD1_FACTOR);
  beattracker->updControl("BeatReferee/br/mrs_real/child2Factor", (mrs_real) CHILD2_FACTOR);
  beattracker->updControl("BeatReferee/br/mrs_real/child3Factor", (mrs_real) CHILD3_FACTOR);
  beattracker->updControl("BeatReferee/br/mrs_natural/eqPeriod", EQ_PERIOD);
  beattracker->updControl("BeatReferee/br/mrs_natural/eqPhase", EQ_PHASE);
  beattracker->updControl("BeatReferee/br/mrs_bool/backtrace", backtraceopt); //Se usa en el modo offline
  beattracker->updControl("BeatReferee/br/mrs_natural/soundFileSize", (mrs_natural) ((inputSize / HOPSIZE))); //Tamaño en ticks del track
  beattracker->updControl("BeatReferee/br/mrs_bool/nonCausal", noncausalopt);
  beattracker->updControl("BeatReferee/br/mrs_real/beatTransitionTol", BEAT_TRANSITION_TOL);
  beattracker->updControl("BeatReferee/br/mrs_real/supervisedTriggerThres", sup_thres); //Humbral que rebaja las puntuaciones un cierto valor para que no sean iguales
  //if(noncausalopt) beattracker->updControl("BeatReferee/br/mrs_bool/resetAfterNewInduction", false);
  //else beattracker->updControl("BeatReferee/br/mrs_bool/resetAfterNewInduction", true);
  //map induction_mode to previous name convention

  //Segun el modo que se halla seleccionado, damos el nombre correspondiente.
  if(strcmp(induction_mode.c_str(), "auto-reset") == 0 || strcmp(induction_mode.c_str(), "auto-regen") == 0)
    induction_mode = "supervised";
  else if(strcmp(induction_mode.c_str(), "repeated-reset") == 0 || strcmp(induction_mode.c_str(), "repeated-regen") == 0)
    induction_mode = "repeated";
  else if(strcmp(induction_mode.c_str(), "random-reset") == 0 || strcmp(induction_mode.c_str(), "random-regen") == 0)
    induction_mode = "random";
  beattracker->updControl("BeatReferee/br/mrs_string/inductionMode", induction_mode);

  //set the file with the groundtruth times of trigger
  //En el caso de que el modo de induccion fuese el de give trasistion reset/regen

  //SonicVisualiser Controls:
  /*
  if(sonicOutFlux)
  {
  	beattracker->updControl("Series/onsetdetectionfunction/SonicVisualiserSink/sonicsink/mrs_string/mode", "frames");
  	beattracker->updControl("Series/onsetdetectionfunction/SonicVisualiserSink/sonicsink/mrs_string/destFileName", path.str() + "_onsetFunction.txt");

  	//if(backtraceopt)
  	//	beattracker->updControl("Series/onsetdetectionfunction/SonicVisualiserSink/sonicsink/mrs_natural/offset", inductionTickCount);
  	//else
  	//	beattracker->updControl("Series/onsetdetectionfunction/SonicVisualiserSink/sonicsink/mrs_natural/offset", 0);
  }
  if(sonicOutFluxFilter)
  {
  	beattracker->updControl("Series/normfiltering/SonicVisualiserSink/sonicsinkfilt/mrs_string/mode", "frames");
  	beattracker->updControl("Series/normfiltering/SonicVisualiserSink/sonicsinkfilt/mrs_string/destFileName", path.str() + "_onsetFunctionFilt.txt");
  }
  */


  //Inicializamos los 2 filtros de orden 3, con una configuracion de cambio de fase 0
  //Estos coeficientes estan cogidos de MATLAB de un filtro paso bajo Butterworth(2, 0.28) --> Orden 3, frecuencia de corte 0.28*Fs
  realvec bcoeffs(1,3);
  bcoeffs(0) = 0.1174;
  bcoeffs(1) = 0.2347;
  bcoeffs(2) = 0.1174;
  beattracker->updControl("Series/normfiltering/Filter/filt1/mrs_realvec/ncoeffs", bcoeffs);
  realvec acoeffs(1,3);
  acoeffs(0) = 1.0;
  acoeffs(1) = -0.8252;
  acoeffs(2) = 0.2946;
  beattracker->updControl("Series/normfiltering/Filter/filt1/mrs_realvec/dcoeffs", acoeffs);

  /*
  // Coefficients taken from MATLAB butter(2, 0.18)
  realvec bcoeffs(1,3);
  bcoeffs(0) = 0.0564;
  bcoeffs(1) = 0.1129;
  bcoeffs(2) = 0.0564;
  beattracker->updControl("Series/normfiltering/Filter/filt1/mrs_realvec/ncoeffs", bcoeffs);
  realvec acoeffs(1,3);
  acoeffs(0) = 1.0000;
  acoeffs(1) = -1.2247;
  acoeffs(2) = 0.4504;
  beattracker->updControl("Series/normfiltering/Filter/filt1/mrs_realvec/dcoeffs", acoeffs);
  */

  /*
  // Coefficients taken from MATLAB butter(2, 0.1)
  realvec bcoeffs(1,3);
  bcoeffs(0) = 0.0201;
  bcoeffs(1) = 0.0402;
  bcoeffs(2) = 0.0201;
  beattracker->updControl("Series/normfiltering/Filter/filt1/mrs_realvec/ncoeffs", bcoeffs);
  realvec acoeffs(1,3);
  acoeffs(0) = 1.0000;
  acoeffs(1) = -1.5610;
  acoeffs(2) = 0.6414;
  beattracker->updControl("Series/normfiltering/Filter/filt1/mrs_realvec/dcoeffs", acoeffs);
  */

  //ostringstream onsetFunction [-> for inputing sonicvisualiser spectral flux];
  //onsetFunction << "C:\\Users\\Joao Lobato\\Desktop\\onsetFunctions\\" << outputFile.nameNoExt() << "_vamp_vamp-aubio_aubioonset_detectionfunction.csv";

  //MATLAB Engine inits
  //used for autocorrelation.m
  //mrs_natural winSize = WINSIZE;
  //mrs_natural hopSize = HOPSIZE;
  //MATLAB_EVAL("clear;");
  //MATLAB_EVAL("Onsets = zeros(1,862);");
  //MATLAB_EVAL("Onsets2 = zeros(1,862);");
  //MATLAB_EVAL("Induction = zeros(1,862);");
  //MATLAB_EVAL("Induction2 = zeros(1,862);");
  //MATLAB_EVAL("FluxTS = [];");
  //MATLAB_EVAL("srcAudio = [];");
  //MATLAB_EVAL("FinalBeats=[];");
  //MATLAB_PUT(winSize, "winSize");
  //MATLAB_PUT(hopSize, "hopSize");
  /*
  MATLAB_PUT(induction_time, "timmbing");
  MATLAB_PUT(fsSrc, "SrcFs");
  MATLAB_PUT(inductionTickCount, "inductionTickCount");
  MATLAB_EVAL("FluxTS = [];");
  MATLAB_EVAL("FinalTS = [];");
  MATLAB_EVAL("BeatAgentTS=[];");
  MATLAB_EVAL("BeatAgentsTS=[];");
  MATLAB_EVAL("bestAgentScore=[];");
  MATLAB_EVAL("Flux_FilterTS=[];");
  */

  ///////////////////////////////////////////////////////////////////////////////////////
  //process input file (till EOF)
  ///////////////////////////////////////////////////////////////////////////////////////
  //Iniciamos un contador de frames (TICKS)
  mrs_natural frameCount = 0;

  inputSize = (mrs_natural) (inputSize / HOPSIZE); //El tamaño del track de entrada se pone en funcion de los ticks 

  //Como en el backtrace se analizan 2 veces la ventana de induccion, al tamaño total habra que adherirle el tiempo de induccion en ticks
  if(backtraceopt)
    inputSize += inductionTickCount;

  #if DEBUG_MODE
    cout << "Micro: " << micinputopt << endl; 
    cout << "induction_time: " << induction_time << endl;
  #endif
  //while(IBTsystem->getctrl("mrs_bool/hasData")->to<mrs_bool>())
  //Mientras el conteo de frames, es decir de HOPSIZES sea menor que el total, seguir ejecutando
  while(frameCount <= inputSize)
  {
    //Se actualiza el Marsystem normal
    audioflow->tick();
    //En el primer tick
    if(frameCount == 1)
    {
      if(micinputopt) //Miramos si se ha marcado la opcion de micro
      //Lo decimos por pantalla
      cout << "Capturing Audio......" << endl;
      //Comienza la induccion
      cout << "Induction........";
    }

    //Justo despues de la induccion
    if(frameCount == inductionTickCount) //CHANGE TO TRIGGER!!!
    {
      //cout << "done" << endl;

      //Si esta lo de tratar 2 veces la ventana de induccion pero no el micro, porque son incompatibles
      if(backtraceopt && !micinputopt)
      {
        //Retrocedemos a la posicion 0 del archivo de audio, para volver a tratar la induccion
        audioflow->updControl("SoundFileSource/src/mrs_natural/pos", 0);
      }

      if(!noncausalopt) //Si estamos en modo causal, lo decimos
      {
        cout << "Real-Time Beat Tracking........" << endl;
      }
      else //Sino, lo decimos tambien
      {
        cout << "Off-Line Beat Tracking........" << endl;
      }
    }
    //Display percentage of processing complete...
    //printf("  %d % \r", (mrs_natural) frameCount*100/inputSize);
    //cout << (mrs_natural) frameCount*100/inputSize << "%" << endl;
    beatDetec = audioflow->getctrl("FlowThru/beattracker/BeatReferee/br/mrs_real/beatDetected")->to<mrs_real>();

    #if DEBUG_MODE
      cout << "beatDetec: " << beatDetec << endl;
    #endif
    if (beatDetec == 1)
    {
      DWORD bytes_written, total_bytes_written = 0;
      char bytes_to_send[1];
      bytes_to_send[0] = '1';
      #if ERROR_DEBUG_MODE
        //fprintf(stderr, "Sending bytes...");
        cout << "Sending bytes..." << endl;
      #endif
      if(!WriteFile(handlerSerialComm, bytes_to_send, 1, &bytes_written, NULL))//If not posible to write on PORT/File
      { 
        #if ERROR_DEBUG_MODE
          //fprintf(stderr, "Error\n");
          cout << "Not possible write on PORT COM" << endl;
        #endif
        CloseHandle(handlerSerialComm);
      }   
      #if ERROR_DEBUG_MODE
        //fprintf(stderr, "%d bytes written\n", bytes_written);
        cout << bytes_written << " bytes written" << endl;
      #endif
    }

    frameCount++; //Actualizamos el contador de frames/ticks
  }
  delete audioflow; //Nos cargamos el MarSystem
  cout << "Finish!" << endl; //Acabamos
}


int
main(int argc, const char **argv)
{
  MRSDIAG("SF+ACF.cpp - Main");

  //Si se escribe ibt, se printeara el help corto. SE PUEDE QUITAR
  if (argc == 1)
  {
    return printUsage("ibt");
  }

  initOptions(); //Hacemos la inicializacion de las opciones. EN NUESTRO CASO, SE INICIARAN SOLO COMO QUERAMOS NOSOTROS Y CON LO QUE SE PONGA EN CONSOLA O SI LO HACEMOS DE MODO QUE EL PC SE COMUNIQUE CON EL MICRO,
                  //PODRIAMOS DEJAR ALGUNAS OPCIONES
  cmd_options.readOptions(argc,argv); //Leer opciones. LO MISMO QUE ANTES
  loadOptions(); //Carga las opciones seleccionadas. 
 
  // Printeamos el Help bueno si se indica, con -h
  if (helpopt)
    return printHelp("ibt");

  if (usageopt) //Printea la help corta
    return printUsage("ibt");

  FileName execFileName(argv[0]); //El archivo del track es el primer argumento de consola
  execPath = execFileName.path(); //Sacamos la ruta

  vector<string> soundfiles = cmd_options.getRemaining(); //Coje todo lo que halla escrito en consola que reste
  //Definimos variables de nombre de archivo(track) y archivo txt(output)
  mrs_string sfName = "";

  //Si todo lo escrito en consola que restaba tiene algun archivo al menos, el primero sera el nombre del archivo de audio
  if (soundfiles.size() > 0)
    sfName = soundfiles[0];
  
  //cout << "COM is " << PORT_COM << endl;

  //Initialise COM

  // Declare variables and structures for serial Communication

  #if ERROR_DEBUG_MODE
      // Open the highest available serial port number
      //fprintf(stderr, "Opening serial port...");
      cout << "Opening serial port..." << endl;
  #endif

  std::string str_COM ("\\\\.\\COM");
  if (PORT_COM.size() == 1) PORT_COM = '0' + PORT_COM;
  str_COM += PORT_COM;
  cout << "Sending data out into Port " + PORT_COM << endl;

  /////////////////////////////////////strncat(str, &PORT_COM[0], 1);
  /////////////////////////////////////strncat(str, &PORT_COM[1], 1);
  //cout << "String de PUERTO COM: " << str_COM << endl;
  //Create serial handler comunicator
  handlerSerialComm = CreateFile(
              str_COM.c_str(),                    //"\\\\.\\COM10",         //FileName, must have the number of the COM PORT     <--- Cambiar a str_COM si funciona
              GENERIC_WRITE,          //Write
              0,                      //Not share
              NULL,                   //Default Security
              OPEN_EXISTING,          //Open file, only if exists
              FILE_ATTRIBUTE_NORMAL,  //Normal file
              NULL                    //no attribute template
              );
  if (handlerSerialComm == INVALID_HANDLE_VALUE){ //If it fails
    #if ERROR_DEBUG_MODE
      //fprintf(stderr, "Error\n"); //Error
      cout << "Problem, invalid handler value" << endl;
    #endif
  }
  else{
    #if ERROR_DEBUG_MODE
      //fprintf(stderr, "OK\n"); //Else, print OK
      cout << "Correct handler value" << endl;
    #endif
  }

  // Set device parameters (115200 baud, 1 start bit,
  // 1 stop bit, no parity)
  dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
  if (GetCommState(handlerSerialComm, &dcbSerialParams) == 0){ //If the state of communication is 
      #if ERROR_DEBUG_MODE
          //fprintf(stderr, "Error getting device state\n"); //Error
          cout << "Problem getting handler serial communication parameters" << endl;
      #endif
      CloseHandle(handlerSerialComm); //Close communication
  }
    
  //Set communication Params
  dcbSerialParams.BaudRate = CBR_115200;
  dcbSerialParams.ByteSize = 8;
  dcbSerialParams.StopBits = ONESTOPBIT;
  dcbSerialParams.Parity = NOPARITY;
  if(SetCommState(handlerSerialComm, &dcbSerialParams) == 0){  //If is not possible to set communication parameters
      #if ERROR_DEBUG_MODE
          //fprintf(stderr, "Error setting device parameters\n"); //Error
          cout << "Problem setting handler serial communication parameters" << endl;
      #endif
      CloseHandle(handlerSerialComm);//Close communication
  }

  // Set COM port timeout settings
  timeouts.ReadIntervalTimeout = 50;
  timeouts.ReadTotalTimeoutConstant = 50;
  timeouts.ReadTotalTimeoutMultiplier = 10;
  timeouts.WriteTotalTimeoutConstant = 50;
  timeouts.WriteTotalTimeoutMultiplier = 10;
  if(SetCommTimeouts(handlerSerialComm, &timeouts) == 0){ //If not posible to set Timeout 
      #if ERROR_DEBUG_MODE
          //fprintf(stderr, "Error setting timeouts\n"); //Error
          cout << "Problem setting handler serial communication timeouts" << endl;
      #endif
      CloseHandle(handlerSerialComm); //Close communication
  }
 
  //Creamos un "objeto" archivo con el nombre del archivo del track
  FileName outputFile(sfName);
  //Si el archivo de entrada (que tiene el mismo nombre que el de salida) no es del tipo de ninguna de estas 4 extensiones y no se ha indicado el uso del micro. ESTO LO PODEMOS QUITAR
  if(strcmp(outputFile.ext().c_str(), "wav") == 0 || strcmp(outputFile.ext().c_str(), "mp3") == 0 ||
      strcmp(outputFile.ext().c_str(), "au") == 0 || strcmp(outputFile.ext().c_str(), "raw") == 0 ||
      micinputopt)
  { 
    cout << "InductionLength: " << induction_time << "secs" << endl;
    if(micinputopt) //Si hemos inicializado el micro, lo decimos 
    {
      sfName = "mic";
      cout << "SoundFile: Captured from Microphone" << endl;
      ibt(sfName); //Hacemos ibt con el microfono
    }
    else //Sino, hacemos ibt normal
    {
      cout << "SoundFile: " << sfName << endl;
      if(existsFile(sfName))
      {
        ibt(sfName);
      }
      else 
      {
        cout << "The file extension is not correct" << endl;
        return(0);
      }
    }
  } 

  // Close serial port
  #if ERROR_DEBUG_MODE
      //fprintf(stderr, "Closing serial port...");
      cout << "Closing serial port..." << endl;
  #endif
  if (CloseHandle(handlerSerialComm) == 0) //If not posible to close COM 
  {  
      #if ERROR_DEBUG_MODE
          //fprintf(stderr, "Error\n");
          cout << "Error closing COM port" << endl;
      #endif
  }
  #if ERROR_DEBUG_MODE
      //fprintf(stderr, "OK\n");
      cout << "COM port closed" << endl;
  #endif
  // exit normally

  cout << "All Done!" << endl;
  return(0);
}
