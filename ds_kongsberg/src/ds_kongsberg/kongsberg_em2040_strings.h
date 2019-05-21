//
// Created by jvaccaro on 4/11/19.
//
/**
* Copyright 2019 Woods Hole Oceanographic Institution
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
*    may be used to endorse or promote products derived from this software
*    without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef SENTRY_WS_KONGSBERGEM2040_STRINGS_H
#define SENTRY_WS_KONGSBERGEM2040_STRINGS_H

namespace ds_kongsberg{

enum RET_CODE{
  SUCCESS = 0,
  WRONG_SOUNDER_NAME,
  BAD_CODE
};

enum K_TO_SIS{
  STATUS_IPI = 12,
  READY = 39,
  // PING AND SETTINGS
  STATUS_IPU = 616,
  // PU PARAMS
  FILEIMPORTED = 476, // 451,476
  PARAMSREAD = 459,
  // VALUES
  VALUES = 401
} ;

// IPI LIST

// IPU LIST

// PARAMS LIST

// VALUES LIST

enum SIS_TO_K{
  START = 13,
  SET_READY = 451,
  // PING
  START_PING = 458,
  STOP_PING = 457,
  LOG_IOP_SVP = 454,
  // SETTINGS
  WC_OFF = 460,
  WC_ON = 461,
  STAVEGEN_OFF = 462,
  STAVEGEN_ON = 463,
  SCOPEDATA_OFF = 464,
  SCOPEDATA_ON = 465,
  PIPETRACK_OFF = 466,
  PIPETRACK_ON = 467,
  // PU PARAMS
  IMPORTFILE = 473, // 450,473
  READPARAMS = 476, // 451,476
  EXPORTPARAM = 472, // 450,472
  // RUNTIME VALUES
  GETVALUES = 261,
  SETVALUES = 262,
  TERMINATE = 24
} ;


// Source: Interface specification K-controller and SIS
// DETECTION OF SOUNDERS
// --> $KSSIS,12,NAME=xyz~~IP=xyz....$$$NAME=xyz... // Each sounder detected will be separated with «$$$», and each
                                                    // token/value pair is separated with «~~»
// <-- $KSSIS,13,SOUNDER_NAME  // A sounder is selected for start.
// --> $KSSIS,39,SOUNDER_NAME  // Sounder is ready for use, see also <$KSSIS,616,...> for more information on status
// <-- $KSSIS,451,SOUNDER_NAME // Operator selects a sounder, as active in case several has been detected.

// CHECK THE RESULTS OF ANY CHANGES
// --> $KSSIS,616,SOUNDER_NAME,$IPU:...$$$) // #IPU is received from PU every sec. and contains pinging status etc.

// PINGING
// <-- $KSSIS,458,SOUNDER_NAME // Start pinging
// <-- $KSSIS,457,SOUNDER_NAME // Stop pinging
// <-- $KSSIS,457,SOUNDER_NAME  // Stop pinging
// LOGGING
// <-- $KSSIS,454,SOUNDER_NAME  // When starting new logged files (i.e. lines) the KM standard is to begin each file
                                // (.kmall) with the #IIP #IOP #SVP datagrams. To trigger the PU to send these datagrams
                                // in the datastream, KSSIS_454 is sent to the K-Controller
// WATERCOLUMN
// <-- $KSSIS,460,SOUNDER_NAME // Turn generation of watercolumn data for logging and/or display off in PU.
// <-- $KSSIS,461,SOUNDER_NAME // Turn generation of watercolumn data for logging and/or display on in PU.
// STAVE DATA
// <-- $KSSIS,462,SOUNDER_NAME // Stave data generation off
// <-- $KSSIS,463,SOUNDER_NAME // Stave data generation on
// SCOPE DATA
// <-- $KSSIS,464,SOUNDER_NAME // Scope data off, for all beams
// <-- $KSSIS,465,SOUNDER_NAME,BEAM_NO=### // Scope data on, for specific Beam
// PIPE TRACKER
// <-- $KSSIS,466,SOUNDER_NAME // Pipe tracker, off
// <-- $KSSIS,467,SOUNDER_NAME,PIPE_TRACKER_DATA // Pipe tracker, on, with optional data

// PARAMETERS
// Import PU param from file : Changes to apply
// <-- $KSSIS,450,473,file name, // Specify file to import
// --> $KSSIS,451,476,SOUNDER_NAME // Open file and retrieve contained sounder ID
// <-- $KSSIS,450,476,SOUNDER_NAME // Use retrieved ID or set new (existing or dumm, -operator selection in SIS)
// --> $KSSIS,459,SOUNDER_NAME,<#IPI datagram>$$$, ...) // Read parameters from file and update for selected sounder id.
                                                        // If this is a dummy sounder, instantiate it first (see comment on SIS side).
// This may be a dummy sounder which must be instantiated. (Dummy sounders are sounders not detected on the network,
// but only used for pre-config and test purposes. As such they can not be started.)
// <-- $KSSIS,450,472,file name,Comments) // PU parameters for active selected sounder is stored together with set
                                          // comments in file with set name.

// RUNTIME VALUES
// <-- $KSSIS,261,SOUNDER_NAME // Request values
// Previous set values are gathered from DB
// --> $KSSIS,401,SOUNDER_NAME,JSON_GUI_DATA // Values returned
// <-- $KSSIS,262,SOUNDER_NAME,TOKEN=NEW_VAL, ... // Set one or more particular values
// K-Controller sets value in PU
// --> $KSSIS,401,SOUNDER_NAME,JSON_GUI_DATA // K-Controller confirms new value

// TERMINATE K CONTROLLER
// <-- $KSSIS,24  // Send to Terminate K-Controller (end process)
}

#endif //SENTRY_WS_KONGSBERGEM2040_STRINGS_H
