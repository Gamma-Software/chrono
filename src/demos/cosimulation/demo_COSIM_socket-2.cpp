// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora
// =============================================================================
//
// Demo on how to implement a basic cosimulation framework where data is passed
// back-forth between Chrono and Simulink.
//
// This example needs test_cosimulation.mdl to be load and run in Simulink.
//
// =============================================================================

#include "chrono/core/ChLog.h"
#include "chrono/core/ChMatrixDynamic.h"

#include "chrono_cosimulation/ChCosimulation.h"
#include "chrono_cosimulation/ChExceptionSocket.h"

using namespace chrono;
using namespace chrono::cosimul;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // To write something to the console, use the chrono::GetLog()

    GetLog() << "CHRONO demo about cosimulation \n\n";
    GetLog() << "NOTE! This requires that you also run a copy of Simulink. \n\n";

    try {
        // Test 1
        // Create a cosimulation interface and exchange some data
        // with Simulink.

        int PORTNUM = 50009;
        int input_nb = 1;
        int output_nb = 2;

        ChMatrixDynamic<int> data_in(input_nb*15, 1);
        ChMatrixDynamic<int> data(input_nb*3, 1);
        ChMatrixDynamic<int> data_out(output_nb, 1);

        // 1) Add a socket framework object
        ChSocketFramework socket_tools;

        // 2) Create the cosimulation interface:

        ChCosimulation cosimul_interface(socket_tools,
            input_nb*15,   // n.input values from Simulink
            output_nb);  // n.output values to Simulink

        // 3) Wait client (Simulink) to connect...

        GetLog() << " *** Waiting Simulink to start... *** \n     (load 'data/cosimulation/test_cosimulation.mdl' in "
            "Simulink and press Start...)\n\n";

        cosimul_interface.WaitConnection(PORTNUM);

        int mytime = 0;
        int histime = 0;
        int iteration = 0;

        while (true) {
            if (cosimul_interface.ReceiveData(histime, &data_in))
            {
                int iteration = 0;
                bool next_data = false;
                do
                {
                    data(iteration + 0) = data_in(9 + 15 * iteration);
                    data(iteration + 1) = data_in(6 + 15 * iteration);
                    data(iteration + 2) = data_in(13 + 15 * iteration);
                    GetLog() << "--- synchronization at time: " << data(0) << "\n\n";
                    GetLog() << "id " << data(1) << "\n";
                    GetLog() << "value " << data(2) << "\n";
                    next_data = data_in(14 + 15 * iteration) != 0;
                    iteration++;
                } while (next_data);

            }
        }

    }
    catch (ChExceptionSocket exception) {
        GetLog() << " ERRROR with socket system: \n" << exception.what() << "\n";
    }

    return 0;
}
