highway {

    agent = "RouteAgent";
//    agent = "ORCAAgent";

    net {
//        folder = "nets/kosik/";
        folder = "nets/x-junction/";
//        folder = "nets/junction-big/";
//        folder = "nets/highway-bidirectional/";
//        folder = "nets/highway-straight/";
    }

    vis {               //visualization
        isOn = true;
    }

    // Dashboard configuration
    dashboard {
        simulators {
            OpenDS {
                launch = "out/artifacts/simulator_jar/run_simulator.sh felfest_demo.xml";
            }
            OpenDS2 {
                launch = "out/artifacts/simulator_jar/run_simulator.sh felfest_demo2.xml";
            }
            SimulatorLite {
                launch = "launchers/simulator-lite.sh"
            }
            Empty {
                launch = "launchers/empty.sh"
            }

        }
        // simulatorToRun is an array of simulators you wish to run,
        // Set of simulators to run can be seen above, note that you need to create a script for each od the simlators.
        //The path to the relevant scripts is to be specified above too.

//        simulatorsToRun = []; //if no simulator, LocalSimulator is used - perfect execution of plans
        simulatorsToRun = ["SimulatorLite"];
//        simulatorsToRun = ["OpenDS"];
//        simulatorsToRun = ["Empty"];
    }

    netLayer {
        lane {
            view = true;
            width = 10;
        }
        edge {
            view = false;
            width = 1;
        }
        crossRoad {
            view = false;
            width = 1;
        }
    }
}
