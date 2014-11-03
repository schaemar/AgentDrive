highway {

//    agent = "SDAgent";
    agent = "RouteAgent";
//    agent = "ORCAAgent";

    net {
//        folder = "nets/junction-big/";
        folder = "nets/highway-straight/";
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

        }

//        simulatorsToRun = []; //if no simulator, LocalSimulator is used - perfect execution of plans
        simulatorsToRun = ["SimulatorLite"];
    }

    netLayer {
        lane {
            view = true;
            width = 4;
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
