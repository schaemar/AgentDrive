highway {
    seed = 0;                          // random number generator seed
    simulationDuration = -1;      // - 1 = infinity
    simulationSpeed = 1.0;             //not relevant
    timestep = 10; //ms               //not relevant

//    agent = "SDAgent";
//   agent = "RouteAgent";
//    agent = "ORCAAgent";
   agent = "DESDAgent";

    net {
//        folder = "nets/kosik/";
//        folder = "nets/x-junction/";
 //    folder = "nets/junction-big/";
 //       folder = "nets/highway-bidirectional/";
   //     folder = "nets/highway-straight/";
//       folder = "nets/test90/";
        folder = "nets/circle/";
 //       folder = "nets/CharlesSquare";

    }
    safeDistanceAgent {
        safetyReserveDistance = 20.0;     // [m] - safety distance offset (including vehicle length and separation gap)
        narrowingModeActive = false;
        distanceToActivateNM = 100.0;
        // [m] - when distance to obstacle is smaller than this value NARROWING MODE is activated

        maneuvers {
            //maneuver is discratization unit
            //available maneuvers are Straight, Acceleration, Deacceleration, LaneLeft, LaneRight
            //parameters of maneuvers are following

            laneChangeManeuverDuration = 1.0; // [s]
            straightManeuverDuration = 0.5;     //[s]
            accelerationManeuverDuration = 0.3;   //[s]
            deaccelerationManueverDuration = 0.3;   //[s]
            acceleration = 4.0;                     //[m/s^2]
            deacceleration = -6.0;                  //[m/s^2]
            maximalSpeed = 25.0;                    //[m/s]
            maxSpeedVariance = 0.30                     //[%]
        }

    }
    protobuf {
        isOn = true;
        uri = "socket://localhost:2222";//"socket://192.168.0.100:2222"; //"socket://147.32.83.240:2222";
        protocol = "simplan";

    }
    rvo {
        visibilityGraphRadius = 1000;
        timestep = 1.0;

        agent {
            agentMaxSpeed = 15.0;
            radiusConstant = 1.0;
            timeHorizon = 100.0;
            timeHorizonObst = 100.0;
            orcaSpeedConstant = 1.0;
            maxNeighbors = 100;
        }



        vis {
            showInflatedObstacles = true;
            timestep = 1;
            maxtime = 100000;
        }
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

      simulatorsToRun = []; //if no simulator, LocalSimulator is used - perfect execution of plans
 //    simulatorsToRun = ["SimulatorLite"];
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
    SimulatorLocal{
        timestep=1;
    }
}
