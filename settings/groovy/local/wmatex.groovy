/**
 * My local highway configuration
 * Created by wmatex on 21.7.14.
 */
highway {
    //agent = "RouteAgent";
    agent = "ADPPAgent";
//    agent = "GSDAgent";
    rvo {
        agent {
            randomRoutes = false;
        }
    }
    SimulatorLocal {
        timestep = 1;
    }
    net {
//        folder = "nets/highway-straight";
//        folder = "nets/CharlesSquare";
//        folder = "nets/junction-big";
//        folder = "nets/graz";
//        folder = "nets/pisek";
//        folder = "nets/test90";
//        folder = "nets/super-collision";
//        folder = "nets/artificialX-junction";
//        folder = "nets/artificialT-junction";
//        folder = "nets/artificialX-junction-smaller";
//        folder = "nets/artificialHighway-funnel";
//        folder = "nets/simple"

//        folder = "nets/experiments/X-junction";
        folder = "nets/experiments/T-junction";

        lane {
            stepSize = 2f;
        }
    }

    ADPPAgent {
        radius = 2;           //[m]
        waitPenalty = 0.1d;
        movePenalty = 0.01d;
        waitDuration = 1d;    //[s]
        maxSpeed = 5d;        //[m/s]
        maxAcceleration = 2d; //[m/s^2]
        planningHorizon = 10; //[s]
        planProlongationFactor = 3;
    }

    dashboard {
        simulators {
            OpenDS_devel {
                launch = "launchers/OpenDS.sh";
            }
            Dummy {
                launch = "launchers/dummy.sh"
            }
            SimulatorLite {
                launch = "launchers/simulator-lite.sh"
            }
        }

        simulatorsToRun = [];
        sumoSimulation = true;
        numberOfCarsInSimulation = 50;
        //numberOfCarsInSimulation = agents.size();

    }
}

