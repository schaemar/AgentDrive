/**
 * My local highway configuration
 * Created by wmatex on 21.7.14.
 */
highway {
    //agent = "RouteAgent";
    agent = "ADPPAgent";
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
//        folder = "nets/artificialX-junction-smaller";
//        folder = "nets/artificialHighway-funnel";
        folder = "nets/simple"
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
    }
}

