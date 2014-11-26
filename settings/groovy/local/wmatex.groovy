/**
 * My local highway configuration
 * Created by wmatex on 21.7.14.
 */
highway {
    //agent = "RouteAgent";
    agent = "ADPPAgent";
    net {
//        folder = "nets/highway-straight";
//        folder = "nets/CharlesSquare";
//        folder = "nets/junction-big";
//        folder = "nets/graz";
//        folder = "nets/vysehrad";
//        folder = "nets/pisek";
//        folder = "nets/test";
        folder = "nets/test90";
    }

    dashboard {
        simulators {
            OpenDS_devel {
                launch = "launchers/OpenDS.sh";
            }
            Dummy {
                launch = "launchers/dummy.sh"
            }
        }

        simulatorsToRun = [];
    }
}

