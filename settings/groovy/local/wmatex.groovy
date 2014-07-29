/**
 * My local highway configuration
 * Created by wmatex on 21.7.14.
 */
highway {
    agent = "RouteAgent";
    net {
//        folder = "nets/highway-straight";
//        folder = "nets/CharlesSquare";
//        folder = "nets/junction-big";
        folder = "nets/pisek";
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

        simulatorsToRun = ["OpenDS_devel"];
    }
}

