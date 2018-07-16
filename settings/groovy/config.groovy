/**
 * Created by wmatex on 4.7.14.
 */
simulator {
    lite {
        name = "Simulator-Lite";
        simulationSpeed = 1;
        protobuf {
            serverUri = "socket://localhost:2222";
            protocol = "simplan";
        }

    }
    net {
        // folder = "nets/mlada-boleslav/"
        folder = "nets/skoda-parking/"
        // folder = "nets/CharlesSquare";
        // folder = "nets/dresden/";
        // folder = "super-collision";
        // folder = "nets/x-junction/";
        // folder = "nets/pisek-all/"
        // folder = "nets/junction-big/";
        // folder = "nets/highway-straight/";
        // folder = "nets/artificialX-junction";
        // folder = "nets/ulesika";
        // folder = "nets/nartest";
        // folder = "nets/artificialHighway-funnel";
        // folder = "nets/hostinne";
        // folder = "nets/artificialXS-junction";
        // folder = "nets/artificialT-junction";
        // folder = "nets/pisek";


    }
    netLayer {
        lane {
            view = true;
            width = 10;
        }
        edge {
            view = true;
            width = 1;
        }
        crossRoad {
            view = false;
            width = 1;
        }
    }
}
