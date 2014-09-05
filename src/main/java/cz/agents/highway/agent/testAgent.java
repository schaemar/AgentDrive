package cz.agents.highway.agent;

import cz.agents.highway.environment.HighwayEnvironment;
import cz.agents.highway.environment.roadnet.Edge;
import cz.agents.highway.environment.roadnet.Lane;
import cz.agents.highway.maneuver.HighwaySituation;
import cz.agents.highway.storage.RoadObject;

import java.util.Collection;
import java.util.Set;
import java.util.TreeSet;
import java.util.Vector;

/**
 * Created by david on 26.8.14.
 */
public class testAgent extends SDAgent {
    private HighwayEnvironment highwayEnvironment;
    private Edge myEdge;
    private Lane myLane;

    public testAgent(int id, HighwayEnvironment hgw) {
        super(id);
        this.highwayEnvironment = hgw;
    }

    @Override
    public HighwaySituation generateSS(RoadObject state, Collection<RoadObject> cars, long from, long to) {
        HighwaySituation situationPrediction = new HighwaySituation();

        logger.debug("GenerateSS:");
        Collection<RoadObject> nearCars = new Vector<RoadObject>();
        // TO OPTIMIZE
        myLane = highwayEnvironment.getRoadNetwork().getLane(state.getPosition());
        myEdge = myLane.getEdge();

       // String myLane = state.getPosition();
        for (RoadObject entry : cars) {

            if(entry.getPosition().distance(state.getPosition()) > 30)
            {
                continue;
            }
            else
            {
                nearCars.add(entry);
            }
        }
        Lane testLane;
        for(RoadObject entry : nearCars)
        {
             testLane = highwayEnvironment.getRoadNetwork().getLane(state.getPosition());
        }

        return null;
    }


}
