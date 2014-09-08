package cz.agents.highway.agent;

import cz.agents.highway.environment.HighwayEnvironment;
import cz.agents.highway.environment.roadnet.Edge;
import cz.agents.highway.environment.roadnet.Lane;
import cz.agents.highway.maneuver.CarManeuver;
import cz.agents.highway.maneuver.HighwaySituation;
import cz.agents.highway.maneuver.StraightManeuver;
import cz.agents.highway.storage.RoadObject;

import javax.vecmath.Point2f;
import javax.vecmath.Vector2f;
import java.util.*;

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
        int myIndexOnRoute=getNearestWaipointIndex(state,myLane);


        for (RoadObject entry : cars) {

            if(entry.getPosition().distance(state.getPosition()) > 60 || state.getPosition().equals(entry.getPosition()))
            {
                continue;
            }
            else
            {
                nearCars.add(entry);
            }
        }
        situationPrediction.addAll(getPlannedManeuvers(state, from, to));

        Lane testLane =null;
        int numberOfLanes = myEdge.getLanes().size();
       // lastManeuver = new StraightManeuver(car.getLaneIndex(), car.getVelocity().length(), getDistance(car), (long) (car.getUpdateTime() * 1000));

        for(RoadObject entry : nearCars)
        {
            ArrayList<CarManeuver> predictedManeuvers = getPlannedManeuvers(entry, from, to);
            situationPrediction.addAll(predictedManeuvers);

            CarManeuver man = predictedManeuvers.get(0);


             testLane = highwayEnvironment.getRoadNetwork().getLane(entry.getPosition());

            if(!testLane.getEdge().equals(myEdge))
            {
                System.out.println("abeceda"); // TODO situation where car is in the different road
                continue;
            }
            if(myLane.getLaneId().equals(testLane.getLaneId())) {
                int mamamamamia = getNearestWaipointIndex(entry,testLane);
                if(mamamamamia > myIndexOnRoute)
                {
                  //  StraightManeuver strh = new StraightManeuver(testLane.getIndex(),(double)entry.getVelocity().length(),getDistance(entry),(long) (entry.getUpdateTime() * 1000));
                    situationPrediction.trySetCarAheadManeuver(man);
                    System.out.println("aaaaaaaaaaaaaaaaaaaaaaa");
                }
                System.out.println("mama ma mamam mia");
            }
            else
            {
                if(myLane.getIndex() < testLane.getIndex())  // TODO fix more than three lanes
                {
                //    StraightManeuver strh = new StraightManeuver(testLane.getIndex(),(double)entry.getVelocity().length(),getDistance(entry),(long) (entry.getUpdateTime() * 1000));
                    situationPrediction.trySetCarRightAheadMan(man);
                }
                else
                {
                    //StraightManeuver strh = new StraightManeuver(testLane.getIndex(),(double)entry.getVelocity().length(),getDistance(entry),(long) (entry.getUpdateTime() * 1000));
                    situationPrediction.trySetCarLeftAheadMan(man);
                }
            }


        }

        System.out.println(testLane);
        return situationPrediction;
    }
   private int getNearestWaipointIndex(RoadObject state,Lane myLane)
   {
       int myIndexOnRoute = 0;

       while(!maneuverTranslator.pointCloseEnough(myLane.getInnerPoints().get(myIndexOnRoute),new Point2f(state.getPosition().x,state.getPosition().y),new Vector2f(state.getVelocity().x,state.getVelocity().y)))
       {
           myIndexOnRoute++;  //TODO fix this
           if(myLane.getInnerPoints().size() == myIndexOnRoute)
           {
               myIndexOnRoute--;
               break;
           }
       }
       return  myIndexOnRoute;
   }


}
