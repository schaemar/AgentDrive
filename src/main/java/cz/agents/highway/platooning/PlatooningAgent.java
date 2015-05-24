package cz.agents.highway.platooning;

import cz.agents.alite.common.event.Event;
import cz.agents.highway.agent.Reaction;
import cz.agents.highway.agent.RouteAgent;
import cz.agents.highway.storage.HighwayEventType;
import cz.agents.highway.storage.RoadObject;
import cz.agents.highway.storage.VehicleSensor;
import cz.agents.highway.storage.plan.Action;
import cz.agents.highway.storage.plan.WPAction;

import javax.vecmath.Point2f;
import javax.vecmath.Point3f;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

/**
 * Created by user on 4/18/15.
 */
public class PlatooningAgent extends RouteAgent {


    public boolean isLV = true;
    public PlatooningAgent backFV = null;
    public PlatooningAgent frontAgent = null;
    public PlatooningAgent LV = null;
    public FVModule fvModule = new FVModule();

    public boolean truck = false;


    private float actSpeed;
    private float preferSpeed;
    private int lane;

    private float hardBrakingAcelDry = 7.5f;
    private float hardBrakingAcelWet = 6f;
    private float hardBrakingAcelSnow = 4f;
    private float acceleration = 4;
    private float brakingAcelNormal = 3f;
    private float brakingAcelNormalH = 3*brakingAcelNormal/4;


    public PlatooningAgent(int id) {
        super(id);
        preferSpeed = VehicleGenerationModule.nextSpeed;
        actSpeed = 0;
    }

    public PlatooningAgent(int id, float hardBrakingAcelDry, float hardBrakingAcelWet, float hardBrakingAcelSnow, float acceleration) {
        super(id);
        preferSpeed = VehicleGenerationModule.nextSpeed;
        actSpeed = 0;
        this.hardBrakingAcelSnow = hardBrakingAcelSnow;
        this.hardBrakingAcelWet = hardBrakingAcelWet;
        this.hardBrakingAcelDry = hardBrakingAcelDry;
        this.acceleration = acceleration;
    }

    public boolean canChangeToLeftLane(){
        return navigator.canChangeLaneLeft();
    }
    public void changeToLeftLane(){
        if(navigator.canChangeLaneLeft())lane++;
        navigator.changeLaneLeft();

    }

    public boolean canChangeToRightLane(){
        return navigator.canChangeLaneRight();
    }
    public void changeToRightLane(){
        if(navigator.canChangeLaneRight())lane--;
        navigator.changeLaneRight();
    }

    //returns actual braking coeficient
    public float getBrakingCoef(){
        float returnValue = 0.45f * hardBrakingAcelDry;
        switch(VehicleGenerationModule.airConditions){
            case 0:
                break;
            case 1:
                returnValue = 0.45f *hardBrakingAcelWet;
                break;
            case 2:
                returnValue = 0.45f* hardBrakingAcelSnow;
                break;
        }
        if(truck){
            return returnValue *2;
        }else{
            return returnValue;
        }
    }

    public float getPreferSpeed(){
        return preferSpeed;
    }
    public void setPreferSpeed(float speed){
        preferSpeed = speed;
    }
    public float getActSpeed() {
        return actSpeed;
    }
    public void setActSpeedAsPrefSpeed(){
        actSpeed = preferSpeed;
    }
    public void setLane(int lane){ this.lane = lane;}
    public int getLane(){
        return lane;
    }
    public int getID(){
        return super.id;
    }


    //lastTimeOfRun is time in milisec which is how log the agent  has to brake
    public void speedUp(long lastTimeOfRun) {
        float diff = getBrakingCoef()*lastTimeOfRun/1000;
        if(actSpeed + diff < preferSpeed) {
            actSpeed += diff;
        }else{
            actSpeed = preferSpeed;
        }
    }
    public void speedUp(long lastTimeOfRun, float limit) {
        float diff = getBrakingCoef()*lastTimeOfRun/1000;
        if(actSpeed + diff < limit) {
            actSpeed += diff;
        }else{
            actSpeed = limit;
        }
    }

    public void slowDown(long lastTimeOfRun) {
        float diff = getBrakingCoef()*((float)(lastTimeOfRun))/1000;
        if(actSpeed - diff >=0){
            actSpeed -= diff;
        }else{
            actSpeed = 0;
        }
    }

    public void STOP(){
        actSpeed = 0;
    }


    @Override
    public List<Action> generateWaypointInLane() {
        RoadObject me = sensor.senseCurrentState();
        LinkedList<Action> actions = new LinkedList<Action>();
        if(navigator.isMyLifeEnds()) {
            actions.add(new WPAction(id, 0d, new Point3f(0, 0, 0), -1));
            return actions;
        }
        ArrayList<Point3f> points;  // list of points on the way, used to be able to set speed to the action later

        int wpCount = (int) me.getVelocity().length() + 1; // how many waypoints before me will be calculated.
        points = new ArrayList<Point3f>();
        navigator.setCheckpoint();

        Point2f position2D = new Point2f(me.getPosition().getX(), me.getPosition().getY());

        List<Point2f> wps = new LinkedList<Point2f>();
        Point2f waypoint = null;

        //try to advance navigator closer to the actual position
        int maxMove = 10;  // how many points will be tried.
        //how many waiponts ahead will be chcecked depending on the update time
        maxMove = (int) (((me.getUpdateTime() - lastUpateTime) * MAX_SPEED) / 1000) + 5;
        if (maxMove < 10) maxMove = 10;

        while (navigator.isMyLifeEnds() == false && maxMove-- > 0 && navigator.getRoutePoint().distance(position2D) > CIRCLE_AROUND) {
            navigator.advanceInRoute();
        }
        if (navigator.getRoutePoint().distance(position2D) > CIRCLE_AROUND) {
            navigator.resetToCheckpoint();
        } else {
            while (navigator.isMyLifeEnds() == false && navigator.getRoutePoint().distance(position2D) <= CIRCLE_AROUND) {
                navigator.advanceInRoute();
            }
            //    navigator.setCheckpoint();
        }
        navigator.setCheckpoint();
        waypoint = navigator.getRoutePoint();

        double minSpeed = Float.MAX_VALUE; // minimal speed on the points before me
        //TODO fix than distance of waipoints is different than 1
        for (int i = 0; i < wpCount; i++) {
            // move 3 waipoints ahead
            while (navigator.isMyLifeEnds() == false && waypoint.distance(navigator.getRoutePoint()) < CIRCLE_AROUND){
                navigator.advanceInRoute();
            }
            if(navigator.isMyLifeEnds()) {
                actions.add(new WPAction(id, 0d, new Point3f(0, 0, 0), -1));
                return actions;
            }
            waypoint = navigator.getRoutePoint();
            wps.add(waypoint);

            points.add(i, new Point3f(waypoint.x, waypoint.y, me.getPosition().z));
        }
        for (int i = 0; i < wpCount; i++) // actual filling my outgoing actions
        {
            //scaling speed to the lowest
            actions.add(new WPAction(sensor.getId(), me.getUpdateTime(), points.get(i), actSpeed));
        }
        navigator.resetToCheckpoint();
        lastUpateTime = me.getUpdateTime();
        return actions;

    }



}
