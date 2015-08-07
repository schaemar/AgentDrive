package cz.agents.highway.agent;

import com.jmatio.io.MatFileWriter;
import com.jmatio.types.MLDouble;
import com.jmatio.types.MLInt64;
import cz.agents.alite.common.event.Event;
import cz.agents.highway.maneuver.*;
import cz.agents.highway.storage.HighwayEventType;
import cz.agents.highway.storage.RoadObject;
import cz.agents.highway.storage.VehicleSensor;
import cz.agents.highway.storage.plan.Action;
import cz.agents.highway.storage.plan.ManeuverAction;
import cz.agents.highway.storage.plan.WPAction;
import cz.agents.highway.simpleMessage.*;

import javax.vecmath.*;
import java.util.*;
import java.lang.Math;
import java.util.concurrent.TimeUnit;


public class ComAgent extends Agent implements ComListener{
    ///SVANDRLIK
    private int convoyPosition = 0;
    private double velocitySetpoint = 0.1;
    private ArrayList<ComListener> listeners = new ArrayList<ComListener>();
    private double actualVelocity;
    private Point3f actualPosition;
    private LinkedHashMap<ComAgent, Boolean> activeListeners = new LinkedHashMap<ComAgent, Boolean>();
    private LinkedHashMap<ComAgent, ComMessage> messageStorage = new LinkedHashMap<ComAgent, ComMessage>();
    private float previousCarDistance;
    private final int visibility = 500;
    private int lane;

    double probability;
    LinkedHashMap<Double, Float> positionMap = new LinkedHashMap<Double, Float>();
    LinkedHashMap<Double, Float> distanceMap = new LinkedHashMap<Double, Float>();
    ArrayList<Long> laneSet = new ArrayList<Long>();
    ArrayList<Double> positionSet = new ArrayList<Double>();
    ArrayList<Double> distanceSet = new ArrayList<Double>();
    ArrayList<Double> velocitySet = new ArrayList<Double>();
    ArrayList<Long> selection = new ArrayList<Long>();
    ArrayList<ComMessageEvent> eventList = new ArrayList<ComMessageEvent>();
    float lastPrevDist = 0;
    float lastSpeedMe = 0;
    float lastSpeedFront = Float.MAX_VALUE;
    double previousAction = 0;
    double action = 0;
    float otherLaneDistance;
    double reference = 9;
    boolean waiting = false;
    boolean gotZipReq = false;
    boolean gotZipAck = false;
    boolean respond = false;
    boolean sendReq = false;

    int countDown = 4;
    int zipperID = -1;

    // RouteAgent variables
    private static final float CIRCLE_AROUND = 6.0f;  // Does not exactly correspond to the actual wayPoint distance, used to make circle around the car
    float MAX_SPEED = 40;


    private static final float WP_COUNT_CONST = 0.2f;
    private double lastUpateTime;
    private static int RIGHT = -1;
    private static int LEFT = 1;
    private static final double RADIUS = 1f;
    private static final double MAX_ANGLE = Math.PI / 2;
    private static final float EPSILON = 0.01f;



    @Override
    public void receiveMessage(ComMessageEvent e) {
        RoadObject me = sensor.senseCurrentState();
        ComMessage message = e.getMessage();
        double gotten = Math.random();

        if (gotten <= probability) {

            if (message.getType() == MesType.CAM) {
                System.out.println("%%%% Agent " + message.getSenderID() +
                        " has number " + message.getConvoyPosition() + " SP " + message.getReferenceVelocity());
                insertToMap(e);

            } else if (!e.getSource().equals(this)) {

                if (message.getType() == MesType.ZIP_REQ) {
                    // TODO ZIP message
                    // start with increasing reference distance
                    float coorY = message.getObject().getPosition().getY();

                    if (coorY > me.getPosition().getY() &
                            coorY < me.getPosition().getY() + previousCarDistance &
                            message.getObject().getLaneIndex() != me.getLaneIndex()) {

                        zipperID = message.getObject().getId();
                        gotZipReq = true;
                        respond = true;
                        otherLaneDistance = coorY - this.getActualPosition().getY();
                    }

                } else if (message.getType() == MesType.ZIP_ACK & message.getReceiverID() == me.getId()) {
                    gotZipAck = true;
                    respond = true;
                } else if (message.getType() == MesType.GOT_YOU & message.getReceiverID() == me.getId() & waiting & !respond) {
                    respond = true;
                } else if (message.getType() == MesType.STAND) {
                    sendReq = true;
                }
            }
        }
    }

    public void insertToMap(ComMessageEvent event){
        ComAgent agent = (ComAgent)event.getSource();
        // hold only last messages, remove old ones
        if (this.messageStorage.containsKey(agent)){
            lastSpeedFront = this.messageStorage.get(agent).getObject().getVelocity().length();
            this.messageStorage.remove(agent);
        }

        this.messageStorage.put(agent, event.getMessage());
    }

    public void addActiveListener(ComAgent key, Boolean value){
        if (this.activeListeners.containsKey(key)){
            this.activeListeners.remove(key);
            this.activeListeners.put(key, value);
        } else {
            this.activeListeners.put(key, value);
        }
    }

    public synchronized void addComListener(int index, ComListener lis){
        listeners.add(index,lis);
    }

    public synchronized void remComListener(ComListener lis){
        listeners.remove(lis);
    }

 /*   private ComMessageEvent generateFakeMessage(){
        Point3f fakePos = new Point3f(-2, -1000, 0);
        Vector3f fakeVel = new Vector3f(0,0,0);
        RoadObject fakeObj = new RoadObject(32, this.lastUpateTime, 0, fakePos, fakeVel);
        ComAgent fakeAgent = this;
        fakeAgent.id = 32;
        fakeAgent.setVelocitySetpoint(0);
        fakeAgent.setActualPosition(fakePos);
        fakeAgent.setActualVelocity(0);
        //fakeAgent.setLane(0);
        ComMessage fakeMes = new ComMessage(MesType.STAND, 32, fakeObj);

        return new ComMessageEvent(fakeAgent, fakeMes);
    }
*/
    private synchronized void fireEvent(ComMessage message){
        ComMessageEvent e = new ComMessageEvent(this, message);

        //todo

        // commented code is for latency simulation
        eventList.add(e);
        if (eventList.size() == 5){
            ComMessageEvent toSend = eventList.remove(1);
            eventList.trimToSize();

            for (ComListener listener : listeners){
                if (this.activeListeners.get(listener)) {
                    //listener.receiveMessage(e); //this works for non-latency behavior
                    listener.receiveMessage(toSend);
                }
            }
        }




        // set my position to be consistent with message data
        this.setActualPosition(sensor.senseCurrentState().getPosition());
    }

    public int getLane() {
        return lane;
    }

    public void setLane(int lane) {
        this.lane = lane;
    }

    public int getConvoyPosition() {
        return convoyPosition;
    }

    public void setConvoyPosition(int convoyPosition) {
        this.convoyPosition = convoyPosition;
    }

    public double getVelocitySetpoint() {
        return velocitySetpoint;
    }

    public void setVelocitySetpoint(double velocitySetpoint) {
        this.velocitySetpoint = velocitySetpoint;
    }

    public double getActualVelocity() {
        return actualVelocity;
    }

    public void setActualVelocity(double actualVelocity) {
        this.actualVelocity = actualVelocity;
    }

    public float getPreviousCarDistance() {
        return previousCarDistance;
    }

    public void setPreviousCarDistance(float previousCarDistance) {
        this.previousCarDistance = previousCarDistance;
    }

    public Point3f getActualPosition() {
        return actualPosition;
    }

    public void setActualPosition(Point3f actualPosition) {
        this.actualPosition = actualPosition;
    }

    public void setCapacity (int size){
        this.listeners.ensureCapacity(size);
    }

    public ArrayList<ComListener> getListeners() {
        return listeners;
    }

    /*
                    * We suppose knowledge only about a car which is in front of us
                     * so we filter out those we cannot see
                    * */
    public RoadObject getVehicleAhead(){
        Collection<RoadObject> cars = sensor.senseCars();
        RoadObject me = sensor.senseCurrentState();
        RoadObject closestCar = null;

        float distance = Float.MAX_VALUE;

        for(RoadObject car : cars){
            if ((waiting | gotZipReq)){
                if (!car.equals(me)) {
                    try {
                        float a = car.getPosition().y - me.getPosition().y;
                        if (a < 0 && distance > Math.abs(a)) { // bacha na smer nerovnosti
                            closestCar = car;
                            distance = Math.abs(a); // opet znamenko
                        }
                    } catch (NullPointerException e){
                        System.out.println("$$ no car here bro");
                    }
                }
            } else {
                if (!car.equals(me)) {
                    try {
                        float a = car.getPosition().y - me.getPosition().y;
                        if (me.getLaneIndex() == car.getLaneIndex() && a < 0 && distance > Math.abs(a)) { // bacha na smer nerovnosti
                            closestCar = car;
                            distance = Math.abs(a); // opet znamenko
                        }
                    } catch (NullPointerException e){
                        System.out.println("$$ no car here bro");
                    }
                }
            }
        }
        this.setPreviousCarDistance(distance);

        // return the closest car , if nothing in front result is null
        return closestCar;
    }

    public void manageMyListeners(){
        Collection<RoadObject> cars = sensor.senseCars();
        RoadObject me = sensor.senseCurrentState();

        for (RoadObject car : cars){
            float distance = car.getPosition().distance(me.getPosition());
            int index = car.getId();
            try {
                ComAgent agent = (ComAgent)this.listeners.get(index);

                if (distance > visibility & this.activeListeners.get(agent)) {
                    this.addActiveListener(agent, false);
                } else if (distance < visibility & !this.activeListeners.get(agent)){
                    this.addActiveListener(agent, true);
                }

            } catch (NullPointerException e){
                System.out.println("car not found");
            }
        }

    }


    // ADDED FROM SDAgent

    protected double getDistance(RoadObject state) {
        return transGeoToDistance(state.getPosition());
    }
    private double transGeoToDistance(double x, double y) {
        return sensor.getRoadDescription().distance(new Point2d(x, y));
    }

    protected double transGeoToDistance(Point3f position) {
        return transGeoToDistance(position.x, position.y);
    }



    // RouteAgent
    @Override
    public Point3f getInitialPosition() {

        Point2f p = navigator.getInitialPosition();
        return new Point3f(p.x, p.y, 0);
    }

    /** THIS IS CONSTRUCTOR
     *
     * @param id
     * @param prob
     */
    public ComAgent(int id, double prob) {
        super(id);
        this.probability = prob;
    }

    public void addSensor(final VehicleSensor sensor) {
        this.sensor = sensor;
        this.sensor.registerReaction(new Reaction() {
            public void react(Event event) {
                if (event.getType().equals(HighwayEventType.UPDATED)) {
                    actuator.act(agentReact());
                }
            }
        });

    }

    /**
     * Generate an action as a reaction
     * Rout agent is adjusting speed according to the degrees of the curves and how many waypoints before himself will calculate.
     *
     * @return
     */
    protected List<Action> agentReact() {

        return generateWaypointInLane();
    }

    protected List<Action> agentReact(CarManeuver maneuver) {
        return translate(maneuver);
    }

    public List<Action> translate(CarManeuver maneuver) {
        if (maneuver == null) {
            LinkedList<Action> actions = new LinkedList<Action>();
            Point2f initial = navigator.getInitialPosition();
            actions.add(new WPAction(id, 0d, new Point3f(initial.x, initial.y, 0), 0));

            return actions;
        }
        RoadObject me = sensor.senseCurrentState();
        // Check the type of maneuver
        if ((maneuver instanceof StraightManeuver) || (maneuver instanceof AccelerationManeuver)
                || (maneuver instanceof DeaccelerationManeuver)) {
            return generateWaypointInLane(0, maneuver);
        } else if (maneuver instanceof LaneLeftManeuver) {
            return generateWaypointInLane(/*me.getLaneIndex() + 1*/ LEFT, maneuver);
        } else if (maneuver instanceof LaneRightManeuver) {
            return generateWaypointInLane(/*me.getLaneIndex() - 1*/ RIGHT, maneuver);
        } else {
            LinkedList<Action> actions = new LinkedList<Action>();
            ManeuverAction res = new ManeuverAction(sensor.getId(), maneuver.getStartTime() / 1000.0,
                    maneuver.getVelocityOut(), maneuver.getLaneOut(), maneuver.getDuration());
            actions.add(res);
            return actions;
        }
    }

    private List<Action> generateWaypointInLane(int relativeLane, CarManeuver maneuver) {
        RoadObject me = sensor.senseCurrentState();
        LinkedList<Action> actions = new LinkedList<Action>();

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
        String uniqueIndex = navigator.getUniqueLaneIndex();
        // finding the nearest wayipont, if changing lane, set the first of the new lane.
        while (maxMove-- > 0 && navigator.getRoutePoint().distance(position2D) > CIRCLE_AROUND && navigator.getUniqueLaneIndex().equals(uniqueIndex)) {
            navigator.advanceInRoute();
        }
        // finding the nearest waipoint in the new lane.
        if (!navigator.getUniqueLaneIndex().equals(uniqueIndex)) {
            float initialPos = position2D.distance(navigator.getRoutePoint());
            do {
                navigator.advanceInRoute();
            } while (position2D.distance(navigator.getRoutePoint()) < initialPos);
            while (navigator.getRoutePoint().distance(position2D) <= CIRCLE_AROUND) {
                navigator.advanceInRoute();
            }
            navigator.setCheckpoint();

        }

        // waipoint not found, reset back
        if (navigator.getRoutePoint().distance(position2D) > CIRCLE_AROUND && navigator.getUniqueLaneIndex().equals(uniqueIndex)) {
            navigator.resetToCheckpoint();
        } else {
            while (navigator.getRoutePoint().distance(position2D) <= CIRCLE_AROUND) {
                navigator.advanceInRoute();
            }
            navigator.setCheckpoint();
        }


        if (relativeLane == RIGHT) {
            navigator.changeLaneRight();
            navigator.setCheckpoint();
        } else if (relativeLane == LEFT) {
            navigator.changeLaneLeft();
            navigator.setCheckpoint();
        }
        waypoint = navigator.getRoutePoint();


        float minSpeed = Float.MAX_VALUE; // minimal speed on the points before me

        for (int i = 0; i <= maneuver.getPositionOut() || i < wpCount; i++) {
            // move 3 waipoints ahead
            while (waypoint.distance(navigator.getRoutePoint()) < CIRCLE_AROUND) {
                navigator.advanceInRoute();
            }
            waypoint = navigator.getRoutePoint();
            wps.add(waypoint);
            // vector from my position to the next waypoint
            Vector3f toNextPoint = new Vector3f(waypoint.x - me.getPosition().x, waypoint.y - me.getPosition().y, 0);
            Vector3f velocity = me.getVelocity();
            float angle = velocity.angle(toNextPoint); // angle between my velocity and vector to the next point
            float speed;
            if (Float.isNaN(angle)) {
                speed = 1;
            } else {
                if (angle < 0.4) speed = MAX_SPEED; // if the curve is less than 20 degrees, go by the max speed.
                else if (angle > 6) speed = 2;    // minimal speed for curves.
                else {
                    speed = 1 / angle * 6;
                }
            }
            if (speed < minSpeed) minSpeed = speed;  // all the next actions get the minimal speed.
            points.add(i, new Point3f(waypoint.x, waypoint.y, me.getPosition().z));
        }
        if (minSpeed > maneuver.getVelocityOut()) {
            minSpeed = (float) maneuver.getVelocityOut();
        }
        float speedChangeConst = (me.getVelocity().length() - minSpeed) / wpCount;
        for (int i = 0; i < wpCount; i++) // actual filling my outgoing actions
        {
            //scaling speed to the lowest
            actions.add(new WPAction(sensor.getId(), me.getUpdateTime(), points.get(i), me.getVelocity().length() - (i + 1) * speedChangeConst));
        }
      /*
      only minimal speed set
      for(int i=0;i<=maneuver.getVelocityOut() && i<wpCount;i++)
        {
            actions.add(new WPAction(sensor.getId(), me.getUpdateTime(),points.get(i),minSpeed));
        }*/


        navigator.resetToCheckpoint();
        lastUpateTime = me.getUpdateTime();
        return actions;

    }
// this is called every time when event
    private List<Action> generateWaypointInLane() {
        RoadObject me = sensor.senseCurrentState();
        LinkedList<Action> actions = new LinkedList<Action>();

        ArrayList<Point3f> points;  // list of points on the way, used to be able to set speed to the action later
        int wpCount = 1;
        //int wpCount = (int) me.getVelocity().length() + 1; // how many waypoints before me will be calculated.
        points = new ArrayList<Point3f>();
        navigator.setCheckpoint();



        Point2f position2D = new Point2f(me.getPosition().getX(), me.getPosition().getY());

        List<Point2f> wps = new LinkedList<Point2f>();
        Point2f waypoint = null;

        //SVANDRLIK
        // i have to update time

        //lastUpateTime = me.getUpdateTime();

        RoadObject carInFront = getVehicleAhead();
        positionSet.add((double) me.getPosition().getY());
        velocitySet.add((double) me.getVelocity().length());
        distanceSet.add((double) this.getPreviousCarDistance());
        laneSet.add((long) me.getLaneIndex());

        // add data to maps
        distanceMap.put(lastUpateTime, this.getPreviousCarDistance());
        positionMap.put(lastUpateTime, me.getPosition().getY());

        this.setLane(me.getLaneIndex());
        // Here we send CAM message

        if (probability != 0) {
            fireEvent(new ComMessage(MesType.CAM, me.getId(), me.getLaneIndex(), "Pos and Vel", me, this.convoyPosition, this.velocitySetpoint, this.getPreviousCarDistance()));
        }

        // if i got zip request i send confirm message that i hear and perform. I degrade flag respond 'cause i responded
        if (gotZipReq & respond){
            fireEvent(new ComMessage(MesType.GOT_YOU, me.getId(), zipperID, "Pos and Vel", me, this.convoyPosition, this.velocitySetpoint, this.getPreviousCarDistance()));
            respond = false;
        }

        // if i got zip request and made the space in front i send ZIP_ACK message
        if (this.getPreviousCarDistance() > this.reference & gotZipReq ){
            fireEvent(new ComMessage(MesType.ZIP_ACK, me.getId(), zipperID, "Pos and Vel", me, this.convoyPosition, this.velocitySetpoint, this.getPreviousCarDistance()));
            gotZipReq = false;
        }

        // i will send message with acknowlegement again if no response
        if (!gotZipReq & respond){
            countDown--;
            if (countDown == 0){
                fireEvent(new ComMessage(MesType.ZIP_ACK, me.getId(), zipperID, "Pos and Vel", me, this.convoyPosition, this.velocitySetpoint, this.getPreviousCarDistance()));
                countDown = 4;
            }
        }
        // if i get zip ack i respond
        if (gotZipAck & respond){
            fireEvent(new ComMessage(MesType.GOT_YOU, me.getId(), zipperID, "Pos and Vel", me, this.convoyPosition, this.velocitySetpoint, this.getPreviousCarDistance()));
            respond = false;
        }

        // if i can zip i zip and degrade the gotZipAck flag
        if (gotZipAck & this.getPreviousCarDistance() > this.reference){
            navigator.changeLaneLeft();
            gotZipAck = false;
        }


        // this works but not using messages
/*        if (this.getPreviousCarDistance() > this.reference & waiting ){
            if (me.getLaneIndex() == 0){
                navigator.changeLaneLeft();
            }
            waiting = false;
        }*/

        // agent fires an event if he is in line No. 0
        if ((me.getUpdateTime() >= 60001 & !respond) & me.getLaneIndex() == 0 /*& this.getConvoyPosition() == 1 & !waiting*/){
            fireEvent(new ComMessage(MesType.ZIP_REQ, me.getId(), me.getLaneIndex(), "Zip request message", me, this.convoyPosition, this.velocitySetpoint, this.getPreviousCarDistance()));
            sendReq = false;
            waiting = true;
        }
        // if i sent message that i wanna zip and did not get response in 4 cycles, send message again
        if (waiting & !respond){
            countDown--;
            if (countDown == 0){
                fireEvent(new ComMessage(MesType.ZIP_REQ, me.getId(), me.getLaneIndex(), "Zip request message", me, this.convoyPosition, this.velocitySetpoint, this.getPreviousCarDistance()));
                countDown = 4;
            }
        }


        //try to advance navigator closer to the actual position
        int maxMove = 10;  // how many points will be tried.
        //how many waiponts ahead will be chcecked depending on the update time

        //maxMove = (int) (((me.getUpdateTime() - lastUpateTime) * MAX_SPEED) / 1000) + 5; // original
        maxMove = (int) (((me.getUpdateTime() - lastUpateTime) * this.getVelocitySetpoint()) / 1000) + 5;

/*        if (me.getId() == 9) {
            maxMove = (int) (((me.getUpdateTime() - lastUpateTime) * this.getVelocitySetpoint()) / 1000) + 5;
        } else {
            maxMove = (int) (((me.getUpdateTime() - lastUpateTime) * this.previousAction) / 1000) + 5;
        }*/
        if (maxMove < 10) maxMove = 10;
        String uniqueIndex = navigator.getUniqueLaneIndex();



        // finding the nearest wayipont, if changing lane, set the first of the new lane.
        while (maxMove-- > 0 && navigator.getRoutePoint().distance(position2D) > CIRCLE_AROUND && navigator.getUniqueLaneIndex().equals(uniqueIndex)) {
            navigator.advanceInRoute();
        }
        // finding the nearest waipoint in the new lane.
        if (!navigator.getUniqueLaneIndex().equals(uniqueIndex)) {
            float initialPos = position2D.distance(navigator.getRoutePoint());
            do {
                navigator.advanceInRoute();
            } while (position2D.distance(navigator.getRoutePoint()) < initialPos);
            while (navigator.getRoutePoint().distance(position2D) <= CIRCLE_AROUND) {
                navigator.advanceInRoute();
            }
            navigator.setCheckpoint();

        }

        // waipoint not found, reset back
        if (navigator.getRoutePoint().distance(position2D) > CIRCLE_AROUND && navigator.getUniqueLaneIndex().equals(uniqueIndex)) {
            navigator.resetToCheckpoint();
        } else {
            while (navigator.getRoutePoint().distance(position2D) <= CIRCLE_AROUND) {
                navigator.advanceInRoute();
            }
            navigator.setCheckpoint();
        }
        waypoint = navigator.getRoutePoint();


        float minSpeed = Float.MAX_VALUE; // minimal speed on the points before me

        for (int i = 0; i < wpCount; i++) {
            // move 3 waipoints ahead
            while (waypoint.distance(navigator.getRoutePoint()) < CIRCLE_AROUND) {
                navigator.advanceInRoute();
            }
            waypoint = navigator.getRoutePoint();
            wps.add(waypoint);
            // vector from my position to the next waypoint
            Vector3f toNextPoint = new Vector3f(waypoint.x - me.getPosition().x, waypoint.y - me.getPosition().y, 0);
            Vector3f velocity = me.getVelocity();
            float angle = velocity.angle(toNextPoint); // angle between my velocity and vector to the next point
            float speed;
            if (Float.isNaN(angle)) {
                speed = 1;
            } else {
                if (angle < 0.4) {
                    // speed = MAX_SPEED; //original
                    speed = (float) this.getVelocitySetpoint();
                }// if the curve is less than 20 degrees, go by the max speed.
                else if (angle > 6) speed = 2;    // minimal speed for curves.
                else {
                    speed = 1 / angle * 6;
                }
            }
            if (speed < minSpeed) minSpeed = speed;  // all the next actions get the minimal speed.
            points.add(i, new Point3f(waypoint.x, waypoint.y, me.getPosition().z));
        }

        //SVANDRLIK
        // here we update the convoy order numbers convoyPosition

        manageMyListeners();
        double time = 0;
        ComMessage message = null;
        //RoadObject carInFront = getVehicleAhead();
        float preVelocity = Float.MAX_VALUE;
        float meVelocity = me.getVelocity().length();

        boolean doIt = false;
        /**
        * This is the place where agents are numbered
        * */
        this.setActualPosition(me.getPosition());

        if (me.getUpdateTime() > 1 & me.getUpdateTime() < 202/* & me.getId() == 28*/){
            // DONT TOUCH
            this.setVelocitySetpoint(20);
            //lastPrevDist = this.getPreviousCarDistance();
        }

        //int numOfCarsInFront = 0;
        // TODO some logic
        // if there is no car in front of agent, he is first
        // if (carInFront == null){
        try {
            if (this.getPreviousCarDistance() == Float.MAX_VALUE) {
                this.setConvoyPosition(1);
            } else {
                // otherwise we look into received messages to find out, who is in front of agent
                for (Map.Entry<ComAgent, ComMessage> entry : messageStorage.entrySet()) {
                    if (me.getId() == entry.getValue().getObject().getId()){
                        continue;
                    }
                    // get message sending time
                    double agentTime = entry.getValue().getObject().getUpdateTime();

                    try {
                        if (agentTime >= me.getUpdateTime() - 2000) {
                        //if (me.getId() + 1 == entry.getValue().getObject().getId()) {
                            float myPos = positionMap.get(agentTime);
                            float myDist = distanceMap.get(agentTime);
                            //float myPos = me.getPosition().getY();
                            //float myDist = this.getPreviousCarDistance();

                            float agentPos = entry.getValue().getObject().getPosition().getY();

                            // if there is agent in front of me, i will take his setpoint
                            if (myPos > agentPos) this.setVelocitySetpoint(entry.getValue().getReferenceVelocity());

                            //if (agentTime >= lastUpateTime - 400) {
                            //if ((myPos + myDist)*1.1 >= agentPos & (myPos + myDist)*0.9 <= agentPos) { // puvodni nefunkcni
                            if ((myPos - myDist*1.3) <= agentPos & (myPos - myDist*0.7) >= agentPos) {
                                preVelocity = entry.getValue().getObject().getVelocity().length();
                                this.setConvoyPosition(entry.getValue().getConvoyPosition()+1);
                                //this.setVelocitySetpoint(entry.getValue().getReferenceVelocity());
                                doIt = true;
                                break;
                            }
                        }
                    } catch (Exception e){
                        //e.printStackTrace();
                    }
                    if (agentTime >= me.getUpdateTime()) {
                        //if (doIt){
                        //doIt = false;
                        //    numOfCarsInFront--;
                        //}

                        float myPos = me.getPosition().getY();
                        float myDist = this.getPreviousCarDistance();
                        float agentPos = entry.getValue().getObject().getPosition().getY();

                        //if (myPos > agentPos) numOfCarsInFront++;

                        //if (agentTime >= lastUpateTime - 400) {
                        //if ((myPos + myDist)*1.1 >= agentPos & (myPos + myDist)*0.9 <= agentPos) { // puvodni nefunkcni
                        if ((myPos - myDist*1.2) <= agentPos & (myPos - myDist*0.8) >= agentPos) {
                            preVelocity = entry.getValue().getObject().getVelocity().length();
                            this.setConvoyPosition(entry.getValue().getConvoyPosition()+1);
                            doIt = true;
                            break;
                        }
                    }

                }
            }
        } catch (Exception e){
            e.printStackTrace();
        }

        //this.setConvoyPosition(numOfCarsInFront+1);

        if (me.getUpdateTime() > 181000){
            System.exit(0);
        }


        // export data to matlab files
        if (me.getUpdateTime() > 180001){
            try {
                // list to put everything into
                ArrayList list = new ArrayList();

                // add vehicle position to arrayList
                Double [] position = positionSet.toArray(new Double[positionSet.size()]);
                MLDouble mldouble = new MLDouble("position", position, position.length);
                list.add(mldouble);

                // add velocity
                Double [] velocity = velocitySet.toArray(new Double[velocitySet.size()]);
                mldouble = new MLDouble("velocity", velocity, velocity.length);
                list.add(mldouble);

                // add distance
                Double [] distance = distanceSet.toArray(new Double[distanceSet.size()]);
                mldouble = new MLDouble("distance", distance, distance.length);
                list.add(mldouble);

                // add selection
                Long [] sel = selection.toArray(new Long[selection.size()]);
                MLInt64 mlint = new MLInt64("selection", sel, sel.length);
                list.add(mlint);

                // add convoy position number
                Long [] number = {(long) this.getConvoyPosition()};
                mlint = new MLInt64("number", number, number.length);
                list.add(mlint);

                // add lane index
                Long [] laneInd = laneSet.toArray(new Long[laneSet.size()]);
                mlint = new MLInt64("lane", laneInd, laneInd.length);
                list.add(mlint);

                // write to file
                new MatFileWriter("/home/michal/output/file"+me.getId()+".mat", list);
                //write.write();


            } catch (Exception e){
                e.printStackTrace();
            }
        }


        /**
         * This is where the control action is taken
         * */
        if (me.getUpdateTime() > 80000){
            this.setVelocitySetpoint(5);
        }

        float dist = this.getPreviousCarDistance();


        //MAX_SPEED = (float) (this.getVelocitySetpoint()*1.25);


        float speedChangeConst = (me.getVelocity().length() - minSpeed) / wpCount;

        for (int i = 0; i < wpCount; i++) // actual filling my outgoing actions
        {
            double speed;

            if (this.getVelocitySetpoint() == 0){
                actions.add(new WPAction(sensor.getId(), me.getUpdateTime(), points.get(i), 0));
                break;
            }
            // try to make regulator
            speed = me.getVelocity().length() - (i + 1) * speedChangeConst;


            // TODO regulator mark
            if (this.getPreviousCarDistance() != Float.MAX_VALUE) {
                // if you are leader you keep your velocity. Others try to keep up
                // simple PI regulator

                reference = (this.getVelocitySetpoint());

                if(this.probability == 0 & me.getUpdateTime() > 1600){
                    me.getId();
                }

                float kp = 0.8f;

                if (doIt & preVelocity != Float.MAX_VALUE) {

                    action = - kp * (reference - dist + 7) + 1 * (preVelocity - meVelocity);
                    previousAction = speed + action;// - 0.3 * previousAction
                    selection.add(0l);
                } else {
                    action = - kp * (reference - dist + 7) - 0.05 * action;
                    previousAction = speed + action;
                    selection.add(1l);
                }

                // you shouldn't go faster than maximum
                if (previousAction > MAX_SPEED){
                    previousAction = MAX_SPEED;
                }

                // you shouldn't give lower actions than zero
                if (previousAction < 0){
                    previousAction = 0;
                }

                if (dist < 5){
                    previousAction = 0;
                }

                actions.add(new WPAction(sensor.getId(), me.getUpdateTime(), points.get(i), previousAction));
            } else {

/*                if (lastUpateTime > 25000) {
                    actions.add(new WPAction(sensor.getId(), me.getUpdateTime(), points.get(i), 0));
                } else {
                    actions.add(new WPAction(sensor.getId(), me.getUpdateTime(), points.get(i), speed));
                }*/
                selection.add(1l);
               // if (me.getUpdateTime() > 50000){
               //     actions.add(new WPAction(sensor.getId(), me.getUpdateTime(), points.get(i), speed + 5*Math.sin(me.getUpdateTime()/10000)));
               // } else {
                    actions.add(new WPAction(sensor.getId(), me.getUpdateTime(), points.get(i), speed));
                //}
            }

            //actions.add(new WPAction(sensor.getId(), me.getUpdateTime(), points.get(i), me.getVelocity().length() - (i + 1) * speedChangeConst));
        }
        lastPrevDist = this.getPreviousCarDistance();
        lastSpeedMe = me.getVelocity().length();

        navigator.resetToCheckpoint();
        lastUpateTime = me.getUpdateTime();
        return actions;

    }

    private WPAction point2Waypoint(Point2f point, CarManeuver maneuver) {
        return new WPAction(sensor.getId(), maneuver.getStartTime() / 1000,
                new Point3f(point.x, point.y, sensor.senseCurrentState().getPosition().z),
                maneuver.getVelocityOut());
    }

    /**
     * This method determines whether the waypoint candidate is close enough (in radius) to the position
     * in the direction given by velocity vector
     *
     * @param innerPoint Waypoint candidate
     * @param position   Position
     * @param velocity   Velocity vector
     * @return
     */
    public boolean pointCloseEnough(Point2f innerPoint, Point2f position, Vector2f velocity) {
        // Direction vector of waypoint candidate relative to position
        Vector2f direction = new Vector2f();
        direction.sub(innerPoint, position);

        if (velocity.x == 0 && velocity.y == 0) {
            return innerPoint.distance(position) < 3;
        } else {
            return velocity.angle(direction) < MAX_ANGLE &&
                    distance(innerPoint, position, direction, velocity) < RADIUS;
        }
    }

    /**
     * This method computes the distance of the waypoint. It is the Euklidian distance of the waypoint
     * multiplied by absolute value of the sin of the angle between the direction of the waypoint
     * and the vector of velocity. This ensures, that waypoints that are less deviating from
     * the direction of the vehicle's movement and are close enough are picked.
     */
    private float distance(Point2f innerPoint, Point2f position, Vector2f direction, Vector2f velocity) {
        float d = innerPoint.distance(position);
        return d * d * Math.abs((float) Math.sin(direction.angle(velocity)) + EPSILON);
    }

}