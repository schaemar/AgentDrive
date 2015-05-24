package cz.agents.highway.platooning;

import cz.agents.highway.storage.HighwayStorage;
import cz.agents.highway.storage.Pair;

import java.io.*;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.LinkedHashSet;
import java.util.Set;

/**
 * Created by Ondra Borovec on 4/17/15.
 */
public class PlatooningCenterModule {
    private long startTime = System.currentTimeMillis();
    private long lastUpdate = System.currentTimeMillis();
    private long lastUpdateSimulator = 0;
//    private long lastStartStaticInfo = System.currentTimeMillis();
    private long lastStartStaticInfo = 0;

    private int numberOfGenVehicles = 0;
    private int numberOfGenPersonalVehicles = 0;
    private int numberOfGenTrucks = 0;
    private int numberOfVehiclesInPlatoon = 0;
    private int numOfColisions = 0;

    private int numberOfRun = 0;
    private int intervalOfSimulator = 100;
    private long totalTimeOfRun = 0;

    private VehicleGenerationModule vehicleGenerationModule = new VehicleGenerationModule();

    private HighwayStorage storage;

    public Set<Integer> placedVehicles = new LinkedHashSet<Integer>();

    SimpleDateFormat ft = new SimpleDateFormat ("dd.MM.'-'hh:mm");


    private int numberOfLanes = 3;


    private float[] minDistFromStart = new float[numberOfLanes];
    private int[] minDistFromStartID = new int[numberOfLanes];
    private ArrayList<ArrayList> vehiclesByLanes = new ArrayList<ArrayList>();


    PrintWriter  writer;


    int overtakingType = 0;
//    0......normla
//    1......at one time
//    2......last first

    public PlatooningCenterModule(HighwayStorage storage){
        this.storage = storage;
        startTime = System.currentTimeMillis();
        resetMinDist();
        for(int i = 0; i < numberOfLanes; i++){
            vehiclesByLanes.add(new ArrayList<Integer>());
        }

        //file for statistic info
        //txt file with columns:
        //time; Vehicles/hour; Num. of generated vehicles;	total number of gen personal vehicles; total number of platooning able vehicles; total number of gen trucks;	Collisions/Hour;	lane 0 num. of vehicle; 	lane 0 avg.speed;	lane 0 speed deviation;	lane 1 num. of vehicle;	lane 1 avg.speed;	lane 1 deviation;	lane 2 num. of vehicle;	lane 2 avg.speed;	lane 2 deviation;	total nm. of vehicle nin all lanes;	total avg.speed;	total deviation
        try {
            if(VehicleGenerationModule.generationLimit){
                writer = new PrintWriter("num.lanes:"+numberOfLanes+"_typeOvertake:"+overtakingType+"_"+VehicleGenerationModule.totalVehiclesPerHour+"_platVeh:"+(int)(VehicleGenerationModule.probOfPlatoon*100)+"%" +"_speed:"+VehicleGenerationModule.avrgSpeed+"-+"+VehicleGenerationModule.speedDispersion+"_trucks:"+VehicleGenerationModule.genTrucks+".txt");
            }else{
                writer = new PrintWriter("num.lanes:"+numberOfLanes+"_typeOvertake:"+overtakingType+"_"+"no-limit"+"_platVeh:"+(int)(VehicleGenerationModule.probOfPlatoon*100)+"%" +"_speed:"+VehicleGenerationModule.avrgSpeed+"-+"+VehicleGenerationModule.speedDispersion+"_trucks:"+VehicleGenerationModule.genTrucks+".txt");
            }
//            if(VehicleGenerationModule.generationLimit){
//                writer = new PrintWriter(ft.format(new Date(startTime))+" number of lanes:"+numberOfLanes+" overtaking type"+overtakingType+"  with conditions "+(int)(VehicleGenerationModule.probOfPlatoon*100)+"% "+VehicleGenerationModule.totalVehiclesPerHour +" limit "+VehicleGenerationModule.avrgSpeed+" -+ "+VehicleGenerationModule.speedDispersion+" trucks "+VehicleGenerationModule.genTrucks+".txt");
//            }else{
//                writer = new PrintWriter(ft.format(new Date(startTime))+" number of lanes:"+numberOfLanes+" overtaking type"+overtakingType+"  with conditions "+(int)(VehicleGenerationModule.probOfPlatoon*100)+"% no-limit "+VehicleGenerationModule.avrgSpeed+" -+ "+VehicleGenerationModule.speedDispersion+" trucks "+VehicleGenerationModule.genTrucks+".txt");
//            }
            writer.print("Time;Vehicles/hour;Num. of generated vehicles;total number of gen personal vehicles;total number of platooning able vehicles;total number of gen trucks;Collisions/Hour;");
            for(int i = 0; i < numberOfLanes; i++) writer.print("lane "+i+" num. of vehicle;lane "+i+" avg.speed;lane "+i+" speed deviation;");
            writer.println("total nm. of vehicle nin all lanes;total avg.speed;total deviation;average speed of passanger vehicles;");

        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
    }

    public void run(){
        System.out.println("___________________________________________________________________________________________");
        System.out.println("___________________________________________________________________________________________");
        long lastTimeOfRun = System.currentTimeMillis() - lastUpdate; // time of running simulator to next start of this run function
        System.out.println("Platooning center module works after "+ (lastTimeOfRun)+" ms");
        lastUpdate = System.currentTimeMillis();
        numberOfRun++;
        totalTimeOfRun += intervalOfSimulator;
        lastUpdateSimulator = totalTimeOfRun;

        resetMinDist();
        removingOfOutVehicles();

        for(Integer vehicle : placedVehicles){
            PlatooningAgent agent = (PlatooningAgent) getAgent(vehicle);
            if(agent == null) {
                placedVehicles.remove(vehicle);

            }else if(agent.truck && agent.isLV){
                // in case it is truck it can only speed up of slow down in lane 0
                int lane = agent.getLane();
                float position = getCurrentYposition(vehicle);
                float actSpeed = agent.getActSpeed();
                float brakingKoef = agent.getBrakingCoef();
                int frontAgentId = getAgentIdInLaneFrontThePos(position, lane, agent.id);
                if(frontAgentId == -1  || position < 3 || frontSafeDistanceInLane(actSpeed, brakingKoef, getActSpeed(frontAgentId), distanceBetweenVehicles(agent.id, frontAgentId), isAgnetTruck(frontAgentId), agent.truck)){
                    agent.speedUp(lastTimeOfRun);
                }else{
                    agent.slowDown(lastTimeOfRun);
                }
                PlatooningAgent FVAgent = agent.backFV;
                while(FVAgent != null){//do it for all vehicles to the last one
                    float FVposition = getCurrentYposition(FVAgent.id);
                    //part for being in right distance from thr front vehicle of platoon
                    if(FVposition < 3 && getCurrentYposition(FVAgent.frontAgent.getID()) > FVAgent.fvModule.getOptTruckDistInPlatoon()){
                        FVAgent.setActSpeedAsPrefSpeed();
                    }
                    if (distanceBetweenVehicles(agent.id, frontAgentId) < agent.getActSpeed() * vehicleGenerationModule.safeTime *0.6f && totalTimeOfRun-lastStartStaticInfo >1000 && getCurrentYposition(agent.id) < 4900) {
                        numOfColisions++;
                    }
                    if (getCurrentYposition(FVAgent.frontAgent.getID()) - FVposition > FVAgent.fvModule.getOptTruckDistInPlatoon()){
                        FVAgent.speedUp(lastTimeOfRun, agent.getActSpeed()+0.1f);
                    }else{
                        FVAgent.slowDown(lastTimeOfRun);
                    }
                    //part to storing orders of Leading vehicle

                    setMinDist(FVposition, FVAgent.getID(), lane);
                    //tries to work with the next following agent of platoon
                    FVAgent = FVAgent.backFV;
                }
                setMinDist(position, vehicle, agent.getLane());
            }else if(agent.isLV && !agent.truck){
                //In case it is personal vehicle and it is leading vehicle
//                printVehicleGeneralInfo(vehicle);
                int lane = agent.getLane();
                float position = getCurrentYposition(vehicle);
                float actSpeed = agent.getActSpeed();
                float brakingKoef = agent.getBrakingCoef();

                PlatooningAgent lastPlatoonAgent = getLastVehicleOfPlatoon(agent);
                //two variable to send orders to following vehicles
                float orderForFVPosition = Float.MAX_VALUE; // where to do it
                int orderForFVDir = 0;// what to do -1 ... change to right; 1 ... change to left
                int targetOfOrders = 0; //0...all FV; 1...only last FV; 2...all insted the last FV


                boolean finish = false; //if it change to right lane it does nothing more
                if(agent.canChangeToRightLane()){
                    int backRightAgentId = getAgentIdInLaneBackThePos(position, lane - 1);
                    int frontRightAgentId = getAgentIdInLaneFrontThePos(position, lane - 1, agent.id);
                    switch (overtakingType){
                        case 0:
                            if((frontRightAgentId == -1 || frontSafeDistanceInLane(actSpeed, brakingKoef, getActSpeed(frontRightAgentId), distanceBetweenVehicles(agent.id, frontRightAgentId), isAgnetTruck(frontRightAgentId), agent.truck))
                            && (backRightAgentId == -1 || overtakingDistance(getActSpeed(backRightAgentId), actSpeed, distanceBetweenVehicles(backRightAgentId, agent.id)))){
                                changeLaneToRight(agent, lane);
                                orderForFVPosition = position;
                                orderForFVDir = -1;
                                finish = true;
                            }
                            break;
                        case 2:
                        case 1:
                            if((frontRightAgentId == -1 || frontSafeDistanceInLane(actSpeed, brakingKoef, getActSpeed(frontRightAgentId), distanceBetweenVehicles(agent.id, frontRightAgentId), isAgnetTruck(frontRightAgentId), agent.truck))
                                    && (backRightAgentId == -1 || overtakingDistance(getActSpeed(backRightAgentId), actSpeed, distanceBetweenVehicles(backRightAgentId, lastPlatoonAgent.id)))
                                    && noVehicleBetween(position, getCurrentYposition(lastPlatoonAgent.getID()), lane -1)){
                                changeLaneToRight(agent, lane);
                                orderForFVPosition = position;
                                orderForFVDir = -1;
                                finish = true;
                            }
                            break;
                    }

                }

                if(!finish){
                    //the vehicle could not change to right
                    int frontAgentId = getAgentIdInLaneFrontThePos(position, lane, agent.id);

                    //test of collision ---- the front vehicle is closer then 60% of safe distance, it also run only every hole second and do not count in last 100m of simulator
                    if (distanceBetweenVehicles(agent.id, frontAgentId) < agent.getActSpeed() * vehicleGenerationModule.safeTime *0.6f && System.currentTimeMillis()- lastStartStaticInfo >1000 && getCurrentYposition(agent.id) < 4900) {
                        numOfColisions++;
                    }

                    if(frontAgentId == -1 || frontSafeDistanceInLane(actSpeed, brakingKoef, getActSpeed(frontAgentId), distanceBetweenVehicles(agent.id, frontAgentId), isAgnetTruck(frontAgentId), agent.truck)){
                        //there is enought space in front of the vehicle
                        agent.speedUp(lastTimeOfRun);
                    }else{
                        //there is not enought space in front of the vehicle and it has to react
                        if(agent.getPreferSpeed() < getAgent(frontAgentId).getActSpeed()){
                            //if the front vehicle is close and faster, the vehicle only slow down, overtaking maneuver does not make sense
                            agent.slowDown(lastTimeOfRun);
                        }else{
                            //the vehicle tries to overtake
                            if(agent.canChangeToLeftLane()){
                                //the left line it there
                                int backLeftAgentId = getAgentIdInLaneBackThePos(position, lane + 1);
                                int frontLeftAgentId = getAgentIdInLaneFrontThePos(position, lane + 1, agent.id);
                                switch (overtakingType){
                                    case 0:
                                        if((frontLeftAgentId == -1 || frontSafeDistanceInLane(actSpeed, brakingKoef, getActSpeed(frontLeftAgentId), distanceBetweenVehicles(agent.id, frontLeftAgentId), isAgnetTruck(frontLeftAgentId), agent.truck))
                                                && (backLeftAgentId == -1 || overtakingDistance(getActSpeed(backLeftAgentId), actSpeed, distanceBetweenVehicles(backLeftAgentId, agent.id)))){
                                            changeLaneToLeft(agent, lane);
                                            orderForFVPosition = position;
                                            orderForFVDir = 1;
                                            agent.speedUp(lastTimeOfRun);
                                        }else{
                                            agent.slowDown(lastTimeOfRun);
                                        }
                                        break;
                                    case 1:
                                        if((frontLeftAgentId == -1 || frontSafeDistanceInLane(actSpeed, brakingKoef, getActSpeed(frontLeftAgentId), distanceBetweenVehicles(agent.id, frontLeftAgentId), isAgnetTruck(frontLeftAgentId), agent.truck))
                                                && (backLeftAgentId == -1 || overtakingDistance(getActSpeed(backLeftAgentId), actSpeed, distanceBetweenVehicles(backLeftAgentId, agent.id)))
                                                && noVehicleBetween(position, getCurrentYposition(lastPlatoonAgent.getID()), lane +1)){
                                            changeLaneToLeft(agent, lane);
                                            orderForFVPosition = position;
                                            orderForFVDir = 1;
                                            agent.speedUp(lastTimeOfRun);
                                        }else{
                                            agent.slowDown(lastTimeOfRun);
                                        }
                                        break;

                                    case 2:
                                        float minSpeedFrontLastFV = minSpeedInRange(position +100, getCurrentYposition(lastPlatoonAgent.id), lane +1);
                                        float possitionOfLastFV = getCurrentYposition(lastPlatoonAgent.getID());
                                        int backLeftAgentIdFromLastFV = getAgentIdInLaneBackThePos(possitionOfLastFV, lane + 1);
                                        int frontLeftAgentIdFromLastFV  = getAgentIdInLaneFrontThePos(possitionOfLastFV, lane + 1, lastPlatoonAgent.id);
                                        if(agent.fvModule.state == 0){
                                            if((lastPlatoonAgent.getActSpeed() < minSpeedFrontLastFV
                                                    && (frontLeftAgentIdFromLastFV == -1 || frontSafeDistanceInLane(lastPlatoonAgent.getActSpeed(), lastPlatoonAgent.getBrakingCoef(), getActSpeed(frontLeftAgentIdFromLastFV), distanceBetweenVehicles(lastPlatoonAgent.id, frontLeftAgentIdFromLastFV), isAgnetTruck(frontLeftAgentIdFromLastFV), lastPlatoonAgent.truck)))
                                                    && (backLeftAgentIdFromLastFV == -1 || overtakingDistance(getActSpeed(backLeftAgentId), lastPlatoonAgent.getActSpeed(), distanceBetweenVehicles(backLeftAgentId, lastPlatoonAgent.id)))){
                                                orderForFVPosition = 0;
                                                orderForFVDir = 1;
                                                targetOfOrders = 1;
                                                agent.slowDown(lastTimeOfRun);
                                                agent.fvModule.state = 1;
                                            }else{
                                                agent.slowDown(lastTimeOfRun);
                                            }
                                        }else if(agent.fvModule.state == 1
                                                && !(frontLeftAgentIdFromLastFV == -1 || frontSafeDistanceInLane(lastPlatoonAgent.getActSpeed(), lastPlatoonAgent.getBrakingCoef(), getActSpeed(frontLeftAgentIdFromLastFV), distanceBetweenVehicles(lastPlatoonAgent.id, frontLeftAgentIdFromLastFV), isAgnetTruck(frontLeftAgentIdFromLastFV), lastPlatoonAgent.truck))){
                                            orderForFVPosition = position;
                                            orderForFVDir = 0;
                                            targetOfOrders = 1;
                                            agent.fvModule.state = 0;
                                            agent.speedUp(lastTimeOfRun);
                                        }else if(agent.fvModule.state == 1
                                                && (frontLeftAgentId == -1 || frontSafeDistanceInLane(actSpeed, brakingKoef, getActSpeed(frontLeftAgentId), distanceBetweenVehicles(agent.id, frontLeftAgentId), isAgnetTruck(frontLeftAgentId), agent.truck))
                                                && noVehicleBetween(position, getCurrentYposition(lastPlatoonAgent.getID())+1f, lane +1)){
                                            changeLaneToLeft(agent, lane);
                                            orderForFVPosition = position;
                                            orderForFVDir = 1;
                                            targetOfOrders = 2;
                                            agent.fvModule.state = 0;
                                            agent.speedUp(lastTimeOfRun);
                                        }else{
                                            agent.slowDown(lastTimeOfRun);
                                        }

                                        break;
                                }

                            }else{
                                //there is no lane for overtaking
                                agent.slowDown(lastTimeOfRun);
                            }
                        }
                    }
                }



                //Part of obeying the commnands of following vehicles
                PlatooningAgent FVAgent = agent.backFV;
                while(FVAgent != null){//do it for all vehicles to the last one
                    float FVposition = getCurrentYposition(FVAgent.id);
                    //part for being in right distance from thr front vehicle of platoon
                    if(FVposition == 0 && getCurrentYposition(FVAgent.frontAgent.getID()) > FVAgent.fvModule.getOptDistInPlatoon()){
                        FVAgent.setActSpeedAsPrefSpeed();
                    }
                    if (getCurrentYposition(FVAgent.frontAgent.getID()) - FVposition > FVAgent.fvModule.getOptDistInPlatoon()){
                        FVAgent.speedUp(lastTimeOfRun, agent.getActSpeed()+0.1f);
                    }else{
                        FVAgent.slowDown(lastTimeOfRun);
                    }
                    //part to storing orders of Leading vehicle
                    switch (overtakingType){
                        case 0:
                            if(orderForFVPosition!=Float.MAX_VALUE) FVAgent.fvModule.addOrder(orderForFVPosition, orderForFVDir);
                            break;
                        case 1:
                            if(orderForFVPosition!=Float.MAX_VALUE) FVAgent.fvModule.addOrder(FVposition, orderForFVDir);
                            break;
                        case 2:
                            if(orderForFVDir == -1){
                                if(orderForFVPosition!=Float.MAX_VALUE) FVAgent.fvModule.addOrder(FVposition, orderForFVDir);
                            }else{
                                switch (targetOfOrders){
                                    case 0:
                                        if(orderForFVPosition!=Float.MAX_VALUE) FVAgent.fvModule.addOrder(FVposition, orderForFVDir);
                                        break;
                                    case 1:
                                        if(orderForFVPosition!=Float.MAX_VALUE && FVAgent.backFV == null){
                                            FVAgent.fvModule.addOrder(FVposition, orderForFVDir);
                                        }
                                        break;
                                    case 2:
                                        if(orderForFVPosition!=Float.MAX_VALUE && FVAgent.backFV != null){
                                            FVAgent.fvModule.addOrder(FVposition, orderForFVDir);
                                        }
                                        break;
                                }
                            }

                    }

                    //part of executing the stored commands
                    Pair<Float, Integer> order = FVAgent.fvModule.executeOrders(getCurrentYposition(FVAgent.getID()));
                    if(order != null){
                        if(order.getValue() == 1){
                            changeLaneToLeft(FVAgent, FVAgent.getLane());
                        }else if(order.getValue() == -1){
                            changeLaneToRight(FVAgent, FVAgent.getLane());
                        }
                    }

                    setMinDist(FVposition, FVAgent.getID(), lane);
                    //tries to work with the next following agent of platoon
                    FVAgent = FVAgent.backFV;
                }

                setMinDist(position, vehicle, agent.getLane());
            }
        }

        generateNewVehicles();

        lastUpdate = System.currentTimeMillis();

        startStaticticInfoPrint();
        //averageSpeed();
        System.out.println(numberOfGenVehicles);
        System.out.println("Platooning module run for ..." + (System.currentTimeMillis() - lastUpdate) + " ms");
    }


    //Function to remover vehicles from PlatooningModule which were deleted in local simulator
    private void removingOfOutVehicles() {
        Set<Integer> toRemove = new LinkedHashSet<Integer>();
        for(Integer id : placedVehicles){
            if(!(storage.getPosCurr().containsKey(id))) toRemove.add(id);
        }
        for(Integer id : toRemove)placedVehicles.remove(id);
        for(ArrayList<Integer> list : vehiclesByLanes){
            for(int i = 0; i < list.size(); i++){
                if(!placedVehicles.contains(list.get(i))){
                    list.remove(i);
                }
            }
        }
    }


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Platooning staff
    ///////////////////////////////////////////////////////////////////////////////////////////
    private PlatooningAgent getLastVehicleOfPlatoon(PlatooningAgent agent){
        while(agent.backFV != null){
            agent = agent.backFV;
        }
        return agent;
    }






    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Distance comparison
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    private boolean frontSafeDistanceInLane(float speedOfBackVehicle, float brakingCoefOfBackVehicle, float speedOfFrontVehicle, float distance, boolean frontTruck, boolean iAmTruck){
        boolean answer;
        float extraDistance = 0;
        if(frontTruck) {
            extraDistance = 15;// length of truck
        }else{
            extraDistance = 4;// length of car
        }
        float distanceOfSafeTime = speedOfBackVehicle * vehicleGenerationModule.safeTime;
        if(frontTruck) distanceOfSafeTime = distanceOfSafeTime * 0.4f;
        float distanceToReact = vehicleGenerationModule.reactionTime * speedOfBackVehicle;
        if (speedOfBackVehicle+0.3f < speedOfFrontVehicle){
            answer = distance > 0.8f*distanceOfSafeTime+distanceToReact+extraDistance;
        }else{
            float distanceToSlowDown = (float) Math.abs((speedOfBackVehicle * (speedOfBackVehicle - speedOfFrontVehicle) / brakingCoefOfBackVehicle - 0.5f * Math.pow(speedOfBackVehicle - speedOfFrontVehicle,2) / (brakingCoefOfBackVehicle)));
            answer = distance > distanceOfSafeTime+distanceToReact+distanceToSlowDown+extraDistance;
        }
        return answer;
    }
    private boolean overtakingDistance(float speedOfBackVehicle, float speedOfFrontVehicle, float distance){
        float distanceOfSafeTime = speedOfBackVehicle * vehicleGenerationModule.safeTime + 4;
        double differenceSpeedDistance = Math.min(Math.pow((speedOfBackVehicle - speedOfFrontVehicle),2), distanceOfSafeTime*0.4f);
        if(speedOfBackVehicle > speedOfFrontVehicle){

            if(distanceOfSafeTime + differenceSpeedDistance < distance) return true;
        }else{

            if(distanceOfSafeTime - differenceSpeedDistance < distance) return true;
        }
        return false;
    }

    private boolean noVehicleBetween(float frontPosition, float backPosition, int lane){
        for(Object obj : vehiclesByLanes.get(lane)) {
            Integer id = (Integer) obj;
            float position = getCurrentYposition(id);
            if(position >= backPosition && position <= frontPosition) return false;
        }
        return true;
    }



    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Controling of agent
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    private void changeLaneToLeft(PlatooningAgent agent, int lane){
        agent.changeToLeftLane();
        ((ArrayList)vehiclesByLanes.get(lane)).remove(new Integer(agent.id));
        ((ArrayList)vehiclesByLanes.get(agent.getLane())).add(agent.id);
        agent.setPreferSpeed(agent.getPreferSpeed() + 1);
    }
    private void changeLaneToRight(PlatooningAgent agent, int lane){
        agent.changeToRightLane();
        ((ArrayList)vehiclesByLanes.get(lane)).remove(new Integer(agent.id));
        ((ArrayList)vehiclesByLanes.get(agent.getLane())).add(agent.id);
        agent.setPreferSpeed(agent.getPreferSpeed() - 1);
    }


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Creating vehicles
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //general function to adding new vehicles
    private void generateNewVehicles() {
        for(int lane = 0; lane < numberOfLanes; lane++){//goes though every lane and test whether there is enought space to add another vehicle
            boolean cannAddToThisLane = false;
            boolean cannAddTruckToThisLane = false;
            boolean truck = false;
            if(minDistFromStart[lane] == Integer.MAX_VALUE) cannAddToThisLane = true;
            else {
                if(minDistFromStart[lane] > 50 ){
                    PlatooningAgent newAgent = getNextAgent();//what next agent should be generated
                    PlatooningAgent frontAgent = getAgent(minDistFromStartID[lane]);
                    if(frontSafeDistanceInLane(newAgent.getPreferSpeed() + lane, newAgent.getBrakingCoef(), frontAgent.getActSpeed(), minDistFromStart[lane], frontAgent.truck, VehicleGenerationModule.addTruck(numberOfGenPersonalVehicles, numberOfGenVehicles))){
                        cannAddToThisLane = true;
                    }
                    if(lane == 0 && VehicleGenerationModule.addTruck(numberOfGenPersonalVehicles, numberOfGenVehicles) && frontSafeDistanceInLane(25, newAgent.getBrakingCoef(), frontAgent.getActSpeed(), minDistFromStart[lane], frontAgent.truck, VehicleGenerationModule.addTruck(numberOfGenPersonalVehicles, numberOfGenVehicles))){
                        cannAddTruckToThisLane = true;
                    }
                }
            }
//            if(cannAddToThisLane && VehicleGenerationModule.canAddNext(lastUpdate - startTime, numberOfGenVehicles)) {

            if(VehicleGenerationModule.canAddNext(totalTimeOfRun, numberOfGenVehicles)) {
                if(cannAddTruckToThisLane){
                    PlatooningAgent agent = startNextAgent(lane);
                    if(agent == null)return;
                    recreateToTruck(agent);
                    agent.isLV = true;
                    generateTruckPlatoon(agent, VehicleGenerationModule.lengthOfTruckPlatoon(), lane);
                }else if(cannAddToThisLane){
                    PlatooningAgent agent = startNextAgent(lane);
                    if(agent == null)return;
                    if (VehicleGenerationModule.genPlatoon(numberOfVehiclesInPlatoon,numberOfGenPersonalVehicles)) {
                        numberOfGenPersonalVehicles++;
                        agent.isLV = true;
                        generatePlatoon(agent, VehicleGenerationModule.lengthOfPlatoon(), lane);
                    }
                    numberOfGenPersonalVehicles++;
                }




            }

        }

    }
    private void recreateToTruck(PlatooningAgent agent){
        agent.truck = true;
        agent.setPreferSpeed(VehicleGenerationModule.truckSpeed);
        agent.setActSpeedAsPrefSpeed();
        numberOfGenTrucks++;
    }
    private void generateTruckPlatoon(PlatooningAgent LV, int lenght, int lane){
        PlatooningAgent frontAgent = LV;
        for(int i = 0; i < lenght; i++){
            PlatooningAgent agent = startNextAgent(lane);
            recreateToTruck(agent);
            agent.isLV = false;
            agent.STOP();
            agent.setPreferSpeed(LV.getPreferSpeed());
            agent.frontAgent = frontAgent;
            frontAgent.backFV = agent;
            frontAgent = agent;
        }
        frontAgent.backFV = null;
    }
    private void generatePlatoon(PlatooningAgent LV, int lenght, int lane){
        PlatooningAgent frontAgent = LV;
        for(int i = 0; i < lenght; i++){
            PlatooningAgent agent = startNextAgent(lane);
            agent.isLV = false;
            agent.setPreferSpeed(LV.getPreferSpeed()+1f);
            agent.STOP();
            agent.LV = LV;
            agent.frontAgent = frontAgent;
            frontAgent.backFV = agent;
            frontAgent = agent;
        }
        frontAgent.backFV = null;
        numberOfVehiclesInPlatoon += lenght +1;
        numberOfGenPersonalVehicles += lenght;
    }
    private PlatooningAgent getNextAgent(){
        return ((PlatooningAgent) storage.getAgents().get(storage.notStartedVehicles.get(0)));
    }

    public PlatooningAgent startNextAgent(){
        return startNextAgent(0);
    }
    public PlatooningAgent startNextAgent(int lane){
        if(storage.notStartedVehicles.isEmpty()) return null;
        int id = storage.notStartedVehicles.remove(0);
        PlatooningAgent agent = ((PlatooningAgent) storage.getAgents().get(id));
        agent.getNavigator().setAgentLaneById(lane);
        agent.setPreferSpeed(agent.getPreferSpeed());
        agent.setPreferSpeed(agent.getPreferSpeed() + lane);
        vehiclesByLanes.get(lane).add(id);
        agent.setLane(lane);
        agent.setActSpeedAsPrefSpeed();
        minDistFromStart[lane] = 0;
        placedVehicles.add(id);
        numberOfGenVehicles++;
        return agent;
    }
    private void setMinDist(float position, int vehicle, int lane){
        if(position < minDistFromStart[lane]){
            minDistFromStart[lane] = position;
            minDistFromStartID[lane] = vehicle;
        }
        if(Float.isNaN(position)){
            minDistFromStart[lane] = 0;
            minDistFromStartID[lane] = vehicle;
        }
    }
    private void resetMinDist(){
        for(int i =0; i < numberOfLanes; i++){
            minDistFromStart[i] = Integer.MAX_VALUE;
        }
    }


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Searching in lanes
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    private PlatooningAgent getAgentInLaneFrontThePos(float position, int lane, int initId){
        float minDist = Float.MAX_VALUE;
        int minId = -1;
        for(Object obj : vehiclesByLanes.get(lane)){
            Integer id = (Integer) obj;
            float pos = getCurrentYposition(id);
            if(pos < minDist && pos >= position && initId != id){
                minDist = pos;
                minId = id;
            }
        }
        return getAgent(minId);
    }
    private Integer getAgentIdInLaneFrontThePos(float position, int lane, int initId){
        float minDist = Float.MAX_VALUE;
        int minId = -1;
        for(Object obj : vehiclesByLanes.get(lane)){
            Integer id = (Integer) obj;
            float pos = getCurrentYposition(id);
            if(pos < minDist && pos >= position && initId != id){
                minDist = pos;
                minId = id;
            }
        }
        return minId;
    }
    private PlatooningAgent getAgentInLaneBackThePos(float position, int lane){
        float minDist = Float.MIN_VALUE;
        int minId = -1;
        for(Object obj : vehiclesByLanes.get(lane)){
            Integer id = (Integer) obj;
            float pos = getCurrentYposition(id);
            if(pos > minDist && pos < position){
                minDist = pos;
                minId = id;
            }
        }
        return getAgent(minId);
    }
    private Integer getAgentIdInLaneBackThePos(float position, int lane){
        float minDist = Float.MIN_VALUE;
        int minId = -1;
        for(Object obj : vehiclesByLanes.get(lane)){
            Integer id = (Integer) obj;
            float pos = getCurrentYposition(id);
            if(pos > minDist && pos < position){
                minDist = pos;
                minId = id;
            }
        }
        return minId;
    }

    private float minSpeedInRange(float front, float back, int lane){
        float minSpeed = Float.MAX_VALUE;
        for(Object obj : vehiclesByLanes.get(lane)){
            Integer id = (Integer) obj;
            float pos = getCurrentYposition(id);
            if(pos < front && pos > back){
                minSpeed = Math.min(minSpeed, getActSpeed(id));
            }
        }
        return minSpeed;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //General
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    private void printVehicleGeneralInfo(Integer vehicle) {
        System.out.println("___________________________________________________________________________________________");
        System.out.println("Vehicle: " + vehicle);
        System.out.println("Actual speed: "+getActSpeed(vehicle)+" m/s");
        System.out.println("Position: "+ getCurrentYposition(vehicle)+" m");
    }
    private float getCurrentYposition(Integer id){
        if(storage.getPosCurr().get(id)==null) return Integer.MAX_VALUE;
        if (id==-1) return Integer.MAX_VALUE;
        if (Float.isNaN(storage.getPosCurr().get(id).getPosition().getY())) return 0;
        return -storage.getPosCurr().get(id).getPosition().getY();
    }
    private float getActSpeed(int id){
        if (getAgent(id)==null) return 25;
        return ((PlatooningAgent) getAgent(id)).getActSpeed();
    }
    private PlatooningAgent getAgent(Integer id){
        return (PlatooningAgent) storage.getAgents().get(new Integer(id));
    }
    private float distanceBetweenVehicles(int id1, int id2){
        return Math.abs(getCurrentYposition(id1) - getCurrentYposition(id2));
    }
    private boolean isAgnetTruck(int id){
        return getAgent(new Integer(id)).truck;
    }


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Statistic
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void startStaticticInfoPrint(){
//        if (System.currentTimeMillis() - lastStartStaticInfo < 1000) return;
//        lastStartStaticInfo = System.currentTimeMillis();
        if (totalTimeOfRun - lastStartStaticInfo < 1000) return;
        lastStartStaticInfo = totalTimeOfRun;
        //print time from start;
//        writer.print((System.currentTimeMillis() - startTime)/1000+";");
        writer.print((totalTimeOfRun)/1000+";");
        //density of generated vehicles (vehicle/hour);
//        writer.print(((long)numberOfGenVehicles) * 3600000 / (lastUpdate - startTime)+";");
        writer.print(((long)numberOfGenVehicles) * 3600000 / totalTimeOfRun+";");
        //total number of gen vehicles; , total number of gen personal vehicles; total number of platooning able vehicles; total number of gen trucks;
        writer.print(numberOfGenVehicles+";"+numberOfGenPersonalVehicles+";"+numberOfVehiclesInPlatoon+";"+numberOfGenTrucks+";");
        //number of collisions (collisions/hour);
        writer.print(((long)numOfColisions) * 3600000 / totalTimeOfRun+";");
        int totalNumberVehicles = 0;
        float totalSumOfSpeed = 0;
        int totalNumberVehiclesPassanger = 0;
        float totalSumOfSpeedPassanger = 0;
        for(ArrayList<Integer> list : vehiclesByLanes){
            int numberOfVehicles = list.size();
            totalNumberVehicles += numberOfVehicles;
            float sumOfSpeeds = 0;
            for(Integer id : list){
                sumOfSpeeds += getActSpeed(id);

                if(!getAgent(id).truck){
                    totalSumOfSpeedPassanger += getActSpeed(id);
                    totalNumberVehiclesPassanger++;
                }
            }
            totalSumOfSpeed += sumOfSpeeds;
            float avgSpeed = sumOfSpeeds/numberOfVehicles;
            float sumOfQSpeeds = 0;
            for(Integer id : list){
                sumOfQSpeeds += Math.pow(getActSpeed(id) - avgSpeed,2);
            }
            double deviation = Math.sqrt(sumOfQSpeeds/numberOfVehicles);
            //act number of vehicles in lane; average speed in lane; deviation of speed in lane;
            writer.print(numberOfVehicles + ";" + avgSpeed+";"+deviation+";");
        }
        float totalAvgSpeed = totalSumOfSpeed/totalNumberVehicles;
        double deviation = 0;
        float sumOfQSpeeds = 0;
        for(ArrayList<Integer> list : vehiclesByLanes){
            for(Integer id : list){
                sumOfQSpeeds += Math.pow(getActSpeed(id) - totalAvgSpeed,2);
            }
        }
        deviation = Math.sqrt(sumOfQSpeeds/totalNumberVehicles);
        //number of act vehicles in all lanes; average speed in all lanes; deviation of speed in all lanes
        writer.print(totalNumberVehicles+";"+totalAvgSpeed+";"+deviation+";"+totalSumOfSpeedPassanger/totalNumberVehiclesPassanger);
        writer.println("");
        writer.flush();
    }

    void averageSpeed(){
        float sumSpeed = 0;
        for(Integer i : placedVehicles){
            sumSpeed += getActSpeed(i);
        }
        writer.println(sumSpeed / placedVehicles.size());
        writer.flush();
    }
}
