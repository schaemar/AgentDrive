package cz.agents.highway.platooning;

import cz.agents.highway.storage.HighwayStorage;
import cz.agents.highway.storage.Pair;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.Set;

/**
 * Created by Ondra Borovec on 4/17/15.
 */
public class PlatooningCenterModule_normal {
    private long startTime = System.currentTimeMillis();
    private long lastUpdate = System.currentTimeMillis();
    private long lastStartStaticInfo = System.currentTimeMillis();

    private int numberOfGenVehicles = 0;
    private int numberOfVehiclesInPlatoon = 0;
    private int numOfColisions = 0;

    private VehicleGenerationModule vehicleGenerationModule = new VehicleGenerationModule();
    private float overtakindDistanceKoef = 0.05f;

    private HighwayStorage storage;

    public Set<Integer> placedVehicles = new LinkedHashSet<Integer>();


    private int numberOfLanes = 3;
    private float[] minDistFromStart = new float[numberOfLanes];
    private int[] minDistFromStartID = new int[numberOfLanes];
    private ArrayList<ArrayList> vehiclesByLanes = new ArrayList<ArrayList>();


    PrintWriter  writer;


    public PlatooningCenterModule_normal(HighwayStorage storage){
        this.storage = storage;
        startTime = System.currentTimeMillis();
        resetMinDist();
        for(int i = 0; i < numberOfLanes; i++){
            vehiclesByLanes.add(new ArrayList<Integer>());
        }

        try {
            writer = new PrintWriter("collision 100% 3000 limit.txt");
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
    }

    public void run(){
        System.out.println("___________________________________________________________________________________________");
        System.out.println("___________________________________________________________________________________________");
        long lastTimeOfRun = System.currentTimeMillis() - lastUpdate;
        System.out.println("Platooning center module works after "+ (lastTimeOfRun)+" ms");
        lastUpdate = System.currentTimeMillis();
        resetMinDist();

        removingOfOutVehicles();

        for(Integer vehicle : placedVehicles){
            PlatooningAgent agent = (PlatooningAgent) getAgent(vehicle);
            if(agent == null){
                placedVehicles.remove(vehicle);
            }else if(agent.isLV){
                boolean finish = false;
//                printVehicleGeneralInfo(vehicle);
                int lane = agent.getLane();
                float position = currentYposition(vehicle);
                float actSpeed = agent.getActSpeed();
                float brakingKoef = agent.getBrakingCoef();

                float orderForFVPosition = Float.MAX_VALUE;
                int orderForFVDir = 0;
//                if(agent.getLane() == 0 && System.currentTimeMillis() - startTime > 5000){
//                    changeLaneToLeft(agent, lane);
//                    orderForFVPosition = position;
//                    orderForFVDir = 1;
//                }

                if(agent.canChangeToRightLane()){
                    int backRightAgentId = getAgentIdInLaneBackThePos(position, lane - 1);
                    int frontRightAgentId = getAgentIdInLaneFrontThePos(position, lane - 1, agent.id);
                    if((frontRightAgentId == -1 || frontSafeDistanceInLane(actSpeed, brakingKoef, getActSpeed(frontRightAgentId), distanceBetweenVehicles(agent.id, frontRightAgentId)))
                            && (backRightAgentId == -1 || overtakingDistance(getActSpeed(backRightAgentId), actSpeed, distanceBetweenVehicles(backRightAgentId, agent.id)))){
                        changeLaneToRight(agent, lane);
                        orderForFVPosition = position;
                        orderForFVDir = -1;
                        finish = true;
                    }
                }
                if(!finish){
                    int frontAgentId = getAgentIdInLaneFrontThePos(position, lane, agent.id);
                    if (distanceBetweenVehicles(agent.id, frontAgentId) < agent.getActSpeed() * vehicleGenerationModule.safeTime *0.6f && System.currentTimeMillis()- lastStartStaticInfo >1000 && currentYposition(agent.id) < 4900) {
                        numOfColisions++;
                    }
                    if(frontAgentId == -1 || frontSafeDistanceInLane(actSpeed, brakingKoef, getActSpeed(frontAgentId), distanceBetweenVehicles(agent.id, frontAgentId))){
                        agent.speedUp(lastTimeOfRun);
                    }else{
                        if(agent.getPreferSpeed() < getAgent(frontAgentId).getActSpeed()){
                            agent.slowDown(lastTimeOfRun);
                        }else{
                            if(agent.canChangeToLeftLane()){
                                int backLeftAgentId = getAgentIdInLaneBackThePos(position, lane + 1);
                                int frontLeftAgentId = getAgentIdInLaneFrontThePos(position, lane + 1, agent.id);
                                if((frontLeftAgentId == -1 || frontSafeDistanceInLane(actSpeed, brakingKoef, getActSpeed(frontLeftAgentId), distanceBetweenVehicles(agent.id, frontLeftAgentId)))
                                        && (backLeftAgentId == -1 || overtakingDistance(getActSpeed(backLeftAgentId), actSpeed, distanceBetweenVehicles(backLeftAgentId, agent.id)))){
                                    changeLaneToLeft(agent, lane);
                                    orderForFVPosition = position;
                                    orderForFVDir = 1;
                                    agent.speedUp(lastTimeOfRun);
                                }else{
                                    agent.slowDown(lastTimeOfRun);
                                }
                            }else{
                                agent.slowDown(lastTimeOfRun);
                            }
                        }
                    }
                }
                PlatooningAgent FVAgent = agent.backFV;
                while(FVAgent != null){
                    float FVposition = currentYposition(FVAgent.id);
                    if(FVposition == 0 && currentYposition(FVAgent.frontAgent.getID()) > FVAgent.fvModule.getOptDistInPlatoon()){
                        FVAgent.setActSpeedAsPrefSpeed();
                    }
                    if (currentYposition(FVAgent.frontAgent.getID()) - FVposition > FVAgent.fvModule.getOptDistInPlatoon()){
                        FVAgent.speedUp(lastTimeOfRun, agent.getActSpeed()+0.1f);
                    }else{
                        FVAgent.slowDown(lastTimeOfRun);
                    }
                    //if(orderForFVPosition!=Float.MAX_VALUE) FVAgent.fvModule.addOrder(orderForFVPosition, orderForFVDir);
                    if(orderForFVPosition!=Float.MAX_VALUE) FVAgent.fvModule.addOrder(orderForFVPosition, orderForFVDir);
                    Pair<Float, Integer> order = FVAgent.fvModule.executeOrders(currentYposition(FVAgent.getID()));
                    if(order != null){
                        if(order.getValue() == 1){
                            changeLaneToLeft(FVAgent, FVAgent.getLane());
                        }else{
                            changeLaneToRight(FVAgent, FVAgent.getLane());
                        }
                    }

                    setMinDist(FVposition, FVAgent.getID(), lane);
                    FVAgent = FVAgent.backFV;
                }

                setMinDist(position, vehicle, agent.getLane());
            }
        }

//        if(placedVehicles.isEmpty()) {
//
//            generatePlatoon(startNextAgent(0), 5, 0);
//        }
//        if(placedVehicles.isEmpty()) {
//            startNextAgent(0);
//            startNextAgent(1);
//        }
        generateNewVehicles();

        lastUpdate = System.currentTimeMillis();
        System.out.println(numberOfGenVehicles+"    "+ (lastUpdate - startTime));
        System.out.println("Was generated " + ((long)numberOfGenVehicles) * 3600000 / (lastUpdate - startTime) + " vehicles per hour");
        startStaticticInfoPrint();
        System.out.println("Platooning module run for ..." + (System.currentTimeMillis() - lastUpdate) + " ms");
    }

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
    //Distance comparison
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    private boolean frontSafeDistanceInLane(float speedOfBackVehicle, float brakingCoefOfBackVehicle, float speedOfFrontVehicle, float distance){
        boolean answer;
        float distanceOfSafeTime = speedOfBackVehicle * vehicleGenerationModule.safeTime;
        float distanceToReact = vehicleGenerationModule.reactionTime * speedOfBackVehicle;
        if (speedOfBackVehicle < speedOfFrontVehicle){
            answer = distance > 0.8f*distanceOfSafeTime+distanceToReact;
        }else{
            float distanceToSlowDown = (float) (0.5f * Math.pow(speedOfBackVehicle - speedOfFrontVehicle,2) / (brakingCoefOfBackVehicle));
            answer = distance > distanceOfSafeTime+distanceToReact+distanceToSlowDown;
        }
        return answer;
    }
    private boolean overtakingDistance(float speedOfBackVehicle, float speedOfFrontVehicle, float distance){
        float distanceOfSafeTime = speedOfBackVehicle * vehicleGenerationModule.safeTime;
        double differenceSpeedDistance = Math.min(Math.pow((speedOfBackVehicle - speedOfFrontVehicle),2), distanceOfSafeTime*0.4f);
        if(speedOfBackVehicle > speedOfFrontVehicle){

            if(distanceOfSafeTime + differenceSpeedDistance < distance) return true;
        }else{

            if(distanceOfSafeTime - differenceSpeedDistance < distance) return true;
        }
        return false;
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
    private void generateNewVehicles() {
        boolean generatingPlatoon = false;
        for(int lane = 0; lane < numberOfLanes; lane++){
            boolean freeLane = false;
            if(minDistFromStart[lane] == Integer.MAX_VALUE) freeLane = true;
            else {
                if(minDistFromStart[lane] > 50 ){
                    PlatooningAgent newAgent = getNextAgent();
                    PlatooningAgent frontAgent = getAgent(minDistFromStartID[lane]);
                    if(frontSafeDistanceInLane(newAgent.getPreferSpeed() + lane, newAgent.getBrakingCoef(), frontAgent.getActSpeed(), minDistFromStart[lane])) freeLane = true;
                }
            }
            if(freeLane && VehicleGenerationModule.canAddNext(lastUpdate - startTime, numberOfGenVehicles)) {
                PlatooningAgent agent = useAnotherVehicle(lane);
                if (VehicleGenerationModule.genPlatoon(numberOfVehiclesInPlatoon,numberOfGenVehicles)) {
                    agent.isLV = true;
                    generatePlatoon(agent, VehicleGenerationModule.lengthOfPlatoon(), lane);
                }

            }

        }

    }
    private void generatePlatoon(PlatooningAgent LV, int lenght, int lane){
        PlatooningAgent frontAgent = LV;
        for(int i = 0; i < lenght; i++){
            PlatooningAgent agent = useAnotherVehicle(lane);
            agent.isLV = false;
            agent.setPreferSpeed(LV.getPreferSpeed());
            agent.STOP();
            agent.frontAgent = frontAgent;
            frontAgent.backFV = agent;
            frontAgent = agent;
        }
        frontAgent.backFV = null;
        numberOfVehiclesInPlatoon += lenght +1;
    }
    private PlatooningAgent getNextAgent(){
        return ((PlatooningAgent) storage.getAgents().get(storage.notStartedVehicles.get(0)));}
    public PlatooningAgent useAnotherVehicle(){
        return useAnotherVehicle(0);
    }
    public PlatooningAgent useAnotherVehicle(int lane){
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
            float pos = currentYposition(id);
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
            float pos = currentYposition(id);
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
            float pos = currentYposition(id);
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
            float pos = currentYposition(id);
            if(pos > minDist && pos < position){
                minDist = pos;
                minId = id;
            }
        }
        return minId;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //General
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    private void printVehicleGeneralInfo(Integer vehicle) {
        System.out.println("___________________________________________________________________________________________");
        System.out.println("Vehicle: " + vehicle);
        System.out.println("Actual speed: "+getActSpeed(vehicle)+" m/s");
        System.out.println("Position: "+currentYposition(vehicle)+" m");
    }
    private float currentYposition(Integer id){
        if(storage.getPosCurr().get(id)==null) return Integer.MAX_VALUE;
        if (id==-1) return Integer.MAX_VALUE;
        if (Float.isNaN(storage.getPosCurr().get(id).getPosition().getY())) return 0;
        return -storage.getPosCurr().get(id).getPosition().getY();
    }
    private float getActSpeed(int id){
        return ((PlatooningAgent) getAgent(id)).getActSpeed();
    }
    private PlatooningAgent getAgent(Integer id){
        return (PlatooningAgent) storage.getAgents().get(new Integer(id));
    }
    private float distanceBetweenVehicles(int id1, int id2){
        return Math.abs(currentYposition(id1) - currentYposition(id2));
    }


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Statistic
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void startStaticticInfoPrint(){
        if (System.currentTimeMillis() - lastStartStaticInfo < 1000) return;
        lastStartStaticInfo = System.currentTimeMillis();
        writer.print((System.currentTimeMillis() - startTime)/1000+";");
        writer.print(((long)numberOfGenVehicles) * 3600000 / (lastUpdate - startTime)+";");
        writer.print(numberOfGenVehicles+";"+numberOfVehiclesInPlatoon+";");
        writer.print(numOfColisions * 3600000 / (lastUpdate - startTime)+";");
        int totalNumberVehicles = 0;
        float totalSumOfSpeed = 0;
        for(ArrayList<Integer> list : vehiclesByLanes){
            int numberOfVehicles = list.size();
            totalNumberVehicles += numberOfVehicles;
            float sumOfSpeeds = 0;
            for(Integer id : list){
                sumOfSpeeds += getActSpeed(id);
            }
            totalSumOfSpeed += sumOfSpeeds;
            float avgSpeed = sumOfSpeeds/numberOfVehicles;
            float sumOfQSpeeds = 0;
            for(Integer id : list){
                sumOfQSpeeds += Math.pow(getActSpeed(id) - avgSpeed,2);
            }
            double deviation = Math.sqrt(sumOfQSpeeds/numberOfVehicles);
            writer.print(numberOfVehicles+";"+avgSpeed+";"+deviation+";");
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
        writer.print(totalNumberVehicles+";"+totalAvgSpeed+";"+deviation);
        writer.println("");
        writer.flush();
    }

    void averageSpeed(){

    }
}
