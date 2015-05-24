package cz.agents.highway.platooning;

import cz.agents.alite.configurator.Configurator;

import java.util.Random;

/**
 * Created by user on 4/20/15.
 */
public class VehicleGenerationModule {

    static Random random = new Random();

//    static float safeTime = 2.1f;
//    static float reactionTime = 0.1f;
//
//    static int totalVehiclesPerHour = 3000;
//    static boolean generationLimit = true;
//
//    static float personalVehicleRation = 0.65f;
//    static float avrgSpeed = 30;
//    static float minSpeed = 25;
//    static float speedDispersion = 3f;
//
//    static float probOfPlatoon = 0.5f;
//    static int maxLengthOfPlatoon = 5;
//
//    static boolean genTrucks = false;
//    static float truckSpeed = 25;


    public static float safeTime = (float) Configurator.getParamDouble("highway.platooning.safeTime",2.1).doubleValue();
    public static float reactionTime = (float) Configurator.getParamDouble("highway.platooning.reactionTime",0.1).doubleValue();

    public static int totalVehiclesPerHour = Configurator.getParamInt("highway.platooning.VehicleGenerationModule.totalNumberOfVehiclesPerHour",3000);
    public static boolean generationLimit = Configurator.getParamBool("highway.platooning.VehicleGenerationModule.generationLimit", true);
    public static int airConditions = 0;//0...normal; 1...wet; 2...snow

    public static float personalVehicleRation = (float) Configurator.getParamDouble("highway.platooning.VehicleGenerationModule.passengerVehicleRatio",0.65).doubleValue();
    public static float avrgSpeed = (float) Configurator.getParamDouble("highway.platooning.VehicleGenerationModule.averageSpeed",30.0).doubleValue();
    public static float minSpeed = (float) Configurator.getParamDouble("highway.platooning.VehicleGenerationModule.minimalSpeed",25.0).doubleValue();
    public static float speedDispersion = (float) Configurator.getParamDouble("highway.platooning.VehicleGenerationModule.speedDispersion",3.0).doubleValue();

    public static float probOfPlatoon = (float) Configurator.getParamDouble("highway.platooning.VehicleGenerationModule.ratioOfPlatooningVehicles",0.5).doubleValue();
    public static int maxLengthOfPlatoon = Configurator.getParamInt("highway.platooning.VehicleGenerationModule.maximalLenghtOfPlatoon",5);

    public static boolean genTrucks = Configurator.getParamBool("highway.platooning.VehicleGenerationModule.generateTrucks", true);
    public static float truckSpeed = (float) Configurator.getParamDouble("highway.platooning.VehicleGenerationModule.truckSpeed",25.0).doubleValue();
    public static int lengthOfTruckPlatoon = 3;



    static float nextSpeed = (float) random.nextGaussian() * speedDispersion + avrgSpeed;

    static public boolean genPlatoon(){
        return random.nextDouble() < probOfPlatoon;
    }

    static public boolean genPlatoon(float platoonVehicles, float totalVehicles){
        return platoonVehicles/totalVehicles < probOfPlatoon;
    }

    static public int lengthOfPlatoon(){
//        return 5;
        return  random.nextInt(maxLengthOfPlatoon) + 1;
    }

    static public int lengthOfTruckPlatoon(){
        return  random.nextInt(lengthOfTruckPlatoon);
    }

    static public float generateNextSpeed(){
        nextSpeed = (float) random.nextGaussian() * speedDispersion + avrgSpeed;
        if(nextSpeed < minSpeed) nextSpeed = minSpeed;
        return nextSpeed;
    }

    static public boolean canAddNext(long time, int numberOfGenVehicles){
        if(!generationLimit)return true;
        long vehicles = numberOfGenVehicles;
        return (vehicles*3600000/time) < totalVehiclesPerHour;
    }
    static public boolean canAddNext(float time, int numberOfGenVehicles){
        if(!generationLimit)return true;
        long vehicles = numberOfGenVehicles;
        return (vehicles*3600000/time) < totalVehiclesPerHour;
    }

    static public boolean addTruck(float numberOfPersonalVehicles, float totaNumberVehicles){
        if(!genTrucks)return false;
        //if(totaNumberVehicles == 0) return false;
        if(numberOfPersonalVehicles/totaNumberVehicles > personalVehicleRation){
            return true;
        }else{
            return false;
        }
    }

}
