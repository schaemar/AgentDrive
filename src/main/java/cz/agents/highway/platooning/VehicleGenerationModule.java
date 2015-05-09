package cz.agents.highway.platooning;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Random;

/**
 * Created by user on 4/20/15.
 */
public class VehicleGenerationModule {

    static float safeTime = 2.1f;
    static float reactionTime = 0.1f;
    static float avrgSpeed = 30;
    static float speedDispersion = 3;
    static Random random = new Random();
    static float probOfPlatoon = 0.5f;
    static float distInPlatoon = 9;
    static int maxLenghtOfPlatoon = 5;
    static int vehiclesPerHour = 3000;
    static boolean generationLimit = false;




    static float nextSpeed = (float) random.nextGaussian() * speedDispersion + avrgSpeed;

    static public boolean genPlatoon(){
        return random.nextDouble() < probOfPlatoon;
    }

    static public boolean genPlatoon(float platoonVehicles, float totalVehicles){
        return platoonVehicles/totalVehicles < probOfPlatoon;
    }

    static public int lengthOfPlatoon(){
        return  random.nextInt(maxLenghtOfPlatoon) + 2;
    }

    static public float generateNextSpeed(){
        nextSpeed = (float) random.nextGaussian() * speedDispersion + avrgSpeed;
        System.out.println(nextSpeed);
        return nextSpeed;
    }

    static public boolean canAddNext(long time, int numberOfGenVehicles){
        if(!generationLimit)return true;
        System.out.println("VehicleGenerationModule: "+numberOfGenVehicles*3600000/time);
        return (numberOfGenVehicles*3600000/time) < vehiclesPerHour;
    }


}
