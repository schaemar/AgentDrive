package cz.agents.highway.platooning;

import cz.agents.highway.storage.Pair;

import java.util.ArrayList;

/**
 * Created by user on 4/27/15.
 */
public class FVModule {

    private ArrayList<Float> changeLaneAt = new ArrayList<Float>();
    private ArrayList<Integer> changeLaneTo = new ArrayList<Integer>();

    private float distanceBetweenVehicles = 10;

    public FVModule(){

    }

    public float getOptDistInPlatoon(){
        return distanceBetweenVehicles;
    }

    public void addOrder(float position, int direction){
        changeLaneAt.add(position);
        changeLaneTo.add(direction);
    }

    public Pair<Float, Integer> executeOrders(float position){
        if(changeLaneAt.isEmpty()) return null;
        if(position > changeLaneAt.get(0)){
            return new Pair<Float, Integer>(changeLaneAt.remove(0), changeLaneTo.remove(0));
        }
        return null;
    }

}
