package cz.agents.agentdrive.simulator.lite.visualization;

import tt.vis.ParameterControlLayer.ParameterProvider;

public class    TimeParameter implements ParameterProvider {

    int time = 0;
    int timeStep;

    public TimeParameter() {
        this(1);
    }

    public TimeParameter(int timeStep) {
        super();
        this.timeStep = timeStep;
    }

    public int getTimeStep() {
        return timeStep;
    }

    @Override
    public String getName() {
        return "Time";
    }

    @Override
    public String getValue() {
        return Integer.toString(time);
    }

    @Override
    public char getIncreaseKey() {
        return 'q';
    }

    @Override
    public char getDecreaseKey() {
        return 'a';
    }

    @Override
    public char getResetKey() {
        return 'r';
    }



    @Override
    public void increased() {
        time += timeStep;
    }

    @Override
    public void decreased() {
        time -= timeStep;
    }

    @Override
    public void reset() {
        time = 0;
    }

    public int getTime() {
        return time;
    }

    public void setTime(int newTime) {
        time = newTime;
    }
}
