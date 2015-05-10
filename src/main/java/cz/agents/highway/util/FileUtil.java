package cz.agents.highway.util;


import cz.agents.alite.configurator.Configurator;
import cz.agents.highway.environment.roadnet.XMLReader;
import cz.agents.highway.storage.Pair;

import javax.vecmath.Point3f;
import java.util.*;
import java.io.*;


/**
 * Created by david on 10.3.15.
 */
public class FileUtil {
    private FileUtil(){}

    public void writeToFile(Map<Integer, Queue<Pair<Long,Float>>> distances, int speedOrDistances)
    {
        String file_name;
        if(speedOrDistances == 0) {
            file_name ="distances.m";
        }
        else
        {
            file_name ="speeds.m";
        }
        char[] alphabet = "ABCDEFGHIJKLMNOPQRSTUVWXYZ".toCharArray();
        try {
            FileWriter fstream = new FileWriter(file_name);
            BufferedWriter out = new BufferedWriter(fstream);
            int index = 0;
            String addstring = "";
            Random rand = new Random();
            for (Map.Entry<Integer, Queue<Pair<Long,Float>>> entry : distances.entrySet())
            {
                if(index > 25)
                {
                    addstring +="A";
                    index = 0;
                }
                out.write(addstring + alphabet[index] + " = [");
                Queue<Pair<Long,Float>> way =  entry.getValue();
                Iterator<Pair<Long,Float>> itr = way.iterator();
                while (itr.hasNext()) {
                    Float element = (Float) itr.next().getValue();
                    out.write(element + " ");
                }
                out.write("]");
                out.newLine();
                out.write(addstring + alphabet[index] + "time" + " = [");
                itr = way.iterator();
                while (itr.hasNext()) {
                    Float element = itr.next().getKey()/1000f;
                    out.write(element + " ");
                }
                out.write("]");
                index++;
                out.newLine();
            }
            out.write("figure;");
            out.newLine();
            out.write("hold on;");
            out.newLine();
            index = 0;
            addstring = "";
            for (Map.Entry<Integer, Queue<Pair<Long,Float>>> entry : distances.entrySet())
            {
                if(index > 25)
                {
                    addstring +="A";
                    index = 0;
                }
                float r = rand.nextFloat();
                float g = rand.nextFloat();
                float b = rand.nextFloat();
                out.write("plot("+addstring + alphabet[index] + "time" + "," + addstring + alphabet[index++] + ",'Color'," +"[" + r +" " + g + " " + b + "])");
                out.newLine();
            }

            if(speedOrDistances == 0) {
                out.write("title('Car distances to the junction')");
                out.newLine();
                out.write("ylabel('distance')");
            }
            else
            {
                out.write("title('Car speeds')");
                out.newLine();
                out.write("ylabel('speed')");
            }
            out.newLine();
            out.write("xlabel('Simulation time')");
            out.newLine();
            out.write("hold off;");
            out.close();
            System.out.println("File created successfully.");

        } catch (Exception e) { // TODO Improve this
            System.out.println(e.getMessage());
        }
    }
    public void writeGraphOfArrivals(Map<Integer, Pair<Float,Float>>  graphOfArrivals)
    {
        String file_name = "graphOfArrivals.m";
        try {
            FileWriter fstream = new FileWriter(file_name);
            BufferedWriter out = new BufferedWriter(fstream);
            out.write("A" + " = [");
            for (Map.Entry<Integer, Pair<Float, Float>> obj : graphOfArrivals.entrySet()) {
                if(obj.getValue().getValue() !=null) {
                    Float element = (Float) obj.getValue().getKey();
                    out.write(element + " ");
                }
            }
            out.write("]");
            out.newLine();
            out.write("Atime" + " = [");
            for (Map.Entry<Integer, Pair<Float, Float>> obj : graphOfArrivals.entrySet()) {
                if(obj.getValue().getValue() !=null) {
                    Float element = obj.getValue().getValue() - obj.getValue().getKey();
                    out.write(element + " ");
                }
            }
            out.write("]");
            out.newLine();
            out.write("p = polyfit(A,Atime,4);");
            out.newLine();
            out.write("x1 = linspace(0,A(length(A)));");
            out.newLine();
            out.write("y1 = polyval(p,x1);");
            out.newLine();
            out.write("figure;");
            out.newLine();
            out.write("hold on;");
            out.newLine();
            out.write("plot(A,Atime,'o');");
            out.newLine();
            out.write("plot(x1,y1);");
            out.newLine();
            out.write("hold off;");
            out.close();
            System.out.println("Graph of arrivals created");
        }
     catch (Exception e) { // TODO Improve this
        System.out.println(e.getMessage());
    }
    }
    public void writeReport(int numberOfCollisions,float numberOfVehiclesPerSecond,long timeOfsimulation,
                            Map<Integer,Float> avspeed, Map<Integer, Pair<Point3f,Float>> lenghtOfjourney,
                            LinkedList<Float> timesOfArrival, LinkedList<Integer> computationTime)
    {
        String file_name = "report.txt";
        try {
            FileWriter fstream = new FileWriter(file_name);
            BufferedWriter out = new BufferedWriter(fstream);
            out.write("Name of the scenario is: " + Configurator.getParamString("highway.net.folder", "nets/junction-big/"));
            out.newLine();
            out.write("Number of collisions is :" + numberOfCollisions);
            out.newLine();
            out.write("Number of vehicles travelling throw junction per seccond is: " + numberOfVehiclesPerSecond);
            out.newLine();
            out.write("Time of simulation was: " + timeOfsimulation / 1000f);
            out.newLine();
            out.write("Maximum number of cars in simulation are: " + Configurator.getParamInt("highway.dashboard.numberOfCarsInSimulation", 40));
            out.newLine();
            out.write("Sumo simulation: " + Configurator.getParamBool("highway.dashboard.sumoSimulation", true));
            if(!Configurator.getParamBool("highway.dashboard.sumoSimulation", true))
            {
                out.newLine();
                out.write("Random routes: " + Configurator.getParamBool("highway.rvo.agent.randomRoutes", true));
            }
            out.newLine();
            out.write("Distance to activate narowing mod: " + Configurator.getParamInt("highway.safeDistanceAgent.distanceToActivateNM", 400));
            out.newLine();
            out.write("Safety reserve distance: " + Configurator.getParamDouble("highway.safeDistanceAgent.safetyReserveDistance", 4.0)
            );
            out.newLine();
            if(Configurator.getParamList("highway.dashboard.simulatorsToRun", String.class).isEmpty())
            {
                out.write("Simulator used: LocalSimulator");
            }
            else {
                out.write("Simulator used: " + Configurator.getParamList("highway.dashboard.simulatorsToRun", String.class).get(0));
            }
            out.newLine();
          /*  out.write("Avarage speed is: "+ (distance/timeOfsimulation)*3.6 + " km/h");
            out.newLine();*/
            final XMLReader reader = XMLReader.getInstance();
            File flowsFile = new File(reader.getFile(Utils.getResourceUrl(Configurator.getParamString("highway.net.folder", "nets/junction-big/")), ".flows.xml"));
            Scanner sc = new Scanner(flowsFile);
            while (sc.hasNextLine()) {
                String s = sc.nextLine();
                out.write(s);
                out.newLine();
            }
            sc.close();
            Map<List<String>,Pair<Integer,Float>> averageJourneySpeed = new HashMap<List<String>, Pair<Integer,Float>>();
            for (Map.Entry<Integer, Float> obj : avspeed.entrySet())
            {
                Pair<Integer, Float> integerFloatPair = averageJourneySpeed.get(reader.getRoutes().get(obj.getKey()));
                Float newSpeed;
                Integer newNumber;
                if(integerFloatPair == null)
                {
                    newSpeed = obj.getValue();
                    newNumber = 1;
                }
                else
                {
                    Float originalavspeed = integerFloatPair.getValue();
                    Integer originalnumer = integerFloatPair.getKey();
                    newSpeed = originalavspeed + obj.getValue();
                    newNumber = originalnumer+1;
                }
                averageJourneySpeed.put(reader.getRoutes().get(obj.getKey()),new Pair<Integer, Float>(newNumber,newSpeed));
                out.write("id: " + obj.getKey() + " route: " + reader.getRoutes().get(obj.getKey())
                        + " avspeed: " + obj.getValue() + " distance traveled: " +
                        lenghtOfjourney.get(obj.getKey()).getValue());
                out.newLine();
            }
            out.newLine();
            for (Map.Entry<List<String>, Pair<Integer,Float>> obj : averageJourneySpeed.entrySet())
            {
                Float spped = obj.getValue().getValue()/obj.getValue().getKey();

                out.write("Journey: " + obj.getKey() + " avspeed: " + spped);
                out.newLine();
            }
            out.write("Times of arrival in seconds:");
            out.newLine();
            out.write(timesOfArrival.toString());
            out.newLine();
            Integer summ=0;
            for(Integer e : computationTime)
            {
                summ+=e;
            }
            Float res = (float)summ/computationTime.size();
            out.write(res.toString() + " miliseconds");
            out.newLine();
            out.close();

            System.out.println("Report created successfully.");
        }
     catch (Exception e) { // TODO Improve this
        System.out.println(e.getMessage());
        }
    }
    public static FileUtil getInstance() {
        return FileUtilHolder.INSTANCE;
    }
    private static class FileUtilHolder {
        private static final FileUtil INSTANCE = new FileUtil();
    }
}

