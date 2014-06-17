package cz.agents.highway.creator;

import cz.agents.alite.creator.CreatorFactory;

public class Main {

    /**
     * @param args
     */
    public static void main(String[] args) {
        System.out.print("RUNNING Highway2013");
        for (int i = 0; i < args.length; i++) {
            System.out.print(" " + args[0]);
        }
        System.out.println(".");
       DefaultCreator creator = (DefaultCreator) CreatorFactory.createCreator(args);
       creator.create();
       creator.runSimulation();
       

    }

}
