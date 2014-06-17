package cz.agents.highway.util;

import java.net.URL;

public class Utils {
	
	public static URL getResourceUrl(String resource){
		URL ret = Utils.class.getClassLoader().getResource(resource);
		return ret;
	}

    public static int name2ID(String name) {
        return Integer.parseInt(name);
    }

}
