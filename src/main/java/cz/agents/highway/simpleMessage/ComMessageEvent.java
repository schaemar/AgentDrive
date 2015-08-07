package cz.agents.highway.simpleMessage;

import java.util.EventObject;

/**
 * Created by michal on 8.2.15.
 */
public class ComMessageEvent extends EventObject {

    private ComMessage message;

    public ComMessageEvent(Object source, ComMessage message) {
        super(source);
        this.message = message;
    }

    public ComMessage getMessage() {
        return message;
    }
}
