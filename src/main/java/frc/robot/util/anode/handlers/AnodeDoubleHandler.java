package frc.robot.util.anode.handlers;

import java.lang.reflect.Type;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import frc.robot.util.LoggedTunableNumber;

public class AnodeDoubleHandler extends AnodeObjectHandler<Double> {
    private LoggedTunableNumber tunableNumber;

    public AnodeDoubleHandler() {}

    /**
     * Constructor 
     * @param name
     * @param path
     * @param hardcodedValue
     */
    private AnodeDoubleHandler(String name, String path,Double hardcodedValue) {
        //Start a loggedTunableNumber and default to the hardcoded value.
        tunableNumber = new LoggedTunableNumber(path + "/" + name, hardcodedValue);
    }

    /**
     * A method to be run on the prototype which constructs an instance of this handler
     */
    @Override
    public AnodeObjectHandler<Double> getNew(String name, String path, Object hardcodedValue) {
        return new AnodeDoubleHandler(name,path,(Double)hardcodedValue);
    }

    /**
     * Returns the type we're handling. Should be the same as T
     * <p>We can't just get this from T because of java's type erasure.
     */
    @Override
    public List<Class<Double>> getHandledTypes() {
        return Arrays.asList(Double.class,Double.TYPE);
    }

    /**
     * Called when we want to update our value
     */
    @Override
    public Double refresh(Object currentVal) {
        //If the number changed, update it. Otherwise, keep it the same.
        if(tunableNumber.hasChanged(this.hashCode())) {
            return tunableNumber.getAsDouble();
        } else {
            return (Double)currentVal;
        }
    }
}
