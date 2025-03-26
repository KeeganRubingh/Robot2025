package frc.robot.util.anode.handlers;

import java.lang.reflect.ParameterizedType;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import frc.robot.util.LoggedTunableNumber;

public class AnodeMeasureHandler<? extends Unit> extends AnodeObjectHandler<Measure<?>>{
    private LoggedTunableNumber tunableNumber;
    private Measure<?> defaultVal;

    public AnodeMeasureHandler() {};

    private AnodeMeasureHandler(String name, String path, Measure<?> hardcodedValue) {
        tunableNumber = new LoggedTunableNumber(path + "/" + name + hardcodedValue.unit().name(), hardcodedValue.magnitude());
        defaultVal = hardcodedValue;
    }
    
    @Override
    public AnodeObjectHandler<Measure<?>> getNew(String name, String path, Object hardcodedValue) {
        @SuppressWarnings("unchecked")
        AnodeMeasureHandler<?> measureHandler = new AnodeMeasureHandler<?>(name, path, (Measure<?>) hardcodedValue);
        return measureHandler;
    }
    
    @Override
    public List<Class<Measure<?>>> getHandledTypes() {
        return Arrays.asList((Class<Measure<T>>Measure.class.getGenericInterfaces()[0]);
    }

    @Override
    public Measure<T> refresh(Object currentVal) {
        return defaultVal;
    }
}
