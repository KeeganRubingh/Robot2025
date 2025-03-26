package frc.robot.util.anode.handlers;

import java.lang.reflect.Type;
import java.util.List;

public abstract class AnodeObjectHandler<T> {
    public abstract AnodeObjectHandler<T> getNew(String name, String path, Object hardcodedValue);
    public abstract List<Class<? extends T>> getHandledTypes();
    public abstract T refresh(Object currentVal);
}
