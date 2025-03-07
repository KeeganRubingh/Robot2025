package frc.robot.util.anode;

import java.lang.reflect.Field;
import java.lang.reflect.Type;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.anode.handlers.AnodeObjectHandler;

public class AnodeManager {
    private static AnodeManager instance;
    private Map<Object, Anode[]> m_anodes = new HashMap<>();
    protected Map<Type, AnodeObjectHandler<?>> m_objectHandlers = new HashMap<>();

    /**
     * Represents a Logged Tunable value
     */
    private static class Anode {
        public Object parent;
        public AnodeObjectHandler<?> handler;
        public Field target;

        public Anode(Object parent, Field target, AnodeObjectHandler<?> handler) {
            this.parent = parent;
            this.handler = handler;
            this.target = target;
        }

        /**
         * This method creates a list of anode loggers
         * @param target
         * @return
         */
        public static Anode[] makeAnodesForObject(Object target, String currentPath) {
            Object lParent = target;
            String lName = "";
            ArrayList<Field> params = new ArrayList<>();
            String instanceName = null;
            ArrayList<Anode> returnedAnodes = new ArrayList<>();
            boolean isStaticObject = false;
            
            //Flag if our object is static
            if(lParent.getClass().getAnnotation(AnodeStaticObjectFlag.class) != null) {
                isStaticObject = true;
            }

            //Check all of our declared fields to find necessary annotations
            for (Field f : lParent.getClass().getDeclaredFields()) {
                //If this field has our instance name, set it.
                if(f.getAnnotation(AnodeInstanceName.class) != null && f.getType().equals(String.class) ) {
                    f.trySetAccessible();
                    try{
                        instanceName = (String)f.get(lParent);
                    } catch (Exception e) {
                        System.err.println(e);
                        instanceName = null;
                    }
                }
                //If this field is a tunable parameter that we have a handler for, add it to our params so we can handle it later.
                if(f.getAnnotation(AnodeTunableParameter.class) != null && AnodeManager.getInstance().m_objectHandlers.containsKey(f.getType())) {
                    f.trySetAccessible();
                    params.add(f);
                    continue;
                }
            }

            if((!isStaticObject) && instanceName == null) {
                instanceName = "" + target.hashCode();
            }
            AnodeObject pathsource = target.getClass().getAnnotation(AnodeObject.class);
            //Setting our path. If we're static we do current path and object name. If we aren't we add on instance name
            lName = isStaticObject 
                ? currentPath + "/" + pathsource.Key() 
                : currentPath + "/" + pathsource.Key() + "/" + instanceName;

            for(Field f : params) {
                String finalName = f.getAnnotation(AnodeTunableParameter.class).Key();
                Object value = null;
                try{
                    value = f.get(lParent);
                } catch (Exception e) {
                    System.err.println(e);
                    value = null;
                }
                var typeHandlerPrototype = AnodeManager.getInstance().m_objectHandlers.get(f.getType());
                if(typeHandlerPrototype.getHandledTypes().contains(value.getClass())) {
                    returnedAnodes.add(
                        new Anode(lParent, f, typeHandlerPrototype.getNew(finalName, currentPath, value))
                    );
                } else {
                    System.err.println("Handler type and value type are mismatching!");
                }
            }

            return returnedAnodes.toArray(new Anode[returnedAnodes.size()]);
        }

        public static Anode[] makeAnodesForObject(Object target) {
            return makeAnodesForObject(target, "");
        }
    }

    private AnodeManager() {}
    public void AddObjects(Object... objects) {
        for (Object target : objects) {
            Anode[] anodes = Anode.makeAnodesForObject(target);
            if(anodes == null) {
                continue;
            }
            m_anodes.put(target, anodes);
        }
    }

    public static void Startup(Object... objects) {
        if(instance != null) {
            DriverStation.reportError("AnodeManager started several times!!!", true);
        }
        
    }

    public static AnodeManager getInstance() {
        if(instance == null) {
            instance = new AnodeManager();
        }
        return instance;
    }

    public void updateObject(Object target) {
        Anode[] anodes = m_anodes.getOrDefault(target, null);
        if(anodes == null) {
            DriverStation.reportError("AnodeManager does not have object \'" + target.getClass().getSimpleName() + "\' !", true);
            return;
        }

        for(Anode a : anodes) {
            try{
                a.handler.refresh(a.target.get(a.parent));
            } catch (Exception e) {
                System.err.println(e);
                continue;
            }
        }
    }

    public void updateAll() {
        for(Object o : m_anodes.keySet()) {
            updateObject(o);
        }
    }

    public void addTypeHandler(AnodeObjectHandler<?> handler) {
        m_objectHandlers.put(handler.getHandledType(), handler);
    }
}