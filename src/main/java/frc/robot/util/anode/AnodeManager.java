package frc.robot.util.anode;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.LoggedTunableNumber;

public class AnodeManager {
    private static AnodeManager instance;
    private Map<Object, Anode[]> m_anodes = new HashMap<>();

    /**
     * Represents a Logged Tunable value
     */
    private static class Anode {
        public Object parent;
        public LoggedTunableNumber ltn;
        public Field target;

        public Anode(Object parent,String key, LoggedTunableNumber ltn, Field target) {
            this.parent = parent;
            this.ltn = ltn;
            this.target = target;
        }

        /**
         * This will go through all the fields of an object using reflection, and attempt to make an Anode for any that have the proper annotations
         * @param target
         * @return
         */
        public static Anode[] makeAnodesForObject(Object target) {
            Object lParent = target;
            String lName = "";
            ArrayList<Field> params = new ArrayList<>();
            Field instanceNameParam = null;
            ArrayList<Anode> returnedAnodes = new ArrayList<>();

            for (Field f : lParent.getClass().getFields()) {
                if(f.getAnnotation(AnodeTunableParameter.class) != null && (f.getType().isInstance(Double.class) || f.getType().isInstance(Double.TYPE))) {
                    f.trySetAccessible();
                    params.add(f);
                }
                if(f.getAnnotation(AnodeInstanceName.class) != null && f.getType().isInstance(String.class) ) {
                    f.trySetAccessible();
                    instanceNameParam = f;
                }
            }
            if(instanceNameParam == null) {
                DriverStation.reportError("Class " + target.getClass().getSimpleName() + " has no InstanceName!", true);
                return null;
            }

            AnodeObject pathsource = target.getClass().getAnnotation(AnodeObject.class);
            if(pathsource != null) {
                try{
                    lName = pathsource.Key() + instanceNameParam.get(lParent);
                } catch (Exception e) {
                    System.err.println(e);
                    return null;
                }
            }

            for(Field f : params) {
                String finalName = lName + f.getAnnotation(AnodeTunableParameter.class).Key();
                Double value = 0.0;
                try{
                    value = f.getDouble(lParent);
                } catch (Exception e) {
                    System.err.println(e);
                }
                returnedAnodes.add(
                    new Anode(lParent, finalName, new LoggedTunableNumber(finalName,value), f)
                );
            }

            return returnedAnodes.toArray(new Anode[returnedAnodes.size()]);
        }
    }

    private AnodeManager(Object... objects) {
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
        instance = new AnodeManager(objects);
    }

    public static AnodeManager getInstance() {
        if(instance == null) {
            DriverStation.reportError("AnodeManager not started up before usage!!!", true);
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
                a.target.setDouble(a.parent, a.ltn.get());
            } catch (Exception e) {
                System.err.println(e);
            }
        }
    }

    public void updateAll() {
        for(Object o : m_anodes.keySet()) {
            updateObject(o);
        }
    }
}