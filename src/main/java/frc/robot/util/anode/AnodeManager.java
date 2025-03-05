package frc.robot.util.anode;

import java.lang.reflect.Field;
import java.lang.reflect.Type;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.anode.CustomAnodeDef;

public class AnodeManager {
    private static AnodeManager instance;
    private Map<Object, Anode[]> m_anodes = new HashMap<>();

    /**
     * Represents a Logged Tunable value
     */
    protected static class Anode {
        public Object parent;
        public LoggedTunableNumber ltn;
        public Field target;
        public static Map<Type,CustomAnodeDef> customTypes = new HashMap<>();

        public Anode(Object parent, Field target, LoggedTunableNumber ltn) {
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
            ArrayList<Field> usableFields = new ArrayList<>();
            ArrayList<CustomAnodeDef> fieldCustomTypes = new ArrayList<>();
            Field instanceNameField = null;
            String instanceName = null;
            ArrayList<Anode> returnedAnodes = new ArrayList<>();

            //Iterate over all the fields of the object
            for (Field f : lParent.getClass().getDeclaredFields()) {
                //Is this field a tunable param?
                if(f.getAnnotation(AnodeTunableParameter.class) != null) {
                    //Is it a double?
                    if((f.getType().equals(Double.TYPE) || f.getType().equals(double.class))) {
                        // If so, try to make it accessible, add the field to usableFields, and give it no custom type (defaults to double)
                        f.trySetAccessible();
                        usableFields.add(f);
                        fieldCustomTypes.add(null);
                    //Do we have a custom type to handle it?
                    } else if(customTypes.containsKey(f.getType())) {
                        // // If so, try to make it accessible, add the field to usableFields, and store the custom type of this field
                        // f.trySetAccessible();
                        // usableFields.add(f);
                        // fieldCustomTypes.add(null);
                    }
                }
                //Is this field our instance name, and a string?
                if(f.getAnnotation(AnodeInstanceName.class) != null && f.getType().equals(String.class) ) {
                    //If so, try to make it accessible and store it as our instance name field
                    f.trySetAccessible();
                    instanceNameField = f;
                }
            }
            //If we got an instance name set it, otherwise use a default.
            if(instanceNameField != null) {
                try {
                    instanceName = instanceNameField.get(lParent).toString();
                } catch (Exception e) {
                    System.err.println(e);
                    return null;
                }
            } else {
                instanceName = lParent.getClass().getSimpleName() + lParent.hashCode();
            }

            AnodeObject pathsource = target.getClass().getAnnotation(AnodeObject.class);
            if(pathsource != null) {
                try{
                    lName = pathsource.Key()  + "/" + instanceName;
                } catch (Exception e) {
                    System.err.println(e);
                    return null;
                }
            } else {
                lName = "UNNAMED/" + instanceName;
            }

            for(int i = 0; i < usableFields.size(); i++) {
                Field f = usableFields.get(i);

                String finalName = lName + "/"+ f.getAnnotation(AnodeTunableParameter.class).Key();

                //If the custom field is null default to double
                if (fieldCustomTypes.get(i) == null) {
                    Double value = 0.0;
                    try{
                        value = f.getDouble(lParent);
                    } catch (Exception e) {
                        System.err.println(e);
                    }
                    returnedAnodes.add(
                        new Anode(lParent, f, new LoggedTunableNumber(finalName,value))
                    );
                //Otherwise let it finish up the initialization
                } else {
                    // returnedAnodes.addAll(fieldCustomTypes.get(i).getAnodes(lParent, f, instanceName));
                }
            }

            return returnedAnodes.toArray(new Anode[returnedAnodes.size()]);
        }
    }

    private AnodeManager() {};

    public void addObjects(Object object) {
        Anode[] anodes = Anode.makeAnodesForObject(object);
            if(anodes == null) {
                DriverStation.reportError("Something went wrong with Anode!", true);
                return;
            }
        m_anodes.put(object, anodes);
    }

    public void addObjects(Object... objects) {
        for (Object target : objects) {
            addObjects(target);
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

    public void addCustomDefs(CustomAnodeDef handler) {
        // Anode.customTypes.put(handler.getTargetType(), handler);
    }

    public void addCustomDefs(CustomAnodeDef... handlers) {
        // for (CustomAnodeDef customAnodeDef : handlers) {
        //     addCustomDefs(handlers);
        // }
    }
}