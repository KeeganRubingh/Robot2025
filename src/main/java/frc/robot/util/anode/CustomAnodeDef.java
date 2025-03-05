package frc.robot.util.anode;

import java.lang.reflect.Field;
import java.lang.reflect.Type;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.util.LoggedTunableNumber;

/**
 * Used to define ways to handle arbitrary types (other than doubles)
 * <p>Note that the way you use this can modify things in dangerous ways. <b>Make sure you know what you're doing when creating a custom def!</b></p>
 */
public class CustomAnodeDef {
    // private Type targetType = null;
    // private Field[] handledFields = null;
    // private String[] fieldNames = null;

    // /**
    //  * Creates a custom anode def
    //  * @param targetType The type that will be handled
    //  * @param handledFields Which fields of the type are to be handled
    //  * @param fieldNames What these fields will be named in the SmartDashboard
    //  */
    // public CustomAnodeDef(Type targetType, Field[] handledFields, String[] fieldNames) {
    //     this.targetType = targetType;
    //     this.handledFields = handledFields;
    //     this.fieldNames = fieldNames;
    // }

    // protected List<AnodeManager.Anode> getAnodes(Object parent, Field target, String name) {
    //     ArrayList<AnodeManager.Anode> anodes = new ArrayList<>();
    //     Object internalParent = null;
    //     try {
    //         internalParent = target.get(parent);
    //     } catch (Exception e) {
    //         System.err.println(e);
    //         return new ArrayList<>();
    //     }

    //     String fullName = "";
    //     for(int i = 0; i < handledFields.length; i++) {
    //         fullName = name + fieldNames[i];
    //         handledFields[i].trySetAccessible();
    //         try {
    //             if(internalParent.getClass().getField(handledFields[i].getName()) == null ) {
    //                 System.err.println("Field doesn't exist!");
    //                 continue;
    //             }
    //             anodes.add(
    //                 new AnodeManager.Anode(
    //                     internalParent, 
    //                     handledFields[i],
    //                     new LoggedTunableNumber(fullName,handledFields[i].getDouble(target.get(parent)))
    //                 )
    //             );
    //         } catch (Exception e) {
    //             System.err.println(e);
    //             return new ArrayList<>();
    //         }
    //     }

    //     return anodes;
    // }

    // public Type getTargetType() {
    //     return targetType;
    // }

    // public static CustomAnodeDef[] getDefaultCustomDefs() {
    //     try {
    //         return new CustomAnodeDef[] {
    //             new CustomAnodeDef(
    //                 PIDController.class,
    //                 new Field[] {
    //                     PIDController.class.getDeclaredField("m_kp"),
    //                     PIDController.class.getDeclaredField("m_ki"),
    //                     PIDController.class.getDeclaredField("m_kd")
    //                 }, 
    //                 new String[] {
    //                     "P",
    //                     "I",
    //                     "D"
    //                 }
    //             )
    //         };
    //     } catch (Exception e) {
    //         System.err.println(e);
    //         return new CustomAnodeDef[0];
    //     }
    // }
}
