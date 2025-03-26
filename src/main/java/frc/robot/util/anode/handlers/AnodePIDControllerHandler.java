package frc.robot.util.anode.handlers;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.util.LoggedTunableNumber;

public class AnodePIDControllerHandler extends AnodeObjectHandler<PIDController>{
    private LoggedTunableNumber m_kP;
    private LoggedTunableNumber m_kI;
    private LoggedTunableNumber m_kD;

    public AnodePIDControllerHandler() {}

    /**
     * Constructor 
     * @param name
     * @param path
     * @param hardcodedValue
     */
    private AnodePIDControllerHandler(String name, String path, PIDController hardcodedValue) {
        //Start a loggedTunableNumber and default to the hardcoded value.
        m_kP = new LoggedTunableNumber(path + "/" + name + "/kP", hardcodedValue.getP());
        m_kI = new LoggedTunableNumber(path + "/" + name + "/kI", hardcodedValue.getI());
        m_kD = new LoggedTunableNumber(path + "/" + name + "/kD", hardcodedValue.getD());
    }

    /**
     * A method to be run on the prototype which constructs an instance of this handler
     */
    @Override
    public AnodeObjectHandler<PIDController> getNew(String name, String path, Object hardcodedValue) {
        return new AnodePIDControllerHandler(name,path,(PIDController)hardcodedValue);
    }

    /**
     * Returns the type we're handling. Should be the same as T
     * <p>We can't just get this from T because of java's type erasure.
     */
    @Override
    public List<Class<PIDController>> getHandledTypes() {
        return Arrays.asList(PIDController.class);
    }

    /**
     * Called when we want to update our value
     */
    @Override
    public PIDController refresh(Object currentVal) {
        //If the number changed, update it. Otherwise, keep it the same.
        return new PIDController(m_kP.getAsDouble(), m_kI.getAsDouble(), m_kD.getAsDouble());
    }
    
}
