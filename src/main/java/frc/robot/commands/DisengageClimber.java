package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.DoubleSupplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.climber.Climber;
import frc.robot.util.LoggedTunableNumber;

public class DisengageClimber extends SequentialCommandGroup {

    
    private static final String className = DisengageClimber.class.getSimpleName();


    
    public DisengageClimber(Climber climber) {
        super(
            climber.getNewSetServoAngleCommand(90.0),
            new WaitCommand(0.1),
            climber.getNewSetVoltsCommand(2.0)
        );
    }
}
