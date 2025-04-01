package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import java.util.function.DoubleSupplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.arm.ArmJoint;
import frc.robot.subsystems.climber.Climber;
import frc.robot.util.LoggedTunableNumber;

public class EngageClimber extends SequentialCommandGroup {

     private static final String className = EngageClimber.class.getSimpleName();

    private enum ShoulderPositions {
        Final(new LoggedTunableNumber(className + "/shoulder/FinalEndgameDegrees", 121.5));

        DoubleSupplier position;
        MutAngle distance;

        ShoulderPositions(DoubleSupplier position) {
            this.position = position;
            this.distance = Degrees.mutable(0.0);
        }

        public Angle angle() {
            this.distance.mut_replace(this.position.getAsDouble(), Degrees);
            return this.distance;
        }
    }

    private enum ElbowPositions {
        Final(new LoggedTunableNumber(className + "/elbow/FinalEndgameDegrees", 120));

        DoubleSupplier position;
        MutAngle distance;

        ElbowPositions(DoubleSupplier position) {
            this.position = position;
            this.distance = Degrees.mutable(0.0);
        }

        public Angle angle() {
            this.distance.mut_replace(this.position.getAsDouble(), Degrees);
            return this.distance;
        }
    }
    
    public EngageClimber(Climber climber, ArmJoint shoulder, ArmJoint elbow) {
        super(
            elbow.getNewSetAngleCommand(ElbowPositions.Final.position),
            shoulder.getNewSetAngleCommand(ShoulderPositions.Final.position),
            climber.getNewSetServoAngleCommand(0.0),
            new WaitCommand(0.1),
            climber.getNewSetVoltsCommand(-2.0) // negative was forward in test
        );
    }
}
