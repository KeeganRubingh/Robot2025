package frc.robot.commands;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;

import java.util.function.DoubleSupplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intakeextender.IntakeExtender;
import frc.robot.util.LoggedTunableNumber;

public class GroundOuttakeCoral extends SequentialCommandGroup {
    private static enum IntakeExtenderPositions {
        Outtake(new LoggedTunableNumber("GroundOuttake/IntakeExtender/OuttakeDegrees",StowToGroundIntake.intakeExtenderFinal.in(Degrees))),
        Stow(new LoggedTunableNumber("GroundOuttake/IntakeExtender/StowDegrees",StowToGroundIntake.intakeExtenderStow.in(Degrees)));

        DoubleSupplier position;
        MutAngle distance;

        IntakeExtenderPositions(DoubleSupplier position) {
            this.position = position;
            this.distance = Degrees.mutable(0.0);
        }

        public Angle angle() {
            this.distance.mut_replace(this.position.getAsDouble(), Degrees);
            return this.distance;
        }
    }

    public GroundOuttakeCoral(Intake intake, IntakeExtender extender) {
        super(
            extender.getNewIntakeExtenderTurnCommand(IntakeExtenderPositions.Outtake.position)
            .alongWith(new WaitUntilCommand(extender.getNewAtAngleTrigger(IntakeExtenderPositions.Outtake.position, Degrees.of(10))).withTimeout(1.0)),
            intake.getNewSetVoltsCommand(-5)
        ); 
    }

    public static Command GroundOuttakeToStow(Intake intake, IntakeExtender extender) {
        return intake.getNewSetVoltsCommand(0)
        .andThen(extender.getNewIntakeExtenderTurnCommand(IntakeExtenderPositions.Stow.position));
    }
}
