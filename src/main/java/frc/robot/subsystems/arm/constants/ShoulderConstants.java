package frc.robot.subsystems.arm.constants;

import static edu.wpi.first.units.Units.*;

import java.util.function.Consumer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import frc.robot.RobotState;
import frc.robot.util.CanDef;
import frc.robot.util.CanDef.CanBus;
import frc.robot.util.Gains;

public class ShoulderConstants extends ArmJointConstants {
    public ShoulderConstants() {
        this.LeaderProfile = CanDef.builder().id(0).bus(CanBus.Rio).build();

        this.SimGains =
            Gains.builder().kS(0.0).kG(0.0).kV(0.0).kA(0.0).kP(0.1).kI(0.0).kD(0.0).build();

        this.TalonFXGains =
            Gains.builder().kS(0.0).kG(0.0).kV(0.0).kA(0.0).kP(0.0).kI(0.0).kD(0.0).build();

        this.MaxVelocity = DegreesPerSecond.of(360);
        this.MaxAcceleration = DegreesPerSecondPerSecond.of(360);
        this.MaxJerk = 0.0;
        this.TorqueCurrentLimit = Amps.of(120);
        this.SupplyCurrentLimit = Amps.of(40);
        this.ForwardTorqueLimit = Amps.of(80);
        this.ReverseTorqueLimit = Amps.of(-80);

        this.NumMotors = 2;
        this.Gearing = 75;
        this.Length = Inches.of(24.719);
        this.Weight = Pounds.of(50);
        this.Motors = DCMotor.getKrakenX60(NumMotors);
        this.MaximumAngle = Degrees.of(360);
        this.MinimumAngle = Degrees.of(0);
        this.StartingAngle = Degrees.zero();

        this.XPosition = Meters.of(0.07);
        this.YPosition = Inches.of(0);
        this.ZPosition = Meters.of(0.377);
        this.PitchModifier = Degrees.of(84);

        this.LoggedName = "Shoulder";
        this.mechanismSimCallback = (d) -> {
            RobotState.instance().setShoulderSource(d);
        };
    }
}
