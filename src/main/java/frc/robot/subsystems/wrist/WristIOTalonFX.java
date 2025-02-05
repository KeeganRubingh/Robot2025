package frc.robot.subsystems.wrist;

<<<<<<< HEAD
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
=======
>>>>>>> a713d9c (Invert Motors based on real-life testing)
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.Rotations;
import edu.wpi.first.units.measure.Angle;
import frc.robot.util.CanDef;
import frc.robot.util.Gains;
import frc.robot.util.PhoenixUtil;

public class WristIOTalonFX implements WristIO {
  public PositionVoltage Request;
  public TalonFX Motor;
  public CANcoder canCoder;

  public WristIOTalonFX(CanDef canbus,CanDef canCoderDef) {
    Motor= new TalonFX(canbus.id(), canbus.bus());
    Request = new PositionVoltage(0);
    canCoder = new CANcoder(canCoderDef.id(), canCoderDef.bus());

    Motor.setControl(Request);
    configureTalons();
  }

  private void configureTalons() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.Voltage.PeakForwardVoltage = 7;
    cfg.Voltage.PeakReverseVoltage = 7;
    cfg.CurrentLimits.StatorCurrentLimit = 80;
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = 40;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;

    cfg.Feedback.FeedbackRemoteSensorID = canCoder.getDeviceID();
    cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    cfg.Feedback.SensorToMechanismRatio = 1.0;
    cfg.Feedback.RotorToSensorRatio = 9.0;

<<<<<<< HEAD
=======
    cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    cfg.Slot0.kP = 1.0;
>>>>>>> a713d9c (Invert Motors based on real-life testing)

    PhoenixUtil.tryUntilOk(5, () -> Motor.getConfigurator().apply(cfg));
  }

  public void setTarget(Angle target) {
    Request = Request.withPosition(target);
    Motor.setControl(Request);
  }

  @Override
  public void updateInputs(WristInputs inputs) {
    inputs.wristAngle.mut_replace(Motor.getPosition().getValue());
    inputs.wristAngularVelocity.mut_replace(Motor.getVelocity().getValue());
    inputs.wristSetPoint.mut_replace(
        Angle.ofRelativeUnits(
            ((MotionMagicVoltage) Motor.getAppliedControl()).Position, Rotations));
    inputs.supplyCurrent.mut_replace(Motor.getStatorCurrent().getValue());
  }

  @Override
  public void setGains(Gains gains) {
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = gains.kP;
    slot0Configs.kI = gains.kI;
    slot0Configs.kD = gains.kD;
    slot0Configs.kS = gains.kS;
    slot0Configs.kG = gains.kG;
    slot0Configs.kV = gains.kV;
    slot0Configs.kA = gains.kA;
    PhoenixUtil.tryUntilOk(5, () -> Motor.getConfigurator().apply(slot0Configs));
  }
}
