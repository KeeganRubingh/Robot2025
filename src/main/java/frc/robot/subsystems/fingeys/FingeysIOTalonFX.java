package frc.robot.subsystems.fingeys;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.CanDef;
import frc.robot.util.PhoenixUtil;

public class FingeysIOTalonFX implements FingeysIO {
  public VoltageOut Request;
  public TalonFX Motor;

  public FingeysIOTalonFX(CanDef canbus) {
    Motor = new TalonFX(canbus.id(),canbus.bus());
    Request = new VoltageOut(0.0);
    
    configureTalons();
  }

  private void configureTalons() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.CurrentLimits.StatorCurrentLimit = 80.00;
    cfg.CurrentLimits.StatorCurrentLimitEnable = true;
    cfg.CurrentLimits.SupplyCurrentLimit = 40.00;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    cfg.Voltage.PeakForwardVoltage = 16;
    cfg.Voltage.PeakReverseVoltage = 16;
    cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    PhoenixUtil.tryUntilOk(5, () -> Motor.getConfigurator().apply(cfg));
  }

  @Override
  public void updateInputs(FingeysInputs inputs) {
    inputs.angularVelocity.mut_replace(Motor.getVelocity().getValue());
    // inputs.voltageSetPoint.mut_replace(
    //     Voltage.ofRelativeUnits(
    //         ((VoltageOut) Motor.getAppliedControl()).Output, Volts));
    inputs.voltage.mut_replace(Motor.getMotorVoltage().getValue());
    inputs.supplyCurrent.mut_replace(Motor.getStatorCurrent().getValue());
  }

  @Override
  public void setTarget(Voltage target) {
    Request = Request.withOutput(target);
    Motor.setControl(Request);
  }

  @Override
  public void stop() {
    Motor.setControl(new StaticBrake());
  }
}
