package frc.robot.util;

import java.util.function.Consumer;

public class LoggedTunableGainsBuilder  {
    private LoggedTunableNumber kP;
    private LoggedTunableNumber kI;
    private LoggedTunableNumber kD;
    private LoggedTunableNumber kS;
    private LoggedTunableNumber kG;
    private LoggedTunableNumber kV;
    private LoggedTunableNumber kA;

    public LoggedTunableGainsBuilder(String key,double kP, double kI, double kD,
        double kS, double kG, double kV, double kA) {
        this.kP = new LoggedTunableNumber(key + "kP", kP);
        this.kI = new LoggedTunableNumber(key + "kI", kI);
        this.kD = new LoggedTunableNumber(key + "kD", kD);
        this.kS = new LoggedTunableNumber(key + "kS", kS);
        this.kG = new LoggedTunableNumber(key + "kG", kG);
        this.kV = new LoggedTunableNumber(key + "kV", kV);
        this.kA = new LoggedTunableNumber(key + "kA", kA);
    }

    public void ifGainsHaveChanged(Consumer<Gains> gainsConsumer) {
        LoggedTunableNumber.ifChanged(hashCode(), () -> {
            gainsConsumer.accept(build());
        }, kP, kI, kD, kS, kG, kV, kA);
    }

    public Gains build() {
        return Gains.builder()
        .kP(kP.get())
        .kI(kI.get())
        .kD(kD.get())
        .kS(kS.get())
        .kG(kG.get())
        .kV(kV.get())
        .kA(kA.get())
        .build();
    }
}