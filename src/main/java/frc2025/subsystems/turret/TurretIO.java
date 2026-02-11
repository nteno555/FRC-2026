package frc2025.subsystems.turret;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
    @AutoLog
    public static class TurretIOInputs {
        double positionTicks;
        double positionDegrees;
        double positionDegreesReal;
        double positionRadians;
        boolean throughboresConnected;
        boolean tb1Connected;
        boolean tb2Connected;
    }

    public void updateInputs(TurretIOInputs inputs);
    public void setPosition(double position);
    public void setPosition(double position, double velocity);
    public void setVoltage(double voltage);
    public void rezero();
    public void rezero(double position);
    public void rezeroCRT();

    public double toRadians(double position);
    public double toRotations(double radians);

}