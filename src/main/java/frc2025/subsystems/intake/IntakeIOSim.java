package frc2025.subsystems.intake;

public class IntakeIOSim implements IntakeIO {
    private double voltage;
    private double position;

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.current = 0;
//        if(voltage<0){
//            inputs.canRangeDistance = 999999;
//        }
        if(voltage>0){
            inputs.current=9999;
        }
        inputs.voltage = voltage;
    }

    @Override
    public void setSpeed(double speed) {
        voltage = speed;
    }

    @Override
    public void setVoltage(double voltage) {
        this.voltage = voltage;
    }

    @Override
    public void setPosition(double position, boolean useVoltage) {
        this.position = position;
    }
}
