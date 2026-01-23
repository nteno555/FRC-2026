package frc2025.subsystems.intake;

import frc2025.config.SubsystemConstants;

public class IntakeIOSim implements IntakeIO {
    private double voltage;
    private double position;

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.current = 0;
        if(voltage<0){
            //inputs.canRangeDistance = 999999;
        }
        if(voltage>0){
            //inputs.canRangeDistance = 0;
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
    public void setDeployPosition() {
        this.position = SubsystemConstants.intakeDeployPosition;
    }

    @Override
    public void setZeroPosition() {
        this.position = 0.0;
    }
}
