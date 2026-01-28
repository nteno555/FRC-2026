package frc2025.subsystems.turret;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc2025.config.SubsystemConstants;
import org.littletonrobotics.junction.Logger;

public class TurretIOReal implements TurretIO{
    private final TalonFX turretMotor = new TalonFX(15, "rio");
    private final DutyCycleEncoder throughBore1 = new DutyCycleEncoder(2);
    private final DutyCycleEncoder throughBore2 = new DutyCycleEncoder(3);

    public TurretIOReal() {
        turretMotor.getConfigurator().apply(new TalonFXConfiguration());

        var talonFXConfigs = new TalonFXConfiguration();
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = SubsystemConstants.turretkS;
        slot0Configs.kV = SubsystemConstants.turretkV;
        slot0Configs.kP = SubsystemConstants.turretkP;
        slot0Configs.kI = SubsystemConstants.turretkI;
        slot0Configs.kD = SubsystemConstants.turretkD;

        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = SubsystemConstants.turretCruiseVelocity;
        motionMagicConfigs.MotionMagicAcceleration = SubsystemConstants.turretAcceleration;
        motionMagicConfigs.MotionMagicJerk = SubsystemConstants.turretJerk;

        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = SubsystemConstants.turretSupplyCurrentLimit;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = SubsystemConstants.turretStatorCurrentLimit;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        TorqueCurrentConfigs configTorque =  new TorqueCurrentConfigs();
        configTorque.PeakForwardTorqueCurrent = SubsystemConstants.turretPeakForwardTorqueCurrent;
        configTorque.PeakReverseTorqueCurrent = SubsystemConstants.turretPeakReverseTorqueCurrent;
        configTorque.TorqueNeutralDeadband = SubsystemConstants.turretTorqueNeutralDeadband;

        turretMotor.getConfigurator().apply(talonFXConfigs);
        turretMotor.getConfigurator().apply(configTorque);
        turretMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public double toRadians(double position) {
        return 1/SubsystemConstants.turretRotationsPerRadian * position;
    }
    public double toRotations(double radians) {
        return SubsystemConstants.turretRotationsPerRadian * radians;
    }

    private ControlRequest createControlRequest(double targetValue, boolean useVoltage) {
        if (useVoltage) {
            return new VoltageOut(targetValue);
        } else {
            return new MotionMagicVoltage(targetValue)
                    .withEnableFOC(true)
                    .withSlot(0)
                    .withOverrideBrakeDurNeutral(true)
                    .withLimitForwardMotion(false)
                    .withLimitReverseMotion(false);
        }
    }


    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.positionTicks = turretMotor.getPosition().getValue().in(Units.Rotation);
        inputs.positionRadians = toRadians(inputs.positionTicks);

        Logger.recordOutput("Turret/voltage", turretMotor.getMotorVoltage().getValue().in(Units.Volts));
        Logger.recordOutput("Turret/current", turretMotor.getSupplyCurrent().getValue().in(Units.Amp));
    }

    @Override
    public void setPosition(double radians) {
        double position = radians * SubsystemConstants.turretRotationsPerRadian;
        turretMotor.setControl(createControlRequest(position, false));
        Logger.recordOutput("Turret/setPosPosition", position);
    }

    @Override
    public void setVoltage(double voltage) {
        turretMotor.setControl(createControlRequest(voltage, true));
    }

    @Override
    public void rezero() {
        double currentPos = throughBore1.get();
        double normalizedPos = (currentPos - SubsystemConstants.throughBoreZero1 + 1) % 1;
        double currentRadians = normalizedPos * 2 * Math.PI;
        turretMotor.setPosition(currentRadians * SubsystemConstants.turretRotationsPerRadian);
//        double tb1RotToRad = (Math.PI*2) / SubsystemConstants.throughBoreRatio1;
//        double tb2RotToRad = (Math.PI*2) / SubsystemConstants.throughBoreRatio2;
//        double smallestPos1 = ((throughBore1.get() - SubsystemConstants.throughBoreZero1 + 1) % 1) * tb1RotToRad;
//        double smallestPos2 = ((1 - throughBore2.get() - SubsystemConstants.throughBoreZero2 + 1) % 1) * tb2RotToRad;
//        double epsilon = 5;
//
//        while(Math.abs(smallestPos1 - smallestPos2) > epsilon) {
//            if(smallestPos1 < smallestPos2) smallestPos1 += tb1RotToRad;
//            else smallestPos2 += tb2RotToRad;
//        }
//
//        turretMotor.setPosition(- 1 * smallestPos1 * SubsystemConstants.turretRotationsPerRadian);
    }
}