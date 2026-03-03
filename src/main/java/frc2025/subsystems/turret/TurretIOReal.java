package frc2025.subsystems.turret;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc2025.RobotState;
import frc2025.config.SubsystemConstants;
import org.littletonrobotics.junction.Logger;

public class TurretIOReal implements TurretIO{
    private final TalonFX turretMotor = new TalonFX(SubsystemConstants.turretID, "rio");
    private final CANcoder throughBore1 = new CANcoder(SubsystemConstants.turretTb1ID);
    private final CANcoder throughBore2 = new CANcoder(SubsystemConstants.turretTb2ID);

    public TurretIOReal() {
        turretMotor.getConfigurator().apply(new TalonFXConfiguration());

        var talonFXConfigs = new TalonFXConfiguration();

        var motorOutputConfigs = talonFXConfigs.MotorOutput;
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;

        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = SubsystemConstants.turretkS;
        slot0Configs.kV = SubsystemConstants.turretkV;
        slot0Configs.kA = SubsystemConstants.turretkA;
        slot0Configs.kP = SubsystemConstants.turretkP;
        slot0Configs.kI = SubsystemConstants.turretkI;
        slot0Configs.kD = SubsystemConstants.turretkD;
        slot0Configs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = SubsystemConstants.turretCruiseVelocity;
        motionMagicConfigs.MotionMagicAcceleration = SubsystemConstants.turretAcceleration;
        motionMagicConfigs.MotionMagicJerk = SubsystemConstants.turretJerk;

        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = SubsystemConstants.turretSupplyCurrentLimit;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = SubsystemConstants.turretStatorCurrentLimit;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        var voltageConfigs = talonFXConfigs.Voltage;
        voltageConfigs.PeakForwardVoltage = SubsystemConstants.turretVoltageLimit;
        voltageConfigs.PeakReverseVoltage = -SubsystemConstants.turretVoltageLimit;

        TorqueCurrentConfigs configTorque =  new TorqueCurrentConfigs();
        configTorque.PeakForwardTorqueCurrent = SubsystemConstants.turretPeakForwardTorqueCurrent;
        configTorque.PeakReverseTorqueCurrent = SubsystemConstants.turretPeakReverseTorqueCurrent;

        var softLimits = talonFXConfigs.SoftwareLimitSwitch;
        softLimits.ForwardSoftLimitThreshold = SubsystemConstants.turretMaxAngleRadians * SubsystemConstants.turretRotationsPerRadian;
        softLimits.ForwardSoftLimitEnable = true;
        softLimits.ReverseSoftLimitThreshold = SubsystemConstants.turretMinAngleRadians * SubsystemConstants.turretRotationsPerRadian;
        softLimits.ReverseSoftLimitEnable = true;

        turretMotor.getConfigurator().apply(motorOutputConfigs, 0.05);
        turretMotor.getConfigurator().apply(talonFXConfigs, 0.05);
        turretMotor.getConfigurator().apply(configTorque, 0.05);
        turretMotor.getConfigurator().apply(voltageConfigs, 0.05);
        turretMotor.setNeutralMode(NeutralModeValue.Brake);
        rezero();

        throughBore1.getConfigurator().apply(new CANcoderConfiguration());
        throughBore2.getConfigurator().apply(new CANcoderConfiguration());

        var tb1Config = new CANcoderConfiguration();
        var tb2Config = new CANcoderConfiguration();

        tb1Config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        tb2Config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        throughBore1.getConfigurator().apply(tb1Config);
        throughBore2.getConfigurator().apply(tb2Config);
    }

    public double toRadians(double position) {
        return 1/SubsystemConstants.turretRotationsPerRadian * position;
    }
    public double toRotations(double radians) {
        return SubsystemConstants.turretRotationsPerRadian * radians;
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.positionTicks = turretMotor.getPosition().getValue().in(Units.Rotation);
        inputs.positionRadians = toRadians(inputs.positionTicks);
        inputs.throughboresConnected = throughBore1.isConnected() && throughBore2.isConnected();
        inputs.tb1Connected = throughBore1.isConnected();
        inputs.tb2Connected = throughBore2.isConnected();
        inputs.velocity = turretMotor.getVelocity().getValueAsDouble() / SubsystemConstants.turretRotationsPerRadian;

        Logger.recordOutput("Turret/voltage", turretMotor.getMotorVoltage().getValue().in(Units.Volts));
        Logger.recordOutput("Turret/current", turretMotor.getSupplyCurrent().getValue().in(Units.Amp));
        Logger.recordOutput("Turret/tb1AbsPosition", throughBore1.getAbsolutePosition().getValue().in(Units.Rotations) + 0.5);
        Logger.recordOutput("Turret/tb2AbsPosition", throughBore2.getAbsolutePosition().getValue().in(Units.Rotations) + 0.5);
//        Logger.recordOutput("Turret/tb1RelativePosition", throughBore1.getPosition().getValue().in(Units.Rotations));
//        Logger.recordOutput("Turret/tb2RelativePosition", throughBore2.getPosition().getValue().in(Units.Rotations));
        Logger.recordOutput("Turret/RPS", turretMotor.getVelocity().getValueAsDouble());
    }

    @Override
    public void setPosition(double position) {
        turretMotor.setControl(new PositionVoltage(position)
                .withSlot(0)
                .withEnableFOC(true));
        Logger.recordOutput("Turret/setPosPosition", position);
    }

    @Override
    public void setPosition(double position, double velocity) {
        turretMotor.setControl(
                new PositionVoltage(position)
                        .withVelocity(velocity)
                        .withSlot(0)
                        //.withEnableFOC(true)
        );
    }

    @Override
    public void setVoltage(double voltage){
        Logger.recordOutput("Turret/softwareVoltage", voltage);
        turretMotor.setControl(new VoltageOut(voltage));
    }

    @Override
    public void rezero() {
//        rezeroCRT();
        turretMotor.setPosition(0.0);
    }

    @Override
    public void rezero(double position) {
        turretMotor.setPosition(position);
    }

    public void rezeroCRT() {
        if (!SubsystemConstants.useTurretThroughBores) {
            turretMotor.setPosition(SubsystemConstants.turretHomeAngleRotations);
        } else {
            if (throughBore1.isConnected() && throughBore2.isConnected()) {
                double tb1Pos = throughBore1.getAbsolutePosition().getValueAsDouble() + 0.5;
                double tb2Pos = throughBore2.getAbsolutePosition().getValueAsDouble() + 0.5;

                double pos1 = ((tb1Pos - SubsystemConstants.throughBoreZero1 + 1) % 1) * SubsystemConstants.throughBoreRatio1;
                double pos2 = ((tb2Pos - SubsystemConstants.throughBoreZero2 + 1) % 1) * SubsystemConstants.throughBoreRatio2;
//                Logger.recordOutput("Turret/tb1WithOffset", tb1Pos - SubsystemConstants.throughBoreZero1);
//                Logger.recordOutput("Turret/tb2WithOffset", tb2Pos - SubsystemConstants.throughBoreZero2);
//                Logger.recordOutput("Turret/initialPos1", pos1);
//                Logger.recordOutput("Turret/initialPos2", pos2);
                double epsilon = 0.2;

                int r1Multiplier = 0;
                int r2Multiplier = 0;

                while (Math.abs(pos1 - pos2) > epsilon && pos1 > -50 && pos2 > -50) {
                    if (pos1 > pos2) {
                        pos1 -= SubsystemConstants.throughBoreRatio1;
                        r1Multiplier--;
                    }
                    else {
                        pos2 -= SubsystemConstants.throughBoreRatio2;
                        r2Multiplier--;
                    }
                }

                while (Math.abs(pos1 - pos2) > epsilon && pos1 < 20 && pos2 < 20) {
                    if (pos1 < pos2) {
                        pos1 += SubsystemConstants.throughBoreRatio1;
                        r1Multiplier++;
                    }
                    else  {
                        pos2 += SubsystemConstants.throughBoreRatio2;
                        r2Multiplier++;
                    }
                }

                turretMotor.setPosition(-(pos1 - toRotations(SubsystemConstants.turretZeroOffsetRadians)));

//                Logger.recordOutput("Turret/finalPos1", pos1);
//                Logger.recordOutput("Turret/finalPos2", pos2);
//                Logger.recordOutput("Turret/r1Multiplier", r1Multiplier);
//                Logger.recordOutput("Turret/r2Multiplier", r2Multiplier);
            } else if (throughBore1.isConnected()) {
                double pos1 = ((throughBore1.getAbsolutePosition().getValueAsDouble() - SubsystemConstants.throughBoreZero1 + 1) % 1) * SubsystemConstants.throughBoreRatio1;
                turretMotor.setPosition(-(pos1 - toRotations(SubsystemConstants.turretZeroOffsetRadians)));
            } else {
                double pos2 = ((throughBore2.getAbsolutePosition().getValueAsDouble() - SubsystemConstants.throughBoreZero2 + 1) % 1) * SubsystemConstants.throughBoreRatio2;
                turretMotor.setPosition(-(pos2 - toRotations(SubsystemConstants.turretZeroOffsetRadians)));
            }
        }
    }

    @Override
    public void addVisionUpdate(double robotRelativeAngleRadians) {

    }

//    @Override
//    public void rezero() {
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
//    }
}