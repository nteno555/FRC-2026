package frc2025.subsystems.indexer;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc2025.config.SubsystemConstants;
import org.littletonrobotics.junction.Logger;

public class IndexerIOReal implements IndexerIO {
    //since the class implements from IndexerIO, it must have all the methods that were created in the interface
    //create 2 talon motors
    //make 2 requests, 1 speedRequest of type DutyCycleOut and 1 voltageRequest of type VoltageOut
    //create constructor for this class
        //first, apply a temporary configuration to the indexer motor
        //make a var indexerConfigs and initialize as a new talon fx configuration
        //set supply and stator current limits as well as enable both of them
        //make a new torque configuration configTorque and set peak current forward and backward at 100
        //apply configTorque and indexerConfigs to both indexers with .apply(var config, int time) method with time = 0.05
        //set indexer2 as a follower of indexer (motor 1) with .setControl()
    //update inputs method
        //set the voltage and current for both indexers with .getMotorVoltage() and .getCurrentVoltage() and .getAsDouble()
        //calculate totalCurrent by adding everything
    //for setSpeed and setVoltage methods, use .setControl() and the 2 requests
}