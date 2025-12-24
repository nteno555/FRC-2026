package frc2025.util;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest;

public class SwerveSysidVoltageRequest implements LegacySwerveRequest {
    private final MotionMagicVoltage m_motionMagicControl = new MotionMagicVoltage(0).withEnableFOC(false).withFeedForward(0).withOverrideBrakeDurNeutral(false).withSlot(0);
    private final VoltageOut m_voltageOutControl = new VoltageOut(0.0);

    private double m_targetVoltage = 0.0;
    private boolean m_driveType = true;

    public SwerveSysidVoltageRequest(boolean driveType) {
        m_driveType = driveType;
    }

    public SwerveSysidVoltageRequest() {
        m_driveType = true;
    }

    /**
     *
     * @param targetVoltage Voltage for all modules to target
     * @return
     */
    public SwerveSysidVoltageRequest withVoltage(double targetVoltage) {
        this.m_targetVoltage = targetVoltage;
        return this;
    }

    @Override
    public StatusCode apply(LegacySwerveControlRequestParameters legacySwerveControlRequestParameters, LegacySwerveModule... legacySwerveModules) {
        for (var module : legacySwerveModules)
        {
            if (m_driveType) {
                // Command steer motor to zero
                module.getSteerMotor().setControl(m_motionMagicControl);

                // Command drive motor to voltage
                module.getDriveMotor().setControl(m_voltageOutControl.withOutput(m_targetVoltage));
            }
            else {
                // Command steer motor to voltage
                module.getSteerMotor().setControl(m_voltageOutControl.withOutput(m_targetVoltage));

                // Command drive motor to zero
                module.getDriveMotor().setControl(m_motionMagicControl);
            }
        }

        return StatusCode.OK;
    }
}
