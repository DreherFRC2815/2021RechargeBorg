// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/** Add your docs here. */
public class MagEncoder {
    private final WPI_TalonSRX talon;
    
    private double dpp = 1;

    public MagEncoder(WPI_TalonSRX t) {
        talon = t;
    }
    
    public void setDistancePerPulse(double d) {
        dpp = d;
    }

    public double getRawDistance() {
        return talon.getSelectedSensorPosition();
    }

    public double getRawVelocity() {
        return talon.getSelectedSensorVelocity();
    }

    public double getDistance() {
        return talon.getSelectedSensorPosition() * dpp;
    }

    public double getRate() {
        return talon.getSelectedSensorVelocity() * 10.0 * dpp;
    }

    public void reset() {
        talon.setSelectedSensorPosition(0);
    }
}
