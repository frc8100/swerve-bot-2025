package frc.robot;

import frc.robot.subsystems.swerve.SwerveConfig;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;

// import 

public final class CTREConfigs {

    public CANcoderConfiguration swerveCanCoderConfig = new CANcoderConfiguration();

    public CTREConfigs() {
        // swerveCanCoderConfig = new CANCoderConfiguration();

        /* Swerve CANCoder Configuration */
        // swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        // swerveCanCoderConfig.sensorDirection = SwerveConfig.canCoderInvert;
        // swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        // swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;

        // Set up the CANCoder configuration

        // TODO: Update to phoenix 6 (missing config initializationStrategy, SensorDirection from boolean to enum)
        // canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive
        swerveCanCoderConfig.MagnetSensor.SensorDirection = SwerveConfig.canCoderInvert ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;
        // canCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0_To360;
        swerveCanCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        // canCoderConfig.MagnetSensor.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        // canCoderConfig.MagnetSensor.sensorTimeBase = SensorTimeBase.PerSecond;
    }
}
