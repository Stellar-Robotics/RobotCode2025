package frc.robot;

import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.BaseConstants.ModuleConstants;
import frc.robot.BaseConstants.PathPlannerConstants;

public final class Configs {

    public static final class MAXSwerveModule {

        public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();
        public static final SparkMaxConfig invertedConfig = new SparkMaxConfig();

        static {
            // Use module constants to calculate conversion factors and feed forward gain.
            double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                    / ModuleConstants.kDrivingMotorReduction;
            double turningFactor = 2 * Math.PI;
            double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;
            double drivingP = 0.04;
            double drivingI = 0;
            double drivingD = 0;

            drivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
            drivingConfig.encoder
                    .positionConversionFactor(drivingFactor) // meters
                    .velocityConversionFactor(drivingFactor / 60.0); // meters per second
            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to modify them for your own robot!
                    .pid(drivingP, drivingI, drivingD)
                    .velocityFF(drivingVelocityFeedForward)
                    .outputRange(-1, 1);

            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);
            turningConfig.absoluteEncoder
                    // Invert the turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering motor in the MAXSwerve Module.
                    .inverted(true)
                    .positionConversionFactor(turningFactor) // radians
                    .velocityConversionFactor(turningFactor / 60.0); // radians per second
            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(1, 0, 0)
                    .outputRange(-1, 1)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route.
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, turningFactor);
            invertedConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit)
                    .inverted(true);
            invertedConfig.encoder
                    .positionConversionFactor(drivingFactor) // meters
                    .velocityConversionFactor(drivingFactor / 60.0); // meters per second
            invertedConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to modify them for your own robot!
                    .pid(drivingP, drivingI, drivingD)
                    .velocityFF(drivingVelocityFeedForward)
                    .outputRange(-1, 1);

        }

    }

    public static final class AlgaeMechConfig {

        public static final SparkMaxConfig pickupMotorConfig = new SparkMaxConfig();
        public static final SparkMaxConfig extensionMotorConfig = new SparkMaxConfig();

        static {
                
            pickupMotorConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(30);

            extensionMotorConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(20);

        }

    }

    public static final class CoralMechConfig {

        public static final SparkMaxConfig rollerMotorConfig = new SparkMaxConfig();
        public static final SparkMaxConfig extensionMotorConfig = new SparkMaxConfig();


        static {

            rollerMotorConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(30);
        
            rollerMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.2, 0, 0);

            extensionMotorConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(25);
            extensionMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.3, 0.0001, 0)
                .maxOutput(0.25)
                .minOutput(-0.25);

        }  

    }

    public static final class ElevatorConfig {

        public static final SparkMaxConfig elevatorMotorConfig = new SparkMaxConfig();

        static {

                elevatorMotorConfig
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(25)
                        .inverted(true);
                elevatorMotorConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .pid(0.2, 0, 0);
                // elevatorMotorConfig.limitSwitch // The elevator will also have a limit switch
                //         .forwardLimitSwitchEnabled(true)
                //         .forwardLimitSwitchType(Type.kNormallyClosed)
                //         .setSparkMaxDataPortConfig();

        }

    }

    public static final class PathPlanner {

        public static RobotConfig pathPlannerConfig;
        
        // Configure Me Please
        public static Translation2d[] moduleOffsets = {
                new Translation2d(0, 0), // FL
                new Translation2d(0, 0), // FR
                new Translation2d(0, 0), // BL
                new Translation2d(0, 0)  // BR         
        };
        
        static {

                try {
                        pathPlannerConfig = RobotConfig.fromGUISettings();
                } catch (Exception e) {
                        System.out.println("PathPlanner Auto Config Failed, Falling back!");
                        pathPlannerConfig = new RobotConfig(
                                PathPlannerConstants.massKG, 
                                PathPlannerConstants.momentOfInertia, 
                                PathPlannerConstants.moduleConfig, 
                                moduleOffsets
                        );   
                }
        }
    }
}