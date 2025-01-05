// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.RobotControl;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.BaseConstants.IOConstants;

/** This class's purpose is to manage drive and operator control to the robot. */
public class ControllerIO {

    // Temporarily set to public
    public XboxController xboxController;
    public StellarController stellarController;
    public Joystick joystickController;

    // Singleton pattern handling
    private static ControllerIO primaryInst;
    private static ControllerIO secondaryInst;

    // Create enum for controller type
    public enum controllerType {
        XBOX,
        JOYSTICK,
        STELLAR
    }

    // Get singleton primary instance
    public static ControllerIO getPrimaryInstance(controllerType controller) {

        // Check if instance already exists
        if(primaryInst == null) {

            // Create new instance if not already exists
            primaryInst = new ControllerIO();

            // Initiate the specified controller type
            switch (controller) {
                case XBOX:
                    primaryInst.InitXBOX(false);
                    break;
                case JOYSTICK:
                    primaryInst.InitJOYSTICK(false);
                    break;
                case STELLAR:
                    primaryInst.InitSTELLAR(false);
                    break;
                default:
                    primaryInst.InitSTELLAR(false);
                    break;
            }
        }
        return primaryInst;
    }

    // Get singleton secondary instance
    public static ControllerIO getSecondaryInstance(controllerType controller) {
        if(secondaryInst == null) {

            // Create new secondary instance if not already exists
            secondaryInst = new ControllerIO();

            // Initiate the specified controller type
            switch (controller) {
                case XBOX:
                    secondaryInst.InitXBOX(true);
                    break;
                case JOYSTICK:
                    secondaryInst.InitJOYSTICK(true);
                    break;
                case STELLAR:
                    secondaryInst.InitSTELLAR(true);
                    break;
                default:
                    secondaryInst.InitSTELLAR(true);
                    break;
            }    
        }
        return secondaryInst;
    }

    // Initiate an xbox controller
    public void InitXBOX(boolean isSecondary) {
        xboxController = new XboxController(isSecondary ? IOConstants.kOperatorControllerPort : IOConstants.kDriverControllerPort);
        System.out.println("Initiated New Xbox Controller");
    }

    // Initiate a joystick
    public void InitJOYSTICK(boolean isSecondary) {
        joystickController = new Joystick(isSecondary ? IOConstants.kOperatorControllerPort : IOConstants.kDriverControllerPort);
        System.out.println("Initiated New Joystick Controller");
    }

    // Initiate a custom stellar controller
    public void InitSTELLAR(boolean isSecondary) {
        stellarController = new StellarController(isSecondary ? IOConstants.kOperatorControllerPort : IOConstants.kDriverControllerPort);
        System.out.println("Initiated New Stellar Controller");
    }

    // THIS CLASS IS INCOMPLETE

}
