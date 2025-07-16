/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot;

// Drive Systems
import edu.wpi.first.wpilibj.DriverStation;

import java.io.File;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc4388.utility.controller.XboxController;
import frc4388.utility.controller.ButtonBox;
import frc4388.utility.controller.DeadbandedXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// Commands
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
// Autos
import frc4388.utility.controller.VirtualController;
import frc4388.robot.commands.wait.waitSupplier;
import frc4388.robot.constants.Constants.OIConstants;
import frc4388.robot.subsystems.differential.DiffDrive;

import com.pathplanner.lib.commands.PathPlannerAuto;

// Utilites
import frc4388.utility.DeferredBlock;
import frc4388.utility.compute.TimesNegativeOne;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (2including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* RobotMap */
    
    public final RobotMap m_robotMap = RobotMap.configureReal();
    
    /* Subsystems */
    public final DiffDrive m_DiffDrive = new DiffDrive(m_robotMap.m_DiffDrive, m_robotMap.m_gyro);
    // public final LED m_robotLED = new LED();
    // public final Vision m_vision = new Vision(m_robotMap.leftCamera, m_robotMap.rightCamera);
    // public final Elevator m_robotElevator = new Elevator(m_robotMap.elevator, m_robotMap.endeffector, m_robotMap.basinLimitSwitch, m_robotMap.endeffectorLimitSwitch, m_robotMap.IRIntakeBeam, m_robotLED);
    // public final SwerveDrive m_robotSwerveDrive = new SwerveDrive(m_robotMap.swerveDrivetrain, m_vision);
    // public final SwerveDrive m_robotSwerveDrive = new SwerveDrive(m_robotMap.swerveDrivetrain);


    /* Controllers */
    private final DeadbandedXboxController m_driverXbox   = new DeadbandedXboxController(OIConstants.XBOX_DRIVER_ID);
    private final DeadbandedXboxController m_operatorXbox = new DeadbandedXboxController(OIConstants.XBOX_OPERATOR_ID);
    private final ButtonBox m_buttonBox = new ButtonBox(OIConstants.BUTTONBOX_ID);

    // public List<Subsystem> subsystems = new ArrayList<>();

    // ! Teleop Commands
    public void stop() {
        // new InstantCommand(()->{}, m_robotSwerveDrive).schedule();
        // m_robotSwerveDrive.stopModules();
        // Constants.AutoConstants.Y_OFFSET_TRIM.set(0);
    }

    // ! /*  Autos */
    private SendableChooser<String> autoChooser;
    private Command autoCommand;

    // private Command waitFeedStation = new waitSupplier(m_robotElevator::readyToMove);
    // private Command waitDebuger = new waitSupplier(m_driverXbox::getYButtonPressed);
    // private Command waitDebugerManual = new waitSupplier(m_driverXbox::getYButtonPressed);
    private Command waitDebuger = new waitSupplier(() -> true);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureButtonBindings();        
        configureVirtualButtonBindings();
        
        DeferredBlock.addBlock(() -> { // Called on first robot enable
            // m_robotSwerveDrive.resetGyro();
        }, false);
        DeferredBlock.addBlock(() -> { // Called on every robot enable
            TimesNegativeOne.update();
        }, true);

        DriverStation.silenceJoystickConnectionWarning(true);
        // CameraServer.startAutomaticCapture();

        /* Default Commands */
        // ! Swerve Drive Default Command (Regular Rotation)
        // drives the robot with a two-axis input from the driver controller
        m_DiffDrive.setDefaultCommand(new RunCommand(() -> {
            m_DiffDrive.arcadeDrive(m_driverXbox.getLeft(), m_driverXbox.getRight());
        }, m_DiffDrive)
        .withName("SwerveDrive DefaultCommand"));

        makeAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        // this.subsystems.add(m_robotSwerveDrive);
        // this.subsystems.add(m_robotMap.leftFront);
        // this.subsystems.add(m_robotMap.rightFront);
        // this.subsystems.add(m_robotMap.rightBack);
        // this.subsystems.add(m_robotMap.leftBack);

        // ! Swerve Drive One Module Test
        // m_robotSwerveDrive.setDefaultCommand(new RunCommand(() -> {
        //     m_robotMap.rightFront.go(getDeadbandedDriverController().getLeft());
        // }

        // ! Swerve Drive Default Command (Orientation Rotation)
        // m_robotSwerveDrive.setDefaultCommand(new RunCommand(() -> {
        //     m_robotSwerveDrive.driveWithInputOrientation(getDeadbandedDriverController().getLeft(), 
        //                                                  getDeadbandedDriverController().getRightX(), 
        //                                                  getDeadbandedDriverController().getRightY(), 
        //                                                  true);
        // }, m_robotSwerveDrive))
        // .withName("SwerveDrive OrientationCommand"));
        // continually sends updates to the Blinkin LED controller to keep the lights on
        // m_robotLED.setDefaultCommand(new RunCommand(() -> m_robotLED.updateLED(), m_robotLED));

        // m_robotSwerveDrive.setDefaultCommand(new RunCommand(() -> {
        //     m_robotSwerveDrive.driveWithInput(
        //                                     getDeadbandedDriverController().getLeft(), 
        //                                     getDeadbandedDriverController().getRight(),
        //                                     true);
        // }, m_robotSwerveDrive));




    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

        new JoystickButton(m_driverXbox, XboxController.A_BUTTON).onTrue(new InstantCommand(() -> {
            m_DiffDrive.resetOdometry();
        }));

        // ? /* Programer Buttons (Controller 3)*/

        // * /* Auto Recording */
        // new JoystickButton(m_autoRecorderXbox, XboxController.LEFT_BUMPER_BUTTON)
        //     .whileTrue(new neoJoystickRecorder(m_robotSwerveDrive,
        //                 new DeadbandedXboxController[]{getDeadbandedDriverController(), getDeadbandedOperatorController()},
        //                                     () -> autoplaybackName.get()))
        //     .onFalse(new InstantCommand());
        
        // new JoystickButton(m_autoRecorderXbox, XboxController.RIGHT_BUMPER_BUTTON)
        //     .onTrue(new neoJoystickPlayback(m_robotSwerveDrive,
        //     () -> autoplaybackName.get(),
        //     new VirtualController[]{getVirtualDriverController(), getVirtualOperatorController()},
        //     true, false))
        //     .onFalse(new InstantCommand());

    }
    
    /**
     * This method is used to replcate {@link Trigger Triggers} for {@link VirtualController Virtual Controllers}. <p/>
     * Please use {@link RobotContainer#DualJoystickButton} in {@link RobotContainer#configureButtonBindings} for standard buttons.
     */
    private void configureVirtualButtonBindings() {

        // ? /* Driver Buttons */
        
        /* Notice: the following buttons have not been replicated
         * Swerve Drive Slow and Fast mode Gear Shifts : Fast mode is known to cause drift, so we disable that feature in Autoplayback
         * Swerve Drive Rotation Gear Shifts           : Same reason as Slow and Fast mode.
         * Auto Recording controls                     : We don't want an Null Ouroboros for an auto.
         */

        // ? /* Operator Buttons */

        /* Notice: the following buttons have not been replicated
         * Override Intake Position Encoder : It's an emergancy overide, for when the position of intake when the robot boots, the intake is not inside the robot.
         *                                    We don't need it in an auto.
         * Climbing controls                : We don't need to climb in auto.
         */
        
         // ? Notice: the Programer Buttons are not to be replicated because they are designed for debuging the robot, and do not need to be replicated in auto.

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {


        //return autoPlayback;
        //return new GotoPositionCommand(m_robotSwerveDrive, m_vision)
        //return autoChooser.getSelected();
	// try{
	// //     // Load the path you want to follow using its name in the GUI
    //     return autoCommand;
	// } catch (Exception e) {
	//     DriverStation.reportError("Path planner error: " + e.getMessage(), e.getStackTrace());
	    return autoCommand;
	// }
    // return new PathPlannerAuto("Line-up-no-arm");
	// zach told me to do the below comment
	//return new GotoPositionCommand(m_robotSwerveDrive, m_vision);
      //  return new GotoPositionCommand(m_robotSwerveDrive, m_vision, AutoConstants.targetpos);
    }

    public boolean autoChooserUpdated = false;
    public void makeAutoChooser() {
        autoChooser = new SendableChooser<String>();
        
        File dir;

        if(RobotBase.isReal()) {
            dir = new File("/home/lvuser/deploy/pathplanner/autos/");
        } else {
            // dir = new File("C:\\Users\\Ridgebotics\\Documents\\GitHub\\2025RidgeScape\\src\\main\\deploy\\pathplanner\\autos\\");
            dir = new File("/home/astatin3/Documents/GitHub/2025RidgeScape/src/main/deploy/pathplanner/autos");
        }

        String[] autos = dir.list();

        if(autos == null) return;

        for (String auto : autos) {
            if (auto.endsWith(".auto"))
                autoChooser.addOption(auto.replaceAll(".auto", ""), auto.replaceAll(".auto", ""));
            // System.out.println(auto);
        }

        autoChooser.onChange((filename) -> {
            autoChooserUpdated = true;
            // if (filename.equals("Taxi")) {
            //     autoCommand = new SequentialCommandGroup(
            //         new MoveForTimeCommand(m_robotSwerveDrive, 
            //             new Translation2d(0, -1), 
            //             new Translation2d(), 1000, true
            //     ), new InstantCommand(()-> {m_robotSwerveDrive.softStop();} , m_robotSwerveDrive));
            // } else {
                autoCommand = new PathPlannerAuto(filename);
            // }
            System.out.println("Robot Auto Changed " + filename);
        });
        // SmartDashboard.putData(autoChooser);

    }

    /**
     * A button binding for two controllers, preferably an {@link DeadbandedXboxController Xbox Controller} and {@link VirtualController Virtual Xbox Controller}
     * @param joystickA A controller
     * @param joystickB A controller
     * @param buttonNumber The button to bind to
     */
    public Trigger DualJoystickButton(GenericHID joystickA, GenericHID joystickB, int buttonNumber) {
        return new Trigger(() -> (joystickA.getRawButton(buttonNumber) || joystickB.getRawButton(buttonNumber)));
    }

    public DeadbandedXboxController getDeadbandedDriverController() {
        return this.m_driverXbox;
    }

    public DeadbandedXboxController getDeadbandedOperatorController() {
        return this.m_operatorXbox;
    }

    public ButtonBox getButtonBox() {
        return this.m_buttonBox;
    }
}
