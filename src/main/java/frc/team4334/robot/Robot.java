package frc.team4334.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// If you rename or move this class, update the build.properties file in the project root
public class Robot extends TimedRobot
{
    // Initialize an Xbox 360 controller to control the robot
    private XboxController primaryController = new XboxController(0);

    // Initialize the drivetrain motors
    private WPI_TalonSRX drivetrainMotorLeft1;
    private WPI_TalonSRX drivetrainMotorLeft2;
    private WPI_TalonSRX drivetrainMotorRight1;
    private WPI_TalonSRX drivetrainMotorRight2;

    // Initialize the sensors on the DIO ports
    private Ultrasonic ultrasonicSensorFront;
    private Ultrasonic ultrasonicSensorLeft;
    private Ultrasonic ultrasonicSensorBack;
    private Ultrasonic ultrasonicSensorRight;

    // Pairs up the drivetrain motors based on their respective side and initializes the drivetrain controlling object
    private SpeedControllerGroup drivetrainMotorGroupLeft;
    private SpeedControllerGroup drivetrainMotorGroupRight;
    private DifferentialDrive robotDrive;

    // Initialize a pneumatic compressor (setup via the roboRIO config page)
    private Compressor pneumaticCompressor = new Compressor(0);

    // Initialize the navX object
    private AHRS navX;

    // Function that is run once when the robot is first powered on
    @Override
    public void robotInit()
    {
        // Assigns all the motors to their respective objects (the number in brackets is the port # of what is connected where)
        drivetrainMotorLeft1 = new WPI_TalonSRX(0);
        drivetrainMotorLeft2 = new WPI_TalonSRX(1);
        drivetrainMotorRight1 = new WPI_TalonSRX(2);
        drivetrainMotorRight2 = new WPI_TalonSRX(3);

        // Assigns all the DIO sensors to their respective objects (the number in brackets is the port # of what is connected where)
        ultrasonicSensorFront = new Ultrasonic(4, 5);
        ultrasonicSensorLeft = new Ultrasonic(2, 3);
        ultrasonicSensorBack = new Ultrasonic(0, 1);
        ultrasonicSensorRight = new Ultrasonic(6, 7);

        // Assigns the drivetrain motors to their respective motor controller group and then passes them on to the drivetrain controller object
        drivetrainMotorGroupLeft = new SpeedControllerGroup(drivetrainMotorLeft1, drivetrainMotorLeft2);
        drivetrainMotorGroupRight = new SpeedControllerGroup(drivetrainMotorRight1, drivetrainMotorRight2);
        robotDrive = new DifferentialDrive(drivetrainMotorGroupLeft, drivetrainMotorGroupRight);

        // Sets the appropriate configuration settings for the motors
        drivetrainMotorGroupLeft.setInverted(true);
        drivetrainMotorGroupRight.setInverted(true);
        robotDrive.setSafetyEnabled(true);
        robotDrive.setExpiration(0.1);
        robotDrive.setMaxOutput(0.80);

        // Sets the appropriate configuration settings for the drivetrain encoders
        drivetrainMotorLeft1.setSensorPhase(true);
        drivetrainMotorRight1.setSensorPhase(false);

        // Enables the ultrasonic sensors to calculate distances (need to be polled to give a reading)
        ultrasonicSensorFront.setEnabled(true);
        ultrasonicSensorLeft.setEnabled(true);
        ultrasonicSensorBack.setEnabled(true);
        ultrasonicSensorRight.setEnabled(true);

        // Attempts to setup the navX object otherwise prints an error
        try
        {
            // Initializes the navX object on the roboRIO's MXP port and resets it
            navX = new AHRS(SPI.Port.kMXP);
            navX.reset();
        } catch (RuntimeException ex)
        {
            DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
        }

        // Initializes and starts a thread to poll the ultrasonics automatically (enables range finding from the ultrasonics)
        ultrasonicPollingThread();
    }

    // Function that is called periodically during test mode
    @Override
    public void testPeriodic()
    {

    }

    // Function that is called periodically during disabled mode
    @Override
    public void disabledPeriodic()
    {
        // Grabs the input values from the driverstation SmartDashboard window
        getSmartDashboardValues();

        // Calls the function to update the SmartDashboard window's values
        updateSmartDashboard();
    }

    // Function that is run once each time the robot enters autonomous mode
    @Override
    public void autonomousInit()
    {
        // Turns off the pneumatic compressor
        pneumaticCompressor.setClosedLoopControl(false);

        // Resets the navX
        navX.reset();

        // Resets the drivetrain encoders
        drivetrainMotorLeft1.setSelectedSensorPosition(0);
        drivetrainMotorRight1.setSelectedSensorPosition(0);

        // Disables motor safety for the drivetrain for autonomous
        robotDrive.setSafetyEnabled(false);
    }

    // Function that is run periodically during autonomous mode
    @Override
    public void autonomousPeriodic()
    {
        // Gets the values from the SmartDashboard
        getSmartDashboardValues();

        // Calls the function to update the SmartDashboard window's values
        updateSmartDashboard();
    }

    // Function that is called once each time the robot enters tele-operated mode
    @Override
    public void teleopInit()
    {
        // Turns on the pneumatic compressor
        pneumaticCompressor.setClosedLoopControl(true);

        // Resets the navX
        navX.reset();

        // Resets the drivetrain encoders
        drivetrainMotorLeft1.setSelectedSensorPosition(0);
        drivetrainMotorRight1.setSelectedSensorPosition(0);

        // Enables motor safety for the drivetrain for teleop
        robotDrive.setSafetyEnabled(true);
    }

    // Function that is called periodically during tele-operated mode
    @Override
    public void teleopPeriodic()
    {
        // Left Bumper
        if (primaryController.getBumper(GenericHID.Hand.kLeft))
        {
            robotDrive.arcadeDrive(1.0, -navX.getAngle() * 0.03);
        }
        // Right Bumper
        else if (primaryController.getBumper(GenericHID.Hand.kRight))
        {
            robotDrive.arcadeDrive(-1.0, -navX.getAngle() * 0.03);
        }
        // Sends the Y axis input from the left stick (speed) and the X axis input from the right stick (rotation) from the primary controller to move the robot if neither the Left Bumper or the Right Bumper were pressed
        else
        {
            robotDrive.arcadeDrive(primaryController.getY(GenericHID.Hand.kRight), primaryController.getX(GenericHID.Hand.kLeft));
        }

        // A button - Increments the navX's target angle by 15 degrees
        if (primaryController.getAButton() && !primaryController.getAButtonPressed() && primaryController.getAButtonReleased())
        {
            navX.setAngleAdjustment(navX.getAngleAdjustment() + 15);
        }
        // B button - Decrements the navX's target angle by 15 degrees
        if (primaryController.getAButton() && !primaryController.getAButtonPressed() && primaryController.getAButtonReleased())
        {
            navX.setAngleAdjustment(navX.getAngleAdjustment() - 15);
        }
        // X button - Resets the navX's target angle
        if (primaryController.getXButton() && !primaryController.getXButtonPressed() && primaryController.getXButtonReleased())
        {
            navX.reset();
        }

        // Gets the values from the SmartDashboard
        getSmartDashboardValues();

        // Calls the function to update the SmartDashboard window's values
        updateSmartDashboard();
    }

    // Function to update the visually presented data in the SmartDashboard window
    public void updateSmartDashboard()
    {
        SmartDashboard.putNumber("Ultrasonic Front", (ultrasonicSensorFront.isRangeValid() == false) ? SmartDashboard.getNumber("Ultrasonic Front", 999.0) : ultrasonicSensorFront.getRangeInches());
        SmartDashboard.putNumber("Ultrasonic Left", (ultrasonicSensorLeft.isRangeValid() == false) ? SmartDashboard.getNumber("Ultrasonic Left", 999.0) : ultrasonicSensorLeft.getRangeInches());
        SmartDashboard.putNumber("Ultrasonic Back", (ultrasonicSensorBack.isRangeValid() == false) ? SmartDashboard.getNumber("Ultrasonic Back", 999.0) : ultrasonicSensorBack.getRangeInches());
        SmartDashboard.putNumber("Ultrasonic Right", (ultrasonicSensorRight.isRangeValid() == false) ? SmartDashboard.getNumber("Ultrasonic Right", 999.0) : ultrasonicSensorRight.getRangeInches());
        SmartDashboard.putNumber("Encoder Left", drivetrainMotorLeft1.getSelectedSensorPosition());
        SmartDashboard.putNumber("Encoder Right", drivetrainMotorRight1.getSelectedSensorPosition());
        SmartDashboard.putNumber("navX Angle", navX.getAngle());
        SmartDashboard.putNumber("navX Angle Adjustment", navX.getAngleAdjustment());
        SmartDashboard.putNumber("navX Compass Heading", navX.getCompassHeading());
        SmartDashboard.putNumber("navX Fused Heading", navX.getFusedHeading());
    }

    // Function to get the values from the SmartDashboard window
    public void getSmartDashboardValues()
    {

    }

    // Function to start a new thread to poll the ultrasonic sensors
    public void ultrasonicPollingThread()
    {
        // Sets up a new thread that polls at a set interval
        Thread thread = new Thread(() -> {
            while (!Thread.interrupted())
            {
                // Pings the ultrasonic sensors
                ultrasonicSensorFront.ping();
                ultrasonicSensorLeft.ping();
                ultrasonicSensorBack.ping();
                ultrasonicSensorRight.ping();
                Timer.delay(.1);
            }
        });
        // Starts the thread
        thread.start();
    }
}
