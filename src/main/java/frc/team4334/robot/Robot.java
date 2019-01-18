package frc.team4334.robot;

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
    private Talon leftDriveMotor;
    private Talon rightDriveMotor;

    // Initialize the sensors on the DIO ports
    private Ultrasonic ultrasonicSensor1;
    private Ultrasonic ultrasonicSensor2;
    private Ultrasonic ultrasonicSensor3;
    private Ultrasonic ultrasonicSensor4;
    private Encoder drivetrainEncoder1;
    private Encoder drivetrainEncoder2;

    // Pairs up the drivetrain motors based on their respective side and initializes the drivetrain controlling object
    private SpeedControllerGroup leftSideDriveMotors;
    private SpeedControllerGroup rightSideDriveMotors;
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
        leftDriveMotor = new Talon(0);
        rightDriveMotor = new Talon(1);

        // Assigns all the DIO sensors to their respective objects (the number in brackets is the port # of what is connected where)
        drivetrainEncoder1 = new Encoder(20, 21, true, Encoder.EncodingType.k4X);
        drivetrainEncoder2 = new Encoder(22, 23, false, Encoder.EncodingType.k4X);
        ultrasonicSensor1 = new Ultrasonic(4, 5);
        ultrasonicSensor2 = new Ultrasonic(2, 3);
        ultrasonicSensor3 = new Ultrasonic(0, 1);
        ultrasonicSensor4 = new Ultrasonic(6, 7);

        // Assigns the drivetrain motors to their respective motor controller group and then passes them on to the drivetrain controller object
        leftSideDriveMotors = new SpeedControllerGroup(leftDriveMotor);
        rightSideDriveMotors = new SpeedControllerGroup(rightDriveMotor);
        robotDrive = new DifferentialDrive(leftSideDriveMotors, rightSideDriveMotors);

        // Sets the appropriate configuration settings for the motors
        leftSideDriveMotors.setInverted(true);
        rightSideDriveMotors.setInverted(true);
        robotDrive.setSafetyEnabled(true);
        robotDrive.setExpiration(0.1);
        robotDrive.setMaxOutput(0.80);

        // Enables the ultrasonic sensors to calculate distances (need to be polled to give a reading)
        ultrasonicSensor1.setEnabled(true);
        ultrasonicSensor2.setEnabled(true);
        ultrasonicSensor3.setEnabled(true);
        ultrasonicSensor4.setEnabled(true);

        // Sets the appropriate configuration settings for drivetrain encoders
        drivetrainEncoder1.setMaxPeriod(.1);
        drivetrainEncoder2.setMaxPeriod(.1);
        drivetrainEncoder1.setMinRate(10);
        drivetrainEncoder2.setMinRate(10);
        drivetrainEncoder1.setDistancePerPulse(5);
        drivetrainEncoder2.setDistancePerPulse(5);
        drivetrainEncoder1.setSamplesToAverage(7);
        drivetrainEncoder2.setSamplesToAverage(7);

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
        //        LiveWindow.run();
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
        drivetrainEncoder1.reset();
        drivetrainEncoder2.reset();

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
        // Resets the navX
        navX.reset();

        // Resets the drivetrain encoders
        drivetrainEncoder1.reset();
        drivetrainEncoder2.reset();

        // Turns on the pneumatic compressor
        pneumaticCompressor.setClosedLoopControl(true);

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
            robotDrive.arcadeDrive(-1.0, -navX.getAngle() * 0.03);
        }
        // Right Bumper
        else if (primaryController.getBumper(GenericHID.Hand.kRight))
        {
            robotDrive.arcadeDrive(1.0, -navX.getAngle() * 0.03);
        }
        // Sends the Y axis input from the left stick (speed) and the X axis input from the right stick (rotation) from the primary controller to move the robot if neither the Left Bumper or the Right Bumper were pressed
        else
        {
            robotDrive.arcadeDrive(primaryController.getY(GenericHID.Hand.kRight), primaryController.getX(GenericHID.Hand.kLeft));
        }

        // X button - Increments the navX's target angle by 15 degrees
        if (primaryController.getAButton() && !primaryController.getAButtonPressed())
        {
            navX.setAngleAdjustment(navX.getAngleAdjustment() + 15);
        }

        // Gets the values from the SmartDashboard
        getSmartDashboardValues();

        // Calls the function to update the SmartDashboard window's values
        updateSmartDashboard();
    }

    // Function to update the visually presented data in the SmartDashboard window
    public void updateSmartDashboard()
    {
        SmartDashboard.putNumber("Ultrasonic 1", ultrasonicSensor1.getRangeInches());
        SmartDashboard.putNumber("Ultrasonic 2", ultrasonicSensor2.getRangeInches());
        SmartDashboard.putNumber("Ultrasonic 3", ultrasonicSensor3.getRangeInches());
        SmartDashboard.putNumber("Ultrasonic 4", ultrasonicSensor4.getRangeInches());
        SmartDashboard.putNumber("Encoder 1", drivetrainEncoder1.get());
        SmartDashboard.putNumber("Encoder 2", drivetrainEncoder2.get());
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
                ultrasonicSensor1.ping();
                ultrasonicSensor2.ping();
                ultrasonicSensor3.ping();
                ultrasonicSensor4.ping();
                Timer.delay(.1);
            }
        });
        // Starts the thread
        thread.start();
    }
}
