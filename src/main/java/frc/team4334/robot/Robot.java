package frc.team4334.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

// If you rename or move this class, update the build.properties file in the project root
public class Robot extends TimedRobot
{
    //
    // Initialize an Xbox 360 controller to control the robot
    private XboxController primaryController = new XboxController(0);

    // Initialize the drivetrain motors
    private WPI_TalonSRX drivetrainMotorLeft1;
    private WPI_TalonSRX drivetrainMotorLeft2;
    private WPI_TalonSRX drivetrainMotorRight1;
    private WPI_TalonSRX drivetrainMotorRight2;

    // Initialize the arm motors
    private VictorSP leftArmMotor;
    private VictorSP rightArmMotor;

    // Initialize the cargo arm intake motors
    private VictorSP cargoArmIntakeMotorLeft;
    private VictorSP cargoArmIntakeMotorRight;

    // Initialize the cargo mecanum floor intake motor
    private VictorSP cargoMecanumIntakeMotor;

    // Initialize solenoids on the PCM ports
    private DoubleSolenoid gearShifterSolenoid;
    private DoubleSolenoid hatchMechanismSolenoid;
    private DoubleSolenoid mecanumIntakeSolenoid;

    // Initialize the sensors on the DIO ports
    private Ultrasonic ultrasonicSensorFront;
    private Ultrasonic ultrasonicSensorLeft;
    private Ultrasonic ultrasonicSensorBack;
    private Ultrasonic ultrasonicSensorRight;
    private DigitalInput armLimitSwitch;
    private DigitalInput cargoArmPushButton;

    // Pairs up the drivetrain motors based on their respective side and initializes the drivetrain controlling object
    private SpeedControllerGroup drivetrainMotorGroupLeft;
    private SpeedControllerGroup drivetrainMotorGroupRight;
    private DifferentialDrive robotDrive;

    // Initialize a pneumatic compressor (setup via the roboRIO config page)
    private Compressor pneumaticCompressor = new Compressor(0);

    // Initialize the navX object
    private AHRS navX;

    // Initialize configuration values that will be used by the autonomous routines generated using PathWeaver
    private static final int encoderTicksPerRevolution = 30000;
    private static final double wheelDiameter = 0.1524;
    private static final double maxVelocity = 4.5;
    private static final String pathWeaverPathName = "Straight1";
    private EncoderFollower drivetrainControllerLeft;
    private EncoderFollower drivetrainControllerRight;
    private Notifier autonomousController;

    // Function that is run once when the robot is first powered on
    @Override
    public void robotInit()
    {
        // Assigns all the motors to their respective objects (the number in brackets is the port # of what is connected where on the CAN bus)
        drivetrainMotorLeft1 = new WPI_TalonSRX(0);
        drivetrainMotorLeft2 = new WPI_TalonSRX(1);
        drivetrainMotorRight1 = new WPI_TalonSRX(2);
        drivetrainMotorRight2 = new WPI_TalonSRX(3);

        // Assigns all the motors to their respective objects (the number in brackets is the port # of what is connected where on PWM)
        leftArmMotor = new VictorSP(0);
        rightArmMotor = new VictorSP(1);
        cargoArmIntakeMotorLeft = new VictorSP(3);
        cargoArmIntakeMotorRight = new VictorSP(2);
        cargoMecanumIntakeMotor = new VictorSP(4);

        // Assigns all the solenoids to their respective objects (the number in brackets is the port # of what is connected where on the PCM)
        gearShifterSolenoid = new DoubleSolenoid(2, 3);
        hatchMechanismSolenoid = new DoubleSolenoid(0, 1);
        mecanumIntakeSolenoid = new DoubleSolenoid(5, 6);

        // Assigns all the DIO sensors to their respective objects (the number in brackets is the port # of what is connected where on the DIO)
        ultrasonicSensorFront = new Ultrasonic(30, 29);
        ultrasonicSensorLeft = new Ultrasonic(28, 27);
        ultrasonicSensorBack = new Ultrasonic(26, 25);
        ultrasonicSensorRight = new Ultrasonic(24, 23);
        armLimitSwitch = new DigitalInput(2);
        cargoArmPushButton = new DigitalInput(0);

        // Assigns the drivetrain motors to their respective motor controller group and then passes them on to the drivetrain controller object
        drivetrainMotorGroupLeft = new SpeedControllerGroup(drivetrainMotorLeft1, drivetrainMotorLeft2);
        drivetrainMotorGroupRight = new SpeedControllerGroup(drivetrainMotorRight1, drivetrainMotorRight2);
        robotDrive = new DifferentialDrive(drivetrainMotorGroupLeft, drivetrainMotorGroupRight);

        // Sets the appropriate configuration settings for the motors
        drivetrainMotorGroupLeft.setInverted(true);
        drivetrainMotorGroupRight.setInverted(true);
        cargoArmIntakeMotorLeft.setInverted(true);
        robotDrive.setSafetyEnabled(true);

        // Sets the appropriate configuration settings for the solenoids
        hatchMechanismSolenoid.set(DoubleSolenoid.Value.kReverse);
        gearShifterSolenoid.set(DoubleSolenoid.Value.kReverse);
        mecanumIntakeSolenoid.set(DoubleSolenoid.Value.kReverse);

        // Sets the appropriate configuration settings for the drivetrain encoders
        drivetrainMotorLeft1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 20);
        drivetrainMotorRight1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 20);
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

        // Instantiates a UsbCamera object from the CameraServer for the first camera for POV driving (starts the SmartDashboard's camera stream)
        UsbCamera camera1 = CameraServer.getInstance().startAutomaticCapture("Microsoft LifeCam HD-3000 (Image Analysis)", 0);

        // Sets the properties for the first camera object
        camera1.setResolution(320, 240);
        camera1.setFPS(30);

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

        // Gets and sets the specified autonomous routine trajectories for the left and right side of the drivetrain
        Trajectory left_trajectory = PathfinderFRC.getTrajectory("output/" + pathWeaverPathName + ".left");
        Trajectory right_trajectory = PathfinderFRC.getTrajectory("output/" + pathWeaverPathName + ".right");
        drivetrainControllerLeft = new EncoderFollower(left_trajectory);
        drivetrainControllerRight = new EncoderFollower(right_trajectory);

        // Configures the drivetrain left and right side controllers to use the appropriate configurations
        drivetrainControllerLeft.configureEncoder(drivetrainMotorLeft1.getSelectedSensorPosition(), encoderTicksPerRevolution, wheelDiameter);
        drivetrainControllerRight.configureEncoder(drivetrainMotorRight1.getSelectedSensorPosition(), encoderTicksPerRevolution, wheelDiameter);
        drivetrainControllerLeft.configurePIDVA(1.0, 0.15, 0.1, 1 / maxVelocity, 0);
        drivetrainControllerRight.configurePIDVA(1.0, 0.15, 0.1, 1 / maxVelocity, 0);

        // Sets up the autonomous controller and starts it
        autonomousController = new Notifier(this::followPath);
        autonomousController.startPeriodic(left_trajectory.get(0).dt);
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

        // Stops the autonomous controller and the drivetrain
        //        autonomousController.stop();
        robotDrive.stopMotor();
    }

    // Function that is called periodically during tele-operated mode
    @Override
    public void teleopPeriodic()
    {
        // A button (Press & Release) - Toggles the hatch panel mechanism solenoid
        if (primaryController.getAButtonReleased())
        {
            if (hatchMechanismSolenoid.get().equals(DoubleSolenoid.Value.kReverse))
                hatchMechanismSolenoid.set(DoubleSolenoid.Value.kForward);
            else hatchMechanismSolenoid.set(DoubleSolenoid.Value.kReverse);
        }

        // B button (Press & Release) - Toggles the mecanum intake solenoid
        if (primaryController.getBButtonReleased())
        {
            if (mecanumIntakeSolenoid.get() == DoubleSolenoid.Value.kReverse)
                mecanumIntakeSolenoid.set(DoubleSolenoid.Value.kForward);
            else mecanumIntakeSolenoid.set(DoubleSolenoid.Value.kReverse);
        }

        // Left Bumper (Press & hold) - Moves the arm down
        if (primaryController.getBumper(GenericHID.Hand.kLeft))
        {
            leftArmMotor.set(1);
            rightArmMotor.set(1);
        }
        // Right Bumper (Press & hold) - Moves the arm up if the limit switch is not pressed
        else if (primaryController.getBumper(GenericHID.Hand.kRight) && armLimitSwitch.get())
        {
            leftArmMotor.set(-1);
            rightArmMotor.set(-1);
        }
        // Stops the arm motors from moving
        else
        {
            leftArmMotor.set(0);
            rightArmMotor.set(0);
        }

        // Right Stick Button (Press & Release) - Toggles the drivetrain gear shifter solenoid
        if (primaryController.getStickButtonReleased(GenericHID.Hand.kRight))
        {
            if (gearShifterSolenoid.get() == DoubleSolenoid.Value.kReverse)
                gearShifterSolenoid.set(DoubleSolenoid.Value.kForward);
            else gearShifterSolenoid.set(DoubleSolenoid.Value.kReverse);
        }

        // Right Trigger (Hold) - Intakes cargo
        if (primaryController.getTriggerAxis(GenericHID.Hand.kRight) >= 0.2)
        {
            cargoArmIntakeMotorLeft.set(primaryController.getTriggerAxis(GenericHID.Hand.kRight));
            cargoArmIntakeMotorRight.set(primaryController.getTriggerAxis(GenericHID.Hand.kRight));
            cargoMecanumIntakeMotor.set(primaryController.getTriggerAxis(GenericHID.Hand.kRight));

            // Retracts the cargo mecanum intake if the ball triggers the arm push button
            if (!cargoArmPushButton.get()) mecanumIntakeSolenoid.set(DoubleSolenoid.Value.kReverse);
        }
        // Left Trigger (Hold) - Outtakes cargo
        else if (primaryController.getTriggerAxis(GenericHID.Hand.kLeft) >= 0.2)
        {
            cargoArmIntakeMotorLeft.set(-primaryController.getTriggerAxis(GenericHID.Hand.kLeft));
            cargoArmIntakeMotorRight.set(-primaryController.getTriggerAxis(GenericHID.Hand.kLeft));
            cargoMecanumIntakeMotor.set(-primaryController.getTriggerAxis(GenericHID.Hand.kLeft));
        }
        // Stops the cargo intake motors
        else
        {
            cargoArmIntakeMotorLeft.set(0);
            cargoArmIntakeMotorRight.set(0);
            cargoMecanumIntakeMotor.set(0);
        }

        // Sends the Y axis input from the left stick (speed) and the X axis input from the right stick (rotation) from the primary controller to move the robot
        robotDrive.arcadeDrive(-primaryController.getY(GenericHID.Hand.kLeft), primaryController.getX(GenericHID.Hand.kRight));

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
                Timer.delay(0.1);
            }
        });
        // Starts the thread
        thread.start();
    }

    // Function that is called to follow the passed in autonomous routine trajectories fed to the drivetrain motor controllers
    private void followPath()
    {
        if (drivetrainControllerLeft.isFinished() || drivetrainControllerRight.isFinished())
        {
            autonomousController.stop();
        } else
        {
            double left_speed = drivetrainControllerLeft.calculate(drivetrainMotorLeft1.getSelectedSensorPosition());
            double right_speed = drivetrainControllerRight.calculate(drivetrainMotorRight1.getSelectedSensorPosition());
            double heading = navX.getAngle();
            double desired_heading = Pathfinder.r2d(drivetrainControllerLeft.getHeading());
            double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
            double turn = 0.8 * (-1.0 / 80.0) * heading_difference;
            drivetrainMotorGroupLeft.set(-left_speed - turn);
            drivetrainMotorGroupRight.set(right_speed + turn);
            System.out.println("--------------------------------------------------------------------------");
            System.out.println("Drivetrain Left Code: " + left_speed);
            System.out.println("Drivetrain Right Code: " + right_speed);
            System.out.println("Drivetrain Left Actual: " + drivetrainMotorGroupLeft.get());
            System.out.println("Drivetrain Right Actual: " + drivetrainMotorGroupRight.get());
            System.out.println("--------------------------------------------------------------------------");
        }
    }
}
