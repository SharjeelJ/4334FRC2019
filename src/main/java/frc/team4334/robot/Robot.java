package frc.team4334.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
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

    // Initialize the arm motors
    // Todo: Switch to VictorSPX for competition
    private VictorSP leftArmMotor;
    private VictorSP rightArmMotor;

    // Initialize the cargo arm intake motors
    // Todo: Switch to VictorSPX for competition)
    private VictorSP cargoArmIntakeMotorLeft;
    private VictorSP cargoArmIntakeMotorRight;

    // Initialize the cargo mecanum floor intake motor
    // Todo: Switch to VictorSPX for competition)
    private VictorSP cargoMecanumIntakeMotor;

    // Initialize solenoids on the PCM ports
    private DoubleSolenoid gearShifterSolenoid;
    private DoubleSolenoid hatchMechanismSolenoid;
    private DoubleSolenoid mecanumIntakeSolenoid;

    // Initialize the sensors on the DIO ports
    private Ultrasonic ultrasonicSensorFrontLeft;
    private Ultrasonic ultrasonicSensorFrontRight;
    private Ultrasonic ultrasonicSensorBack;
    private DigitalInput armPushButton;
    private DigitalInput cargoArmPushButton;

    // Initialize the sensors on the Analog ports
    private Potentiometer armPotentiometer;

    // Initialize PID controller objects to handle the arm's movement
    private PIDController armPIDLeft;
    private PIDController armPIDRight;

    // Pairs up the drivetrain motors based on their respective side and initializes the drivetrain controlling object
    private SpeedControllerGroup drivetrainMotorGroupLeft;
    private SpeedControllerGroup drivetrainMotorGroupRight;
    private DifferentialDrive robotDrive;

    // Initialize a pneumatic compressor (setup via the roboRIO config page)
    private Compressor pneumaticCompressor = new Compressor(0);

    // Initialize the navX object
    private AHRS navX;

    // Initialize configuration values that will be used by the autonomous routines generated using PathWeaver
    //    private static final int encoderTicksPerRevolution = 30000;
    //    private static final double wheelDiameter = 0.1524;
    //    private static final double maxVelocity = 4.5;
    //    private static final String pathWeaverPathName = "Test2";
    //    private EncoderFollower drivetrainControllerLeft;
    //    private EncoderFollower drivetrainControllerRight;
    //    private Notifier autonomousController;

    // Initialize miscellaneous configuration values
    private static int reverseDrivetrainDirection = -1;
    private static int armPIDSetpoint = 90;
    private static int armPIDScale = 1800;
    private static int armPIDOffset = -918; // Todo: Tune offset at competition (adding moves the setpoint further into the robot, subtracting moves it lower to the ground)
    private static final int armPIDAcceptableError = 2;
    private static final int armPIDHatchOuttakeSetpoint = 90;
    private static final int armPIDHatchIntakeCargoOuttakeSetpoint = 110;
    private static final int armPIDHatchIntakeSetpoint = 200;
    private static final int armPIDCargoIntakeSetpoint = 10;

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
        leftArmMotor = new VictorSP(0); // Todo: Port 5 & change to VictorSPX for competition
        rightArmMotor = new VictorSP(1); // Todo: Port 8 & change to VictorSPX for competition
        cargoArmIntakeMotorLeft = new VictorSP(3); // Todo: Port 6 & change to VictorSPX for competition
        cargoArmIntakeMotorRight = new VictorSP(2); // Todo: Port 4 & change to VictorSPX for competition
        cargoMecanumIntakeMotor = new VictorSP(4); // Todo: Port 7 & change to VictorSPX for competition

        // Assigns all the solenoids to their respective objects (the number in brackets is the port # of what is connected where on the PCM)
        gearShifterSolenoid = new DoubleSolenoid(2, 3);
        hatchMechanismSolenoid = new DoubleSolenoid(0, 1);
        mecanumIntakeSolenoid = new DoubleSolenoid(5, 6); // Todo: Port 6, 7 for competition

        // Assigns all the DIO sensors to their respective objects (the number in brackets is the port # of what is connected where on the DIO)
        ultrasonicSensorFrontLeft = new Ultrasonic(30, 29); // Todo: Adjust ports for competition
        ultrasonicSensorFrontRight = new Ultrasonic(28, 27); // Todo: Adjust ports for competition
        ultrasonicSensorBack = new Ultrasonic(26, 25); // Todo: Adjust ports for competition
        armPushButton = new DigitalInput(2);
        cargoArmPushButton = new DigitalInput(0);

        // Assigns all the Analog sensors to their respective objects (the number in brackets is the port # of what is connected where on the Analog)
        armPotentiometer = new AnalogPotentiometer(0, armPIDScale, armPIDOffset);

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

        // Sets the appropriate configuration settings for the arm PID controllers
        armPIDLeft = new PIDController(0.05, 0, 0, armPotentiometer, leftArmMotor);
        armPIDRight = new PIDController(0.05, 0, 0, armPotentiometer, rightArmMotor);

        // Sets the appropriate configuration settings for the drivetrain encoders
        drivetrainMotorLeft1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 20);
        drivetrainMotorRight1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 20);
        drivetrainMotorLeft1.setSensorPhase(true);
        drivetrainMotorRight1.setSensorPhase(false);

        // Enables the ultrasonic sensors to calculate distances (need to be polled to give a reading)
        ultrasonicSensorFrontLeft.setEnabled(true);
        ultrasonicSensorFrontRight.setEnabled(true);
        ultrasonicSensorBack.setEnabled(true);

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
        UsbCamera camera1 = CameraServer.getInstance().startAutomaticCapture("Microsoft LifeCam HD-3000", 0);

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

    // Function that is run once each time the robot enters disabled mode
    @Override
    public void disabledInit()
    {
        // Disables all PID controllers
        armPIDLeft.disable();
        armPIDRight.disable();
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

        //        try
        //        {
        //            // Gets and sets the specified autonomous routine trajectories for the left and right side of the drivetrain
        //            Trajectory left_trajectory = PathfinderFRC.getTrajectory("output/" + pathWeaverPathName + ".left");
        //            Trajectory right_trajectory = PathfinderFRC.getTrajectory("output/" + pathWeaverPathName + ".right");
        //            drivetrainControllerLeft = new EncoderFollower(left_trajectory);
        //            drivetrainControllerRight = new EncoderFollower(right_trajectory);
        //
        //            // Configures the drivetrain left and right side controllers to use the appropriate configurations
        //            drivetrainControllerLeft.configureEncoder(drivetrainMotorLeft1.getSelectedSensorPosition(), encoderTicksPerRevolution, wheelDiameter);
        //            drivetrainControllerRight.configureEncoder(drivetrainMotorRight1.getSelectedSensorPosition(), encoderTicksPerRevolution, wheelDiameter);
        //            drivetrainControllerLeft.configurePIDVA(0.05, 0.0, 0.0, 1 / maxVelocity, 0);
        //            drivetrainControllerRight.configurePIDVA(0.05, 0.0, 0.0, 1 / maxVelocity, 0);
        //
        //            // Sets up the autonomous controller and starts it
        //            autonomousController = new Notifier(this::followPath);
        //            autonomousController.startPeriodic(left_trajectory.get(0).dt);
        //        } catch (IOException e)
        //        {
        //        }
    }

    // Function that is run periodically during autonomous mode
    @Override
    public void autonomousPeriodic()
    {
        // Calls the function for tele-operated mode
        teleopPeriodic();

        // Gets the values from the SmartDashboard
        //        getSmartDashboardValues();

        // Calls the function to update the SmartDashboard window's values
        //        updateSmartDashboard();
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
        // A button (Press & hold) - Engages the hatch panel mechanism solenoid
        if (primaryController.getAButton()) hatchMechanismSolenoid.set(DoubleSolenoid.Value.kForward);
        else hatchMechanismSolenoid.set(DoubleSolenoid.Value.kReverse);

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
            // Todo: Use ControlMode.PercentOutput as the first parameter for competition
            leftArmMotor.set(1);
            rightArmMotor.set(1);
        }
        // Right Bumper (Press & hold) - Moves the arm up if the push button is not pressed
        else if (primaryController.getBumper(GenericHID.Hand.kRight) && armPushButton.get())
        {
            // Todo: Use ControlMode.PercentOutput as the first parameter for competition
            leftArmMotor.set(-1);
            rightArmMotor.set(-1);
        }
        // Stops the arm motors from moving
        else if (!(armPIDLeft.isEnabled() || armPIDRight.isEnabled()))
        {
            // Todo: Use ControlMode.PercentOutput as the first parameter for competition
            leftArmMotor.set(0);
            rightArmMotor.set(0);
        }

        // Left Stick Button (Press & Release) - Toggles the forward direction of the drivetrain
        if (primaryController.getStickButtonReleased(GenericHID.Hand.kLeft)) reverseDrivetrainDirection *= -1;

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
            // Todo: Use ControlMode.PercentOutput as the first parameter for competition
            cargoArmIntakeMotorLeft.set(primaryController.getTriggerAxis(GenericHID.Hand.kRight));
            cargoArmIntakeMotorRight.set(primaryController.getTriggerAxis(GenericHID.Hand.kRight));
            cargoMecanumIntakeMotor.set(primaryController.getTriggerAxis(GenericHID.Hand.kRight));

            // Retracts the cargo mecanum intake if the ball triggers the arm push button and sets the arm to the cargo outtake setpoint
            if (!cargoArmPushButton.get())
            {
                armPIDSetpoint = armPIDHatchIntakeCargoOuttakeSetpoint + 1;
                armPIDLeft.setSetpoint(armPIDSetpoint);
                armPIDRight.setSetpoint(armPIDSetpoint);
                armPIDLeft.enable();
                armPIDRight.enable();
                mecanumIntakeSolenoid.set(DoubleSolenoid.Value.kReverse);
            }
        }
        // Left Trigger (Hold) - Outtakes cargo
        else if (primaryController.getTriggerAxis(GenericHID.Hand.kLeft) >= 0.2)
        {
            // Todo: Use ControlMode.PercentOutput as the first parameter for competition
            cargoArmIntakeMotorLeft.set(-primaryController.getTriggerAxis(GenericHID.Hand.kLeft));
            cargoArmIntakeMotorRight.set(-primaryController.getTriggerAxis(GenericHID.Hand.kLeft));
            cargoMecanumIntakeMotor.set(-primaryController.getTriggerAxis(GenericHID.Hand.kLeft));
        }
        // Stops the cargo intake motors
        else
        {
            // Todo: Use ControlMode.PercentOutput as the first parameter for competition
            cargoArmIntakeMotorLeft.set(0);
            cargoArmIntakeMotorRight.set(0);
            cargoMecanumIntakeMotor.set(0);
        }

        // Up D-Pad (Press & Release) - Sets the PID setpoint to intake / outtake the hatch panel and retracts the mecanum intake
        if (primaryController.getPOV() == 0)
        {
            armPIDSetpoint = armPIDHatchOuttakeSetpoint;
            armPIDLeft.setSetpoint(armPIDSetpoint);
            armPIDRight.setSetpoint(armPIDSetpoint);
            armPIDLeft.enable();
            armPIDRight.enable();
            mecanumIntakeSolenoid.set(DoubleSolenoid.Value.kReverse);
        }
        // Right D-Pad (Press & Release) - Sets the PID setpoint to outtake the cargo and retracts the mecanum intake
        else if (primaryController.getPOV() == 90)
        {
            armPIDSetpoint = armPIDHatchIntakeCargoOuttakeSetpoint;
            armPIDLeft.setSetpoint(armPIDSetpoint);
            armPIDRight.setSetpoint(armPIDSetpoint);
            armPIDLeft.enable();
            armPIDRight.enable();
            mecanumIntakeSolenoid.set(DoubleSolenoid.Value.kReverse);
        }
        // Down D-Pad (Press & Release) - Sets the PID setpoint to intake the hatch panel off the ground and retracts the mecanum intake
        else if (primaryController.getPOV() == 180)
        {
            armPIDSetpoint = armPIDHatchIntakeSetpoint;
            armPIDLeft.setSetpoint(armPIDSetpoint);
            armPIDRight.setSetpoint(armPIDSetpoint);
            armPIDLeft.enable();
            armPIDRight.enable();
            mecanumIntakeSolenoid.set(DoubleSolenoid.Value.kReverse);
        }
        // Left D-Pad (Press & Release) - Sets the PID setpoint to intake the cargo from the mecanum intake
        else if (primaryController.getPOV() == 270)
        {
            armPIDSetpoint = armPIDCargoIntakeSetpoint;
            armPIDLeft.setSetpoint(armPIDSetpoint);
            armPIDRight.setSetpoint(armPIDSetpoint);
            armPIDLeft.enable();
            armPIDRight.enable();
            mecanumIntakeSolenoid.set(DoubleSolenoid.Value.kForward);
        }
        // Disables the PID controller objects if the potentiometer reading is reasonably close to the setpoint
        else if ((armPIDLeft.isEnabled() || armPIDRight.isEnabled()) && (armPotentiometer.get() >= armPIDSetpoint - armPIDAcceptableError && armPotentiometer.get() <= armPIDSetpoint + armPIDAcceptableError))
        {
            armPIDLeft.disable();
            armPIDRight.disable();
        }
        // Disables the PID controller objects if the arm push button is pressed and adjusts the offset value
        else if ((armPIDLeft.isEnabled() || armPIDRight.isEnabled()) && !armPushButton.get() && armPIDSetpoint != (armPIDHatchIntakeCargoOuttakeSetpoint + 1))
        {
            armPIDLeft.disable();
            armPIDRight.disable();
            armPIDOffset += armPotentiometer.get();
        }

        // Sends the Y axis input from the left stick (speed) and the X axis input from the right stick (rotation) from the primary controller to move the robot
        robotDrive.arcadeDrive(primaryController.getY(GenericHID.Hand.kLeft) * reverseDrivetrainDirection, primaryController.getX(GenericHID.Hand.kRight) * 0.8);

        // Gets the values from the SmartDashboard
        getSmartDashboardValues();

        // Calls the function to update the SmartDashboard window's values
        updateSmartDashboard();
    }

    // Function to update the visually presented data in the SmartDashboard window
    public void updateSmartDashboard()
    {
        SmartDashboard.putNumber("Ultrasonic Front Left", (ultrasonicSensorFrontLeft.isRangeValid() == false) ? SmartDashboard.getNumber("Ultrasonic Front Left", 999.0) : ultrasonicSensorFrontLeft.getRangeInches());
        SmartDashboard.putNumber("Ultrasonic Front Right", (ultrasonicSensorFrontRight.isRangeValid() == false) ? SmartDashboard.getNumber("Ultrasonic Front Right", 999.0) : ultrasonicSensorFrontRight.getRangeInches());
        SmartDashboard.putNumber("Ultrasonic Back", (ultrasonicSensorBack.isRangeValid() == false) ? SmartDashboard.getNumber("Ultrasonic Back", 999.0) : ultrasonicSensorBack.getRangeInches());
        SmartDashboard.putNumber("Encoder Left", drivetrainMotorLeft1.getSelectedSensorPosition());
        SmartDashboard.putNumber("Encoder Right", drivetrainMotorRight1.getSelectedSensorPosition());
        SmartDashboard.putNumber("navX Angle", navX.getAngle());
        SmartDashboard.putNumber("navX Angle Adjustment", navX.getAngleAdjustment());
        SmartDashboard.putNumber("navX Compass Heading", navX.getCompassHeading());
        SmartDashboard.putNumber("navX Fused Heading", navX.getFusedHeading());
        SmartDashboard.putNumber("Arm Potentiometer Angle", armPotentiometer.get());
        SmartDashboard.putNumber("Arm Potentiometer Setpoint", armPIDSetpoint);
        SmartDashboard.putNumber("Arm PID Offset", armPIDOffset);
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
                ultrasonicSensorFrontLeft.ping();
                ultrasonicSensorFrontRight.ping();
                ultrasonicSensorBack.ping();
                Timer.delay(0.1);
            }
        });
        // Starts the thread
        thread.start();
    }

    // Function that is called to follow the passed in autonomous routine trajectories fed to the drivetrain motor controllers
    //    private void followPath()
    //    {
    //        if (drivetrainControllerLeft.isFinished() || drivetrainControllerRight.isFinished())
    //        {
    //            autonomousController.stop();
    //        } else
    //        {
    //            double left_speed = drivetrainControllerLeft.calculate(drivetrainMotorLeft1.getSelectedSensorPosition());
    //            double right_speed = drivetrainControllerRight.calculate(drivetrainMotorRight1.getSelectedSensorPosition());
    //            double heading = navX.getAngle();
    //            double desired_heading = Pathfinder.r2d(drivetrainControllerLeft.getHeading());
    //            double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
    //            double turn = 0.35 * (-1.0 / 80.0) * heading_difference;
    //            drivetrainMotorGroupLeft.set(left_speed - turn);
    //            drivetrainMotorGroupRight.set(-right_speed - turn);
    //            System.out.println("--------------------------------------------------------------------------");
    //            System.out.println("Drivetrain Left Code: " + left_speed);
    //            System.out.println("Drivetrain Right Code: " + right_speed);
    //            System.out.println("Drivetrain Left Actual: " + drivetrainMotorGroupLeft.get());
    //            System.out.println("Drivetrain Right Actual: " + drivetrainMotorGroupRight.get());
    //            System.out.println("--------------------------------------------------------------------------");
    //        }
    //    }
}
