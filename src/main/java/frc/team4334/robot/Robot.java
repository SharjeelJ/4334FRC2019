package frc.team4334.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*
    Controls:
        Left Stick [Hold] = Moves the robot on the Y axis (forward / back)
        Right Stick [Hold] = Moves the robot on the X axis (left / right)
        Left Stick [Press & Release] = Toggles the forward direction of the drivetrain
        Right Stick [Press & Release] = Switches the drivetrain gear shifter solenoid to high gear (shifts to low gear automatically when the robot slows down)
        Left Bumper [Press & Hold] = Moves the arm down
        Right Bumper [Press & Hold] = Moves the arm up
        Left Trigger [Hold] = Outtakes cargo
        Right Trigger [Hold] = Intakes cargo
        A Button [Press & Hold] = Engages the hatch panel solenoids (disengages when released)
        B Button [Press & Release] = Toggles the mecanum intake solenoid
        Up D-Pad [Press & Release] = Sets the PID setpoint to intake / outtake the hatch panel and retracts the mecanum intake
        Right D-Pad [Press & Release] = Sets the PID setpoint to outtake the cargo and retracts the mecanum intake
        Down D-Pad [Press & Release] = Sets the PID setpoint to intake the hatch panel off the ground and retracts the mecanum intake
        Left D-Pad [Press & Release] = Sets the PID setpoint to intake the cargo from the mecanum intake
    Networking Config:
        OpenMesh Radio [Event Configured]:
            IP Address = 10.43.34.1
        roboRIO [Static]:
            IP Address = 10.43.34.2
            Subnet Mask = 255.255.255.0
            Gateway = 10.43.34.1
            DNS Server = 10.43.34.1
        Raspberry Pi [Static]:
            IP Address = 10.43.34.10
            Subnet Mask = 255.255.255.0
            Gateway = 10.43.34.1
            DNS Server = 10.43.34.1
        Driver Station [Static]:
            IP Address = 10.43.34.5
            Subnet Mask = 255.0.0.0
            Gateway = 10.43.34.1
            DNS Server = 10.43.34.1
 */

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
    private VictorSP leftArmMotor;
    private VictorSP rightArmMotor;

    // Initialize the cargo arm intake motors
    private VictorSP cargoArmIntakeMotorLeft;
    private VictorSP cargoArmIntakeMotorRight;

    // Initialize the side winder motor
    private VictorSP sideWinderMotor;

    // Initialize the potentiometer and hall effect sensors on the hatch
    private DigitalInput hatchHallEffectLeft;
    private DigitalInput hatchHallEffectRight;
    private Potentiometer hatchPotentiometer;

    // Initialize the cargo mecanum floor intake motor
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

    // Initialize PID controller objects to handle hatch movement
    private PIDSubsystem sideWinderPID;
    private boolean sideWinderPIDisEnabled;

    // Pairs up the drivetrain motors based on their respective side and initializes the drivetrain controlling object
    private SpeedControllerGroup drivetrainMotorGroupLeft;
    private SpeedControllerGroup drivetrainMotorGroupRight;
    private DifferentialDrive robotDrive;

    // Initialize a pneumatic compressor (setup via the roboRIO config page)
    private Compressor pneumaticCompressor = new Compressor(0);

    // Initialize the navX object
    private AHRS navX;

    // Initialize miscellaneous configuration values
    private static int reverseDrivetrainDirection = -1;
    private static int armPIDSetpoint = 90;
    private static int armPIDScale = 1800;
    private static int armPIDOffset = -593; // Todo: Tune offset at competition (adding moves the setpoint further into the robot, subtracting moves it lower to the ground OR manually set arm to 90 and then replace with the displayed Correct Offset value)
    private static final int armPIDAcceptableError = 2;
    private static final int armPIDHatchOuttakeSetpoint = 90;
    private static final int armPIDHatchIntakeCargoOuttakeSetpoint = 110;
    private static final int armPIDHatchIntakeSetpoint = 185;
    private static final int armPIDCargoIntakeSetpoint = 2;
    private double hatchPotentiometerOffset = -5.6;
    private int hatchPotentiometerScale = 60; //Todo: Tune scale

    private ShuffleboardTab dynamicSettingsTab = Shuffleboard.getTab("Competition");
    public NetworkTableEntry HALL_EFFECT_LEFT = dynamicSettingsTab.addPersistent("Hall Effect Left", false).getEntry();
    public NetworkTableEntry HALL_EFFECT_RIGHT = dynamicSettingsTab.addPersistent("Hall Effect Right", false).getEntry();

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
        leftArmMotor = new VictorSP(2);
        rightArmMotor = new VictorSP(3);
        cargoArmIntakeMotorLeft = new VictorSP(1);
        cargoArmIntakeMotorRight = new VictorSP(0);
        cargoMecanumIntakeMotor = new VictorSP(4);
        sideWinderMotor = new VictorSP(6);

        // Assigns all the solenoids to their respective object (the number in brackets is the port # of what is connected where on the PCM)
        gearShifterSolenoid = new DoubleSolenoid(1, 0);
        hatchMechanismSolenoid = new DoubleSolenoid(2, 3);
        mecanumIntakeSolenoid = new DoubleSolenoid(7, 6);

        // Assigns all the DIO sensors to their respective objects (the number in brackets is the port # of what is connected where on the DIO)
        ultrasonicSensorFrontLeft = new Ultrasonic(30, 29); // Todo: Adjust ports for competition
        ultrasonicSensorFrontRight = new Ultrasonic(28, 27); // Todo: Adjust ports for competition
        ultrasonicSensorBack = new Ultrasonic(26, 25); // Todo: Adjust ports for competition
        armPushButton = new DigitalInput(2);
        cargoArmPushButton = new DigitalInput(4);

        // Assigns all the Analog sensors to their respective objects (the number in brackets is the port # of what is connected where on the Analog)
        armPotentiometer = new AnalogPotentiometer(1, armPIDScale, armPIDOffset);

        // Assigns all hatch sensors to their respective object
        hatchHallEffectLeft = new DigitalInput(0);
        hatchHallEffectRight = new DigitalInput(1);
        hatchPotentiometer = new AnalogPotentiometer(4, hatchPotentiometerScale, hatchPotentiometerOffset);

        // Assigns the drivetrain motors to their respective motor controller group and then passes them on to the drivetrain controller object
        drivetrainMotorGroupLeft = new SpeedControllerGroup(drivetrainMotorLeft1, drivetrainMotorLeft2);
        drivetrainMotorGroupRight = new SpeedControllerGroup(drivetrainMotorRight1, drivetrainMotorRight2);
        robotDrive = new DifferentialDrive(drivetrainMotorGroupLeft, drivetrainMotorGroupRight);

        // Sets the appropriate configuration settings for the motors
        drivetrainMotorGroupLeft.setInverted(true);
        drivetrainMotorGroupRight.setInverted(true);
        cargoArmIntakeMotorLeft.setInverted(false);
        cargoArmIntakeMotorRight.setInverted(true);
        robotDrive.setSafetyEnabled(true);

        // Sets the appropriate configuration settings for the solenoids
        hatchMechanismSolenoid.set(DoubleSolenoid.Value.kReverse);
        gearShifterSolenoid.set(DoubleSolenoid.Value.kReverse);
        mecanumIntakeSolenoid.set(DoubleSolenoid.Value.kReverse);

        // Sets the appropriate configuration settings for the arm PID controllers
        armPIDLeft = new PIDController(0.05, 0, 0, armPotentiometer, leftArmMotor);
        armPIDRight = new PIDController(0.05, 0, 0, armPotentiometer, rightArmMotor);

        // Sets the appropriate configuration settings for the hatch controller
        sideWinderPID = new PIDSubsystem(0.01,0,0) {
            @Override
            protected double returnPIDInput() {
                return hatchPotentiometer.get();
            }

            @Override
            protected void usePIDOutput(double output) {
                sideWinderMotor.set(output);
            }

            @Override
            protected void initDefaultCommand() {
                super.setAbsoluteTolerance(0.0);
                super.setOutputRange(-0.2, 0.2);
                super.setInputRange(-30, 30);
            }

            @Override
            public void enable(){
                super.enable();
                sideWinderPIDisEnabled = true;
            }

            @Override
            public void disable(){
                super.disable();
                sideWinderPIDisEnabled = false;
            }
        };
        sideWinderPID.disable();

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
        camera1.setFPS(15);

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
        // Grabs the input values from the driver station SmartDashboard window
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
    }

    // Function that is run periodically during autonomous mode
    @Override
    public void autonomousPeriodic()
    {
        // Calls the function for tele-operated mode
        teleopPeriodic();
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

        // Stops the drivetrain
        robotDrive.stopMotor();
    }

    // Function that is called periodically during tele-operated mode
    @Override
    public void teleopPeriodic()
    {
        // A button (Press & Hold) - Engages the hatch panel mechanism solenoid
        if (primaryController.getAButton()) hatchMechanismSolenoid.set(DoubleSolenoid.Value.kForward);
        else hatchMechanismSolenoid.set(DoubleSolenoid.Value.kReverse);

        // B button (Press & Release) - Toggles the mecanum intake solenoid
        if (primaryController.getBButtonReleased())
        {
            if (mecanumIntakeSolenoid.get() == DoubleSolenoid.Value.kReverse)
                mecanumIntakeSolenoid.set(DoubleSolenoid.Value.kForward);
            else mecanumIntakeSolenoid.set(DoubleSolenoid.Value.kReverse);
        }

        // Left Bumper (Press & Hold) - Moves the arm down
        if (primaryController.getBumper(GenericHID.Hand.kLeft))
        {
            leftArmMotor.set(1);
            rightArmMotor.set(1);
        }
        // Right Bumper (Press & Hold) - Moves the arm up if the push button is not pressed
        else if (primaryController.getBumper(GenericHID.Hand.kRight) && armPushButton.get())
        {
            leftArmMotor.set(-1);
            rightArmMotor.set(-1);
        }
        // Stops the arm motors from moving
        else if (!(armPIDLeft.isEnabled() || armPIDRight.isEnabled()))
        {
            leftArmMotor.set(0);
            rightArmMotor.set(0);
        }

        // Left Stick Button (Press & Release) - Toggles the forward direction of the drivetrain
        if (primaryController.getStickButtonReleased(GenericHID.Hand.kLeft)) reverseDrivetrainDirection *= -1;

        // Right Stick Button (Press & Release) - Enables the drivetrain gear shifter solenoid (switches to high gear) and disables it (switches to low gear) when the robot slows down
        if (primaryController.getStickButtonReleased(GenericHID.Hand.kRight))
            gearShifterSolenoid.set(DoubleSolenoid.Value.kForward);
        else if (Math.abs(primaryController.getY(GenericHID.Hand.kLeft)) <= 0.20)
            gearShifterSolenoid.set(DoubleSolenoid.Value.kReverse);

        // Right Trigger (Hold) - Intakes cargo
        if (primaryController.getTriggerAxis(GenericHID.Hand.kRight) >= 0.2)
        {
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
            cargoArmIntakeMotorLeft.set(-primaryController.getTriggerAxis(GenericHID.Hand.kLeft) * 0.80);
            cargoArmIntakeMotorRight.set(-primaryController.getTriggerAxis(GenericHID.Hand.kLeft) * 0.80);
            cargoMecanumIntakeMotor.set(-primaryController.getTriggerAxis(GenericHID.Hand.kLeft));
        }
        // Stops the cargo intake motors
        else
        {
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

        //Update ShuffleBoard with HallEffect Sensor Status
        if(hatchHallEffectRight.get()){
            HALL_EFFECT_RIGHT.setBoolean(true);
        } else {
            HALL_EFFECT_RIGHT.setBoolean(false);
        }

        if(hatchHallEffectLeft.get()){
            HALL_EFFECT_LEFT.setBoolean(true);
        } else {
            HALL_EFFECT_LEFT.setBoolean(false);
        }

        //System.out.println("Hatch PID Corrected Offset: " + (hatchPotentiometerOffset-hatchPotentiometer.get()));

        if(primaryController.getStartButtonReleased() && !sideWinderPIDisEnabled){
            sideWinderPID.setSetpoint(0);
            sideWinderPID.enable();
            System.out.println("Started PID");
            System.out.println(hatchPotentiometer.get());
        }



        if(sideWinderPIDisEnabled && sideWinderPID.onTarget()){
            sideWinderPID.disable();
        }

        //Move Hatch Left and Right using X(left) and Y(right)
        if((primaryController.getYButton()) && hatchHallEffectRight.get()){
            //Move Right
            sideWinderMotor.set(0.2);
        } else if((primaryController.getXButton()) && hatchHallEffectLeft.get()){
            //Move Left
            sideWinderMotor.set(-0.2);
        } else if(!sideWinderPIDisEnabled){
            sideWinderMotor.set(0);
        }

        // Disable SideWinder PID when manual controls are in use
        if(primaryController.getYButton() || primaryController.getXButton()){
            sideWinderPID.disable();
        }

        // Sends the Y axis input from the left stick (speed) and the X axis input from the right stick (rotation) from the primary controller to move the robot
        robotDrive.arcadeDrive(primaryController.getY(GenericHID.Hand.kLeft) * reverseDrivetrainDirection, primaryController.getX(GenericHID.Hand.kRight) * 0.80);

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
        SmartDashboard.putNumber("Arm Potentiometer Angle", armPotentiometer.get());
        SmartDashboard.putNumber("Arm Potentiometer Setpoint", armPIDSetpoint);
        SmartDashboard.putNumber("Arm PID Offset", armPIDOffset);
        SmartDashboard.putNumber("Arm PID Corrected Offset", 90 + armPIDOffset - armPotentiometer.get());
        SmartDashboard.putString("Arm PID Push Button", String.valueOf(armPushButton.get()));
        SmartDashboard.putString("Arm Cargo Push Button", String.valueOf(cargoArmPushButton.get()));
        SmartDashboard.putString("Hatch Potentiometer", String.valueOf(hatchPotentiometer.get()));
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
}
