
package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * ShooterTestStand opmode runs a single shooter motor, launch flap servo, using limelight on a test stand
 * Its purpose is to improve artifact shots (3 in a row, accuracy)
 * Data logging is used to record real-time data.
 */
@TeleOp(name = "ShooterTestStand")
//@Disabled //comment this out when ready to add to android phone
public class ShooterTestStand extends OpMode {
    LimelightDecode limelight; // Team 7462 class for the limelight camera
    Shooter shooter; // Team 7462 class for the shooter

    Servo launchFlap;

    // Timers
    ElapsedTime timer = new ElapsedTime();

    private static double IDLEPOWER = 20;
    private boolean shooterAtSpeed = true; // needs to be true initially

    private boolean readyForNextShoot = false;
    private boolean emergencyMode = false;

    private static final double FLAPDOWN = 0.45;
    private static final double FLAPUP = 0.75;

    ShooterTestStand.Datalog datalog = new ShooterTestStand.Datalog("ShooterLog");

    int i=0;

    double cameraA = 0.044; // fixed tilt, in radians

    double robotYaw = -17.0; // Used by MegaTag 2  to get correct botpose (MT2)

    @Override
    public void init() {
        launchFlap = hardwareMap.get(Servo.class, "launchFlapLeft");

        shooter = new Shooter(hardwareMap, "shooterLeft", true);
        shooter.setControllerValues(0.3, 0.0243);

        limelight = new LimelightDecode();
        limelight.init(hardwareMap, 16.8,cameraA);

        timer.reset();

        launchFlap.setPosition(FLAPDOWN);

        /*
        The telemetry.setMsTransmissionInterval() method in the FIRST Tech Challenge (FTC) SDK controls
        how frequently telemetry data is sent from the Robot Controller to the Driver Station
        250 (milliseconds) is the default value and a good general-purpose interval that balances updates with bandwidth usage.
        100 to 50 (milliseconds) are useful for debugging or operations requiring faster updates (e.g., vision processing, rapid sensor changes).
        A lower interval provides a more real-time view of data on the Driver Station but increases communication bandwidth usage,
        which might impact other network-dependent tasks (like vision processing).
         */
        telemetry.setMsTransmissionInterval(50);
    }

    //we are using the methods from OpMode and @Override is so that we can write our own stuff for this method
    @Override
    @SuppressLint("DefaultLocale")
    public void init_loop() {
        //telemetry.addData("Pattern", limelight.getObelisk());
        telemetry.addData("team ID", limelight.getID());

        telemetry.addLine("Press B for red, X for blue");
        if (gamepad1.bWasPressed()) {
            // goalTag.targetAprilTagID = 24; RED
            limelight.setTeam(24);
            robotYaw = 175.0;
        } else if (gamepad1.xWasPressed()) {
            // goalTag.targetAprilTagID = 20;  BLUE
            limelight.setTeam(20);
            robotYaw = -19.0;
        }

//        telemetry.addLine("ADJUST ROBOT YAW USING DPAD UP AND DOWN");
//        telemetry.addLine("TO GET THE CORRECT MT2 VALUES");
//        if (gamepad1.dpadUpWasPressed()) {
//            robotYaw += 2.0;
//        } else if (gamepad1.dpadDownWasPressed()) {
//            robotYaw -= 2.0;
//        }
        telemetry.addData("FIXED Robot YAW for MT2 ",robotYaw);

        limelight.processMT2(telemetry, robotYaw);

        limelight.process(telemetry);  // this processes MT1 and gets tx ty

        telemetry.addData("Apriltag tx YAW (DEG) SHOULD BE ZERO",String.format(" %.1f", limelight.getTx()));

        telemetry.update();
    }

    @Override
    public void start() {
        shooter.targetVelocity = IDLEPOWER;
        timer.reset();
    }

    @Override
    @SuppressLint("DefaultLocale")
    public void loop() {
        limelight.process(telemetry);

        shooter.overridePower(); // shooter motor speed controller

        telemetry.addLine("Left Bumper to shoot");

        telemetry.addData("Apriltag tx YAW (DEG) SHOULD BE ZERO",String.format(" %.1f", limelight.getTx()));

        telemetry.addData("shooter Current Velocity",String.format(" %.1f", shooter.getVelocity()));
        telemetry.addData("shooter Target Velocity",String.format(" %.1f", shooter.getShooterVelo(limelight)));
        telemetry.addData("Apriltag RANGE (in)",String.format(" %.1f", limelight.getRange()));

        telemetry.update();


        // Driver commands launch sequence, repeating shots
        if (gamepad1.left_bumper && readyForNextShoot && (limelight.isDataCurrent || emergencyMode)) {
            shooter.targetVelocity = shooter.getShooterVelo(limelight);
            shooterAtSpeed = false;
            readyForNextShoot = false;
            timer.reset();
        }

        // Shoot when at speed
        if (!shooterAtSpeed) {
            if (shooter.atSpeed()) {
                timer.reset();
                launchFlap.setPosition(FLAPUP);
                shooterAtSpeed = true;
            }
        }

        // Command launch Flap Down
        if (timer.seconds() > 0.4 && shooterAtSpeed) launchFlap.setPosition(FLAPDOWN);

        // Allow time for launch flap to go Down and artifact to enter. Toggle ready for next shoot
        if (timer.seconds() > 0.7 && shooterAtSpeed) readyForNextShoot =true;

        // Finger is off of the button, set shooter speed back to idle
        if (!gamepad1.left_bumper) {
            shooter.targetVelocity = IDLEPOWER;
            readyForNextShoot = true;
        }

        // If camera not working press this and shoot near point of close V.
        if (gamepad1.leftStickButtonWasPressed()) {
            shooter.targetVelocity = 30;
            //IDLEPOWER = 30;
            emergencyMode = true;
        }

        // Data log
        // Note that the order in which we set datalog fields
        // does *not* matter! Order is configured inside the Datalog class constructor.
        datalog.loopCounter.set(i);
        datalog.flapPos.set(launchFlap.getPosition());
        datalog.shooterVelocity.set(shooter.getVelocity());
        datalog.targetVelocity.set(shooter.targetVelocity);
        datalog.shooterPower.set(shooter.getPower());
        datalog.targetRange.set(limelight.getRange());
        datalog.writeLine();

        i++;
    }

    /**
    * Datalog class encapsulates all the fields that will go into the datalog.
    */
    public static class Datalog {
    // The underlying datalogger object - it cares only about an array of loggable fields
    private final Datalogger datalogger;

    // These are all of the fields that we want in the datalog.
    // Note that order here is NOT important. The order is important in the setFields() call below
    public Datalogger.GenericField loopCounter = new Datalogger.GenericField("LoopCounter");
    public Datalogger.GenericField flapPos = new Datalogger.GenericField("FlapPos");
    public Datalogger.GenericField shooterVelocity = new Datalogger.GenericField("shooterVelocity");
    public Datalogger.GenericField targetVelocity = new Datalogger.GenericField("targetVelocity");
    public Datalogger.GenericField shooterPower = new Datalogger.GenericField("shooterPower");
    public Datalogger.GenericField targetRange = new Datalogger.GenericField("targetRange");

    public Datalog(String name) {
        // Build the underlying datalog object
        datalogger = new Datalogger.Builder()

                // Pass through the filename
                .setFilename(name)

                // Request an automatic timestamp field
                .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)

                // Tell it about the fields we care to log.
                // Note that order *IS* important here! The order in which we list
                // the fields is the order in which they will appear in the log.
                .setFields(
                        loopCounter,
                        flapPos,
                        //posError,
                        shooterVelocity,
                        targetVelocity,
                        shooterPower,
                        targetRange
                )
                .build();
    }

    // Tell the datalogger to gather the values of the fields
    // and write a new line in the log.
    public void writeLine() {
        datalogger.writeLine();
    }
    }
}