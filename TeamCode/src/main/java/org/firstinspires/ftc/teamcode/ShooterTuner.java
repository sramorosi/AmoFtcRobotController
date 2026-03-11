
package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * ShooterTuner opmode runs a single shooter motor,
 * Use to determine Kvelo and Kp
 * Data logging is used to record real-time data.
 */
@TeleOp(name = "Shooter Tuner")
//@Disabled //comment this out when ready to add to android phone
public class ShooterTuner extends OpMode {
    Shooter shooter;

    private double Kvelo = 0.001;

    private double Kp = 1.0;
    private static final double IDLEPOWER = 25;

    ShooterTuner.Datalog datalog = new ShooterTuner.Datalog("ShooterLog");

    @Override
    public void init() {
        shooter = new Shooter(hardwareMap, "shooter", true);
        shooter.setControllerValues(0.3, 0.0243);

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

        // allow the user to change the motor power and see the velocity and Kvelo
        double power = 0.1;
        double velocity = shooter.getVelocity();
        Kvelo = power/velocity;

        if (gamepad1.dpadUpWasPressed()) power += 0.01;
        else if (gamepad1.dpadDownWasPressed()) power -= 0.01;

        telemetry.addData("Shooter power +UP -DOWN", power);

        shooter.setPower(power);
        telemetry.addData("Shooter Speed"," %.1f", velocity);
        telemetry.addData("Kvelo term"," %.1f", Kvelo);

        telemetry.update();
    }

    @Override
    public void start() {
        shooter.setTargetVelocity(IDLEPOWER);
        shooter.setControllerValues(Kp, Kvelo);
    }

    @Override
    @SuppressLint("DefaultLocale")
    public void loop() {


        telemetry.addLine("Slow shooter down and look at response, adjust Kp");
        telemetry.addData("Kp term +UP -DOWN", Kp);

        if (gamepad1.dpadUpWasPressed()) Kp += 0.1;
        else if (gamepad1.dpadDownWasPressed()) Kp -= 0.1;

        telemetry.addData("Target velocity"," %.1f", IDLEPOWER);
        telemetry.addData("shooter Current Velocity",String.format(" %.1f", shooter.getVelocity()));
        telemetry.addData("Shooter Power"," %.1f", shooter.getPower());

        telemetry.update();

        shooter.setControllerValues(Kp, Kvelo);
        shooter.overridePower(); // shooter motor speed controller

        // Data log
        // Note that the order in which we set datalog fields
        // does *not* matter! Order is configured inside the Datalog class constructor.
        datalog.shooterVelocity.set(shooter.getVelocity());
        datalog.targetVelocity.set(shooter.getTargetVelocity());
        datalog.shooterPower.set(shooter.getPower());
        datalog.writeLine();

    }

    /**
    * Datalog class encapsulates all the fields that will go into the datalog.
    */
    public static class Datalog {
    // The underlying datalogger object - it cares only about an array of loggable fields
    private final Datalogger datalogger;

    // These are all of the fields that we want in the datalog.
    // Note that order here is NOT important. The order is important in the setFields() call below
    public Datalogger.GenericField shooterVelocity = new Datalogger.GenericField("shooterVelocity");
    public Datalogger.GenericField targetVelocity = new Datalogger.GenericField("targetVelocity");
    public Datalogger.GenericField shooterPower = new Datalogger.GenericField("shooterPower");

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
                        shooterVelocity,
                        targetVelocity,
                        shooterPower
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