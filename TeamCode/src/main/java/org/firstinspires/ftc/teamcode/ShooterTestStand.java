/* Copyright (c) 2025 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * ShooterTestStand opmode runs a single shooter motor, launch flap servo, using limelight on a test stand
 * Its purpose is to improve artifact shots (3 in a row, accuracy)
 * Data logging is used to record real-time data.
 */
@TeleOp(name = "ShooterTestStand")
//@Disabled //comment this out when ready to add to android phone
public class ShooterTestStand extends OpMode {
    LimelightDecode limelight;
    Shooter shooterLeft;

    Servo launchFlapLeft;

    // Timers
    ElapsedTime timerLeft = new ElapsedTime();

    private static double IDLEPOWER = 20;
    //private double kP = 0.3; // was 0.14 before adding 0 breaking
    private boolean shooterAtSpeed = true; // needs to be true initially

    private boolean readyForNextShoot = false;
    private boolean emergencyMode = false;

    private static final double FLAPDOWN = 0.45;
    private static final double FLAPUP = 0.75;

    ShooterTestStand.Datalog datalog = new ShooterTestStand.Datalog("ShooterLog");

    int i=0;

    double cameraA = 0.042; // fixed tilt, in radians

    @Override
    public void init() {
        launchFlapLeft = hardwareMap.get(Servo.class, "launchFlapLeft");

        shooterLeft = new Shooter(hardwareMap, "shooterLeft", true);
        shooterLeft.setControllerValues(0.3, 0.0243);

        limelight = new LimelightDecode();
        limelight.init(hardwareMap, 17.0,cameraA);

        timerLeft.reset();

        launchFlapLeft.setPosition(FLAPDOWN);

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
    public void init_loop() {
        //telemetry.addData("Pattern", limelight.getObelisk());
        telemetry.addData("team ID", limelight.getID());

        telemetry.addLine("Press B for red, X for blue");
        if (gamepad1.bWasPressed()) {
            // goalTag.targetAprilTagID = 24; RED
            limelight.setTeam(24);
        } else if (gamepad1.xWasPressed()) {
            // goalTag.targetAprilTagID = 20;  BLUE
            limelight.setTeam(20);
        }

        telemetry.addLine("ADJUST CAMERA ANGLE USING DPAD LEFT AND RIGHT");
        telemetry.addLine("TO GET THE CORRECT COMPUTED RANGE");
        if (gamepad1.dpadLeftWasPressed()) {
            cameraA += 0.002;
            limelight.setCameraAngle(cameraA);
        } else if (gamepad1.dpadRightWasPressed()) {
            cameraA -= 0.002;
            limelight.setCameraAngle(cameraA);
        }
        telemetry.addData("Camera Angle ",cameraA);

        // Add limelight processing so we can see if range is good.
        limelight.process(telemetry);
        //telemetry.addData("range", String.format("%.01f mm", sensorDistance.getDistance(DistanceUnit.MM)));

        telemetry.addData("Apriltag tx YAW (DEG) SHOULD BE ZERO",String.format(" %.1f", limelight.getTx()));
        telemetry.addData("Apriltag ty PITCH (DEG)",String.format(" %.1f", limelight.getTy()));
        telemetry.addData("Apriltag computed range (in)",String.format(" %.1f", limelight.getRange()));

        telemetry.update();
    }

    @Override
    public void start() {
        shooterLeft.targetVelocity = IDLEPOWER;
        timerLeft.reset();
    }

    @Override
    public void loop() {
        limelight.process(telemetry);

        shooterLeft.overridePower(); // shooter motor speed controller

        telemetry.addLine("Left Bumper to shoot");

        telemetry.addData("shooterLeftCurrentVelocity",String.format(" %.1f", shooterLeft.getVelocity()));
        telemetry.addData("shooterLeftTargetVelocity",String.format(" %.1f", shooterLeft.getShooterVelo(limelight)));
        telemetry.addData("Apriltag computed range (in)",String.format(" %.1f", limelight.getRange()));

        telemetry.update();


        // Driver commands launch sequence, repeating shots
        if (gamepad1.left_bumper && readyForNextShoot && (limelight.isDataCurrent || emergencyMode)) {
            shooterLeft.targetVelocity = shooterLeft.getShooterVelo(limelight);
            shooterAtSpeed = false;
            readyForNextShoot = false;
            timerLeft.reset();
        }

        // Shoot when at speed
        if (!shooterAtSpeed) {
            if (shooterLeft.atSpeed()) {
                timerLeft.reset();
                launchFlapLeft.setPosition(FLAPUP);
                shooterAtSpeed = true;
            }
        }

        // Command launch Flap Down
        if (timerLeft.seconds() > 0.4 && shooterAtSpeed) launchFlapLeft.setPosition(FLAPDOWN);

        // Allow time for launch flap to go Down and artifact to enter. Toggle ready for next shoot
        if (timerLeft.seconds() > 0.7 && shooterAtSpeed) readyForNextShoot =true;

        // Finger is off of the button, set shooter speed back to idle
        if (!gamepad1.left_bumper) {
            shooterLeft.targetVelocity = IDLEPOWER;
            readyForNextShoot = true;
        }

        // If camera not working press this and shoot near point of close V.
        if (gamepad1.leftStickButtonWasPressed()) {
            shooterLeft.targetVelocity = 30;
            //IDLEPOWER = 30;
            emergencyMode = true;
        }

        // Data log
        // Note that the order in which we set datalog fields
        // does *not* matter! Order is configured inside the Datalog class constructor.
        datalog.loopCounter.set(i);
        datalog.flapPos.set(launchFlapLeft.getPosition());
        datalog.shooterVelocity.set(shooterLeft.getVelocity());
        datalog.targetVelocity.set(shooterLeft.targetVelocity);
        datalog.shooterPower.set(shooterLeft.getPower());
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