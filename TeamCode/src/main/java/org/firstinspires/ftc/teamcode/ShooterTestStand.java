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

/*
 * This opmode is a copy of 7462 DECODE Mecanum Teleop
 * Its purpose is to run a single shooter motor, launch flap servo, using limelight on a test stand
 * The overall purpose is to optimize artefact shots.
 * Data logging
 * All references to chassis have been removed
 */
@TeleOp(name = "ShooterTestStand")
//@Disabled //comment this out when ready to add to android phone
public class ShooterTestStand extends OpMode {
    GoalTagLimelight limelight;
    Shooter shooterLeft;

    Servo launchFlapLeft;

    // Timers
    ElapsedTime timerLeft = new ElapsedTime();
    ElapsedTime timerFlipper = new ElapsedTime();

    private static double IDLEPOWER = 20;
    //private double kP = 0.3; // was 0.14 before adding 0 breaking
    private boolean leftIsRunning;

    private boolean readyToShoot = true;
    private boolean emergencyMode = false;

    private static double FLAPDOWN = 0.45;
    private static double FLAPUP = 0.75;

    ShooterTestStand.Datalog datalog = new ShooterTestStand.Datalog("ShooterLog");

    int i=0;

    @Override
    public void init() {
        launchFlapLeft = hardwareMap.get(Servo.class, "launchFlapLeft");

        shooterLeft = new Shooter(hardwareMap, "shooterLeft", true);
        shooterLeft.setControllerValues(0.3, 0.0243);

        limelight = new GoalTagLimelight();
        limelight.init(hardwareMap, telemetry);

        timerLeft.reset();
        timerFlipper.reset();

        launchFlapLeft.setPosition(FLAPDOWN);
    }

    //we are using the methods from OpMode and @Override is so that we can write our own stuff for this method
    @Override
    public void init_loop() {
        //telemetry.addData("Pattern", limelight.getObelisk());
        telemetry.addData("team ID", limelight.getID());

        telemetry.addLine("Left Bumper to shoot");
        telemetry.addLine("Press b for red, x for blue");
        telemetry.update();
        if (gamepad1.bWasPressed()) {
//            goalTag.targetAprilTagID = 24;
            limelight.setTeam(24);
        } else if (gamepad1.xWasPressed()) {
            //goalTag.targetAprilTagID = 20;
            limelight.setTeam(20);
        }
    }

    @Override
    public void start() {
        shooterLeft.targetVelocity = IDLEPOWER;
    }

    @Override
    public void loop() {
        limelight.process(telemetry);

        shooterLeft.overridePower();

        telemetry.addData("limelight angle", limelight.getTx());
        telemetry.addData("shooterLeftCurrentVelocity", shooterLeft.getVelocity());
        telemetry.addData("shooterLeftTargetVelocity", getShooterVelo());
        //telemetry.addData("Kp", kP);
        telemetry.addData("TimerLeft", timerLeft.seconds());
        telemetry.update();


        // Driver Controls launch
        //if (gamepad1.leftBumperWasPressed() && (limelight.isDataCurrent || emergencyMode)) {
        if (gamepad1.left_bumper && readyToShoot && (limelight.isDataCurrent || emergencyMode)) {
                shooterLeft.targetVelocity = getShooterVelo();
            leftIsRunning = true;
            readyToShoot = false;
            timerLeft.reset();
        }

        // Shoot when at speed
        if (leftIsRunning) {
            if (shooterLeft.atSpeed()) {
                timerLeft.reset();
                launchFlapLeft.setPosition(FLAPUP);
                leftIsRunning = false;
            }
        }

        // Command launch Flap Down
        if (timerLeft.seconds() > 0.4 && !leftIsRunning) {
            launchFlapLeft.setPosition(FLAPDOWN);
            //shooterLeft.targetVelocity = IDLEPOWER;
            //readyToShoot=true;
        }
        // Flap Down and next artifact is ready
        if (timerLeft.seconds() > 0.8 && !leftIsRunning) {
            //launchFlapLeft.setPosition(FLAPDOWN);
            shooterLeft.targetVelocity = IDLEPOWER;
            readyToShoot=true;
        }
        // If camera not working press this and shoot near point of close V.
        if (gamepad1.leftStickButtonWasPressed()) {
            shooterLeft.targetVelocity = 30;
            IDLEPOWER = 30;
            emergencyMode = true;
        }

        // Data log
        // Note that the order in which we set datalog fields
        // does *not* matter! Order is configured inside the Datalog class constructor.
        datalog.loopCounter.set(i);
        //datalog.runTime.set(timerLeft.seconds());
        datalog.flapPos.set(launchFlapLeft.getPosition());
        datalog.shooterVelocity.set(shooterLeft.getVelocity());
        datalog.targetVelocity.set(shooterLeft.targetVelocity);
        datalog.writeLine();

        i++;
    }

    public double getShooterVelo() {
        // do math here
        //return (limelight.getRange() + 202.17 - 10) / 8.92124;
        return (limelight.getRange()+100.99)/7.3712;
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
    public Datalogger.GenericField runTime = new Datalogger.GenericField("RunTime");
    public Datalogger.GenericField flapPos = new Datalogger.GenericField("FlapPos");
    //public Datalogger.GenericField posError = new Datalogger.GenericField("posError");

    public Datalogger.GenericField shooterVelocity = new Datalogger.GenericField("shooterVelocity");
    public Datalogger.GenericField targetVelocity = new Datalogger.GenericField("targetVelocity");

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
                        runTime,
                        flapPos,
                        //posError,
                        shooterVelocity,
                        targetVelocity
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