package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Servo Sweep Tester is to test how fast a servo can sweep from 0 to 1
 *
 * @author Steve Amorosi
 * date 1/11/2026
 */
@TeleOp(name = "ServoSpeedTester", group="Util")
public class ServoSpeedTester extends OpMode {

    String DEVICE_NAME = "launchFlapLeft";

    Servo   servo;
    double increment = 0.05;
    double sweepTime = 1.0;  // seconds for one sweep

    ElapsedTime timer = new ElapsedTime();

    boolean reverse = true;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, DEVICE_NAME);
    }

    @SuppressLint("DefaultLocale")
    public void init_loop() {

        telemetry.addLine("Servo will sweep back and forth");
        telemetry.addLine(String.format("Sweep Time %.2f", sweepTime));
        telemetry.update();
    }

    public void start() {

        servo.setPosition(0.0);
        timer.reset();

    }

    @SuppressLint("DefaultLocale")
    public void loop() {
        telemetry.addLine(String.format("DPad Left: Decrease sweep time by %.2f",increment));
        telemetry.addLine(String.format("DPad Right: Increase sweep time by %.2f",increment));
        telemetry.addData("Sweep Time", "%.2f", sweepTime);
        telemetry.update();

        if (gamepad1.dpadLeftWasPressed()) {
            sweepTime -= increment;
            sweepTime = Math.max(0., sweepTime);
        }
        if (gamepad1.dpadRightWasPressed()) {
            sweepTime += increment;
        }

        if (timer.seconds() > sweepTime) {

            if (reverse) servo.setPosition(1.0);
            else servo.setPosition(0.0);

            timer.reset();

            reverse = !reverse;
        }
    }
}