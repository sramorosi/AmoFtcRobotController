package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.qualcomm.robotcore.hardware.IMU;
import com.sun.tools.javac.util.BasicDiagnosticFormatter;

import java.util.List;


public class LimelightDecode {
    private Limelight3A limelight;
    private IMU imu;
    //private double goalYaw; // inches
    private double goalRange; // in

    private int teamID;

    public boolean GPP = false; // id 21
    public boolean PGP = false; // id 22
    public boolean PPG = false; // id 23
    public boolean seeObelisk = false;
    private double tx;
    private double ty;
    private double camera_height = 1.0; // in
    private final double target_height = 29.5; // in
    private double camera_angle = 0.0; // radians old was 0.0418

    public boolean isDataCurrent;
    //Pipeline 5 is 20(blue) pipeline 1 is 24(red)

    public final static double MtoINCH = 39.3701;

    double GoalX = -58.3727;  // where 0,0 is field center and X is toward audience wall
    double blueGoalY = -55.6425;  // where 0,0 is field center and Y is toward blue alliance
    double redGoalY = 55.6425;  // where 0,0 is field center and Y is toward blue alliance

    public void init(HardwareMap hardwareMap, double cameraY, double cameraA) {
        camera_height = cameraY;
        camera_angle = cameraA;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start(); // This tells Limelight to start looking!

        /*
        100 Hz is the default and recommended setting for many applications, especially in the FTC robotics competition using the Limelight 3A model.
        This rate ensures you get the fastest possible updates (100 times per second), which is crucial for responsive, real-time robot control and
        tracking of moving targets.
        50 Hz can be a good option if you are experiencing performance issues or network congestion, as it reduces the frequency of data requests.
        It is still frequent enough for many vision tasks
         */
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)


        imu = hardwareMap.get(IMU.class,"imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
    }

    public void readObelisk(Telemetry telemetry) {
        limelight.pipelineSwitch(6); //targets closest
        LLResult result = limelight.getLatestResult();

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials.isEmpty()) {
            seeObelisk = false;
        }

        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int tagId = fiducial.getFiducialId();
            if (tagId == 21) {
                telemetry.addData("Detected", "Tag ID 21");
                PGP = false;
                PPG = false;
                GPP = true;
                seeObelisk = true;
            } else if (tagId == 22) {
                telemetry.addData("Detected", "Tag ID 22");
                PGP = true;
                PPG = false;
                GPP = false;
                seeObelisk = true;
            } else if (tagId == 23) {
                telemetry.addData("Detected", "Tag ID 23");
                PGP = false;
                PPG = true;
                GPP = false;
                seeObelisk = true;
            }
        }
    }
    public void process(Telemetry telemetry) {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            tx = result.getTx(); // How far left or right the target is (degrees)
            ty = result.getTy(); // How far up or down the target is (degrees)

            //telemetry.addData("pipeline", result.getPipelineIndex());

            //double ta = result.getTa(); // How big the target looks (0%-100% of the image)
            Pose3D botpose = result.getBotpose();
            Pose3D botposeMT2 = result.getBotpose_MT2();
            if (botposeMT2 != null) {
                double x = botpose.getPosition().x*MtoINCH;
                double y = botpose.getPosition().y*MtoINCH;
                telemetry.addLine(String.format("MT1 Location %6.2f %6.2f (m)",x,y));
                telemetry.addData("MT1 rotation", String.format(" %.2f",botpose.getOrientation().getYaw()));

                double distance = 0.0;
                if (result.getPipelineIndex() == 5) distance = calculateDistance(x, y,GoalX,blueGoalY);
                if (result.getPipelineIndex() == 1) distance = calculateDistance(x, y,GoalX,redGoalY);
                telemetry.addData("MT1 Distance", String.format(" %.2f",distance));

                double xMT2 = botposeMT2.getPosition().x;
                double yMT2 = botposeMT2.getPosition().y;
                telemetry.addLine(String.format("MT2 Location %6.2f %6.2f (m)",xMT2,yMT2));
                telemetry.addData("MT2 rotation", String.format(" %.2f",botposeMT2.getOrientation().getYaw()));

                //goalYaw = botposeMT2.getOrientation().getYaw();

                // Compute distance (range) from ty (pitch angle), assuming a fixed camera angle
                goalRange = (target_height - camera_height) / (Math.tan(Math.toRadians(ty)+camera_angle));

                isDataCurrent = true;

            } else {
                isDataCurrent = false;
            }

        } else {
            isDataCurrent = false;
            telemetry.addData("Limelight", "No Targets");
        }
    }

    public void setTeam(int id) {
        if (id == 20) {
            limelight.pipelineSwitch(5);
            teamID = 20;
        } else if (id == 24) {
            limelight.pipelineSwitch(1);
            teamID = 24;
        }
    }

    public String getObelisk() {
        if (PGP) {
            return "PGP";
        } else if (GPP) {
            return "GPP";
        } else if (PPG) {
            return "PPG";
        } else {
            return "No Tag Detected";
        }
    }
    public double getRange() {
        return goalRange;
    }
    public double getTx() {
        return tx;
    }
    public double getTy() {
        return ty;
    }
    public int getID() {
        return teamID;
    }
    public void setCameraAngle(double CameraA) { camera_angle = CameraA; }

    /**
     * Calculates the Euclidean distance between two points (x1, y1) and (x2, y2).
     *
     * @param x1 the x-coordinate of the first point
     * @param y1 the y-coordinate of the first point
     * @param x2 the x-coordinate of the second point
     * @param y2 the y-coordinate of the second point
     * @return the distance between the two points as a double
     */
    public static double calculateDistance(double x1, double y1, double x2, double y2) {
        // Calculate the differences in coordinates
        double deltaX = x2 - x1;
        double deltaY = y2 - y1;

        // Square the differences
        //double squaredDeltaX = Math.pow(deltaX, 2);
        //double squaredDeltaY = Math.pow(deltaY, 2);
        // Alternatively, use multiplication for performance:
        double squaredDeltaX = deltaX * deltaX;
        double squaredDeltaY = deltaY * deltaY;

        // Sum the squares
        double sumOfSquares = squaredDeltaX + squaredDeltaY;

        // Take the square root of the sum
        double distance = Math.sqrt(sumOfSquares);

        return distance;
    }
}
