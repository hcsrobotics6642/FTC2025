package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * ============================================================================
 *  MecanumAutoBlue.java
 *  Runs BLUE alliance autonomous.
 *
 *  Robot starts against the back wall, centered on the launch line.
 *  Steps:
 *    1. Detect the obelisk pattern with Limelight.
 *    2. Use Limelight TX to auto-aim.
 *    3. Shoot long-range from the back launch zone.
 *    4. Drive forward slightly to park.
 *    5. Save ending pose for TeleOp.
 * ============================================================================
 */
@Autonomous(name = "Auto: BLUE Alliance", group = "Robot")
public class MecanumAutoBlue extends LinearOpMode {

    private RobotHardware robot;

    @Override
    public void runOpMode() {

        telemetry.addLine("Initializing robot...");
        telemetry.update();

        robot = new RobotHardware(hardwareMap);

        // Reset all shared match data
        RobotData.resetAutoData();
        RobotData.alliance = RobotData.Alliance.Blue;

        telemetry.addLine("BLUE Auto Ready.");
        telemetry.addLine("Place robot against back wall, centered on launch line.");
        telemetry.addLine("Press PLAY to begin.");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;


        // ---------------------------------------------------------------------
        // STEP 1 — DETECT PATTERN
        // ---------------------------------------------------------------------
        telemetry.addLine("Detecting pattern...");
        telemetry.update();

        robot.detectPattern();

        telemetry.addData("Pattern", RobotData.pattern);
        telemetry.update();


        // ---------------------------------------------------------------------
        // STEP 2 — GET AIM ANGLE FROM LIMELIGHT TX
        // ---------------------------------------------------------------------
        telemetry.addLine("Reading Limelight aim...");
        telemetry.update();

        // For Blue side, tx sign will be reversed compared to Red
        double aimAngle = robot.getAimAngle();

        telemetry.addData("Aim Angle (tx)", aimAngle);
        telemetry.update();

        sleep(200);


        // ---------------------------------------------------------------------
        // STEP 3 — TURN TOWARD TARGET
        // ---------------------------------------------------------------------
        telemetry.addLine("Turning toward target...");
        telemetry.update();

        turnRelativeDegrees(aimAngle);


        // ---------------------------------------------------------------------
        // STEP 4 — SPIN UP FLYWHEEL & SHOOT
        // ---------------------------------------------------------------------
        telemetry.addLine("Spinning up shooter...");
        telemetry.update();

        robot.startFlywheel();

        // Give PID time to stabilize
        sleep(700);

        // Fire artifact
        robot.getLifterServo().setPosition(1.0);
        sleep(400);
        robot.getLifterServo().setPosition(0.0);

        robot.stopFlywheel();

        telemetry.addLine("Shot Fired!");
        telemetry.update();


        // ---------------------------------------------------------------------
        // STEP 5 — DRIVE FORWARD TO PARK
        // ---------------------------------------------------------------------
        telemetry.addLine("Driving to park...");
        telemetry.update();

        robot.drive(0.4, 0, 0);
        sleep(600);
        robot.drive(0, 0, 0);


        // ---------------------------------------------------------------------
        // STEP 6 — SAVE POSE FOR TELEOP
        // ---------------------------------------------------------------------
        Pose2D finalPose = robot.updatePoseWithFusion();
        RobotData.finalAutoPose = finalPose;
        RobotData.autoCompleted = true;

        telemetry.addLine("BLUE Auto Complete.");
        telemetry.addData("Final Pose", finalPose);
        telemetry.update();

        while (opModeIsActive()) idle();
    }


    // =========================================================================
    // TURN RELATIVE USING HEADING FEEDBACK
    // =========================================================================
    private void turnRelativeDegrees(double relativeAngleDeg) {

        double maxTurnPower = 0.35;
        double tolerance = 2.0;
        double k_h = 0.02;

        Pose2D pose = robot.updatePoseWithFusion();
        double initial = pose.getHeading(AngleUnit.DEGREES);

        double target = AngleUnit.normalizeDegrees(initial + relativeAngleDeg);

        while (opModeIsActive()) {

            pose = robot.updatePoseWithFusion();
            double current = pose.getHeading(AngleUnit.DEGREES);
            double error = AngleUnit.normalizeDegrees(target - current);

            if (Math.abs(error) <= tolerance) break;

            double turnPower = k_h * error;
            turnPower = Math.max(-maxTurnPower, Math.min(maxTurnPower, turnPower));

            robot.drive(0, 0, turnPower);

            telemetry.addData("Turning", error);
            telemetry.update();
        }

        robot.drive(0, 0, 0);
        sleep(150);
    }
}
