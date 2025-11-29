package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * ============================================================================
 *  MecanumAutoRed.java
 *  Runs RED alliance autonomous.
 *  - Starts at the far back wall, centered on the launch line.
 *  - Detects the obelisk pattern using Limelight.
 *  - Uses Limelight TX for auto-aim.
 *  - Shoots from long range.
 *  - Saves ending pose for TeleOp.
 * ============================================================================
 */
@Autonomous(name = "Auto: RED Alliance", group = "Robot")
public class MecanumAutoRed extends LinearOpMode {

    private RobotHardware robot;

    @Override
    public void runOpMode() {

        telemetry.addLine("Initializing robot...");
        telemetry.update();

        robot = new RobotHardware(hardwareMap);

        // Make sure we start clean
        RobotData.resetAutoData();
        RobotData.alliance = RobotData.Alliance.Red;

        telemetry.addLine("RED Auto Ready.");
        telemetry.addLine("Position robot against the back wall.");
        telemetry.addLine("Press PLAY to run.");
        telemetry.update();

        waitForStart();

        if (!opModeIsActive()) return;

        // ---------------------------------------------------------------------
        // STEP 1 — DETECT PATTERN AT START
        // ---------------------------------------------------------------------
        telemetry.addLine("Detecting pattern...");
        telemetry.update();

        robot.detectPattern();

        telemetry.addData("Pattern", RobotData.pattern);
        telemetry.update();


        // ---------------------------------------------------------------------
        // STEP 2 — GET AIM ANGLE FROM LIMELIGHT
        // ---------------------------------------------------------------------
        telemetry.addLine("Reading Limelight...");
        telemetry.update();

        // Get TX (horizontal error in degrees)
        double aimAngle = robot.getAimAngle();    // For Red: usually negative

        telemetry.addData("Aim Angle (tx)", aimAngle);
        telemetry.update();

        sleep(200);


        // ---------------------------------------------------------------------
        // STEP 3 — TURN TOWARD THE TARGET USING TX
        // ---------------------------------------------------------------------
        telemetry.addLine("Turning toward target...");
        telemetry.update();

        turnRelativeDegrees(aimAngle);


        // ---------------------------------------------------------------------
        // STEP 4 — SPIN UP SHOOTER AND FIRE
        // ---------------------------------------------------------------------
        telemetry.addLine("Spinning up shooter...");
        telemetry.update();

        robot.startFlywheel();

        // Allow PID to settle
        sleep(700);

        // Lift artifact into flywheel
        robot.getLifterServo().setPosition(1.0);
        sleep(400);
        robot.getLifterServo().setPosition(0.0);

        // Stop shooter after firing
        robot.stopFlywheel();

        telemetry.addLine("Shot Fired!");
        telemetry.update();


        // ---------------------------------------------------------------------
        // STEP 5 — OPTIONAL: DRIVE FORWARD TO PARK
        // ---------------------------------------------------------------------
        telemetry.addLine("Driving to park...");
        telemetry.update();

        robot.drive(0.4, 0, 0);
        sleep(600);
        robot.drive(0, 0, 0);


        // ---------------------------------------------------------------------
        // STEP 6 — SAVE ENDING POSE FOR TELEOP HANDOFF
        // ---------------------------------------------------------------------
        Pose2D finalPose = robot.updatePoseWithFusion();
        RobotData.finalAutoPose = finalPose;
        RobotData.autoCompleted = true;

        telemetry.addData("Final Pose", finalPose);
        telemetry.addLine("RED Auto Complete.");
        telemetry.update();

        // Standby until end of autonomous period
        while (opModeIsActive()) {
            idle();
        }
    }

    // =========================================================================
    // TURN RELATIVE USING HEADING FEEDBACK
    // =========================================================================
    private void turnRelativeDegrees(double relativeAngleDeg) {

        double maxTurnPower = 0.35;
        double tolerance = 2.0;         // Within 2 degrees is good
        double k_h = 0.02;              // Proportional gain

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
