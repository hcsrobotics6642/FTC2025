package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Auto Red", group = "Robot")
public class AutoRed extends LinearOpMode {

    private RobotHardware robot;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotHardware(hardwareMap);
        RobotData.resetAutoData();
        RobotData.alliance = RobotData.Alliance.Red;  // Critical!

        telemetry.addLine("RED Autonomous Ready");
        telemetry.addLine("Detecting obelisk pattern...");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // -------------------------------------------------------
        // STEP 1 — DETECT OBELISK PATTERN USING APRIL TAGS
        // -------------------------------------------------------
        RobotData.pattern = RobotData.Pattern.NotSet;

        long detectStart = System.currentTimeMillis();
        long detectTimeout = 4000;

        while (opModeIsActive() &&
                RobotData.pattern == RobotData.Pattern.NotSet &&
                System.currentTimeMillis() - detectStart < detectTimeout) {

            LLResult result = robot.getLimelight().getLatestResult();

            if (result != null && result.isValid()) {
                for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
                    int id = fiducial.getFiducialId();

                    if (id == RobotHardware.GPP_APRILTAG_ID) {
                        RobotData.pattern = RobotData.Pattern.GPP;
                    } else if (id == RobotHardware.PGP_APRILTAG_ID) {
                        RobotData.pattern = RobotData.Pattern.PGP;
                    } else if (id == RobotHardware.PPG_APRILTAG_ID) {
                        RobotData.pattern = RobotData.Pattern.PPG;
                    }
                }
            }
            idle();
        }

        // Fallback if nothing detected
        if (RobotData.pattern == RobotData.Pattern.NotSet) {
            RobotData.pattern = RobotData.Pattern.GPP;
        }

        telemetry.addData("Detected Pattern", RobotData.pattern);
        telemetry.update();
        sleep(500);

        // -------------------------------------------------------
        // STEP 2 — GET SHOT ORDER FROM PATTERN
        // -------------------------------------------------------
        int[] shotOrder = robot.getMotifShotOrder();

        telemetry.addData("Shot Order", "%d → %d → %d", shotOrder[0], shotOrder[1], shotOrder[2]);
        telemetry.update();
        sleep(500);

        // -------------------------------------------------------
        // STEP 3 — SHOOT 3 ARTIFACTS IN CORRECT ORDER
        // -------------------------------------------------------
        for (int slot : shotOrder) {
            telemetry.addData("Aiming", "Slot " + slot);
            telemetry.update();

            switch (slot) {
                case 1: robot.rotateToSlot1Shoot(); break;
                case 2: robot.rotateToSlot2Shoot(); break;
                case 3: robot.rotateToSlot3Shoot(); break;
            }

            sleep(500);                    // Let servo settle
            robot.prepareShot();           // Spin up + wait for stable RPM
            robot.shootArtifact();         // Fire!
            sleep(300);                    // Recovery time
        }

        robot.stopFlywheel();

        // -------------------------------------------------------
        // STEP 4 — PARK (RED ALLIANCE = strafe LEFT)
        // -------------------------------------------------------
        robot.parkRed();

        // -------------------------------------------------------
        // STEP 5 — SAVE FINAL POSE FOR TELEOP HANDOFF
        // -------------------------------------------------------
        RobotData.finalAutoPose = robot.updatePose();  // or updatePoseWithFusion() if you have it
        RobotData.autoCompleted = true;

        telemetry.addLine("RED AUTO COMPLETE!");
        telemetry.addData("Pattern", RobotData.pattern);
        telemetry.addData("Final Pose", RobotData.finalAutoPose);
        telemetry.update();

        while (opModeIsActive()) {
            idle();
        }
    }
}