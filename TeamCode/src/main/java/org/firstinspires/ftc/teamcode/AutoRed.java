package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Auto Red - Back Wall Start", group = "Robot")
public class AutoRed extends LinearOpMode {

    private RobotHardware robot;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotHardware(hardwareMap, this);  // Pass 'this' OpMode
        RobotData.resetAutoData();
        RobotData.alliance = RobotData.Alliance.Red;

        telemetry.addLine("RED AUTO READY");
        telemetry.addLine("Starting flat against back wall");
        telemetry.addLine("Detecting Obelisk motif...");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // ========================================================
        // STEP 1: DETECT OBELISK PATTERN (AprilTags 21-23)
        // ========================================================
        RobotData.pattern = RobotData.Pattern.NotSet;

        long detectStart = System.currentTimeMillis();
        while (opModeIsActive() &&
                RobotData.pattern == RobotData.Pattern.NotSet &&
                System.currentTimeMillis() - detectStart < 4000) {

            LLResult result = robot.getLimelight().getLatestResult();
            if (result != null && result.isValid()) {
                for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
                    int id = fiducial.getFiducialId();
                    if (id == 21) RobotData.pattern = RobotData.Pattern.GPP;
                    else if (id == 22) RobotData.pattern = RobotData.Pattern.PGP;
                    else if (id == 23) RobotData.pattern = RobotData.Pattern.PPG;
                }
            }
            idle();
        }

        if (RobotData.pattern == RobotData.Pattern.NotSet) {
            RobotData.pattern = RobotData.Pattern.GPP;  // Safe default
        }
        // After you set RobotData.pattern = ...
        robot.setAllianceLED();   // Show Blue/Red
        robot.setMotifLEDs();     // Show GPP/PGP/PPG pattern

        // ========================================================
        // STEP 2: GET SHOT ORDER
        // ========================================================
        int[] shotOrder = robot.getMotifShotOrder();

        telemetry.addData("Shot Order", "%d → %d → %d", shotOrder[0], shotOrder[1], shotOrder[2]);
        telemetry.update();
        sleep(500);

        // ========================================================
        // STEP 3: TURN TO SHOOTING HEADING (Red = 180 degrees)
        // ========================================================
        robot.turnToHeading(165,2);  // Face straight down field on Red side
        sleep(300);

        // ========================================================
        // STEP 4: FIRE ALL 3 ARTIFACTS
        // ========================================================
        for (int slot : shotOrder) {
            telemetry.addData("Firing Slot", slot);
            telemetry.update();

            switch (slot) {
                case 1: robot.rotateToShootSlot(1); break;
                case 2: robot.rotateToShootSlot(2); break;
                case 3: robot.rotateToShootSlot(3); break;
            }

            sleep(600);                    // Let indexer settle
            robot.prepareShot();           // Spin up + stabilize
            robot.shootArtifact();         // Lifter up
            sleep(900);                   // Full lifter cycle + buffer
            robot.returnLifter();          // Lifter down
            sleep(600);                    // Recovery
        }

        robot.stopFlywheel();

        // ========================================================
        // STEP 5: DRIVE INTO OBSERVATION ZONE (~26 inches forward)
        // ========================================================
        telemetry.addLine("Driving into Observation Zone...");
        telemetry.update();

        robot.driveStraight(0.2, 26);   // 26 inches forward at 60% power

        // ========================================================
        // STEP 6: SAVE POSE FOR TELEOP HANDOFF
        // ========================================================
        RobotData.finalAutoPose = robot.updatePose();
        RobotData.autoCompleted = true;

        telemetry.addLine("RED AUTO COMPLETE!");
        telemetry.addData("Final Pose", RobotData.finalAutoPose);
        telemetry.update();

        while (opModeIsActive()) {
            idle();
        }
    }
}