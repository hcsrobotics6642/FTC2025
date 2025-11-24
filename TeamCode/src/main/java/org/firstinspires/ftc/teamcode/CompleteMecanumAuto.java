package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Complete Autonomous with:
 * - Camera management for AprilTag/Obelisk detection
 * - Alliance and pattern detection
 * - Indexer control for preloaded artifacts
 * - Flywheel shooter for scoring
 * - Navigation to scoring position
 */
@Autonomous(name = "Robot: Complete Autonomous", group = "Robot")
@Disabled
public class CompleteMecanumAuto extends LinearOpMode {
    private RobotHardware robot;
    private CameraPositionManager cameraManager;
    private IndexerManager indexer;

    @Override
    public void runOpMode() {
        // Initialize hardware
        robot = new RobotHardware(hardwareMap);

        // Initialize camera position manager
        cameraManager = new CameraPositionManager(
                hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "cameraTilt"),
                robot.getLimelight()
        );

        // Initialize indexer
        indexer = new IndexerManager(
                hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "indexer"),
                hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "lifter")
        );

        // Set initial pose (adjust based on starting position)
        robot.getPinpoint().setPosition(new Pose2D(DistanceUnit.INCH, 63, -9, AngleUnit.DEGREES, 0));

        // ========== PRELOAD ARTIFACTS ==========
        // STANDARD PRELOAD: Always load the same way every match
        // Slot 1 = GREEN
        // Slot 2 = PURPLE
        // Slot 3 = PURPLE
        // The indexer will automatically shoot in the correct order based on detected motif
        indexer.loadArtifactManual(1, IndexerManager.ArtifactColor.GREEN);
        indexer.loadArtifactManual(2, IndexerManager.ArtifactColor.PURPLE);
        indexer.loadArtifactManual(3, IndexerManager.ArtifactColor.PURPLE);

        telemetry.addData(">", "Robot Ready. Press Play.");
        telemetry.addData("Preload", "Slot 1=GREEN, Slots 2&3=PURPLE");
        telemetry.addData(">", "Motif will be detected and shot automatically");
        telemetry.update();

        waitForStart();

        if (!opModeIsActive()) return;

        robot.resetAllPIDs();
        robot.getPinpoint().resetPosAndIMU();

        // ========== STEP 1: DETECT ALLIANCE ==========
        telemetry.addData("Step 1", "Detecting Alliance...");
        telemetry.update();

        cameraManager.moveTo(CameraPositionManager.CameraPosition.APRILTAG_VIEW);
        sleep(400); // Wait for camera to settle

        robot.detectAlliance();


        telemetry.addData("Alliance", RobotData.alliance);
        if (!RobotData.teamIndicatorSeen) {
            telemetry.addData("Warning", "Alliance Detection Failed");
        }
        telemetry.update();
        sleep(500);

        // ========== STEP 2: TURN TO FACE OBELISK ==========
        telemetry.addData("Step 2", "Turning to obelisk...");
        telemetry.update();

        if (RobotData.alliance.equals("Red")) {
            turnRelativeDegrees(90); // CCW for Red
        } else if (RobotData.alliance.equals("Blue")) {
            turnRelativeDegrees(-90); // CW for Blue
        }

        // ========== STEP 3: DETECT PATTERN/MOTIF ==========
        telemetry.addData("Step 3", "Detecting Pattern...");
        telemetry.update();

        cameraManager.moveTo(CameraPositionManager.CameraPosition.OBELISK_VIEW);
        sleep(400); // Wait for camera to settle

        robot.detectPattern();

        telemetry.addData("Pattern", RobotData.pattern);
        if (RobotData.pattern.equals("NotSet")) {
            telemetry.addData("Warning", "Pattern Detection Failed");
        }
        telemetry.update();
        sleep(500);

        // ========== STEP 4: POINT CAMERA BACK UP FOR NAVIGATION ==========
        cameraManager.moveTo(CameraPositionManager.CameraPosition.APRILTAG_VIEW);
        sleep(400);

        // ========== STEP 5: NAVIGATE TO SCORING POSITION ==========
        if (!RobotData.alliance.equals("NotSet")) {
            telemetry.addData("Step 4", "Navigating to scoring position...");
            telemetry.update();

            Pose2D targetPose = RobotData.alliance.equals("Red") ?
                    RobotData.RED_SCORING_POSE :
                    RobotData.BLUE_SCORING_POSE;

            robot.driveToPose(targetPose, this);

            telemetry.addData("Navigation", "Reached scoring position");
            telemetry.update();
            sleep(500);
        } else {
            telemetry.addData("Error", "Cannot navigate - alliance unknown");
            telemetry.update();
        }

        // ========== STEP 6: SHOOT PRELOADED ARTIFACTS ==========
        telemetry.addData("Step 5", "Shooting preloaded artifacts...");
        telemetry.update();

        shootArtifacts();

        // ========== STEP 7: OPTIONAL - COLLECT MORE ARTIFACTS ==========
        // Uncomment this section if you want to collect more artifacts during auto
        /*
        telemetry.addData("Step 6", "Collecting more artifacts...");
        telemetry.update();

        // Drive to artifact collection area (adjust pose as needed)
        Pose2D collectionPose = new Pose2D(DistanceUnit.INCH, 24, 0, AngleUnit.DEGREES, 0);
        robot.driveToPose(collectionPose, this);

        // Intake artifacts
        intakeArtifacts(3); // Collect up to 3 more artifacts

        // Return to scoring position
        if (!RobotData.alliance.equals("NotSet")) {
            Pose2D targetPose = RobotData.alliance.equals("Red") ?
                               RobotData.RED_SCORING_POSE :
                               RobotData.BLUE_SCORING_POSE;
            robot.driveToPose(targetPose, this);
        }

        // Shoot collected artifacts
        shootArtifacts();
        */

        // ========== STEP 8: PARK ==========
        telemetry.addData("Step 6", "Parking...");
        telemetry.update();

        // Adjust parking position based on your strategy
        // Example: Park in observation zone
        // Pose2D parkPose = new Pose2D(DistanceUnit.INCH, 0, -60, AngleUnit.DEGREES, 0);
        // robot.driveToPose(parkPose, this);

        // ========== SAVE FINAL POSE FOR TELEOP ==========
        RobotData.finalAutoPose = robot.updatePoseWithFusion();
        RobotData.autoCompleted = true;

        telemetry.addData("Autonomous", "Complete!");
        telemetry.addData("Final Pose", "X: %.1f, Y: %.1f, H: %.1f",
                RobotData.finalAutoPose.getX(DistanceUnit.INCH),
                RobotData.finalAutoPose.getY(DistanceUnit.INCH),
                RobotData.finalAutoPose.getHeading(AngleUnit.DEGREES));
        telemetry.update();

        // Wait until end of autonomous
        while (opModeIsActive()) {
            idle();
        }
    }

    /**
     * Turn the robot by a relative angle
     */
    private void turnRelativeDegrees(double relativeAngleDeg) {
        double maxTurnPower = 0.3;
        double tolerance = 2.0;

        robot.headingPID.setTolerance(tolerance);
        robot.headingPID.setTarget(0);

        double initialHeading = robot.getHeading(AngleUnit.DEGREES);
        double targetHeading = AngleUnit.normalizeDegrees(initialHeading + relativeAngleDeg);

        while (opModeIsActive() &&
                Math.abs(AngleUnit.normalizeDegrees(targetHeading - robot.getHeading(AngleUnit.DEGREES))) > tolerance) {

            double headingError = AngleUnit.normalizeDegrees(targetHeading - robot.getHeading(AngleUnit.DEGREES));

            double rotate = robot.headingPID.calculate(headingError);
            rotate = Math.max(-maxTurnPower, Math.min(maxTurnPower, rotate));

            robot.drive(0, 0, rotate);

            telemetry.addData("Turning", "%.1f°", relativeAngleDeg);
            telemetry.addData("Target", "%.1f°", targetHeading);
            telemetry.addData("Current", "%.1f°", robot.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Error", "%.1f°", headingError);
            telemetry.update();

            sleep(10);
        }

        robot.drive(0, 0, 0);
        robot.headingPID.reset();
    }

    /**
     * Shoot all artifacts in the indexer
     */
    private void shootArtifacts() {
        // Start flywheel
        robot.startFlywheel();
        telemetry.addData("Flywheel", "Spinning up...");
        telemetry.update();

        // Wait for flywheel to reach speed using PID ready check
        while (opModeIsActive() && !robot.isFlywheelReady()) {
            robot.updateFlywheel();
            sleep(50);
        }

        // Start the shooting sequence
        boolean sequenceStarted = indexer.startShooting(RobotData.pattern);

        if (!sequenceStarted) {
            telemetry.addData("Shooting", "No artifacts to shoot");
            telemetry.update();
            robot.stopFlywheel();
            return;
        }

        telemetry.addData("Shooting", "Sequence started");
        telemetry.update();

        // Run the shooting state machine until complete
        while (opModeIsActive() && !indexer.isShootingComplete()) {
            // Update flywheel PID
            robot.updateFlywheel();

            // Flywheel is ready (checked via PID)
            indexer.updateShooting(robot.isFlywheelReady());

            telemetry.addData("Shooting", indexer.getShootingState());
            telemetry.addData("Shot", "%d of 3", indexer.getCurrentShotNumber());
            telemetry.update();

            sleep(50); // Small delay to prevent loop from running too fast
        }

        // Stop flywheel
        robot.stopFlywheel();

        telemetry.addData("Shooting", "Complete!");
        telemetry.update();
        sleep(500);
    }

    /**
     * Intake artifacts using the indexer and camera
     * @param targetCount How many artifacts to collect (max 3)
     */
    private void intakeArtifacts(int targetCount) {
        // Point camera down at intake
        cameraManager.moveTo(CameraPositionManager.CameraPosition.ARTIFACT_INTAKE);
        sleep(400);

        // Start loading
        indexer.startLoading();

        int artifactsCollected = 0;
        long startTime = System.currentTimeMillis();
        long timeout = 10000; // 10 second timeout

        telemetry.addData("Intake", "Collecting artifacts...");
        telemetry.update();

        while (opModeIsActive() &&
                artifactsCollected < targetCount &&
                !indexer.isLoadingComplete() &&
                (System.currentTimeMillis() - startTime) < timeout) {

            // Update camera manager
            cameraManager.update();

            // Detect artifact color (this would need your actual color detection implementation)
            IndexerManager.ArtifactColor detectedColor = detectArtifactColorSimple();

            // Update indexer with detected color
            IndexerManager.LoadingState previousState = indexer.getLoadingState();
            indexer.updateLoading(detectedColor);

            // Check if we just loaded an artifact
            if (previousState == IndexerManager.LoadingState.ARTIFACT_DETECTED &&
                    indexer.getLoadingState() == IndexerManager.LoadingState.WAITING_FOR_ARTIFACT) {
                artifactsCollected++;
                telemetry.addData("Collected", "%d of %d", artifactsCollected, targetCount);
                telemetry.update();
            }

            sleep(50);
        }

        indexer.stopLoading();

        // Point camera back up
        cameraManager.moveTo(CameraPositionManager.CameraPosition.APRILTAG_VIEW);
        sleep(400);

        telemetry.addData("Intake", "Collected %d artifacts", artifactsCollected);
        telemetry.update();
    }

    /**
     * Simple artifact color detection for autonomous
     * Replace this with actual color detection logic from LimelightColorDetector
     */
    private IndexerManager.ArtifactColor detectArtifactColorSimple() {
        // Placeholder - implement actual color detection
        // For now, returns NONE
        // You would use the LimelightColorDetector here
        return IndexerManager.ArtifactColor.NONE;
    }
}
