package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Decode Auto – NO ENCODERS on flywheel
 * 1. Detect alliance (Limelight) Default is blue.
 * 2. Flywheel = 0.80 power (hard-coded)
 * 3. Shoot pre-loaded artifact (slot 1)
 * 4. Turn 90 degrees (Blue -> CCW, Red -> CW)
 * 5. Drive straight forward ~48 in to leave launch zone
 */
@Autonomous(name = "Decode Auto: 80% Flywheel (No Encoders)", preselectTeleOp ="Robot: Complete Mecanum Drive", group= "Robot")
public class DecodeSimpleAuto extends LinearOpMode {
    /* --------------------------------------------------------------- */
    /* -------------------------- HARDWARE --------------------------- */
    /* --------------------------------------------------------------- */
    private RobotHardware robot;
    private CameraPositionManager cameraManager;
    private IndexerManager indexer;

    // Flywheel power – 80 %
    private static final double FLYWHEEL_POWER = 0.8;

    // Drive distance after turn (tune on field)
    private static final double DRIVE_DISTANCE_INCHES = 36.0;

    // Turn tolerance (degrees)
    private static final double TURN_TOLERANCE = 2.0;

    /* --------------------------------------------------------------- */
    /* ------------------------------ MAIN --------------------------- */
    /* --------------------------------------------------------------- */
    @Override
    public void runOpMode() {
        /* ---------- INITIALISE ---------- */
        robot = new RobotHardware(hardwareMap);
        robot.resetAllPIDs();

        cameraManager = new CameraPositionManager(
                robot.getCameraTilt(),
                robot.getLimelight()
        );
        cameraManager.moveTo(CameraPositionManager.CameraPosition.APRILTAG_VIEW);

        indexer = new IndexerManager(
                robot.getIndexerServo(),
                robot.getLifterServo()
        );

        // Pre-load artifact into slot 1 (color irrelevant)
        indexer.loadArtifactManual(1, IndexerManager.ArtifactColor.GREEN);

        // Odometry start pose
        robot.getPinpoint().setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

        telemetry.addData("Status", "Ready – Press START");
        telemetry.update();

        waitForStart();

        /* ---------- 1. DETECT ALLIANCE ---------- */
        robot.detectAlliance();
        while (!cameraManager.isSettled() && opModeIsActive()) {
            cameraManager.update();
            sleep(10);
        }

        if (RobotData.alliance.equals("NotSet")) {
            telemetry.addData("WARN", "No tag – default BLUE");
            RobotData.alliance = "Blue";
        }
        telemetry.addData("Alliance", RobotData.alliance);
        telemetry.update();

        /* ---------- 2. SPIN FLYWHEEL TO 0.80 ---------- */
       // robot.flywheelMotorLeft.setPower(FLYWHEEL_POWER);
       // robot.flywheelMotorRight.setPower(FLYWHEEL_POWER);

        // Give flywheel time to spin up (no encoder feedback)
       // sleep(1200);   // 1.2 s – adjust if needed

        /* ---------- 3. SHOOT SLOT 1 ---------- */
        //shootSlot(1);
        //telemetry.addData("Shoot", "Done");
        //telemetry.update()

        /* ---------- 4. TURN 90 degrees ---------- */
        //double turnDeg = RobotData.alliance.equals("Blue") ? 90.0 : -90.0;
        //turnRelative(turnDeg);


        /* ---------- 5. DRIVE FORWARD ---------- */
        driveForward(DRIVE_DISTANCE_INCHES);
        telemetry.addData("Drive", "Exited launch zone");
        telemetry.update();

        /* ---------- CLEANUP ---------- */
        robot.stopFlywheel();
        RobotData.autoCompleted = true;
        RobotData.finalAutoPose = robot.updatePoseWithFusion();
    }

    /* --------------------------------------------------------------- */
    /* ------------------------- SHOOT SLOT -------------------------- */
    /* --------------------------------------------------------------- */
    private void shootSlot(int slot) {
        // Rotate to shooter
        indexer.rotateSlotToShooter(slot);
        sleep(600);

        // Lift
        robot.getLifterServo().setPosition(0.5);   // LIFTER_LIFT
        sleep(300);

        // Shoot (flywheel already on)
        sleep(500);

        // Lower
        robot.getLifterServo().setPosition(0.0);   // LIFTER_HOME
        sleep(300);

        // Clear slot
        indexer.clearSlot(slot);
    }

    /* --------------------------------------------------------------- */
    /* ------------------------- TURN RELATIVE ----------------------- */
    /* --------------------------------------------------------------- */
    private void turnRelative(double relativeDegrees) {
        Pose2D cur = robot.updatePoseWithFusion();
        double target = AngleUnit.normalizeDegrees(cur.getHeading(AngleUnit.DEGREES) + relativeDegrees);

        robot.headingPID.setTarget(0);
        robot.headingPID.setTolerance(TURN_TOLERANCE);

        while (opModeIsActive()) {
            cur = robot.updatePoseWithFusion();
            double error = AngleUnit.normalizeDegrees(target - cur.getHeading(AngleUnit.DEGREES));
            double power = robot.headingPID.calculate(error);
            robot.drive(0, 0, power);

            if (Math.abs(error) <= TURN_TOLERANCE) break;
            sleep(10);
        }
        robot.drive(0, 0, 0);
    }

    /* --------------------------------------------------------------- */
    /* ------------------------- DRIVE FORWARD ----------------------- */
    /* --------------------------------------------------------------- */
    private void driveForward(double inches) {
        Pose2D cur = robot.updatePoseWithFusion();
        double rad = cur.getHeading(AngleUnit.RADIANS);

        Pose2D target = new Pose2D(
                DistanceUnit.INCH,
                cur.getX(DistanceUnit.INCH) + inches * Math.cos(rad),
                cur.getY(DistanceUnit.INCH) + inches * Math.sin(rad),
                AngleUnit.DEGREES,
                cur.getHeading(AngleUnit.DEGREES)
        );

        robot.driveToPose(target, this);
    }
}