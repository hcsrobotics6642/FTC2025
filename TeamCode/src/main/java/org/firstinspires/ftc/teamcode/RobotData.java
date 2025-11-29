package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * ========================================================================
 * RobotData.java — simple shared storage between Autonomous and TeleOp.
 * ========================================================================
 *
 * This class contains information BOTH opmodes need to access:
 *
 *   ✔ Alliance (set by driver in TeleOp using X/B buttons)
 *   ✔ Pattern (read by Limelight during Auto)
 *   ✔ Final robot pose after Auto (used to update TeleOp start position)
 *   ✔ Whether Auto completed successfully
 *
 * The advantages of keeping this here:
 *   — No need for global variables inside opmodes
 *   — TeleOp can instantly read results from Autonomous
 *   — Very easy for students to understand
 */
public class RobotData {

    // --------------------------------------------------------------------
    // ALLIANCE ENUM
    // --------------------------------------------------------------------
    public enum Alliance {
        Red,
        Blue,
        NotSet
    }

    // The alliance to run for Auto and TeleOp.
    public static Alliance alliance = Alliance.NotSet;

    // --------------------------------------------------------------------
    // PATTERN ENUM (Obelisk motif)
    // --------------------------------------------------------------------
    public enum Pattern {
        GPP,
        PGP,
        PPG,
        NotSet
    }

    public static Pattern pattern = Pattern.NotSet;

    // --------------------------------------------------------------------
    // AUTO RESULTS
    // --------------------------------------------------------------------

    /** Robot pose at the end of Autonomous. TeleOp uses this for smoother handoff. */
    public static Pose2D finalAutoPose = null;

    /** True once Autonomous finishes. */
    public static boolean autoCompleted = false;

    // --------------------------------------------------------------------
    // RESET METHODS
    // --------------------------------------------------------------------

    /** Reset EVERYTHING before starting a new match. */
    public static void resetAll() {
        alliance = Alliance.NotSet;
        pattern = Pattern.NotSet;
        finalAutoPose = null;
        autoCompleted = false;
    }

    /** Reset only data used by Autonomous. */
    public static void resetAutoData() {
        pattern = Pattern.NotSet;
        finalAutoPose = null;
        autoCompleted = false;
    }
}
