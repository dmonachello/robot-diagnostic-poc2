// package frc.robot.diag.core;

// import java.util.HashMap;
// import java.util.Map;

// import edu.wpi.first.networktables.GenericEntry;
// import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
// import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

// /**
//  * Centralized Shuffleboard construction for diagnostics.
//  *
//  * IMPORTANT RULE:
//  *  - Never pass NetworkTableEntry into Shuffleboard.add(...)
//  *  - Always add using a primitive default, then retrieve the entry
//  *
//  * WPILib 2025 note:
//  *  - Widgets return GenericEntry (not NetworkTableEntry).
//  */
// public final class DiagDashboard {

//     private static final String TAB_NAME = "Diag";

//     private static ShuffleboardTab tab;
//     private static ShuffleboardLayout layout;

//     private static final Map<String, GenericEntry> entries = new HashMap<>();

//     private static boolean built = false;

//     private DiagDashboard() {}

//     // --------------------------------------------------------------------
//     // Public API
//     // --------------------------------------------------------------------

//     // Keep your existing call sites happy.
//     public static void build() {
//         buildOnce();
//     }

//     /**
//      * Build the dashboard container exactly once.
//      * Safe to call multiple times.
//      */
//     public static void buildOnce() {
//         if (built) {
//             return;
//         }
//         built = true;

//         tab = Shuffleboard.getTab(TAB_NAME);

//         layout = tab.getLayout("Devices", BuiltInLayouts.kList)
//                     .withPosition(0, 0)
//                     .withSize(10, 20);
//     }

//     /**
//      * Register the standard controls and status fields for a device.
//      * All booleans are forced to false at startup.
//      */
//     public static void registerDevice(String diagName) {
//         buildOnce();

//         addToggle(diagName, "Enable", false);
//         addToggle(diagName, "Retry", false);

//         addString(diagName, "LastStatusHex", "0x00000000");
//         addString(diagName, "Health", "UNKNOWN");
//         addString(diagName, "StatusSummary", "test not run");
//     }

//     /**
//      * Register a UseTerm_* toggle for a terminator attached to a DUT.
//      * Default state can be chosen per call (you said you want false at start).
//      */
//     public static void registerUseTerm(String dutName, String termName, boolean defaultValue) {
//         buildOnce();
//         addToggle(dutName, "UseTerm_" + termName, defaultValue);
//     }

//     /**
//      * Retrieve a previously registered entry.
//      */
//     public static GenericEntry getEntry(String diagName, String field) {
//         return entries.get(key(diagName, field));
//     }

//     // --------------------------------------------------------------------
//     // Internal helpers
//     // --------------------------------------------------------------------

//     private static String key(String diagName, String field) {
//         return diagName + "/" + field;
//     }

//     private static GenericEntry addToggle(String diagName, String field, boolean defaultValue) {
//         String k = key(diagName, field);

//         SimpleWidget w = layout.add(k, defaultValue)
//                                .withWidget(BuiltInWidgets.kToggleButton);

//         GenericEntry e = w.getEntry();
//         e.setBoolean(defaultValue);

//         entries.put(k, e);
//         return e;
//     }

//     private static GenericEntry addString(String diagName, String field, String defaultValue) {
//         String k = key(diagName, field);

//         SimpleWidget w = layout.add(k, defaultValue);
//         GenericEntry e = w.getEntry();
//         e.setString(defaultValue);

//         entries.put(k, e);
//         return e;
//     }
// }
