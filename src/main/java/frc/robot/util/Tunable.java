// package frc.robot.util;

// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// public class Tunable {
    
//     private static final String TABLE_KEY = "Tunables";
//     private static final boolean TUNING_MODE = false;
//     private static final ShuffleboardTab TAB = Shuffleboard.getTab("Tunables");
//     private String key;
//     private double defaultVal;
    
//     public Tunable(String key) {
//         this.key = TABLE_KEY + "/" + key;
//     }

//     public Tunable(String key, double defaultVal) {
//         this(key);
//         setDefault(defaultVal);
//     }

//     public double getDefault() {
//         return defaultVal;
//     }

//     // public void setDefault(double defaultVal) {
//     //     this.defaultVal = defaultVal;
//     //     if(TUNING_MODE) {
//     //         TAB.addDouble(key, TAB.getx);
//     //     }
//     // }

// }
