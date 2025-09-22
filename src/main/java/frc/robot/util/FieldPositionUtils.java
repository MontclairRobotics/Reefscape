// package frc.robot.util;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.util.Units;

// public class FieldPositionUtils {
//     public static final double ROBOT_X_SIZE = Units.inchesToMeters(32);
//     public static final double ROBOT_Y_SIZE = Units.inchesToMeters(32);

//     public static final double FIELD_X_SIZE = 17.548;
//     public static final double FIELD_Y_SIZE = 8.052;
//     public static final double FIELD_MIN_X = 0 + ROBOT_X_SIZE / 2;
//     public static final double FIELD_MAX_X = FIELD_X_SIZE - ROBOT_X_SIZE / 2;
//     public static final double FIELD_MIN_Y = 0 + ROBOT_Y_SIZE / 2;
//     public static final double FIELD_MAX_Y = FIELD_Y_SIZE - ROBOT_Y_SIZE / 2;


//     public static final double CORAL_STATION_X_SIZE = 1.672;
//     public static final double CORAL_STATION_Y_SIZE = 1.270;

//     public static final double CORAL_STATION_BLUE_X = FIELD_MIN_X + CORAL_STATION_X_SIZE;
//     public static final double CORAL_STATION_BLUE_Y1 = FIELD_MIN_Y + CORAL_STATION_Y_SIZE;
//     public static final double CORAL_STATION_BLUE_Y2 = FIELD_MAX_Y - CORAL_STATION_Y_SIZE;

//     public static final double CORAL_STATION_RED_X = FIELD_MAX_X - CORAL_STATION_X_SIZE;
//     public static final double CORAL_STATION_RED_Y1 = FIELD_MIN_Y + CORAL_STATION_Y_SIZE;
//     public static final double CORAL_STATION_RED_Y2 = FIELD_MAX_Y - CORAL_STATION_Y_SIZE;

//     public static final double REEF_X = 4.489;
//     public static final double REEF_Y = 4.026;
//     public static final double REEF_X_SIZE = 1.6633;
//     public static final double REEF_Y_SIZE = 1.6633;
//     public static final double REEF_Y_1 = 3.546;
//     public static final double REEF_Y_2 = 4.506;

//     public static final double REEF_BLUE_X = REEF_X;
//     public static final double REEF_BLUE_Y = REEF_Y;
//     public static final double REEF_BLUE_Y_1 = REEF_Y_1 - Math.sin(Units.degreesToRadians(30)) * ROBOT_Y_SIZE / 2;
//     public static final double REEF_BLUE_Y_2 = REEF_Y_2 + Math.sin(Units.degreesToRadians(30)) * ROBOT_Y_SIZE / 2;
//     public static final double REEF_BLUE_MIN_X = REEF_X - REEF_X_SIZE / 2 - ROBOT_X_SIZE / 2;
//     public static final double REEF_BLUE_MAX_X = REEF_X + REEF_X_SIZE / 2 + ROBOT_X_SIZE / 2;
//     public static final double REEF_BLUE_MIN_Y = REEF_Y - REEF_Y_SIZE / 2 - ROBOT_Y_SIZE / 2;
//     public static final double REEF_BLUE_MAX_Y = REEF_Y + REEF_Y_SIZE / 2 + ROBOT_Y_SIZE / 2;


//     public static final double REEF_RED_X = FIELD_X_SIZE - REEF_X;
//     public static final double REEF_RED_Y = FIELD_X_SIZE - REEF_Y;
//     public static final double REEF_RED_Y_1 = REEF_Y_1 - Math.sin(Units.degreesToRadians(30)) * ROBOT_Y_SIZE / 2;
//     public static final double REEF_RED_Y_2 = REEF_Y_2 + Math.sin(Units.degreesToRadians(30)) * ROBOT_Y_SIZE / 2;
//     public static final double REEF_RED_MIN_X = FIELD_X_SIZE - REEF_X - REEF_X_SIZE / 2 - ROBOT_X_SIZE / 2;
//     public static final double REEF_RED_MAX_X = FIELD_X_SIZE - REEF_X + REEF_X_SIZE / 2 + ROBOT_X_SIZE / 2;
//     public static final double REEF_RED_MIN_Y = FIELD_Y_SIZE - REEF_Y - REEF_Y_SIZE / 2 - ROBOT_Y_SIZE / 2;
//     public static final double REEF_RED_MAX_Y = FIELD_Y_SIZE - REEF_Y + REEF_Y_SIZE / 2 + ROBOT_Y_SIZE / 2;



//     public enum LineDirection {
//         MOVE_ABOVE,
//         MOVE_BELOW,
//         MOVE_BOTH;
//     }
//     public static Translation2d getClosestPointToLine(double x, double y, double x1, double y1, double x2, double y2, LineDirection direction) {
//         // Calculate the direction vector of the line
//         double dx = x2 - x1;
//         double dy = y2 - y1;

//         // Calculate the vector from p1 to p0
//         double dp = x - x1;
//         double dq = y - y1;

//         if (direction != LineDirection.MOVE_BOTH) {
//             double m = dy / dx;

//             // Calculate the y-intercept
//             double y_intercept = y1 - m * x1;

//             // Calculate y value on the line for the given x-coordinate of the point
//             double y_on_line = m * x + y_intercept;
            
//             if (direction == LineDirection.MOVE_ABOVE && y >= y_on_line) {
//                 return new Translation2d(x, y);
//             } else if (direction == LineDirection.MOVE_BELOW && y <= y_on_line) {
//                 return new Translation2d(x, y);
//             }
//         }

//         // Calculate the dot product
//         double dotProduct = dx * dp + dy * dq;

//         // Calculate the squared length of the direction vector
//         double squaredLength = dx * dx + dy * dy;

//         // Calculate the parameter 't'
//         double t = dotProduct / squaredLength;

//         // Calculate the coordinates of the closest point
//         double closestX = x1 + t * dx;
//         double closestY = y1 + t * dy;

//         return new Translation2d(closestX, closestY);
//     }
    


//     /**
//      * Returns the nearest position on the field to the given position
//      * 
//      * @param position Robot position
//      * @return The nearest position on the field that is not inside anything
//      */
//     public static Pose2d getNearestPositionOnField(Pose2d pose) {
//         return new Pose2d(getNearestPositionOnField(pose.getTranslation()), pose.getRotation());
//     }

//     /**
//      * Returns the nearest position on the field to the given position
//      * 
//      * @param position Robot position
//      * @return The nearest position on the field that is not inside anything
//      */
//     public static Translation2d getNearestPositionOnField(Translation2d position) {
//         double x = position.getX();
//         double y = position.getY();

//         // Keep robot in X and Y bounds
//         x = MathUtil.clamp(x, FIELD_MIN_X, FIELD_MAX_X);
//         y = MathUtil.clamp(y, FIELD_MIN_Y, FIELD_MAX_Y);

//         // Get robot out of coral stations
//         if (x < CORAL_STATION_BLUE_X) {
//             if (y < CORAL_STATION_BLUE_Y1) {
//                 // We are in bottom left corner
//                 return getClosestPointToLine(x, y, FIELD_MIN_X, CORAL_STATION_BLUE_Y1, CORAL_STATION_BLUE_X, FIELD_MIN_Y, LineDirection.MOVE_ABOVE);
//             } else if (y > CORAL_STATION_BLUE_Y2) {
//                 // We are in top left corner
//                 return getClosestPointToLine(x, y, FIELD_MIN_X, CORAL_STATION_BLUE_Y2, CORAL_STATION_BLUE_X, FIELD_MAX_Y, LineDirection.MOVE_BELOW);
//             }


//         } else if (x > CORAL_STATION_RED_X) {
//             if (y < CORAL_STATION_RED_Y1) {
//                 // We are in bottom right corner
//                 return getClosestPointToLine(x, y, FIELD_MAX_X, CORAL_STATION_RED_Y1, CORAL_STATION_RED_X, FIELD_MIN_Y, LineDirection.MOVE_ABOVE);
//             } else if (y > CORAL_STATION_RED_Y2) {
//                 // We are in top right corner
//                 return getClosestPointToLine(x, y, FIELD_MAX_X, CORAL_STATION_RED_Y2, CORAL_STATION_RED_X, FIELD_MAX_Y, LineDirection.MOVE_BELOW);
//             }
//         }



//         // Get robot out of blue reef
//         else if ((x > REEF_BLUE_MIN_X) && (x < REEF_BLUE_MAX_X) && (y > REEF_BLUE_MIN_Y) && (y < REEF_BLUE_MAX_Y)) {
//             // In ref bounding box
//             if (y < REEF_BLUE_Y_1) {
//                 if (x < REEF_BLUE_X) {
//                     // We are in 
//                     return getClosestPointToLine(x, y, REEF_BLUE_MIN_X, REEF_BLUE_Y_1, REEF_BLUE_X, REEF_BLUE_MIN_Y, LineDirection.MOVE_BELOW);
//                 } else {
//                     return getClosestPointToLine(x, y, REEF_BLUE_X, REEF_BLUE_MIN_Y, REEF_BLUE_MAX_X, REEF_BLUE_Y_1, LineDirection.MOVE_BELOW);
//                 }
//             } else if (y < REEF_BLUE_Y_2) {
//                 if (x < REEF_BLUE_X) {
//                     return new Translation2d(REEF_BLUE_MIN_X, y);
//                 } else {
//                     return new Translation2d(REEF_BLUE_MAX_X, y);
//                 }
//             } else {
//                 if (x < REEF_BLUE_X) {
//                     return getClosestPointToLine(x, y, REEF_BLUE_MIN_X, REEF_BLUE_Y_2, REEF_BLUE_X, REEF_BLUE_MAX_Y, LineDirection.MOVE_ABOVE);
//                 } else {
//                     return getClosestPointToLine(x, y, REEF_BLUE_X, REEF_BLUE_MAX_Y, REEF_BLUE_MAX_X, REEF_BLUE_Y_2, LineDirection.MOVE_ABOVE);
//                 }
//             }
//         }

//         // Get robot out of red reef
//         else if ((x > REEF_RED_MIN_X) && (x < REEF_RED_MAX_X) && (y > REEF_RED_MIN_Y) && (y < REEF_RED_MAX_Y)) {
//             // In ref bounding box
//             if (y < REEF_RED_Y_1) {
//                 if (x < REEF_RED_X) {
//                     // We are in 
//                     return getClosestPointToLine(x, y, REEF_RED_MIN_X, REEF_RED_Y_1, REEF_RED_X, REEF_RED_MIN_Y, LineDirection.MOVE_BELOW);
//                 } else {
//                     return getClosestPointToLine(x, y, REEF_RED_X, REEF_RED_MIN_Y, REEF_RED_MAX_X, REEF_RED_Y_1, LineDirection.MOVE_BELOW);
//                 }
//             } else if (y < REEF_RED_Y_2) {
//                 if (x < REEF_RED_X) {
//                     return new Translation2d(REEF_RED_MIN_X, y);
//                 } else {
//                     return new Translation2d(REEF_RED_MAX_X, y);
//                 }
//             } else {
//                 if (x < REEF_RED_X) {
//                     return getClosestPointToLine(x, y, REEF_RED_MIN_X, REEF_RED_Y_2, REEF_RED_X, REEF_RED_MAX_Y, LineDirection.MOVE_ABOVE);
//                 } else {
//                     return getClosestPointToLine(x, y, REEF_RED_X, REEF_RED_MAX_Y, REEF_RED_MAX_X, REEF_RED_Y_2, LineDirection.MOVE_ABOVE);
//                 }
//             }        
//         }

//         return new Translation2d(x, y);
//     }
    
// }
