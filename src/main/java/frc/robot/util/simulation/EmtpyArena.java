package frc.robot.util.simulation;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.Arena2025Reefscape;

import edu.wpi.first.math.geometry.Translation2d;

public class EmtpyArena extends SimulatedArena {

    public static final double FIELD_X = -2.0; // 0
    public static final double FIELD_Y = -2.0; // 0
    public static final double FIELD_LENGTH = 20; // 17.548;
    public static final double FIELD_WIDTH = 10; // 8.052;
    
    public static final class EmptyFieldMap extends FieldMap {
        public EmptyFieldMap() {
            super();

            super.addBorderLine(new Translation2d(FIELD_X, FIELD_Y), new Translation2d(FIELD_X, FIELD_WIDTH));
            super.addBorderLine(new Translation2d(FIELD_LENGTH, FIELD_Y), new Translation2d(FIELD_LENGTH, FIELD_WIDTH));

            super.addBorderLine(new Translation2d(FIELD_X, FIELD_Y), new Translation2d(FIELD_LENGTH, FIELD_Y));
            super.addBorderLine(new Translation2d(FIELD_X, FIELD_WIDTH), new Translation2d(FIELD_LENGTH, FIELD_WIDTH));
        }
    }

    public EmtpyArena() {
        super(new EmptyFieldMap());
    }

    @Override
    public void placeGamePiecesOnField() {
    }

    @Override
    public void competitionPeriodic() {
    }
    
}
