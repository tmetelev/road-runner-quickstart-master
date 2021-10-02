package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.MatrixUtils;

import java.util.List;

public abstract class TwoTrackingWheelLocalizer {
    private Pose2d poseEstimate;
    private List<Double> lastWheelPositions;
    private Double lastHeading;

    private DecompositionSolver forwardSolver;

    public TwoTrackingWheelLocalizer(List<Pose2d> wheelPoses){
        Array2DRowRealMatrix inverseMatrix = new Array2DRowRealMatrix(3, 3);

        for(int i = 0; i < 2; i++)
        {
            Vector2d orientationVector = wheelPoses.get(i).headingVec();
            Vector2d positionVector = wheelPoses.get(i).vec();
            inverseMatrix.setEntry(i, 0, orientationVector.getX());
            inverseMatrix.setEntry(i, 1, orientationVector.getY());
            inverseMatrix.setEntry(
                    i,
                    2,
                    positionVector.getX() * orientationVector.getY() - positionVector.getY() * orientationVector.getX()
            );
            inverseMatrix.setEntry(2,2,1.0);

            forwardSolver = new LUDecomposition(inverseMatrix).getSolver();
        }
    }

//    private Pose2d calculatePoseDelta(List<Double> wheelDeltas, Double headingDelta) {
//        val rawPoseDelta = forwardSolver.solve(
//                MatrixUtils.createRealMatrix(
//                        ((wheelDeltas + headingDelta).toDoubleArray())
//                ).transpose()
//        );
//        return Pose2d(
//                rawPoseDelta.getEntry(0, 0),
//                rawPoseDelta.getEntry(1, 0),
//                rawPoseDelta.getEntry(2, 0)
//        );
//    }
}
