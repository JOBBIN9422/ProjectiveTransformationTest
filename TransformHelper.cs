using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;
using MathNet.Numerics.LinearAlgebra;
using SixLabors.Primitives;

namespace TransformTest
{
    public class TransformHelper
    {
        public static Matrix4x4 ComputeTransformMatrix(int width, int height, Point newTopLeft, Point newTopRight, Point newBottomLeft, Point newBottomRight)
        {
            //FIX - generalize the calculation of A & B matrices into function - code too W E T rn
            //compute A matrix
            Matrix<double> solveA = Matrix<double>.Build.DenseOfArray(new double[,] {
                { 0, width, width },
                { 0, 0, height },
                { 1, 1, 1 }
            });

            MathNet.Numerics.LinearAlgebra.Vector<double> augmentA = MathNet.Numerics.LinearAlgebra.Vector<double>.Build.Dense(new double[]
                { 0, height, 1}
            );

            MathNet.Numerics.LinearAlgebra.Vector<double> coefficientsA = solveA.Solve(augmentA);

            Matrix<double> A = Matrix<double>.Build.DenseOfArray(new double[,] {
                { 0, coefficientsA[1] * width, coefficientsA[2] * width },
                { 0, 0, coefficientsA[2] * height },
                { coefficientsA[0], coefficientsA[1], coefficientsA[2] }
            });

            //compute B matrix
            Matrix<double> solveB = Matrix<double>.Build.DenseOfArray(new double[,] {
                { newTopLeft.X, newTopRight.X, newBottomRight.X },
                { newTopLeft.Y, newTopRight.Y, newBottomRight.Y },
                { 1, 1, 1 }
            });

            MathNet.Numerics.LinearAlgebra.Vector<double> augmentB = MathNet.Numerics.LinearAlgebra.Vector<double>.Build.Dense(new double[]
                { newBottomLeft.X, newBottomLeft.Y, 1}
            );

            MathNet.Numerics.LinearAlgebra.Vector<double> coefficientsB = solveB.Solve(augmentB);

            Matrix<double> B = Matrix<double>.Build.DenseOfArray(new double[,] {
                { coefficientsB[0] * newTopLeft.X, coefficientsB[1] * newTopRight.X, coefficientsB[2] * newBottomRight.X },
                { coefficientsB[0] * newTopLeft.Y, coefficientsB[1] * newTopRight.Y, coefficientsB[2] * newBottomRight.Y },
                { coefficientsB[0], coefficientsB[1], coefficientsB[2] }
            });

            //compute matrix C = B * A^-1 
            Matrix<double> AInv = A.Inverse();
            Matrix<double> C = B * AInv;

            return new Matrix4x4((float)C[0, 0], (float)C[1, 0], 0, (float)C[2, 0],
                (float)C[0, 1], (float)C[1, 1], 0, (float)C[2, 1],
                0, 0, 1, 0,
                (float)C[0, 2], (float)C[1, 2], 0, (float)C[2, 2]);
        }

        public static Matrix4x4 CalculateProjectiveTransformationMatrix(int width, int height, Point newTopLeft, Point newTopRight, Point newBottomLeft, Point newBottomRight)
        {
            var s = MapBasisToPoints(
                new Point(0, 0),
                new Point(width, 0),
                new Point(0, height),
                new Point(width, height)
            );
            var d = MapBasisToPoints(newTopLeft, newTopRight, newBottomLeft, newBottomRight);
            var result = d.Multiply(AdjugateMatrix(s));
            var normalized = result.Divide(result[2, 2]);
            return new Matrix4x4(
                (float)normalized[0, 0], (float)normalized[1, 0], 0, (float)normalized[2, 0],
                (float)normalized[0, 1], (float)normalized[1, 1], 0, (float)normalized[2, 1],
                0, 0, 1, 0,
                (float)normalized[0, 2], (float)normalized[1, 2], 0, (float)normalized[2, 2]
            );
        }
        private static Matrix<double> AdjugateMatrix(Matrix<double> matrix)
        {
            if (matrix.RowCount != 3 || matrix.ColumnCount != 3)
            {
                throw new ArgumentException("Must provide a 3x3 matrix.");
            }

            var adj = matrix.Clone();
            adj[0, 0] = matrix[1, 1] * matrix[2, 2] - matrix[1, 2] * matrix[2, 1];
            adj[0, 1] = matrix[0, 2] * matrix[2, 1] - matrix[0, 1] * matrix[2, 2];
            adj[0, 2] = matrix[0, 1] * matrix[1, 2] - matrix[0, 2] * matrix[1, 1];
            adj[1, 0] = matrix[1, 2] * matrix[2, 0] - matrix[1, 0] * matrix[2, 2];
            adj[1, 1] = matrix[0, 0] * matrix[2, 2] - matrix[0, 2] * matrix[2, 0];
            adj[1, 2] = matrix[0, 2] * matrix[1, 0] - matrix[0, 0] * matrix[1, 2];
            adj[2, 0] = matrix[1, 0] * matrix[2, 1] - matrix[1, 1] * matrix[2, 0];
            adj[2, 1] = matrix[0, 1] * matrix[2, 0] - matrix[0, 0] * matrix[2, 1];
            adj[2, 2] = matrix[0, 0] * matrix[1, 1] - matrix[0, 1] * matrix[1, 0];

            return adj;
        }

        private static Matrix<double> MapBasisToPoints(Point p1, Point p2, Point p3, Point p4)
        {
            var A = Matrix<double>.Build.DenseOfArray(new double[,]
            {
                {p1.X, p2.X, p3.X},
                {p1.Y, p2.Y, p3.Y},
                {1, 1, 1}
            });
            var b = MathNet.Numerics.LinearAlgebra.Vector<double>.Build.Dense(new double[] { p4.X, p4.Y, 1 });
            var aj = AdjugateMatrix(A);
            var v = aj.Multiply(b);
            var m = Matrix<double>.Build.DenseOfArray(new[,]
            {
                {v[0], 0, 0 },
                {0, v[1], 0 },
                {0, 0, v[2] }
            });
            return A.Multiply(m);
        }
    }
}
