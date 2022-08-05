using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Emgu.CV;
using Emgu.CV.Util;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;

namespace RASDK.Vision
{
    public class CameraParameter
    {
        public readonly VectorOfDouble RotationVector;

        public readonly VectorOfDouble TranslationVector;

        private readonly Matrix<double> _distortionCoefficients = new Matrix<double>(8, 1);

        private readonly Matrix<double> _intrinsicMatrix = new Matrix<double>(3, 3);

        public CameraParameter(double cX,
                               double cY,
                               double fX,
                               double fY,
                               double skew,
                               Matrix<double> distrotionCoeffs,
                               VectorOfDouble rotationVector,
                               VectorOfDouble translationVector)
        {
            _intrinsicMatrix = new Matrix<double>(3, 3);
            _intrinsicMatrix.Data[0, 0] = fX;
            _intrinsicMatrix.Data[0, 1] = skew;
            _intrinsicMatrix.Data[0, 2] = cX;

            _intrinsicMatrix.Data[1, 0] = 0;
            _intrinsicMatrix.Data[1, 1] = fY;
            _intrinsicMatrix.Data[1, 2] = cY;

            _intrinsicMatrix.Data[2, 0] = 0;
            _intrinsicMatrix.Data[2, 1] = 0;
            _intrinsicMatrix.Data[2, 2] = 1;

            _distortionCoefficients = distrotionCoeffs;
            RotationVector = rotationVector;
            TranslationVector = translationVector;
        }

        public CameraParameter(Matrix<double> intrinsicMatrix,
                               Matrix<double> distrotionCoeffs,
                               VectorOfDouble rotationVector,
                               VectorOfDouble translationVector)
        {
            _intrinsicMatrix = intrinsicMatrix;
            _distortionCoefficients = distrotionCoeffs;
            RotationVector = rotationVector;
            TranslationVector = translationVector;
        }

        /// <summary>
        /// X of principle point.
        /// </summary>
        public double Cx => _intrinsicMatrix.Data[0, 2];

        /// <summary>
        /// Y of principle point.
        /// </summary>
        public double Cy => _intrinsicMatrix.Data[1, 2];

        /// <summary>
        /// X of focal length.
        /// </summary>
        public double Fx => _intrinsicMatrix.Data[0, 0];

        /// <summary>
        /// Y of focal length.
        /// </summary>
        public double Fy => _intrinsicMatrix.Data[1, 1];

        /// <summary>
        /// Skew parameter.
        /// </summary>
        public double Skew => _intrinsicMatrix.Data[0, 1];

        /// <summary>
        /// The distortion parameter vector.
        /// </summary>
        public Matrix<double> DistortionCoefficients => _distortionCoefficients;

        /// <summary>
        /// The upper triangular 3*3 matrix of camera intrinsic parameter, a.k.a. Camera matrix.
        /// </summary>
        public Matrix<double> IntrinsicMatrix => _intrinsicMatrix;

        public Matrix<double> RotationMatrix
        {
            get
            {
                var matrix = new Matrix<double>(3, 3);
                CvInvoke.Rodrigues(RotationVector, matrix);
                return matrix;
            }

            set
            {
                CvInvoke.Rodrigues(value, RotationVector);
            }
        }

        public static CameraParameter LoadFromCsv(string filename = "camera_parameter.csv")
        {
            var csvData = Basic.Csv.Read(filename);

            var cx = double.Parse(csvData[0][1]);
            var cy = double.Parse(csvData[1][1]);
            var fx = double.Parse(csvData[2][1]);
            var fy = double.Parse(csvData[3][1]);
            var skew = double.Parse(csvData[4][1]);

            var dc = new Matrix<double>(8, 1);
            for (int i = 0; i < csvData[5].Count; i++)
            {
                dc.Data[i, 0] = double.Parse(csvData[5][i]);
            }

            var rv = new double[csvData[6].Count];
            for (int i = 0; i < csvData[6].Count; i++)
            {
                rv[i] = double.Parse(csvData[6][i]);
            }

            var tv = new double[csvData[7].Count];
            for (int i = 0; i < csvData[7].Count; i++)
            {
                tv[i] = double.Parse(csvData[7][i]);
            }

            return new CameraParameter(cx, cy, fx, fy, skew, dc, new VectorOfDouble(rv), new VectorOfDouble(tv));
        }

        public void SaveToCsv(string filename = "camera_parameter.csv")
        {
            var dc = new List<string>() { "DistCoeffs" };
            for (int r = 0; r < _distortionCoefficients.Rows; r++)
            {
                dc.Add(_distortionCoefficients[r, 0].ToString() ?? "");
            }

            var rv = new List<string>() { "RotationVector" };
            foreach (var v in RotationVector.ToArray())
            {
                rv.Add(v.ToString() ?? "");
            }

            var tv = new List<string>() { "TranslationVector" };
            foreach (var v in TranslationVector.ToArray())
            {
                tv.Add(v.ToString() ?? "");
            }

            var csvData = new List<List<string>>()
            {
                new List<string>(){"cx",Cx.ToString()},
                new List<string>(){"cy",Cy.ToString()},
                new List<string>(){"fx",Fx.ToString()},
                new List<string>(){"fy",Fy.ToString()},
                new List<string>(){"skew",Skew.ToString()},
                dc,
                rv,
                tv
            };

            Basic.Csv.Write(filename, csvData);
        }
    }
}