using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Drawing;
using Emgu.CV;
using Emgu.CV.Util;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using RASDK.Vision.ExtensionMethods;

namespace RASDK.Vision.Positioning
{
    /// <summary>
    /// 進階的單應性（Homography）視覺定位。
    /// </summary>
    public class AdvancedHomographyPositioner : IVisionPositioning
    {
        private readonly PointF[] _imagePoints = new PointF[4];

        private readonly PointF[] _worldPoints = new PointF[4];

        private readonly HomographyPositioner _homographyPositioner;

        private readonly CameraParameter _cameraParameter;

        /// <summary>
        /// 進階的單應性（Homography）視覺定位。
        /// </summary>
        /// <remarks>worldPoints order: Top-Left, Top-Right, Bottom-Left, Bottom-Right.</remarks>
        public AdvancedHomographyPositioner(PointF[] worldPoints,
                                            CameraParameter cameraParameter,
                                            Image<Bgr, byte> checkboardImage,
                                            Size checkboardSize)
        {
            _cameraParameter = cameraParameter ?? throw new ArgumentNullException($"{nameof(cameraParameter)}", "Should not be null.");
            _worldPoints = worldPoints.Length == 4 ? worldPoints : throw new ArgumentException($"{nameof(worldPoints)}", "長度需爲 4.");

            _imagePoints = UpdateImagePoints(cameraParameter, checkboardImage, checkboardSize);
            _homographyPositioner = new HomographyPositioner(_imagePoints, _worldPoints);
            WorldOffset = new PointF(0, 0);
        }

        /// <summary>
        /// 進階的單應性（Homography）視覺定位。
        /// </summary>
        /// <remarks>worldPoints order: Top-Left, Top-Right, Bottom-Left, Bottom-Right.</remarks>
        public AdvancedHomographyPositioner(PointF[] worldPoints,
                                            CameraCalibration cameraCalibration,
                                            List<Image<Bgr, byte>> checkboardImages = null)
        {
            _worldPoints = worldPoints.Length == 4 ? worldPoints : throw new ArgumentException($"{nameof(worldPoints)}", "長度需爲 4.");

            if (cameraCalibration.CameraParameter == null)
            {
                if (checkboardImages == null)
                {
                    throw new Exception("未執行過 Camera Calibration。");
                }
                else
                {
                    cameraCalibration.CalCameraParameter(checkboardImages, 0);
                }
            }

            _cameraParameter = cameraCalibration.CameraParameter;

            _imagePoints = UpdateImagePoints(_cameraParameter,
                                             cameraCalibration.SourceImageRepresentative,
                                             cameraCalibration.CheckboardSize);

            _homographyPositioner = new HomographyPositioner(_imagePoints, _worldPoints);
            WorldOffset = new PointF(0, 0);
        }

        public CameraParameter CameraParameter => _cameraParameter;

        public PointF[] ImagePoints => _imagePoints;

        public PointF[] WorldPoints => _worldPoints;

        public HomographyPositioner HomographyPositioner => _homographyPositioner;

        public PointF WorldOffset { get; set; }

        public void ImageToWorld(double pixelX, double pixelY, out double worldX, out double worldY)
        {
            _homographyPositioner.ImageToWorld(pixelX, pixelY, out worldX, out worldY);
        }

        public PointF ImageToWorld(PointF pixel)
        {
            return _homographyPositioner.ImageToWorld(pixel);
        }

        // XXX
        private PointF[] InferWorldPoints(PointF topLeftWorldPoint, PointF[] imagePoints, Size checkboardSize, float checkBoardSquareSideLength)
        {
            var topLeftImgPoint = imagePoints[0];
            var topRightImgPoint = imagePoints[1];

            var angleRad = Math.Atan2(topRightImgPoint.Y - topLeftImgPoint.Y, topRightImgPoint.X - topLeftImgPoint.X);
            var angleDeg = angleRad * 180.0 / Math.PI;

            var rotationMatrix = new Matrix<double>(2, 3);
            CvInvoke.GetRotationMatrix2D(topLeftWorldPoint, angleDeg, 1, rotationMatrix);

            // XXX.
            var z = 1;

            var topRightMatrix = new Matrix<double>(3, 1);
            topRightMatrix[0, 0] = topLeftWorldPoint.X + ((checkboardSize.Width - 2) * checkBoardSquareSideLength);
            topRightMatrix[1, 0] = topLeftWorldPoint.Y;
            topRightMatrix[2, 0] = z;

            var bottomLeftMatrix = new Matrix<double>(3, 1);
            bottomLeftMatrix[0, 0] = topLeftWorldPoint.X;
            bottomLeftMatrix[1, 0] = topLeftWorldPoint.Y - ((checkboardSize.Height - 2) * checkBoardSquareSideLength);
            bottomLeftMatrix[2, 0] = z;

            var bottomRightMatrix = new Matrix<double>(3, 1);
            bottomRightMatrix[0, 0] = topLeftWorldPoint.X + ((checkboardSize.Width - 2) * checkBoardSquareSideLength);
            bottomRightMatrix[1, 0] = topLeftWorldPoint.Y - ((checkboardSize.Height - 2) * checkBoardSquareSideLength);
            bottomRightMatrix[2, 0] = z;

            var trm = rotationMatrix * topRightMatrix;
            var blm = rotationMatrix * bottomLeftMatrix;
            var brm = rotationMatrix * bottomRightMatrix;

            return new PointF[]
            {
                topLeftWorldPoint,
                new PointF((float)trm[0,0], (float)trm[1,0]),
                new PointF((float)blm[0,0], (float)blm[1,0]),
                new PointF((float)brm[0,0], (float)brm[1,0]),
            };
        }

        private PointF[] UpdateImagePoints(CameraParameter cameraParameter, Image<Bgr, byte> image, Size checkboardSize)
        {
            var undistortImage = image.UndistortImage(cameraParameter);
            var patternSize = new Size(checkboardSize.Width - 1, checkboardSize.Height - 1);
            var corners = CameraCalibration.FindCorners(undistortImage.Convert<Gray, byte>(),
                                                        patternSize);
            var cornersArray = corners.ToArray();

            var imagePoints = new PointF[]
            {
            // Top Left.
            cornersArray[0],

            // Top Right.
            cornersArray[patternSize.Width - 1],

            // Bottom Left.
            cornersArray[(patternSize.Height - 1) * patternSize.Width],

            // Bottom Right.
            cornersArray[((patternSize.Height - 1) * patternSize.Width) + patternSize.Width - 1]
            };

            return imagePoints;
        }
    }
}