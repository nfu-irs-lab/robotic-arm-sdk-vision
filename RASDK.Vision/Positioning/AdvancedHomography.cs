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

namespace RASDK.Vision.Positioning
{
    /// <summary>
    /// 進階的單應性（Homography）視覺定位。
    /// </summary>
    public class AdvancedHomography : IVisionPositioning
    {
        private readonly PointF[] _imagePoints = new PointF[4];

        private readonly PointF[] _worldPoints = new PointF[4];

        private readonly Homography _homography;

        private readonly CameraCalibration _cameraCalibration;

        /// <summary>
        /// 進階的單應性（Homography）視覺定位。
        /// </summary>
        public AdvancedHomography(CameraCalibration cameraCalibration,
                                  List<Image<Bgr, byte>> checkboardImages,
                                  PointF[] worldPoints)
        {
            _cameraCalibration = cameraCalibration ?? throw new ArgumentNullException();
            _worldPoints = worldPoints.Length == 4 ? worldPoints : throw new ArgumentException();

            var error = _cameraCalibration.Run(checkboardImages, out _, out _, out _, out _);
            var undistortImage = _cameraCalibration.UndistortImage(checkboardImages[0]);
            var patternSize = _cameraCalibration.PatternSize;
            var corners = _cameraCalibration.FindCorners(undistortImage.Convert<Gray, byte>(),
                                                         patternSize);
            var cornersArray = corners.ToArray();

            // Top Left.
            _imagePoints[0] = cornersArray[0];

            // Top Right.
            _imagePoints[1] = cornersArray[patternSize.Width - 1];

            // Botton Left.
            _imagePoints[2] = cornersArray[patternSize.Height - 1 * patternSize.Width];

            // Botton Right.
            _imagePoints[3] = cornersArray[patternSize.Height - 1 * patternSize.Width + patternSize.Width - 1];

            _homography = new Homography(_imagePoints, _worldPoints);

            WorldOffset = new PointF(0, 0);
        }

        public CameraCalibration CameraCalibration => _cameraCalibration;

        public PointF[] ImagePoints => _imagePoints;

        public PointF[] WorldPoints => _worldPoints;

        public Homography Homography => _homography;

        public PointF WorldOffset { get; set; }

        public Image<Bgr, byte> Undistort(Image<Bgr, byte> srcImage)
        {
            return _cameraCalibration.UndistortImage(srcImage);
        }

        public void ImageToWorld(double pixelX, double pixelY, out double worldX, out double worldY)
        {
            _homography.ImageToWorld(pixelX, pixelY, out worldX, out worldY);
        }

        public PointF ImageToWorld(PointF pixel)
        {
            return _homography.ImageToWorld(pixel);
        }
    }
}