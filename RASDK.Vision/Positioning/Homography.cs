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
    /// 單應性（Homography）視覺定位。
    /// </summary>
    public class Homography : IVisionPositioning
    {
        private readonly Matrix<double> _homography = new Matrix<double>(3, 3);

        /// <summary>
        /// 單應性（Homography）視覺定位。
        /// </summary>
        public Homography(PointF[] imagePoints, PointF[] worldPoints)
        {
            UpdateHomographyMatrix(imagePoints, worldPoints);
        }

        /// <summary>
        /// 單應性（Homography）視覺定位。
        /// </summary>
        public Homography(VectorOfPointF imagePoints, VectorOfPointF worldPoints)
        {
            UpdateHomographyMatrix(imagePoints.ToArray(), worldPoints.ToArray());
        }

        /// <summary>
        /// 單應性（Homography）視覺定位。
        /// </summary>
        public Homography(Mat homographyMatrix)
        {
            homographyMatrix.ConvertTo(_homography, DepthType.Cv64F);
        }

        /// <summary>
        /// 單應性（Homography）視覺定位。
        /// </summary>
        public Homography(Matrix<double> homographyMatrix)
        {
            _homography = homographyMatrix;
        }

        /// <summary>
        /// 單應性矩陣。
        /// </summary>
        public Matrix<double> HomographyMatrix => _homography;

        public PointF WorldOffset { get; set; }

        public void ImageToWorld(double pixelX, double pixelY, out double worldX, out double worldY)
        {
            var world = ImageToWorld(new PointF((float)pixelX, (float)pixelY));

            worldX = world.X;
            worldY = world.Y;
        }

        public PointF ImageToWorld(PointF pixel)
        {
            var srcPoints = new PointF[] { pixel };
            var worldPoints = CvInvoke.PerspectiveTransform(srcPoints, _homography);

            if (WorldOffset == null)
            {
                WorldOffset = new PointF(0, 0);
            }
            worldPoints[0].X += WorldOffset.X;
            worldPoints[0].Y += WorldOffset.Y;

            return worldPoints[0];
        }

        private void UpdateHomographyMatrix(PointF[] imagePoints, PointF[] worldPoints)
        {
            var h = CvInvoke.FindHomography(imagePoints, worldPoints);

            if (h == null)
            {
                throw new Exception("Con't find homography matrix.");
            }

            h.ConvertTo(_homography, DepthType.Cv64F);
        }
    }
}