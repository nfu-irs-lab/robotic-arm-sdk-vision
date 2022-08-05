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
    public class HomographyPositioner : IVisionPositioning
    {
        private Matrix<double> _homographyMatrix = null;

        /// <summary>
        /// 單應性（Homography）視覺定位。
        /// </summary>
        public HomographyPositioner(PointF[] imagePoints, PointF[] worldPoints)
        {
            UpdateHomographyMatrix(imagePoints, worldPoints);
        }

        /// <summary>
        /// 單應性（Homography）視覺定位。
        /// </summary>
        public HomographyPositioner(VectorOfPointF imagePoints, VectorOfPointF worldPoints)
        {
            UpdateHomographyMatrix(imagePoints.ToArray(), worldPoints.ToArray());
        }

        /// <summary>
        /// 單應性（Homography）視覺定位。
        /// </summary>
        public HomographyPositioner(Mat homographyMatrix)
        {
            if (_homographyMatrix == null)
            {
                _homographyMatrix = new Matrix<double>(3, 3);
            }

            if (homographyMatrix.Rows != 3 || homographyMatrix.Cols != 3)
            {
                throw new ArgumentException($"homographyMatrix 必須爲 3*3 矩陣，" +
                                            $"實際爲{homographyMatrix.Rows}*{homographyMatrix.Cols}。");
            }

            homographyMatrix.ConvertTo(_homographyMatrix, DepthType.Cv64F);
        }

        /// <summary>
        /// 單應性（Homography）視覺定位。
        /// </summary>
        public HomographyPositioner(Matrix<double> homographyMatrix)
        {
            if (homographyMatrix.Rows != 3 || homographyMatrix.Cols != 3)
            {
                throw new ArgumentException($"homographyMatrix 必須爲 3*3 矩陣，" +
                                            $"實際爲{homographyMatrix.Rows}*{homographyMatrix.Cols}。");
            }

            _homographyMatrix = homographyMatrix.Clone();
        }

        /// <summary>
        /// 單應性矩陣。
        /// </summary>
        public Matrix<double> HomographyMatrix => _homographyMatrix;

        public PointF WorldOffset { get; set; }

        public void ImageToWorld(double pixelX, double pixelY, out double worldX, out double worldY)
        {
            var world = ImageToWorld(new PointF((float)pixelX, (float)pixelY));

            worldX = world.X;
            worldY = world.Y;
        }

        public PointF ImageToWorld(PointF pixel)
        {
            if (_homographyMatrix == null)
            {
                throw new Exception("Homography 矩陣不存在（未找到）。");
            }

            var srcPoints = new PointF[] { pixel };
            var worldPoints = CvInvoke.PerspectiveTransform(srcPoints, _homographyMatrix);

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
            if (imagePoints == null)
            {
                throw new ArgumentNullException($"{nameof(imagePoints)}", "Should not be null.");
            }

            if (worldPoints == null)
            {
                throw new ArgumentNullException($"{nameof(worldPoints)}", "Should not be null.");
            }

            if (imagePoints.Length < 4 || worldPoints.Length < 4)
            {
                throw new ArgumentException($"imagePoints 和 worldPoints 的長度必須大於等於 4，" +
                                            $"實際爲 imagePoints:{imagePoints.Length}, worldPoints:{worldPoints.Length}.");
            }

            if (worldPoints.Length != imagePoints.Length)
            {
                throw new ArgumentException($"imagePoints 和 worldPoints 的長度不相等，" +
                                            $"實際爲 imagePoints:{imagePoints.Length}, worldPoints:{worldPoints.Length}.");
            }

            var h = CvInvoke.FindHomography(imagePoints, worldPoints);

            if (h == null)
            {
                throw new Exception("Con't find homography matrix.");
            }

            if (_homographyMatrix == null)
            {
                _homographyMatrix = new Matrix<double>(3, 3);
            }
            h.ConvertTo(_homographyMatrix, DepthType.Cv64F);
        }
    }
}