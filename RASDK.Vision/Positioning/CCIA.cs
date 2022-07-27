using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Emgu.CV;
using Emgu.CV.Structure;
using Emgu.CV.Util;

namespace RASDK.Vision.Positioning
{
    /// <summary>
    /// Vision positioning by Camera Calibration with Iterative Approximation.<br/>
    /// 疊代逼近相機標定視覺定位法。
    /// </summary>
    public class CCIA : IVisionPositioning
    {
        /// <summary>
        /// 世界座標偏移。
        /// </summary>
        public PointF WorldOffset = new PointF(0, 0);

        /// <summary>
        /// 反轉X軸。
        /// </summary>
        public bool InvertedX = false;

        /// <summary>
        /// 反轉Y軸。
        /// </summary>
        /// <remarks>
        /// 圖片和手臂座標系的Y軸通常是相反的，因此預設爲true。
        /// </remarks>
        public bool InvertedY = true;

        private readonly CameraParameter _cameraParameter;

        private readonly Approximation _approximation;

        private readonly TransferFunctionOfVirtualCheckBoardToWorld _transferFunctionOfVirtualCheckBoardToWorld;

        private double _allowablePixelError;

        /// <summary>
        /// Vision positioning by Camera Calibration with Iterative Approximation.<br/>
        /// 疊代逼近相機標定視覺定位法。
        /// </summary>
        /// <remarks>
        /// 此方法的運作方式爲先給定一個預測虛擬定位板座標作，將其透過相機標定法來算出對應的預測像素座標。<br/>
        /// 如果預測像素座標與實際像素座標的差距大於容許誤差，就調整預測虛擬定位板座標，再重複上述步驟。<br/>
        /// 如果預測像素座標與實際像素座標的差距小於等於容許誤差，就視目前的預測虛擬定位板座標爲正確的，再將其透過一組變換來轉換成世界座標。<br/>
        /// 虛擬定位板座標是一個以相機成像平面投影到實物平面的假想平面座標系。其原點在鏡頭中心，也就是主點的投影位置。
        /// </remarks>
        public CCIA(CameraParameter cameraParameter,
                    double allowablePixelError = 5,
                    TransferFunctionOfVirtualCheckBoardToWorld tf = null,
                    Approximation approximation = null)
        {
            _cameraParameter = cameraParameter;
            _allowablePixelError = allowablePixelError;
            _transferFunctionOfVirtualCheckBoardToWorld = tf ?? BasicTransferFunctionOfVirtualCheckBoardToWorld;
            _approximation = approximation ?? BasicApproximation;
        }

        /// <summary>
        /// 誤差逼近算法。
        /// </summary>
        public delegate void Approximation(double errorX,
                                           double errorY,
                                           ref double valueX,
                                           ref double valueY);

        /// <summary>
        /// 座標轉換算法。
        /// </summary>
        public delegate void TransferFunctionOfVirtualCheckBoardToWorld(double vX,
                                                                        double vY,
                                                                        out double armX,
                                                                        out double armY);

        public double AllowableError
        {
            get => _allowablePixelError;
            set => _allowablePixelError = value;
        }

        public PointF ImageToWorld(Point pixel)
        {
            ImageToWorld(pixel.X, pixel.Y, out var worldX, out var worldY);
            return new PointF((float)worldX, (float)worldY);
        }

        public void ImageToWorld(int pixelX, int pixelY, out double worldX, out double worldY)
        {
            // 給定一個預測虛擬定位板座標。
            double virtualCheckBoardX = 0;
            double virtualCheckBoardY = 0;

            while (true)
            {
                // 將預測虛擬定位板座標透過相機標定法來算出對應的預測像素座標。
                var forecastPixel = CvInvoke.ProjectPoints(
                    new[] { new MCvPoint3D32f((float)virtualCheckBoardX, (float)virtualCheckBoardY, 0) },
                    _cameraParameter.RotationVectors,
                    _cameraParameter.TranslationVectors,
                    _cameraParameter.IntrinsicMatrix,
                    _cameraParameter.DistortionCoefficients);

                // 計算預測像素座標與實際像素座標的差距。
                double errorX = pixelX - forecastPixel[0].X;
                double errorY = pixelY - forecastPixel[0].Y;

                // 判定差距是否大於容許誤差。
                if (Math.Abs(errorX) > _allowablePixelError || Math.Abs(errorY) > _allowablePixelError)
                {
                    // 差距大於容許誤差，調整預測虛擬定位板座標。
                    _approximation(errorX, errorY, ref virtualCheckBoardX, ref virtualCheckBoardY);
                }
                else
                {
                    // 差距小於等於容許誤差，跳出無限迴圈。
                    break;
                }
            }

            // 將虛擬定位板座標轉換成世界座標。
            _transferFunctionOfVirtualCheckBoardToWorld(virtualCheckBoardX,
                                                        virtualCheckBoardY,
                                                        out worldX,
                                                        out worldY);

            // 座標偏移。
            if (WorldOffset == null)
            {
                WorldOffset = new PointF(0, 0);
            }
            worldX += WorldOffset.X;
            worldY += WorldOffset.Y;

            if (InvertedX)
            {
                worldX = -worldX;
            }

            if (InvertedY)
            {
                worldY = -worldY;
            }
        }

        private void BasicTransferFunctionOfVirtualCheckBoardToWorld(double vX,
                                                                     double vY,
                                                                     out double worldX,
                                                                     out double worldY)
        {
            worldX = vX;
            worldY = vY;
        }

        private void BasicApproximation(double errorX, double errorY, ref double vX, ref double vY)
        {
            if (errorX > 0)
                vX++;
            else if (errorX < 0)
                vX--;

            if (errorY > 0)
                vY++;
            else if (errorY < 0)
                vY--;
        }
    }
}