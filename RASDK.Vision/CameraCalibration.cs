using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Drawing;
using System.Threading.Tasks;
using System.Windows.Forms;
using Emgu.CV;
using Emgu.CV.Util;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;

namespace RASDK.Vision
{
    /// <summary>
    /// 相機標定。
    /// </summary>
    public class CameraCalibration
    {
        /// <summary>
        /// 定位板內角點數量尺寸。
        /// </summary>
        private readonly Size _patternSize;

        /// <summary>
        /// 定位板內角點數量。
        /// </summary>
        private readonly int _cornersCount;

        /// <summary>
        /// 定位板方格邊長，以mm爲單位。
        /// </summary>
        private readonly float _checkBoardSquareSideLength;

        /// <summary>
        /// 影像數量。
        /// </summary>
        private int _imageCount;

        /// <summary>
        /// 所有影像的角點。
        /// </summary>
        private List<VectorOfPointF> _allCorners = new List<VectorOfPointF>();

        /// <summary>
        /// 原始影像代表。
        /// </summary>
        private Image<Bgr, byte> _sourceImageRepresentative;

        /// <summary>
        /// 相機內參數矩陣。
        /// </summary>
        private Matrix<double> _cameraMatrix = new Matrix<double>(3, 3);

        /// <summary>
        /// 相機畸變參數。
        /// </summary>
        private VectorOfDouble _distortionCoeffs = new VectorOfDouble();

        /// <summary>
        /// 所有影像的旋轉向量。
        /// </summary>
        private VectorOfDouble[] _rotationVectors;

        /// <summary>
        /// 所有影像的平移向量。
        /// </summary>
        private VectorOfDouble[] _translationVectors;

        private CameraParameter _cameraParameter;

        /// <summary>
        /// 相機標定。
        /// </summary>
        /// <param name="checkBoardSize">定位板的黑白方格尺寸，如12*9。</param>
        /// <param name="checkBoardSquareSideLength">方格邊長。</param>
        public CameraCalibration(Size checkBoardSize, float checkBoardSquareSideLength)
        {
            _patternSize = new Size(checkBoardSize.Width - 1, checkBoardSize.Height - 1);
            _cornersCount = _patternSize.Width * _patternSize.Height;

            _checkBoardSquareSideLength = checkBoardSquareSideLength;
        }

        /// <summary>
        /// 定位板內角點數量尺寸。
        /// </summary>
        public Size PatternSize => _patternSize;

        /// <summary>
        /// 定位板數量尺寸。
        /// </summary>
        public Size CheckboardSize => new Size(_patternSize.Width + 1, _patternSize.Height + 1);

        /// <summary>
        /// 相機內參數矩陣。
        /// </summary>
        public Matrix<double> CameraMatrix => _cameraMatrix;

        /// <summary>
        /// 相機畸變參數。
        /// </summary>
        public VectorOfDouble DistortionCoeffs => _distortionCoeffs;

        /// <summary>
        /// 所有影像的旋轉向量。
        /// </summary>
        public VectorOfDouble[] RotationVectors => _rotationVectors;

        /// <summary>
        /// 所有影像的平移向量。
        /// </summary>
        public VectorOfDouble[] TranslationVectors => _translationVectors;

        /// <summary>
        /// 影像數量。
        /// </summary>
        public int ImageCount => _imageCount;

        /// <summary>
        /// 原始影像代表。
        /// </summary>
        public Image<Bgr, byte> SourceImageRepresentative => _sourceImageRepresentative;

        public List<VectorOfPointF> AllCorners => _allCorners;

        public CameraParameter CameraParameter => _cameraParameter;

        /// <summary>
        /// 畸變矯正。
        /// </summary>
        public static Image<Bgr, byte> UndistortImage(Image<Bgr, byte> image,
                                                      Matrix<double> cameraMatrix,
                                                      VectorOfDouble distCoeffs)
        {
            if (image == null)
            {
                throw new ArgumentNullException($"{nameof(image)}", "should not be null.");
            }
            if (cameraMatrix == null)
            {
                throw new ArgumentNullException($"{nameof(cameraMatrix)}", "Should not be null.");
            }
            if (distCoeffs == null)
            {
                throw new ArgumentNullException($"{nameof(distCoeffs)}", "Should not be null.");
            }

            var outImg = image.Clone();
            CvInvoke.Undistort(image, outImg, cameraMatrix, distCoeffs);
            return outImg;
        }

        /// <summary>
        /// 畸變矯正。
        /// </summary>
        public static Image<Bgr, byte> UndistortImage(Image<Bgr, byte> image,
                                                      CameraParameter cameraParameter)
        {
            if (cameraParameter == null)
            {
                throw new ArgumentNullException($"{nameof(cameraParameter)}", "Should not be null.");
            }
            return UndistortImage(image, cameraParameter.IntrinsicMatrix, cameraParameter.DistortionCoefficients);
        }

        /// <summary>
        /// 找定位板內角點。
        /// </summary>
        /// <param name="image">定位板照片。</param>
        /// <param name="patternSize">內角點數量尺寸。</param>
        /// <returns>角點。</returns>
        public static VectorOfPointF FindCorners(Image<Gray, byte> image, Size patternSize)
        {
            var corners = new VectorOfPointF();

            var find = CvInvoke.FindChessboardCorners(image,
                                                      patternSize,
                                                      corners,
                                                      CalibCbType.AdaptiveThresh |
                                                      CalibCbType.FastCheck |
                                                      CalibCbType.NormalizeImage);

            if (find)
            {
                // For accuracy.
                CvInvoke.CornerSubPix(image,
                                      corners,
                                      new Size(11, 11),
                                      new Size(-1, -1),
                                      new MCvTermCriteria(30, 0.1));

                return corners;
            }

            // Not find.
            return null;
        }

        /// <summary>
        /// 畫上角點標記的影像。
        /// </summary>
        public Image<Bgr, byte> DrawCheckBoardImage(bool undistort = false)
        {
            var img = _sourceImageRepresentative.Clone();
            CvInvoke.DrawChessboardCorners(img, _patternSize, _allCorners[0], true);

            if (undistort)
            {
                img = UndistortImage(img);
            }

            return img;
        }

        /// <summary>
        /// 畸變矯正。
        /// </summary>
        /// <param name="image"></param>
        /// <returns></returns>
        public Image<Bgr, byte> UndistortImage(Image<Bgr, byte> image)
        {
            return UndistortImage(image, _cameraMatrix, _distortionCoeffs);
        }

        /// <summary>
        /// 計算相機參數（內、外參數，畸變係數）。
        /// </summary>
        /// <param name="checkboardImage">定位板照片。</param>
        /// <returns>相機參數。</returns>
        public CameraParameter CalCameraParameter(Image<Bgr, byte> checkboardImage)
        {
            var images = new List<Image<Bgr, byte>>()
            {
                checkboardImage
            };
            return CalCameraParameter(images, 0);
        }

        /// <summary>
        /// 計算相機參數（內、外參數，畸變係數）。
        /// </summary>
        /// <param name="checkboardImages">定位板照片。</param>
        /// <param name="representativeIndex">要作爲代表的照片索引。</param>
        /// <returns>相機參數。</returns>
        public CameraParameter CalCameraParameter(List<Image<Bgr, byte>> checkboardImages,
                                                  uint representativeIndex = 0)
        {
            return CalCameraParameter(checkboardImages, out _, out _, out _, out _, out _, representativeIndex);
        }

        /// <summary>
        /// 計算相機參數（內、外參數，畸變係數）。
        /// </summary>
        /// <returns>相機參數。</returns>
        public CameraParameter CalCameraParameter(List<Image<Bgr, byte>> checkboardImages,
                                                  out Matrix<double> cameraMatrix,
                                                  out VectorOfDouble distortionCoeffs,
                                                  out VectorOfDouble[] rotationVectors,
                                                  out VectorOfDouble[] translationVectors,
                                                  out double reprojectionError,
                                                  uint representativeIndex = 0)
        {
            if (checkboardImages == null)
            {
                throw new ArgumentNullException($"{nameof(checkboardImages)}", "should not be null.");
            }

            _allCorners.Clear();
            _imageCount = checkboardImages.Count;
            for (int i = 0; i < _imageCount; i++)
            {
                if (checkboardImages[i] == null)
                {
                    continue;
                }

                var sourceImage = checkboardImages[i].Clone();
                var grayImage = sourceImage.Convert<Gray, byte>();
                var corners = FindCorners(grayImage, _patternSize);
                _allCorners.Add(corners);

                if (i == representativeIndex)
                {
                    _sourceImageRepresentative = sourceImage;
                }
            }

            reprojectionError = Calibrate();
            cameraMatrix = _cameraMatrix;
            distortionCoeffs = _distortionCoeffs;
            rotationVectors = _rotationVectors;
            translationVectors = _translationVectors;

            _cameraParameter = new CameraParameter(cameraMatrix,
                                                   distortionCoeffs,
                                                   rotationVectors[representativeIndex],
                                                   translationVectors[representativeIndex]);

            return _cameraParameter;
        }

        /// <summary>
        /// 進行標定。
        /// </summary>
        /// <returns>重投影誤差</returns>
        private double Calibrate()
        {
            var cornersCount = _patternSize.Width * _patternSize.Height;
            var objPoints = MakeObjectPoints();

            PointF[][] imagePoints = new PointF[_imageCount][];
            for (int i = 0; i < _imageCount; i++)
            {
                imagePoints[i] = new PointF[cornersCount];
                imagePoints[i] = _allCorners[i].ToArray();
            }

            var error = CvInvoke.CalibrateCamera(objPoints,
                                                 imagePoints,
                                                 _sourceImageRepresentative.Size,
                                                 _cameraMatrix,
                                                 _distortionCoeffs,
                                                 CalibType.RationalModel,
                                                 new MCvTermCriteria(30, 0.1),
                                                 out var rvs,
                                                 out var tvs);

            _rotationVectors = new VectorOfDouble[rvs.Length];
            for (int i = 0; i < rvs.Length; i++)
            {
                _rotationVectors[i] = new VectorOfDouble();
                rvs[i].ConvertTo(_rotationVectors[i], DepthType.Cv64F);
            }

            _translationVectors = new VectorOfDouble[tvs.Length];
            for (int i = 0; i < tvs.Length; i++)
            {
                _translationVectors[i] = new VectorOfDouble();
                tvs[i].ConvertTo(_translationVectors[i], DepthType.Cv64F);
            }

            return error;
        }

        /// <summary>
        /// 產生角點的世界座標位置（工廠模式）。
        /// </summary>
        /// <returns></returns>
        private MCvPoint3D32f[][] MakeObjectPoints()
        {
            var objPoints = new MCvPoint3D32f[_imageCount][];
            for (int i = 0; i < _imageCount; i++)
            {
                objPoints[i] = new MCvPoint3D32f[_cornersCount];
            }

            for (int currentImage = 0; currentImage < _imageCount; currentImage++)
            {
                for (int currentRow = 0; currentRow < _patternSize.Height; currentRow++)
                {
                    for (int currentCol = 0; currentCol < _patternSize.Width; currentCol++)
                    {
                        int nPoint = currentRow * _patternSize.Width + currentCol;
                        objPoints[currentImage][nPoint].X = (float)currentCol * _checkBoardSquareSideLength;
                        objPoints[currentImage][nPoint].Y = (float)currentRow * _checkBoardSquareSideLength;
                        objPoints[currentImage][nPoint].Z = (float)0;
                    }
                }
            }

            return objPoints;
        }
    }
}