using Emgu.CV;
using Emgu.CV.Structure;
using Emgu.CV.Util;

namespace RASDK.Vision.ExtensionMethods
{
    public static class Image
    {
        /// <summary>
        /// 畸變矯正。
        /// </summary>
        public static Image<Bgr, byte> UndistortImage(this Image<Bgr, byte> image,
                                                      Matrix<double> cameraMatrix,
                                                      VectorOfDouble distCoeffs)
        {
            return CameraCalibration.UndistortImage(image, cameraMatrix, distCoeffs);
        }

        /// <summary>
        /// 畸變矯正。
        /// </summary>
        public static Image<Bgr, byte> UndistortImage(this Image<Bgr, byte> image,
                                                      CameraParameter cameraParameter)
        {
            return CameraCalibration.UndistortImage(image, cameraParameter);
        }
    }
}