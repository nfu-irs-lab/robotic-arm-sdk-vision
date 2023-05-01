using Emgu.CV.Aruco;
using Emgu.CV.Structure;
using Emgu.CV.Util;
using Emgu.CV;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using RASDK.Vision.Zed;
using RASDK.Arm;
using static Emgu.CV.Aruco.Dictionary;
using Emgu.CV.Mcc;
using System.Collections;

namespace RASDK.Vision.Positioning
{
    public class ArucoCalibrateCamera
    {

        private readonly Aruco _aruco;
        private VectorOfVectorOfPointF _corners;
        private  VectorOfInt _ids;
        private readonly RoboticArm _arm;
        private readonly Zed2i _camera;
        private readonly double _kp = 5;
        private readonly Dictionary _dictionary;
        private readonly PredefinedDictionaryName _predefinedName;
        private readonly Emgu.CV.Aruco.DetectorParameters _detectorParameters;


        public ArucoCalibrateCamera(
                                    Aruco aruco = null,
                                    VectorOfVectorOfPointF corners = null,
                                    VectorOfInt ids = null,
                                    Dictionary dictionary = null,
                                    Emgu.CV.Aruco.DetectorParameters? detectorParameters = null
                                    )
        {
            _aruco = aruco ?? new Aruco();
            _corners = corners ?? new VectorOfVectorOfPointF();
            _ids = ids ?? new VectorOfInt();
            _dictionary = dictionary ?? new Dictionary(Dictionary.PredefinedDictionaryName.Dict7X7_1000);
            _predefinedName = Dictionary.PredefinedDictionaryName.Dict7X7_1000;
            _detectorParameters = detectorParameters ?? Emgu.CV.Aruco.DetectorParameters.GetDefault();

        }

        /// <summary>
        ///  獲得特定的Aruco的特定角落的位置(PointF)
        /// </summary>
        /// <param name="camera"></param>
        /// <param name="arucoId">你要尋找的ArucoID</param>
        /// <param name="dictionary"></param>
        /// <param name="detectorParameters"></param>
        /// <param name="cornerIndex">你要尋找的第幾個角落，(左上=0順時鐘)</param>
        /// <returns></returns>
        /// <exception cref="ArgumentOutOfRangeException"></exception>
        public static Func<PointF> GetSpecificArucoCornerPixelFunc  (Zed2i camera,
                                                                      int arucoId,
                                                                      Dictionary dictionary = null,
                                                                      Emgu.CV.Aruco.DetectorParameters? detectorParameters = null,
                                                                      int cornerIndex = 0)
        {
            if (cornerIndex > 3 || cornerIndex < 0)
            {
                throw new ArgumentOutOfRangeException($"cornerIndex sholud be 0~3 but {cornerIndex}.", nameof(cornerIndex));
            }

            Func<PointF> func = () =>
            {
                var image = camera.GetImage(Zed2i.ImageType.Gray).ToImage<Bgr, byte>();

                var corners = new VectorOfVectorOfPointF();
                var ids = new VectorOfInt();

                // Detect Aruco markers.
                ArucoInvoke.DetectMarkers(image,
                                          dictionary ?? new Dictionary(Dictionary.PredefinedDictionaryName.Dict4X4_100),
                                          corners,
                                          ids,
                                          detectorParameters ?? Emgu.CV.Aruco.DetectorParameters.GetDefault());

                // Find aruco by specific ID.
                var idsArray = ids.ToArray();
                for (int i = 0; i < idsArray.Length; i++)
                {
                    if (idsArray[i] == arucoId)
                    {
                        var c = corners.ToArrayOfArray();
                        return c[i][cornerIndex];
                    }
                }

                // Not find.
                return new PointF(float.NaN, float.NaN);
            };

            return func;
        }


        public Func<PointF> GetEveryArucoCornerPointFunc(Image<Bgr, byte> image)
        {

            Func<PointF> func = () =>
            {
                // Detect Aruco markers.
                ArucoInvoke.DetectMarkers(image, _dictionary, _corners, _ids, _detectorParameters);

                var arucoIdArray = _ids.ToArray();
                var arucoCornerList = _corners.ToArrayOfArray();
                for (int i = 0; i < arucoIdArray.Length; i++)
                {
                    for (int j = 0; j < 4; j++)
                    {
                        return arucoCornerList[i][j];
                    }
                }

                // Not find.
                return new PointF(float.NaN, float.NaN);
            };

            return func;
        }

        public void GoEveryArucoCorner(Image<Bgr,byte> image)
        {
            //可獲得所有Aruco的角落的XY座標
            
            _aruco.Detect(image, out _corners,out _ids);

            var arucoIdsList = _ids.ToArray();
            var arucoCornerPointList = _corners.ToArrayOfArray();

            double timeOut = 0;

            for (int i=0;i< arucoIdsList.Length;i++)
            {
                for (int j=0;j<4;j++)
                {
                    var a = GetSpecificArucoCornerPointFunc(_camera,i, _dictionary, _detectorParameters,j);
                    var b = VisualServo.MakeBasicArmMoveFunc(_arm, _kp);
                    var c=VisualServo.Tracking(image.Size, timeOut, a, b);
                    VisualServo.Tracking(c, timeOut, a, b);
                }
            }



        }
    }
}
