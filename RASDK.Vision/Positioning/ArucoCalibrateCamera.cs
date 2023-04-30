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

namespace RASDK.Vision.Positioning
{
    public class ArucoCalibrateCamera
    {

        /// <summary>
        ///  
        ///
        /// </summary>
        /// <param name="camera"></param>
        /// <param name="arucoId">你要尋找的ArucoID</param>
        /// <param name="dictionary"></param>
        /// <param name="detectorParameters"></param>
        /// <param name="cornerIndex">你要尋找的第幾個角落，(左上=0順時鐘)</param>
        /// <returns></returns>
        /// <exception cref="ArgumentOutOfRangeException"></exception>
        public static Func<PointF> MakeBasicArucoGetCurrentPixelFunc(Zed2i camera,
                                                                     int arucoId,
                                                                     Dictionary dictionary = null,
                                                                     DetectorParameters? detectorParameters = null,
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
                                          dictionary ?? new Dictionary(Dictionary.PredefinedDictionaryName.Dict7X7_1000),
                                          corners,
                                          ids,
                                          detectorParameters ?? DetectorParameters.GetDefault());

                var arucoIdArray = ids.ToArray();
                for (int i = 0; i < arucoIdArray.Length; i++)
                {
                    if (arucoIdArray[i] == arucoId)
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
    }
}
