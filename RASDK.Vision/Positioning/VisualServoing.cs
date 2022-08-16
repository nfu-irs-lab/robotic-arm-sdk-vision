using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Timers;
using System.Drawing;
using RASDK.Arm;
using Emgu.CV;
using Emgu.CV.Util;
using Emgu.CV.Aruco;
using Emgu.CV.Structure;


namespace RASDK.Vision.Positioning
{
    public class VisualServo
    {
        public static Action<PointF> MakeBasicArmMoveFunc(RoboticArm arm,
                                                          double kp,
                                                          bool invertedX = false,
                                                          bool invertedY = true)
        {
            Action<PointF> action = (errorPixel) =>
            {
                // Proportional control.
                var armPosition = new double[]
                {
                    kp * errorPixel.X,
                    kp * errorPixel.Y,
                    0,

                    0,
                    0,
                    0
                };

                if (invertedX)
                {
                    armPosition[0] *= -1;
                }
                if (invertedY)
                {
                    armPosition[1] *= -1;
                }

                var motionParam = new AdditionalMotionParameters
                {
                    MotionType = RASDK.Arm.Type.MotionType.Linear,
                    CoordinateType = RASDK.Arm.Type.CoordinateType.Descartes,
                    NeedWait = true
                };

                arm.MoveRelative(armPosition, motionParam);
            };

            return action;
        }

        public static Func<PointF> MakeBasicArucoGetCurrentPixelFunc(IDS.IDSCamera camera,
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
                var image = camera.GetImage().ToImage<Bgr, byte>();

                var corners = new VectorOfVectorOfPointF();
                var ids = new VectorOfInt();

                // Detect Aruco markers.
                ArucoInvoke.DetectMarkers(image,
                                          dictionary ?? new Dictionary(Dictionary.PredefinedDictionaryName.Dict4X4_100),
                                          corners,
                                          ids,
                                          detectorParameters ?? DetectorParameters.GetDefault());

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

        public static PointF Tracking(Size imageSize, double timeout, Func<PointF> getCurrentPixelFunc, Action<PointF> armMoveFunc, double allowableError = 5)
        {
            // Get center.
            var center = new PointF((imageSize.Width / 2) - 1, (imageSize.Height / 2) - 1);

            return Tracking(center, timeout, getCurrentPixelFunc, armMoveFunc, allowableError);
        }

        public static PointF Tracking(PointF goalPixel, double timeout, Func<PointF> getCurrentPixelFunc, Action<PointF> armMoveFunc, double allowableError = 5)
        {
            var error = new PointF(float.NaN, float.NaN);

            var timerCount = 0.0;
            var timer = new Timer(100);

            if (timeout >= 0)
            {
                timer.Stop();
                timer.Elapsed += (s, e) => { timerCount += 0.1; };
                timer.Start();
            }

            // Interative.
            while (timerCount < timeout || timeout < 0)
            {
                PointF currentPixel = new PointF();
                try
                {
                    currentPixel = getCurrentPixelFunc();
                }
                catch (Exception)
                {
                    continue;
                }

                if (currentPixel.X == float.NaN ||
                    currentPixel.Y == float.NaN)
                {
                    continue;
                }

                error = new PointF
                {
                    X = currentPixel.X - goalPixel.X,
                    Y = currentPixel.Y - goalPixel.Y
                };

                // Arm moving.
                armMoveFunc(error);

                if (Math.Abs(error.X) <= allowableError &&
                    Math.Abs(error.Y) <= allowableError &&
                    timeout >= 0)
                {
                    break;
                }
            }

            timer.Stop();
            return error;
        }
    }
}