using Emgu.CV.Aruco;
using Emgu.CV.Structure;
using Emgu.CV.Util;
using RASDK.Arm;
using RASDK.Basic.Message;
using RASDK.Vision.Zed;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Timers;

namespace RASDK.Vision.Positioning
{
    public class ArucoCalibrateCamera
    {
        private static double _allowablePixelError;
        private readonly CameraParameter _cameraParameter;
        private static readonly MessageHandler _messageHandler = new EmptyMessageHandler();

        public ArucoCalibrateCamera(CameraParameter cameraParameter,
                                    double allowablePixelError = 3
                                    )
        {
            _allowablePixelError = allowablePixelError;
            WorldOffset = new PointF(0, 0);
        }

        public PointF WorldOffset { get; set; }

        public static Action<PointF> MakeBasicArmMoveFunc(RoboticArm arm,
                                                           double kp,
                                                           bool invertedX = false,
                                                           bool invertedY = true
                                                           )
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

                if (double.IsNaN(armPosition[0])|| double.IsNaN(armPosition[1]))
                {
                    //armPosition[0] = 1;
                    //armPosition[1] = 1;
                    return;
                }

                var motionParam = new AdditionalMotionParameters
                {
                    MotionType = RASDK.Arm.Type.MotionType.PointToPoint,
                    CoordinateType = RASDK.Arm.Type.CoordinateType.Descartes,
                    NeedWait = true
                };
                arm.MoveRelative(armPosition, motionParam);
            };

            return action;
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
        public static Func<PointF> MakeBasicArucoGetCurrentPixelFunc(Zed2i camera,
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

                var ids = new VectorOfInt();
                var corners = new VectorOfVectorOfPointF();



                // Detect Aruco markers.
                ArucoInvoke.DetectMarkers(image,
                                          dictionary ?? new Dictionary(Dictionary.PredefinedDictionaryName.Dict7X7_1000),
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


        public static PointF Tracking(Size imageSize, double timeout, Func<PointF> getCurrentPixelFunc, Action<PointF> armMoveFunc, int allowablePixelError = 3)
        {
            // Get center.
            var center = new PointF((imageSize.Width / 2) - 1, (imageSize.Height / 2) - 1);

            return Tracking(center, timeout, getCurrentPixelFunc, armMoveFunc, allowablePixelError);
        }

        /// <summary>
        ///
        /// </summary>
        /// <param name="goalPixel"></param>
        /// <param name="timeout">Unit in ms</param>
        /// <param name="getCurrentPixelFunc"></param>
        /// <param name="armMoveFunc"></param>
        /// <param name="allowablePixelError"></param>
        /// <returns></returns>
        public static PointF Tracking(PointF goalPixel, double timeout, Func<PointF> getCurrentPixelFunc, Action<PointF> armMoveFunc, int allowablePixelError = 3)
        {
            var error = new PointF(float.NaN, float.NaN);

            var timerCount = 0;
            var timer = new Timer(1);

            if (timeout >= 0)
            {
                timer.Stop();
                timer.Elapsed += (s, e) => { timerCount += 10; }; // XXXtimer部分與官方標示之單位不符
                                                                  // 此代表每經過1毫秒timerCount增加10
                                                                  // timeout的數字單位為毫秒
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

                if (currentPixel.X == float.NaN || currentPixel.Y == float.NaN)
                {
                    continue;
                }

                error = new PointF
                {
                    X = currentPixel.X - goalPixel.X,
                    Y = currentPixel.Y - goalPixel.Y
                };

                if (Math.Abs(currentPixel.X) < Math.Abs(error.X) || Math.Abs(currentPixel.X) < allowablePixelError)
                {
                    error.X = currentPixel.X;
                }

                if (Math.Abs(currentPixel.Y) < Math.Abs(error.Y) || Math.Abs(currentPixel.Y) < allowablePixelError)
                {
                    error.Y = currentPixel.Y;
                }

                // Arm moving.
                armMoveFunc(error);

                if ((Math.Abs(error.X) <= allowablePixelError && Math.Abs(error.Y) <= allowablePixelError) ||
                    timeout <= timerCount)
                {
                    break;
                }
            }

            timer.Stop();
            return error;
        }

        #region Record Aruco ID and ConerNum



        #endregion Record Aruco ID and ConerNum

        #region Record Offset and AllowablePixekError

        /// <summary>
        ///
        /// </summary>
        /// <param name="armAxis"></param>
        /// <param name="cameraFinalPixel"></param>
        /// <param name="picturewFinalPixel"></param>
        /// <param name="offset">cameraFinalPixel-picturewFinalPixel</param>
        /// <param name="allowablePixelError"></param>
        /// <param name="path"></param>
        public static void RecordResult(int arucoId, int cornerNum, PointF[][] initCornerPoint, double[] armAxis, string path = "acc_parameters.csv")
        {
            var colData = new List<string>();
            colData.Add(arucoId.ToString());
            colData.Add(cornerNum.ToString());

            colData.Add(initCornerPoint[arucoId][cornerNum].X.ToString());
            colData.Add(initCornerPoint[arucoId][cornerNum].Y.ToString());

            colData.Add(armAxis[0].ToString());
            colData.Add(armAxis[1].ToString());
            colData.Add(armAxis[2].ToString());
            colData.Add(armAxis[3].ToString());
            colData.Add(armAxis[4].ToString());
            colData.Add(armAxis[5].ToString());

            var csvData = new List<List<string>>();
            csvData.Add(colData);

            var header = new List<string>() { "aruco_id", "corner_num",
                                              "initCornerPointX", "initCornerPointY",
                                               "arm_x", "arm_y","arm_z","arm_a", "arm_b", "arm_c"};

            Basic.Csv.Write(path, csvData, header);
        }

        #endregion Record Offset and AllowablePixekError

        public static void RecordResult(int initPhotoPixel, string path = "acc_parameters.csv")
        {
            var csvData = new List<List<string>>()
            {
                new List<string>(){ "ArucoId=" + initPhotoPixel},
            };

            Basic.Csv.Write(path, csvData);
        }

        public static ArucoCalibrateCamera LoadFromCsv(string filename = "acc_parameters.csv")
        {
            var csvData = Basic.Csv.Read(filename);

            var cx = double.Parse(csvData[0][1]);
            var cy = double.Parse(csvData[1][1]);
            var fx = double.Parse(csvData[2][1]);
            var fy = double.Parse(csvData[3][1]);
            var skew = double.Parse(csvData[4][1]);

            var dc = new double[csvData[5].Count - 1];
            for (int i = 1; i < csvData[5].Count; i++)
            {
                dc[i - 1] = double.Parse(csvData[5][i]);
            }

            var rv = new double[csvData[6].Count - 1];
            for (int i = 1; i < csvData[6].Count; i++)
            {
                rv[i - 1] = double.Parse(csvData[6][i]);
            }

            var tv = new double[csvData[7].Count - 1];
            for (int i = 1; i < csvData[7].Count; i++)
            {
                tv[i - 1] = double.Parse(csvData[7][i]);
            }

            var offset = new PointF(0, 0)
            {
                X = float.Parse(csvData[8][1]),
                Y = float.Parse(csvData[9][1])
            };

            var cp = new CameraParameter(cx, cy, fx, fy, skew, new VectorOfDouble(dc), new VectorOfDouble(rv), new VectorOfDouble(tv));
            var acc = new ArucoCalibrateCamera(cp)
            {
                WorldOffset = offset
            };

            return acc;
        }

        public void SaveToCsv(string filename = "acc_parameters.csv")
        {
            if (_cameraParameter == null)
            {
                throw new Exception($"cameraParameter should not be null.");
            }

            if (System.IO.File.Exists(filename))
            {
                System.IO.File.Delete(filename);
            }

            var dc = new List<string>() { "DistCoeffs" };
            foreach (var v in _cameraParameter.DistortionCoefficients.ToArray())
            {
                dc.Add(v.ToString() ?? "");
            }

            var rv = new List<string>() { "RotationVector" };
            foreach (var v in _cameraParameter.RotationVector.ToArray())
            {
                rv.Add(v.ToString() ?? "");
            }

            var tv = new List<string>() { "TranslationVector" };
            foreach (var v in _cameraParameter.TranslationVector.ToArray())
            {
                tv.Add(v.ToString() ?? "");
            }

            var csvData = new List<List<string>>()
            {
                new List<string>(){"cx",_cameraParameter.Cx.ToString()},
                new List<string>(){"cy",_cameraParameter.Cy.ToString()},
                new List<string>(){"fx",_cameraParameter.Fx.ToString()},
                new List<string>(){"fy",_cameraParameter.Fy.ToString()},
                new List<string>(){"skew",_cameraParameter.Skew.ToString()},
                dc,
                rv,
                tv,
                new List<string>(){"offsetX", WorldOffset.X.ToString()},
                new List<string>(){"offsety", WorldOffset.Y.ToString()}
            };

            Basic.Csv.Write(filename, csvData);
        }
    }
}