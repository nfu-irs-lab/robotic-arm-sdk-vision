using Emgu.CV.Aruco;
using Emgu.CV.Structure;
using Emgu.CV.Util;
using RASDK.Arm;
using RASDK.Basic.Message;
using RASDK.Vision.Zed;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Timers;
using RASDK.Vision.Positioning;
using Emgu.CV;
using Emgu.CV.CvEnum;
using RASDK.Basic;

namespace RASDK.Vision.Positioning
{
    public class ArUcoPositioner:IVisionPositioning
    {

        private Matrix<double> _homographyMatrix = null;

        /// <summary>
        /// 單應性（Homography）視覺定位。
        /// </summary>
        public ArUcoPositioner(PointF[] imagePoints, PointF[] worldPoints)
        {
            UpdateHomographyMatrix(imagePoints, worldPoints);
        }

        /// <summary>
        /// 單應性（Homography）視覺定位。
        /// </summary>
        public ArUcoPositioner(VectorOfPointF imagePoints, VectorOfPointF worldPoints)
        {
            UpdateHomographyMatrix(imagePoints.ToArray(), worldPoints.ToArray());
        }


        /// <summary>
        /// 單應性（Homography）視覺定位。
        /// </summary>
        public ArUcoPositioner(Mat homographyMatrix)
        {
            if (homographyMatrix.Rows != 3 || homographyMatrix.Cols != 3)
            {
                throw new ArgumentException($"homographyMatrix 必須爲 3*3 矩陣，" +
                                            $"實際爲{homographyMatrix.Rows}*{homographyMatrix.Cols}。");
            }

            if (_homographyMatrix == null)
            {
                _homographyMatrix = new Matrix<double>(3, 3);
            }

            homographyMatrix.ConvertTo(_homographyMatrix, DepthType.Cv64F);
        }

        /// <summary>
        /// 單應性（Homography）視覺定位。
        /// </summary>
        public ArUcoPositioner(Matrix<double> homographyMatrix)
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


        public static ArUcoPositioner LoadFromCsv(string filename = @"..\..\..\Tool\acc_parametersFull.csv")
        {
            var csvData = RASDK.Basic.Csv.Read(filename);

            var homographyMatrix = new Matrix<double>(3, 3);
            for (int r = 0; r < homographyMatrix.Rows; r++)
            {
                homographyMatrix[r, 0] = double.Parse(csvData[r][0]);
                homographyMatrix[r, 1] = double.Parse(csvData[r][1]);
                homographyMatrix[r, 2] = double.Parse(csvData[r][2]);
            }

            return new ArUcoPositioner(homographyMatrix);
        }

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


        public void SaveToCsv(string filename = @"../../../Tool/acchomography_matrix.csv")
        {
            if (System.IO.File.Exists(filename))
            {
                System.IO.File.Delete(filename);
            }

            var csvData = new List<List<string>>();
            for (int r = 0; r < _homographyMatrix.Rows; r++)
            {
                var row = new List<string>()
                {
                    _homographyMatrix[r,0].ToString(),
                    _homographyMatrix[r,1].ToString(),
                    _homographyMatrix[r,2].ToString(),
                };
                csvData.Add(row);
            }
            RASDK.Basic.Csv.Write(filename, csvData);
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
                                            $"目前輸入爲 imagePoints:{imagePoints.Length}, worldPoints:{worldPoints.Length}.");
            }

            if (worldPoints.Length != imagePoints.Length)
            {
                throw new ArgumentException($"imagePoints 和 worldPoints 的長度不相等，" +
                                            $"目前輸入爲 imagePoints:{imagePoints.Length}, worldPoints:{worldPoints.Length}.");
            }

            var h = CvInvoke.FindHomography(imagePoints, worldPoints);

            if (h == null)
            {
                throw new Exception("Can't find homography matrix.");
            }

            if (_homographyMatrix == null)
            {
                _homographyMatrix = new Matrix<double>(3, 3);
            }
            h.ConvertTo(_homographyMatrix, DepthType.Cv64F);
        }
    



        public static PointF[] FindClosestArUco(PointF[] DesPointF)
        {
            //csvData[row][col]，row[0]的header都要算一排
            var csvData = Csv.Read(@"../../../Tool/temp/acc_parametersFull.csv");

            PointF[] ArUcoPoint = new PointF[DesPointF.Length];

            double minimum = 0;
            for (int i = 0; i < DesPointF.Length; i++)
            {
                for (int row = 1; row < csvData.Count; row++)
                {
                    var InitPixelX = float.Parse(csvData[row][2]);
                    var InitPixelY = float.Parse(csvData[row][3]);

                    var resultX = DesPointF[i].X - InitPixelX;
                    var resultY = DesPointF[i].Y - InitPixelY;

                    //判斷哪個點與desPoint的直線距離最近
                    var resultTotal = Math.Sqrt(resultX * resultX + resultY * resultY);

                    if (resultTotal == 0)
                    {
                        ArUcoPoint[i].X = float.Parse(csvData[row][4]);
                        ArUcoPoint[i].Y = float.Parse(csvData[row][5]);
                        break;
                    }

                    if (minimum == 0  || resultTotal < minimum)
                    {
                        minimum = resultTotal;
                        ArUcoPoint[i].X = float.Parse(csvData[row][4]);
                        ArUcoPoint[i].Y = float.Parse(csvData[row][5]);
                    }
                }

            }
            return ArUcoPoint;
        }


    public static Action<PointF> MakeBasicArmMoveFunc(RoboticArm arm,
                                                           double kp,
                                                           bool invertedX = true,
                                                           bool invertedY = false
                                                           )
        {

            PointF PreviouserrorPixel = new PointF(0,0);
            Action<PointF> action = (errorPixel) =>
            {
                if (errorPixel.X>80||errorPixel.Y>80)
                {
                    arm.Speed = 80;
                }
                else
                {
                    arm.Speed = 20;
                }

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



                if (double.IsNaN(armPosition[0])|| 
                    double.IsNaN(armPosition[1])||
                    double.IsNaN(errorPixel.X)  ||
                    double.IsNaN(errorPixel.Y)
                    )

                {
                    var xFixDirection = PreviouserrorPixel.X / Math.Abs(PreviouserrorPixel.X);
                    var yFixDirection = PreviouserrorPixel.Y / Math.Abs(PreviouserrorPixel.Y);

                    armPosition[0] = xFixDirection;
                    armPosition[1] = yFixDirection;
                }


                if (!double.IsNaN(errorPixel.X) ||
                    !double.IsNaN(errorPixel.Y))
                {
                    PreviouserrorPixel = errorPixel;
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

                //image.Save(@"..\..\..\Tool\temimage.jpg");

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



        /// <summary>
        ///
        /// </summary>
        /// <param name="armAxis"></param>
        /// <param name="cameraFinalPixel"></param>
        /// <param name="picturewFinalPixel"></param>
        /// <param name="offset">cameraFinalPixel-picturewFinalPixel</param>
        /// <param name="allowablePixelError"></param>
        /// <param name="path"></param>
        public static void RecordResult(int arucoId, int cornerNum, PointF[][] initCornerPoint, double[] armAxis, VectorOfInt ids,string path = @"..\..\..\Tool\acc_parameters.csv")
        {
            var colData = new List<string>();
            colData.Add(arucoId.ToString());
            colData.Add(cornerNum.ToString());

            var index = ids.ToArray().ToList<int>().IndexOf(arucoId);
            if (index != -1)
            {
                colData.Add(initCornerPoint[index][cornerNum].X.ToString());
                colData.Add(initCornerPoint[index][cornerNum].Y.ToString());
            }
            else
            {
                throw new Exception("Can't find ArUco ID.");
            }

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






    }
}