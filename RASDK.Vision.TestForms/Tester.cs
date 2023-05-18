using Emgu.CV;
using Emgu.CV.Aruco;
using Emgu.CV.Structure;
using Emgu.CV.Util;
using RASDK.Arm.Hiwin;
using RASDK.Arm.Type;
using RASDK.Basic;
using RASDK.Basic.Message;
using RASDK.Vision.Positioning;
using RASDK.Vision.Zed;
using System;
using System.Threading;
using MotionParam = RASDK.Arm.AdditionalMotionParameters;

namespace RASDK.Vision
{
    public class Tester
    {
        private readonly RoboticArm _arm;
        private readonly LogHandler _logHandler = new GeneralLogHandler($"../../../../log/");
        private readonly MessageHandler _messageHandler;
        private bool connected = false;
        private readonly Zed2i _zed2i;

        public Tester(Zed2i zed2i,MessageHandler messageHandler = null, RoboticArm arm = null)
        {
            _zed2i = zed2i ;
            _messageHandler = messageHandler = new GeneralMessageHandler(_logHandler);
            _arm = arm ?? new RoboticArm(_messageHandler, "192.168.0.3");

        }

        public void ArmConnect()
        {
            try
            {
                _arm.Connect();
                connected = true;
            }
            catch (Exception)
            {
                //Do nothing
            }
        }

        public void ArmDisconnect()
        {
            _arm.Disconnect();
        }

        public void MoveToCapturePosition()
        {
            _arm.Speed = 95;
            _arm.MoveAbsolute(-24.806, 413.409, 517.716, -94.362, -89.136, 4.766);
        }

        public void MoveToPoint1()
        {
            _arm.Speed = 100;
            MotionParam add = new MotionParam() { CoordinateType = CoordinateType.Descartes };
            _arm.MoveAbsolute(1.047, 294.649, 290.807, -90, -90, 0, add);
        }


        public void MoveToPoint2()
        {
            _arm.Speed = 100;
            MotionParam add = new MotionParam() { CoordinateType = CoordinateType.Descartes };
            _arm.MoveAbsolute(164.072, 294.649, 290.807, -90, -90, 0, add);
        }
        public void MoveToPoint3()
        {
            _arm.Speed = 100;
            MotionParam add = new MotionParam() { CoordinateType = CoordinateType.Descartes };
            _arm.MoveAbsolute(-150.291, 294.649, 290.805, -90.001, -90, 0, add);
        }


        public void MoveFixOffset(int offsetX=0,int offsetY=0) 
        {
            MotionParam add = new MotionParam() { CoordinateType = CoordinateType.Descartes };
            _arm.MoveAbsolute(164.072+offsetX, 294.649+ offsetY, 290.807, -90, -90, 0, add);

        }


        public void OneTimeInterare(int id)
        {
            MoveToCapturePosition();
            Thread.Sleep(500);

            Aruco aruco = new Aruco();
            var frame = _zed2i.GetImage(Zed2i.ImageType.Gray);
            VectorOfInt ids = new VectorOfInt();
            VectorOfVectorOfPointF Corner = new VectorOfVectorOfPointF();
            aruco.Dictionary = new Dictionary(Dictionary.PredefinedDictionaryName.Dict7X7_1000);
            aruco.Detect(frame.ToImage<Bgr, byte>(), out Corner, out ids);
            var InitArucoPixel = Corner.ToArrayOfArray();
            

            if (id % 36 < 18)
            {
                MoveToPoint2();
            }
            else
            {
                MoveToPoint3();
            }
            try
            {
                for (int i = 0; i < 4; i++)
                {
                    var makeArmMoveFunc = ArUcoPositioner.MakeBasicArmMoveFunc(_arm, 0.05, true, false);
                    var arucoIterate = ArUcoPositioner.MakeBasicArucoGetCurrentPixelFunc(_zed2i, id, null, null, i);
                    double timeout = 8000;//毫秒
                    ArUcoPositioner.Tracking(frame.Size, timeout, arucoIterate, makeArmMoveFunc, 3);
                    var armDescartesAxis = _arm.GetNowPosition(CoordinateType.Descartes);

                    //ids只是為了在RecordResult內當成index找尋目標而已，並不會被記錄在csv檔內
                    ArUcoPositioner.RecordResult(id, i, InitArucoPixel, armDescartesAxis, ids);
                }
            }
            catch 
            {
                throw new Exception();
            }
            MoveToCapturePosition();

        }

        public void AutoFullInterate()
        {
            MoveToCapturePosition();

            Thread.Sleep(500);

            if (System.IO.File.Exists(@"..\..\..\Tool\acc_parameters.csv"))
            {
                System.IO.File.Delete(@"..\..\..\Tool\acc_parameters.csv");
            }


            //先拍照檢驗有多少aruco

            Aruco aruco = new Aruco();
            var frame = _zed2i.GetImage(Zed2i.ImageType.Gray);
            VectorOfInt ids = new VectorOfInt();
            VectorOfVectorOfPointF Corner = new VectorOfVectorOfPointF();
            aruco.Dictionary = new Dictionary(Dictionary.PredefinedDictionaryName.Dict7X7_1000);
            aruco.Detect(frame.ToImage<Bgr, byte>(), out Corner, out ids);
            var InitArucoPixel=Corner.ToArrayOfArray();

            //存原始被偵測過ID的相片
            Image<Bgr, byte> colorFrame = frame.ToImage<Bgr, byte>().Clone();
            var color = new MCvScalar(0, 255, 0);
            ArucoInvoke.DrawDetectedMarkers(colorFrame, Corner, ids, color);
            colorFrame.Save(@"..\..\..\Tool\AutoArUcoInitFImage.jpg");

            int[] ArucoIdList = ids.ToArray();

            MoveToPoint2();

            double[] LastPosition = { 0, 0, 0, 0, 0, 0 };
            for (int id = 0; id < ArucoIdList.Length; id++)
            {
                //最左邊是0、36、72...
                if (id % 36 == 0)
                {
                    var Offsety = id / 36 * 18;
                    MoveFixOffset(0, Offsety);
                }


                for (int corners = 0; corners < 4; corners++)
                {
                    try
                    {
                        var makeArmMoveFunc = ArUcoPositioner.MakeBasicArmMoveFunc(_arm, 0.05, true, false);
                        var arucoIterate = ArUcoPositioner.MakeBasicArucoGetCurrentPixelFunc(_zed2i, id, aruco.Dictionary, DetectorParameters.GetDefault(), corners);
                        double timeout = 8000;//毫秒
                        ArUcoPositioner.Tracking(frame.Size, timeout, arucoIterate, makeArmMoveFunc, 3);
                        var armDescartesAxis = _arm.GetNowPosition(CoordinateType.Descartes);

                        ArUcoPositioner.RecordResult(id, corners, InitArucoPixel, armDescartesAxis, ids, @"..\..\..\Tool\acc_parameters.csv");
                    }
                    catch(Exception ex)
                    {
                        if(ex.Message == "Can't find ArUco ID.")
                        {
                            continue;
                        }
                        else
                        {
                            throw new Exception(ex.Message); 
                        }

                    }
                }
            }
        }

        public void TestMove()
        {
            _arm.Speed = 100;
            MotionParam add = new MotionParam() { CoordinateType = CoordinateType.Descartes };
            _arm.MoveAbsolute( 293.272, 215.499, 290.807, -90, -90, 0, add);
        }
    }
}