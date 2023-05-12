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
            _arm.MoveAbsolute(0, 413.407, 517.803, -90, -90, 0);
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


        public void MoveFixOffset(int offsetX,int offsetY=0) 
        {
            MotionParam add = new MotionParam() { CoordinateType = CoordinateType.Descartes };
            _arm.MoveAbsolute(164.072+offsetX, 294.649+ offsetY, 290.807, -90, -90, 0, add);

        }


        public void OneTimeInterare(int id)
        {

            Aruco aruco = new Aruco();
            var frame = _zed2i.GetImage(Zed2i.ImageType.Gray);
            VectorOfInt ids = new VectorOfInt();
            VectorOfVectorOfPointF Corner = new VectorOfVectorOfPointF();
            aruco.Dictionary = new Dictionary(Dictionary.PredefinedDictionaryName.Dict7X7_1000);
            aruco.Detect(frame.ToImage<Bgr, byte>(), out Corner, out ids);
            var InitArucoPixel = Corner.ToArrayOfArray();
            _arm.Speed = 20;
            
            MoveToPoint1();

            for (int i = 0; i < 4; i++)
            {
                var makeArmMoveFunc = ArucoCalibrateCamera.MakeBasicArmMoveFunc(_arm, 0.05, true, false);
                var arucoIterate = ArucoCalibrateCamera.MakeBasicArucoGetCurrentPixelFunc(_zed2i, id, aruco.Dictionary, DetectorParameters.GetDefault(),i);
                double timeout = 8000;//毫秒
                ArucoCalibrateCamera.Tracking(frame.Size, timeout, arucoIterate, makeArmMoveFunc, 3);
                var armDescartesAxis = _arm.GetNowPosition(CoordinateType.Descartes);

                ArucoCalibrateCamera.RecordResult(id, i, InitArucoPixel, armDescartesAxis);
            }
            MoveToPoint1();

        }

        public void AutoFullInterare()
        {
            if (System.IO.File.Exists("acc_parameters.csv"))
            {
                System.IO.File.Delete("acc_parameters.csv");
            }

            _arm.Speed = 20;

            //先拍照檢驗有多少aruco

            Aruco aruco = new Aruco();
            var frame = _zed2i.GetImage(Zed2i.ImageType.Gray);
            VectorOfInt ids = new VectorOfInt();
            VectorOfVectorOfPointF Corner = new VectorOfVectorOfPointF();
            aruco.Dictionary = new Dictionary(Dictionary.PredefinedDictionaryName.Dict7X7_1000);
            aruco.Detect(frame.ToImage<Bgr, byte>(), out Corner, out ids);
            var InitArucoPixel=Corner.ToArrayOfArray();


            int[] ArucoIdList = ids.ToArray();

            MoveToPoint2();

            for (int i = 0; i < ArucoIdList.Length; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    try
                    {
                        _arm.Speed = 20;

                        var makeArmMoveFunc = ArucoCalibrateCamera.MakeBasicArmMoveFunc(_arm, 0.05, true, false);
                        var arucoIterate = ArucoCalibrateCamera.MakeBasicArucoGetCurrentPixelFunc(_zed2i, i, aruco.Dictionary, DetectorParameters.GetDefault(), j);
                        double timeout = 8000;//毫秒
                        ArucoCalibrateCamera.Tracking(frame.Size, timeout, arucoIterate, makeArmMoveFunc, 3);
                        var armDescartesAxis = _arm.GetNowPosition(CoordinateType.Descartes);

                        ArucoCalibrateCamera.RecordResult(i, j, InitArucoPixel, armDescartesAxis);
                    }
                    catch
                    {
                        throw new Exception(); 
                    }
                }
                //每35回到左邊
                if (i%35<18)
                {
                    MoveToPoint2();
                }
                else if( i>=18 && i%35<35)
                {
                    MoveToPoint3();
                }
                else if(i%36==0)
                {
                    MoveFixOffset(0,18);

                }


            }
        }



        public Emgu.CV.Mat TakePicture()
        {
            return _zed2i.GetImage(Zed2i.ImageType.Gray);
        }
    }
}