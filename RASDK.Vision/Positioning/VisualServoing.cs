using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Timers;
using RASDK.Arm;
using Emgu.CV;
using Emgu.CV.Util;
using System.Drawing;

namespace RASDK.Vision.Positioning
{
    public class VisualServo
    {
        public static bool InvertedX = false;

        public static bool InvertedY = true;

        private readonly RoboticArm _arm;

        public VisualServo(RoboticArm arm)
        {
            _arm = arm;
        }

        public PointF Tracking(double kp, double allowableError, double timeout, Func<PointF> getCurrentPixelFunc, Size imageSize)
        {
            // Get center.
            var goalPixel = new PointF((imageSize.Width / 2) - 1, (imageSize.Height / 2) - 1);
            return Tracking(kp, allowableError, timeout, getCurrentPixelFunc, goalPixel);
        }

        public PointF Tracking(double kp, double allowableError, double timeout, Func<PointF> getCurrentPixelFunc, PointF goalPixel)
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
                var currentPixel = getCurrentPixelFunc();
                error = TrackingInterative(_arm, kp, currentPixel, goalPixel);

                if (Math.Abs(error.X) >= allowableError &&
                    Math.Abs(error.Y) >= allowableError &&
                    timeout >= 0)
                {
                    break;
                }
            }

            timer.Stop();
            return error;
        }

        private static PointF TrackingInterative(RoboticArm arm, double kp, PointF currentPixel, PointF goalPixel)
        {
            var error = new PointF
            {
                X = currentPixel.X - goalPixel.X,
                Y = currentPixel.Y - goalPixel.Y
            };

            // Proportional control.
            var armPosition = new double[]
            {
                kp * error.X,
                kp * error.Y,
                0,

                0,
                0,
                0
            };

            if (InvertedX)
            {
                armPosition[0] *= -1;
            }
            if (InvertedY)
            {
                armPosition[1] *= -1;
            }

            var motionParam = new AdditionalMotionParameters
            {
                MotionType = Arm.Type.MotionType.Linear,
                CoordinateType = Arm.Type.CoordinateType.Descartes,
                NeedWait = true
            };

            arm.MoveRelative(armPosition, motionParam);

            return error;
        }
    }
}