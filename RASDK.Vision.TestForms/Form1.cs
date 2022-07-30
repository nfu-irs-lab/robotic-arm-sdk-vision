using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using RASDK.Basic;
using RASDK.Basic.Message;
using Emgu.CV;
using Emgu.CV.Util;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;

namespace RASDK.Vision.TestForms
{
    public partial class Form1 : Form
    {
        private readonly MessageHandler _messageHandler;

        public Form1()
        {
            InitializeComponent();
            _messageHandler = new GeneralMessageHandler(new EmptyLogHandler());
        }

        ~Form1()
        {
            try
            {
                if (_idsCamera != null)
                {
                    if (_idsCamera.Connected)
                    {
                        _idsCamera.Disconnect();
                    }
                }
            }
            catch
            { /* Do nothing. */ }
        }

        #region Positioning

        private void buttonConvert_Click(object sender, EventArgs e)
        {
            var rv = new VectorOfDouble(new double[] {(double)numericUpDownRV1.Value,
                                                      (double)numericUpDownRV2.Value,
                                                      (double)numericUpDownRV3.Value});
            var tv = new VectorOfDouble(new double[] {(double)numericUpDownTV1.Value,
                                                      (double)numericUpDownTV2.Value,
                                                      (double)numericUpDownTV3.Value});

            var cp = new CameraParameter((double)numericUpDownCPCx.Value,
                                         (double)numericUpDownCPCY.Value,
                                         (double)numericUpDownCPFX.Value,
                                         (double)numericUpDownCPFY.Value,
                                         (double)numericUpDownCPSkew.Value,
                                         _distCoeffs,
                                         rv,
                                         tv);

            var vp = new Vision.Positioning.CCIA(cp, 10)
            {
                InvertedX = false,
                InvertedY = true,
                WorldOffset = new PointF((float)numericUpDownOffsetX.Value,
                                         (float)-numericUpDownOffsetY.Value)
            };
            vp.ImageToWorld((int)numericUpDownConvPX.Value,
                            (int)numericUpDownConvPY.Value,
                            out var ax,
                            out var ay);
            numericUpDownConvAX.Value = (decimal)ax;
            numericUpDownConvAY.Value = (decimal)ay;
        }

        private void buttonPositioningCopy_Click(object sender, EventArgs e)
        {
            numericUpDownCPCx.Value = Decimal.Parse(textBoxCameraMatrix02.Text);
            numericUpDownCPCY.Value = Decimal.Parse(textBoxCameraMatrix12.Text);
            numericUpDownCPFX.Value = Decimal.Parse(textBoxCameraMatrix00.Text);
            numericUpDownCPFY.Value = Decimal.Parse(textBoxCameraMatrix11.Text);
            numericUpDownCPSkew.Value = Decimal.Parse(textBoxCameraMatrix01.Text);

            var rv = _rvec.ToArray();
            numericUpDownRV1.Value = Convert.ToDecimal(rv[0]);
            numericUpDownRV2.Value = Convert.ToDecimal(rv[1]);

            var tv = _tvec.ToArray();
            numericUpDownTV1.Value = Convert.ToDecimal(tv[0]);
            numericUpDownTV2.Value = Convert.ToDecimal(tv[1]);
            numericUpDownTV3.Value = Convert.ToDecimal(tv[2]);
        }

        #endregion Positioning

        #region IDS Camera

        private Vision.IDS.IDSCamera _idsCamera;

        private void buttonIdsConnection_Click(object sender, EventArgs e)
        {
            if (_idsCamera == null)
            {
                _idsCamera = new IDS.IDSCamera(_messageHandler);
            }

            if (_idsCamera.Connected)
            {
                // 進行斷線。
                if (_idsCamera.Disconnect())
                {
                    _messageHandler.Show("Disconnected.");
                    buttonIdsConnection.Text = "Connect";
                    buttonIdsGetImage.Enabled = false;
                    buttonIdsCameraSetting.Enabled = false;

                    _idsCamera = null;
                }
            }
            else
            {
                // 進行連線。
                if (_idsCamera.Connect())
                {
                    _messageHandler.Show("Connected.");
                    buttonIdsConnection.Text = "Disconnect";
                    buttonIdsGetImage.Enabled = true;
                    buttonIdsCameraSetting.Enabled = true;
                }
            }
        }

        private void buttonIdsGetImage_Click(object sender, EventArgs e)
        {
            var image = _idsCamera.GetImage();
            pictureBoxIds.Image = image;
        }

        private void buttonIdsCameraSetting_Click(object sender, EventArgs e)
        {
            _idsCamera.ShowSettingForm();
        }

        private void buttonLoadParameterFromFile_Click(object sender, EventArgs e)
        {
            _idsCamera.LoadParameterFromFile();
        }

        private void buttonLoadFromEEPROM_Click(object sender, EventArgs e)
        {
            _idsCamera.LoadParameterFromEEPROM();
        }

        private void buttonSaveToFile_Click(object sender, EventArgs e)
        {
            _idsCamera.SaveParameterToFile();
        }

        private void buttonSaveToEEPROM_Click(object sender, EventArgs e)
        {
            _idsCamera.SaveParameterToEEPROM();
        }

        #endregion IDS Camera

        #region Camera Calibration

        private VectorOfDouble _rvec = new VectorOfDouble();
        private VectorOfDouble _tvec = new VectorOfDouble();

        private Matrix<double> _cameraMatrix;
        private Matrix<double> _distCoeffs;

        private void pictureBoxCameraCalibratioin_MouseMove(object sender, MouseEventArgs e)
        {
            if (pictureBoxCameraCalibratioin.Image == null)
            {
                return;
            }

            var mousePosition = e.Location;
            var imageSize = pictureBoxCameraCalibratioin.Image.Size;
            var boxSize = pictureBoxCameraCalibratioin.Size;
            var pixelPoint = new PointF(mousePosition.X, mousePosition.Y);

            if (imageSize.Width / imageSize.Height <= boxSize.Width / boxSize.Height)
            {
                // The image is limited by the width.

                var scale = (float)boxSize.Width / (float)imageSize.Width;
                var blankPart = (boxSize.Height - scale * imageSize.Height) / 2.0;

                pixelPoint.Y -= (float)blankPart;

                pixelPoint.X /= scale;
                pixelPoint.Y /= scale;
            }
            else
            {
                // The image is limited by the hight.

                var scale = (float)boxSize.Height / (float)imageSize.Height;
                var blankPart = (boxSize.Width - (imageSize.Width * scale)) / 2.0;

                pixelPoint.X -= (float)blankPart;

                pixelPoint.X /= scale;
                pixelPoint.Y /= scale;
            }

            textBoxMousePosition.Text = $"X: {pixelPoint.X}, Y: {pixelPoint.Y}";

            //var scaledImg = new Bitmap(pictureBoxCameraCalibratioin.Image);
            //var cent = new Point(0, 0);
            //if (pixelPoint.X - 25 <= 0)
            //{
            //    cent.X = (int)pixelPoint.X;
            //}
            //else if (pixelPoint.X + 25 >= imageSize.Width)
            //{
            //    cent.X = (int)pixelPoint.X - 50;
            //}
            //else
            //{
            //    cent.X = (int)pixelPoint.X - 25;
            //}

            //if (pixelPoint.Y - 25 <= 0)
            //{
            //    cent.Y = (int)pixelPoint.Y + 50;
            //}
            //else if (pixelPoint.Y + 25 >= imageSize.Height)
            //{
            //    cent.Y = (int)pixelPoint.Y + 50;
            //}
            //else
            //{
            //    cent.Y = (int)pixelPoint.Y - 25;
            //}
            //scaledImg = scaledImg.Clone(new Rectangle(cent, new Size(50, 50)), scaledImg.PixelFormat);
            //pictureBoxCameraCalibratioinScale.Image = scaledImg;
        }

        private void buttonCameraCalibrate_Click(object sender, EventArgs e)
        {
            var checkBoardSize = new Size((int)numericUpDownCheckBoardX.Value, (int)numericUpDownCheckBoardY.Value);
            var cc = new Vision.CameraCalibration(checkBoardSize, (float)numericUpDownCheckBoardSideLength.Value);

            var error = cc.Run(out _cameraMatrix, out _distCoeffs, out var rvs, out var tvs);

            _rvec = rvs[0];
            _tvec = tvs[0];

            textBoxCameraCalibrationError.Text = error.ToString();

            textBoxCameraMatrix00.Text = _cameraMatrix.Data[0, 0].ToString();
            textBoxCameraMatrix01.Text = _cameraMatrix.Data[0, 1].ToString();
            textBoxCameraMatrix02.Text = _cameraMatrix.Data[0, 2].ToString();
            textBoxCameraMatrix10.Text = _cameraMatrix.Data[1, 0].ToString();
            textBoxCameraMatrix11.Text = _cameraMatrix.Data[1, 1].ToString();
            textBoxCameraMatrix12.Text = _cameraMatrix.Data[1, 2].ToString();
            textBoxCameraMatrix20.Text = _cameraMatrix.Data[2, 0].ToString();
            textBoxCameraMatrix21.Text = _cameraMatrix.Data[2, 1].ToString();
            textBoxCameraMatrix22.Text = _cameraMatrix.Data[2, 2].ToString();

            var cornersText = "";
            for (int row = 0; row < checkBoardSize.Height - 1; row++)
            {
                for (int col = 0; col < checkBoardSize.Width - 1; col++)
                {
                    var index = row * (checkBoardSize.Width - 1) + col;
                    var corner = cc.AllCorners[0].ToArray()[index];
                    cornersText += $"[{row}-{col}] X:{Math.Round(corner.X)}, Y:{Math.Round(corner.Y)}\r\n";
                }
                cornersText += "\r\n";
            }
            textBoxCorners.Text = cornersText;

            pictureBoxCameraCalibratioin.Image = cc.DrawCheckBoardImage(checkBoxDistort.Checked).ToBitmap();
        }

        #endregion Camera Calibration
    }
}