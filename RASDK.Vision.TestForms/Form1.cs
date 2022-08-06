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
            //Application.Idle += UpdatePixelTracking;
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

            RASDK.Vision.Positioning.IVisionPositioning vp;
            if (radioButtonCCIA.Checked)
            {
                vp = new Vision.Positioning.CCIA(cp, 10)
                {
                    InvertedX = false,
                    InvertedY = true,
                    WorldOffset = new PointF((float)numericUpDownOffsetX.Value,
                                             (float)-numericUpDownOffsetY.Value)
                };
            }
            else if (radioButtonAdvHomography.Checked)
            {
                //var cc = new CameraCalibration(new Size((int)numericUpDownCheckBoardX.Value, (int)numericUpDownCheckBoardY.Value), (float)numericUpDownCheckBoardSideLength.Value);
                //    cc.Run(out _, out _, out _, out _);
                var worldPoints = new PointF[]
                {
                    new PointF((float)numericUpDownHWTLX.Value, (float)numericUpDownHWTLY.Value),
                    new PointF((float)numericUpDownHWTRX.Value, (float)numericUpDownHWTRY.Value),
                    new PointF((float)numericUpDownHWBLX.Value, (float)numericUpDownHWBLY.Value),
                    new PointF((float)numericUpDownHWBRX.Value, (float)numericUpDownHWBRY.Value)
                };
                vp = new Vision.Positioning.AdvancedHomographyPositioner(worldPoints,
                                                                         _cameraCalibration);

                (vp as Positioning.AdvancedHomographyPositioner).HomographyPositioner.SaveToCsv();

                var undistImg = _cameraCalibration.UndistortImage(_cameraCalibration.SourceImageRepresentative);
                undistImg.Save("AdvancedHomography_undistort.jpg");
            }
            else
            {
                throw new Exception();
            }
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
                    buttonIdsCapture.Enabled = false;
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
                    buttonIdsCapture.Enabled = true;
                    buttonIdsCameraSetting.Enabled = true;
                }
            }
        }

        private void buttonIdsGetImage_Click(object sender, EventArgs e)
        {
            var image = _idsCamera.GetImage();
            pictureBoxMain.Image = image;
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
        private VectorOfDouble _distCoeffs;

        private CameraCalibration _cameraCalibration;

        private PointF _mousePosition = new PointF(0, 0);

        private void pictureBoxMain_MouseMove(object sender, MouseEventArgs e)
        {
            // XXX
            return;

            if (pictureBoxMain.Image == null)
            {
                return;
            }

            var mousePosition = e.Location;
            var imageSize = pictureBoxMain.Image.Size;
            var boxSize = pictureBoxMain.Size;
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

            _mousePosition = pixelPoint;
            textBoxPixelPosition.Text = $"X: {pixelPoint.X}, Y: {pixelPoint.Y}";
        }

        private void UpdatePixelTracking(object sender, EventArgs args)
        {
            var srcImg = pictureBoxMain.Image;
            if (srcImg == null) return;

            var image = new Bitmap(srcImg);
            var boxSize = pictureBoxMain.Size;

            var cent = new Point(0, 0);
            if (_mousePosition.X - 25 <= 0)
            {
                cent.X = (int)_mousePosition.X;
            }
            else if (_mousePosition.X + 25 >= image.Size.Width)
            {
                cent.X = (int)_mousePosition.X - 50;
            }
            else
            {
                cent.X = (int)_mousePosition.X - 25;
            }

            if (_mousePosition.Y - 25 <= 0)
            {
                cent.Y = (int)_mousePosition.Y + 50;
            }
            else if (_mousePosition.Y + 25 >= image.Size.Height)
            {
                cent.Y = (int)_mousePosition.Y + 50;
            }
            else
            {
                cent.Y = (int)_mousePosition.Y - 25;
            }

            try
            {
                image = image.Clone(new Rectangle(cent, new Size(50, 50)), image.PixelFormat);
                pictureBoxSub.Image = image;
            }
            catch
            { }
        }

        private void buttonCameraCalibrate_Click(object sender, EventArgs e)
        {
            var checkBoardSize = new Size((int)numericUpDownCheckBoardX.Value, (int)numericUpDownCheckBoardY.Value);
            _cameraCalibration = new Vision.CameraCalibration(checkBoardSize, (float)numericUpDownCheckBoardSideLength.Value);

            var paths = SelectImagePaths();
            var images = new List<Image<Bgr, byte>>();
            foreach (var p in paths)
            {
                images.Add(new Image<Bgr, byte>(p));
            }

            var cp = _cameraCalibration.CalCameraParameter(images, out _cameraMatrix, out _distCoeffs, out var rvs, out var tvs, out var error);
            cp.SaveToCsv();

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
                    var corner = _cameraCalibration.AllCorners[0].ToArray()[index];
                    cornersText += $"[{row}-{col}] X:{Math.Round(corner.X)}, Y:{Math.Round(corner.Y)}\r\n";
                }
                cornersText += "\r\n";
            }
            textBoxCorners.Text = cornersText;

            UpdatePictureBox();
        }

        private void UpdatePictureBox()
        {
            pictureBoxMain.Image = _cameraCalibration.DrawCheckBoardImage(checkBoxDistort.Checked).ToBitmap();
        }

        private void checkBoxDistort_CheckedChanged(object sender, EventArgs e)
        {
            UpdatePictureBox();
        }

        #endregion Camera Calibration

        private string[] SelectImagePaths()
        {
            var dialog = new OpenFileDialog
            {
                Multiselect = true
            };

            if (dialog.ShowDialog() == DialogResult.OK)
            {
                return dialog.FileNames;
            }

            throw new Exception();
        }
    }
}