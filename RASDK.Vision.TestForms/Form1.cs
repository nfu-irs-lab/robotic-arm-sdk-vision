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
            var cp = new CameraParameter((double)numericUpDownCPCx.Value,
                                         (double)numericUpDownCPCY.Value,
                                         (double)numericUpDownCPFX.Value,
                                         (double)numericUpDownCPFY.Value,
                                         (double)numericUpDownCPSkew.Value,
                                         _distCoeffs,
                                         _rvecs[0],
                                         _tvecs[0]);

            var vp = new Vision.Positioning.CCIA(cp, TF) { AllowableError = 10 };
            vp.ImageToArm((int)numericUpDownConvPX.Value,
                          (int)numericUpDownConvPY.Value,
                          out var ax,
                          out var ay);
            numericUpDownConvAX.Value = (decimal)ax;
            numericUpDownConvAY.Value = (decimal)ay;
        }

        private void TF(double vx, double vy, out double ax, out double ay)
        {
            ax = vx + (double)numericUpDownOffsetX.Value;
            ay = vy + (double)numericUpDownOffsetY.Value;
        }

        private void buttonPositioningCopy_Click(object sender, EventArgs e)
        {
            numericUpDownCPCx.Value = Decimal.Parse(textBoxCameraMatrix02.Text);
            numericUpDownCPCY.Value = Decimal.Parse(textBoxCameraMatrix02.Text);
            numericUpDownCPFX.Value = Decimal.Parse(textBoxCameraMatrix00.Text);
            numericUpDownCPFY.Value = Decimal.Parse(textBoxCameraMatrix11.Text);
            numericUpDownCPSkew.Value = Decimal.Parse(textBoxCameraMatrix01.Text);
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
                if (_idsCamera.Disconnect())
                {
                    _messageHandler.Show("Disconnected.");
                    buttonIdsConnection.Text = "Connect";
                    buttonIdsGetImage.Enabled = false;
                    buttonIdsCameraSetting.Enabled = false;
                }
            }
            else
            {
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

        #endregion IDS Camera

        #region Camera Calibration

        private Mat[] _rvecs;
        private Mat[] _tvecs;
        private Matrix<double> _cameraMatrix;
        private Matrix<double> _distCoeffs;

        private void buttonCameraCalibrate_Click(object sender, EventArgs e)
        {
            var checkBoardSize = new Size((int)numericUpDownCheckBoardX.Value, (int)numericUpDownCheckBoardY.Value);
            var cc = new Vision.CameraCalibration(checkBoardSize, (float)numericUpDownCheckBoardSideLength.Value);

            var error = cc.Run(out _cameraMatrix, out _distCoeffs, out _rvecs, out _tvecs);

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

            pictureBoxCameraCalibratioin.Image = cc.DrawedImage.ToBitmap();
        }

        #endregion Camera Calibration
    }
}