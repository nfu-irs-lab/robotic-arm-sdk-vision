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
                                         new double[]
                                         {
                                             (double)numericUpDownRV1.Value,
                                             (double)numericUpDownRV2.Value,
                                             (double)numericUpDownRV3.Value
                                         },
                                         new double[]
                                         {
                                             (double)numericUpDownTV1.Value,
                                             (double)numericUpDownTV2.Value,
                                             (double)numericUpDownTV3.Value
                                         });

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
    }
}