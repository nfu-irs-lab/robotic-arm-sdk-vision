using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.DepthAI;
using Emgu.CV.ML;
using RASDK.Basic;
using RASDK.Vision.IDS;
using sl;
using System;
using System.Collections.Generic;
using System.Diagnostics.Eventing.Reader;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using Camera = sl.Camera;

namespace RASDK.Vision.Zed
{
    /// <summary>
    /// Zed2i,3D攝影機
    /// </summary>
    public class Zed2i : IDevice
    {
        private readonly Camera _camera;
        private  InitParameters initParameters;
        private  uint mmWidth;
        private  uint mmHeight;

        public Zed2i(InitParameters initParameters = null)
        {
            _camera = new Camera(0);
            this.initParameters = initParameters ?? new InitParameters()//lazy initialization
            {
                resolution = RESOLUTION.HD2K,
                cameraFPS = 15,
                coordinateUnits = UNIT.MILLIMETER,
                depthMode = DEPTH_MODE.ULTRA,

            };
        }


        public int CameraId { get; private set; }
        public int DeviceId { get; private set; }
        public InitParameters InitParameters { get { return initParameters; } }

        #region aboutConnect
        public bool Connected => _camera.IsOpened();

        public bool Connect()
        {
            var initParam = initParameters;
            ERROR_CODE err = _camera.Open(ref initParameters);
            if (err != ERROR_CODE.SUCCESS)
            {
                Environment.Exit(-1);
                throw new Exception(err.ToString());
            }
            else
            {
                mmWidth = (uint)_camera.ImageWidth;
                mmHeight = (uint)_camera.ImageHeight;
                return true;
            }

        }

        public bool Disconnect()
        {
            try
            {
                _camera.Close();
                return true;
            }
            catch (Exception e)
            {
                throw new Exception(e.ToString());
            }
        }
        #endregion

        #region General Feature
        /// <summary>
        /// MAT_TYPE
        /// </summary>
        public enum ImageType
        {
            ColorLeft,
            ColorRight,
            Depth,
            Gray
        }

        /// <summary>
        /// 返回一Image<Bgr,byte>，使用Tobitmap()輸出至picturebox
        /// 設定RGB或Depth的初始通道，預設為ColorLeft
        /// </summary>
        /// <param name="mat">From ZedSDK</param>
        /// <param name="view">From ZedSDK</param>
        /// <returns></returns>
        /// <exception cref="Exception"></exception>

        public Emgu.CV.Mat GetImage(ImageType imageType)
        {
            InitParameters init = new InitParameters();
            sl.Camera _camera = new sl.Camera(0);
            if (_camera.Open(ref init) != ERROR_CODE.SUCCESS)
            {
                Environment.Exit(-1);
            }

            VIEW view;
            MAT_TYPE matType;

            switch (imageType)
            {
                case ImageType.ColorLeft:
                    view = VIEW.LEFT;
                    matType = MAT_TYPE.MAT_8U_C4;
                    break;

                case ImageType.ColorRight:
                    view = VIEW.RIGHT;
                    matType = MAT_TYPE.MAT_8U_C4;
                    break;

                case ImageType.Depth:
                    view = VIEW.DEPTH;
                    matType = MAT_TYPE.MAT_32F_C1;
                    break;
                case ImageType.Gray:
                    view = VIEW.LEFT_GREY;
                    matType = MAT_TYPE.MAT_8U_C1;
                    break;
                default:
                    throw new ArgumentException("錯誤的 Zed ImageType");        
            }



            //確認相機是否可以回傳影像

            sl.Mat mat = new sl.Mat();

            mat.Create(mmWidth, mmHeight, matType, MEM.CPU);

            RuntimeParameters runtimeParameters = new RuntimeParameters();
            if (_camera.Grab(ref runtimeParameters) != ERROR_CODE.SUCCESS)
            { throw new Exception(); }

            if (_camera.RetrieveImage(mat, view) != ERROR_CODE.SUCCESS)
            { throw new Exception(); }

            Emgu.CV.Mat image = new Emgu.CV.Mat(
                mat.GetHeight(),
                mat.GetWidth(),
                Emgu.CV.CvEnum.DepthType.Cv8U,
                mat.GetChannels(),
                mat.GetPtr(),
                0);
            return image; 
        }

        #endregion

        #region Auto Features/Set VideoFeature
        /// <summary>
        /// 如果對Gain與Exposure指定值，會將AEC_AGC設為0
        /// </summary>
        public int AutoGainAndExposure
        {
            get => _camera.GetCameraSettings(VIDEO_SETTINGS.AEC_AGC);
            set => _camera.SetCameraSettings(VIDEO_SETTINGS.AEC_AGC, value);
        }

        /// <summary>
        /// 自動設定白平衡，value=-1為自動。
        /// </summary>
        public bool AutoWhiteBalance
        {
            get
            {
                var state = _camera.GetCameraSettings(VIDEO_SETTINGS.WHITEBALANCE_AUTO);
                if (state == -1)
                    return true;
                else
                    return false;
            }
            set
            {
                if (value == true)
                {
                    _camera.SetCameraSettings(VIDEO_SETTINGS.WHITEBALANCE_AUTO, -1);
                }
                else
                {
                    _camera.SetCameraSettings(VIDEO_SETTINGS.WHITEBALANCE_AUTO, 0);
                }
            }

        }

        /// <summary>
        /// 範圍0~11
        /// </summary>
        public int SetHue
        {
            get => _camera.GetCameraSettings(VIDEO_SETTINGS.HUE);
            set => _camera.SetCameraSettings(VIDEO_SETTINGS.HUE, value);
        }

        #endregion

        #region Form
        /// <summary>
        /// 顯示選擇相機視窗。
        /// </summary>
        public void ChooseCamera()
        {
            var chooseForm = new CameraChoose();
            if (chooseForm.ShowDialog() == DialogResult.OK)
            {
                DeviceId = chooseForm.DeviceID;
                CameraId = chooseForm.CameraID;
            }
        }


        #endregion

        #region Event



        #endregion
    }
}
