using Emgu.CV.CvEnum;
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
        private Camera _camera;
        private InitParameters _initParameters;
        private RuntimeParameters _runtimeParameters;
        private readonly uint _mmWidth;
        private readonly uint _mmHeight;
        private sl.Mat _RGBimage;
        private sl.Mat _Depthimage;

        public Zed2i(InitParameters initParameters = null, RuntimeParameters runtimeParameters = null)
        {
            _camera = new Camera(0);
            _initParameters = initParameters ?? new InitParameters()//lazy initialization
            {
                resolution = RESOLUTION.HD2K,
                cameraFPS = 15,
                coordinateUnits = UNIT.MILLIMETER,
                depthMode = DEPTH_MODE.ULTRA,
                depthStabilization = 100
            };
            _runtimeParameters = runtimeParameters ?? new RuntimeParameters();
        }


        public int CameraId { get; private set; }
        public int DeviceId { get; private set; }
        public InitParameters InitParameters { get { return _initParameters; } }
        public RuntimeParameters RuntimeParameters { get { return _runtimeParameters; } }

        #region aboutConnect
        public bool Connected => _camera.IsOpened();

        public bool Connect()
        {
            var initParam = _initParameters;
            ERROR_CODE err = _camera.Open(ref _initParameters);
            if (err != ERROR_CODE.SUCCESS)
            {
                Environment.Exit(-1);
                throw new Exception(err.ToString());
            }
            else
            {
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
            DepthLeft,
            DepthRight
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
            //確認相機是否可以獲取影像
            RuntimeParameters runTimeParam = _runtimeParameters;
            var GrabErr = _camera.Grab(ref runTimeParam);
            if (GrabErr != ERROR_CODE.SUCCESS)
            {
                throw new Exception(GrabErr.ToString());
            }

            MAT_TYPE matType = MAT_TYPE.MAT_8U_C4; ;
            VIEW view = VIEW.LEFT;

            switch (imageType)
            {
                case ImageType.ColorRight:
                    matType = MAT_TYPE.MAT_8U_C4;
                    view = VIEW.RIGHT;
                    break;

                case ImageType.DepthLeft:
                    matType = MAT_TYPE.MAT_32F_C1;
                    view = VIEW.DEPTH;
                    break;

                case ImageType.DepthRight:
                    matType = MAT_TYPE.MAT_32F_C1;
                    view = VIEW.DEPTH_RIGHT;
                    break;

                default:
                    throw new ArgumentException("錯誤的 Zed imageType");
            }

            //確認相機是否可以回傳影像
            sl.Mat mat = new sl.Mat();
            var RetrieveErr = _camera.RetrieveImage(mat, view);
            if (RetrieveErr != ERROR_CODE.SUCCESS)
            {
                throw new Exception(RetrieveErr.ToString());
            }

            uint mmWidth = (uint)_camera.ImageWidth;
            uint mmHeight = (uint)_camera.ImageHeight;

            mat.Create(mmWidth, mmHeight, matType, MEM.CPU);
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
                var state = _camera.GetCameraSettings(VIDEO_SETTINGS.AUTO_WHITEBALANCE);
                if (state == -1)
                    return true;
                else
                    return false;
            }
            set
            {
                if (value == true)
                {
                    _camera.SetCameraSettings(VIDEO_SETTINGS.AUTO_WHITEBALANCE, -1);
                }
                else
                {
                    _camera.SetCameraSettings(VIDEO_SETTINGS.AUTO_WHITEBALANCE, 0);
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
