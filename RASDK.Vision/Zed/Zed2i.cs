using RASDK.Basic;
using sl;
using System;
using Camera = sl.Camera;

namespace RASDK.Vision.Zed
{
    /// <summary>
    /// Zed2i,3D攝影機
    /// </summary>
    public class Zed2i : IDevice
    {
        private readonly Camera _camera;
        private InitParameters initParameters;
        private RuntimeParameters runtimeParameters;
        private uint mmWidth;
        private uint mmHeight;

        public Zed2i(InitParameters initParameters = null,RuntimeParameters runtimeParameters=null)
        {
            _camera = new Camera(0);
            this.initParameters = initParameters ?? new InitParameters()//lazy initialization
            {
                resolution = RESOLUTION.HD2K,
                cameraFPS = 15,
                coordinateUnits = UNIT.MILLIMETER,
                depthMode = DEPTH_MODE.ULTRA,
            };
            this.runtimeParameters = runtimeParameters ?? new RuntimeParameters();
            mmWidth =(uint) _camera.ImageWidth;
            mmHeight =(uint) _camera.ImageHeight;
        }

        public int CameraId { get; private set; }
        public int DeviceId { get; private set; }
        public InitParameters InitParameters
        { get { return initParameters; } }

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

        #endregion aboutConnect

        #region General Feature

        /// <summary>
        /// MAT_TYPE
        /// </summary>
        public enum ImageType
        {
            ColorLeft,
            ColorRight,
            Depth,
            Gray,
            SideBySide
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
            sl.Camera _camera = new sl.Camera(0);
            if (_camera.Open(ref initParameters) != ERROR_CODE.SUCCESS)
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

                case ImageType.SideBySide:
                    view = VIEW.SIDE_BY_SIDE;
                    matType=MAT_TYPE.MAT_8U_C4;
                    break;

                default:
                    throw new ArgumentException("錯誤的 Zed ImageType");
            }


            //確認相機是否可以回傳影像

            sl.Mat mat = new sl.Mat();

            mat.Create(mmWidth, mmHeight, matType, MEM.GPU);

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

        /// <summary>
        /// 獲取影像的深度資訊，xPixel、yPixel大小由拍照畫質決定
        /// 2K=2208*1242
        /// </summary>
        /// <param name="xPixelLength"></param>
        /// <param name="yPixelLength"></param>
        /// <returns></returns>
        /// <exception cref="Exception"></exception>
        public float GetDepthInfo(int xPixelLength = 2208, int yPixelLength = 1242)
        {

            if (_camera == null)
            {
                throw new Exception("Camera should not be null");
            }

            if (xPixelLength > mmWidth)
            {
                throw new Exception("xPiexl Maximum=" + mmWidth);
            }

            if (yPixelLength > mmHeight)
            {
                throw new Exception("yPiexl Maximum=" + mmHeight);
            }


            if (_camera.Open(ref initParameters) != ERROR_CODE.SUCCESS)
            {
                Environment.Exit(-1);
            }
            Mat depthImage = new Mat();
            depthImage.Create(mmWidth, mmHeight, MAT_TYPE.MAT_32F_C1, MEM.CPU);


            if (_camera.Grab(ref runtimeParameters) == ERROR_CODE.SUCCESS)
            {
                _camera.RetrieveMeasure(depthImage, MEASURE.DEPTH);
            }

            var err = depthImage.GetValue(xPixelLength, yPixelLength, out float depthValue);
            if (err == ERROR_CODE.SUCCESS)
            {
                if (float.IsNaN(depthValue))
                {
                    return 0;
                }
                //尚未解決出現無限符號
                return depthValue;

            }
            else
            {
                throw new Exception("GetValueErrorCode=" + err.ToString());
            }
            return -1;

        }

        #endregion General Feature

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

        #endregion Auto Features/Set VideoFeature

    }
}