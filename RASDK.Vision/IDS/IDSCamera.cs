using Microsoft.Win32;
using RASDK.Basic;
using RASDK.Basic.Message;
using System;
using System.Collections.ObjectModel;
using System.Drawing;
using System.Windows.Forms;
using uEye;
using uEye.Defines;
using DisplayMode = uEye.Defines.DisplayMode;

namespace RASDK.Vision.IDS
{
    /// <summary>
    /// iDS工業攝影機。
    /// </summary>
    public class IDSCamera : IDevice
    {
        private const int _cnNumberOfSeqBuffers = 3;
        private readonly MessageHandler _message;
        private readonly Timer _updateTimer;
        private Camera _camera;
        private PictureBox _pictureBox;

        /// <summary>
        /// iDS工業攝影機。
        /// </summary>
        /// <param name="message"></param>
        public IDSCamera(MessageHandler message = null)
        {
            _message = message ?? new GeneralMessageHandler(new EmptyLogHandler());

            if (CheckRuntimeVersion())
            {
                DeviceId = 1;
                CameraId = 1;
                IsLive = false;
                RenderMode = DisplayRenderMode.FitToWindow;

                _updateTimer = new Timer();
                _updateTimer.Interval = 100;
                _updateTimer.Tick += UpdateControls;
            }
            else
            {
                _message.Show(".NET Runtime Version 3.5.0 is required", LoggingLevel.Error);
            }
        }

        /// <summary>
        /// iDS工業攝影機。
        /// </summary>
        /// <param name="message"></param>
        /// <param name="pictureBox"></param>
        public IDSCamera(MessageHandler message, PictureBox pictureBox)
        {
            _message = message;

            if (CheckRuntimeVersion())
            {
                _pictureBox = pictureBox;
                _pictureBox.SizeMode = PictureBoxSizeMode.CenterImage;

                DeviceId = 1;
                CameraId = 1;
                IsLive = false;
                RenderMode = DisplayRenderMode.FitToWindow;

                _updateTimer = new Timer();
                _updateTimer.Interval = 100;
                _updateTimer.Tick += UpdateControls;
            }
            else
            {
                _message.Show(".NET Runtime Version 3.5.0 is required", LoggingLevel.Error);
            }
        }

        ~IDSCamera()
        {
            try
            {
                Disconnect();
            }
            catch
            { /* Do nothing. */ }
        }

        public int CameraId { get; private set; }
        public int DeviceId { get; private set; }
        public string Failed { get; private set; }
        public double Fps { get; private set; }
        public Int32 FrameCount { get; private set; }
        public Boolean IsLive { get; private set; }
        public DisplayRenderMode RenderMode { get; private set; }
        public string SensorName { get; private set; }

        #region - Dis/Connect, Init -

        public bool Connected
        {
            get
            {
                if (_camera != null)
                {
                    return _camera.IsOpened;
                }

                return false;
            }
        }

        public bool Connect()
        {
            if (_camera == null)
            {
                Init();
            }

            var status = ChangeCaptureMode(CaptureMode.FreeRun);
            if (!status && _camera.IsOpened)
            {
                _camera.Exit();
            }
            else
            {
                AutoGain = true;
                AutoShutter = true;
                AutoWhiteBalance = true;
            }

            return _camera.IsOpened;
        }

        public bool Disconnect()
        {
            _updateTimer.Stop();
            IsLive = false;

            _camera.EventFrame -= FrameEvent;
            _camera.Exit();

            if (_pictureBox != null)
            {
                _pictureBox.Invalidate();
                _pictureBox.SizeMode = PictureBoxSizeMode.CenterImage;
            }
            RenderMode = DisplayRenderMode.FitToWindow;

            return !_camera.IsOpened;
        }

        /// <summary>
        /// 相機初始化。
        /// </summary>
        /// <returns></returns>
        public bool Init()
        {
            if (_camera == null)
            {
                _camera = new Camera();
            }

            var id = DeviceId | (Int32)DeviceEnumeration.UseDeviceID;

            var status = _pictureBox == null ? _camera.Init(id) : _camera.Init(id, _pictureBox.Handle);
            if (status != Status.SUCCESS)
            {
                _message.Show("Initializing the camera failed", LoggingLevel.Error);
                return false;
            }

            status = MemoryHelper.AllocImageMems(_camera, _cnNumberOfSeqBuffers);
            if (status != Status.SUCCESS)
            {
                _message.Show("Allocating memory failed", LoggingLevel.Error);
                return false;
            }

            status = MemoryHelper.InitSequence(_camera);
            if (status != Status.SUCCESS)
            {
                _message.Show("Add to sequence failed", LoggingLevel.Error);
                return false;
            }

            _camera.EventFrame += FrameEvent;
            FrameCount = 0;
            _updateTimer.Start();
            _camera.Information.GetSensorInfo(out var sensorInfo);
            SensorName = sensorInfo.SensorName;
            return true;
        }

        #endregion - Dis/Connect, Init -

        #region - General Feature -

        /// <summary>
        /// 改變相機截取模式。
        /// </summary>
        /// <param name="captureMode"></param>
        /// <returns></returns>
        /// <exception cref="ArgumentException"></exception>
        public bool ChangeCaptureMode(CaptureMode captureMode)
        {
            Func<Status> func;
            bool expectIsLive;

            switch (captureMode)
            {
                case CaptureMode.FreeRun:
                    func = _camera.Acquisition.Capture;
                    expectIsLive = true;
                    break;

                case CaptureMode.Stop:
                    func = _camera.Acquisition.Stop;
                    expectIsLive = false;
                    break;

                case CaptureMode.Snapshot:
                    func = _camera.Acquisition.Freeze;
                    expectIsLive = false;
                    break;

                default:
                    throw new ArgumentException("錯誤的 iDS CaptureMode。");
            }

            var status = func();
            if (status != Status.SUCCESS)
            {
                _message.Show($"Change iDS camera capture mode failed.\r\n{status}", LoggingLevel.Error);
                return false;
            }
            else
            {
                // Everything is ok.
                IsLive = expectIsLive;
            }
            return true;
        }

        /// <summary>
        /// 讀取影像。
        /// </summary>
        /// <returns>Image.</returns>
        public Bitmap GetImage()
        {
            Bitmap img = null;
            if (_camera != null)
            {
                _camera.Memory.GetLast(out int memoryId);
                _camera.Memory.SetActive(memoryId);
                _camera.Acquisition.Freeze();
                _camera.Memory.ToBitmap(memoryId, out img);
            }
            return img;
        }

        /// <summary>
        /// 設定感興趣區域。
        /// </summary>
        /// <param name="width"></param>
        /// <param name="height"></param>
        /// <param name="x"></param>
        /// <param name="y"></param>
        public void SetAoiSize(int width, int height, int x = 0, int y = 0)
        {
            _camera.Size.AOI.Set(x, y, width, height);
        }

        /// <summary>
        /// 儲存參數到EEPROM。
        /// </summary>
        public void SaveParameterToEEPROM()
        {
            SaveParameter(null);
        }

        /// <summary>
        /// 儲存參數爲檔案。
        /// </summary>
        /// <param name="filename"></param>
        public void SaveParameterToFile(string filename = "")
        {
            SaveParameter(filename);
        }

        /// <summary>
        /// 從EEPROM載入參數。
        /// </summary>
        public void LoadParameterFromEEPROM()
        {
            LoadParameter(null);
        }

        /// <summary>
        /// 從檔案載入參數。
        /// </summary>
        /// <param name="filename"></param>
        public void LoadParameterFromFile(string filename = "")
        {
            LoadParameter(filename);
        }

        private void SaveParameter(string filename = null)
        {
            if (filename == null)
            {
                _camera.Parameter.Save();
            }
            else
            {
                _camera.Parameter.Save(filename);
            }
        }

        private void LoadParameter(string filename = null)
        {
            if (IsLive)
            {
                _camera.Acquisition.Stop();
            }

            MemoryHelper.ClearSequence(_camera);
            MemoryHelper.FreeImageMems(_camera);

            if (filename == null)
            {
                _camera.Parameter.Load();
            }
            else
            {
                _camera.Parameter.Load(filename);
            }

            _camera.PixelFormat.Get(out _);

            // Allocate new standard memory.
            MemoryHelper.AllocImageMems(_camera, _cnNumberOfSeqBuffers);
            MemoryHelper.InitSequence(_camera);

            if (IsLive)
            {
                _camera.Acquisition.Capture();
            }
        }

        #endregion - General Feature -

        #region - Auto Features -

        private delegate Status GetFunc(out bool enable);

        /// <summary>
        /// 自動設定增益。
        /// </summary>
        public bool AutoGain
        {
            get => GetAutoFeatures(_camera.AutoFeatures.Software.Gain.GetEnable);
            set => _camera.AutoFeatures.Software.Gain.SetEnable(value);
        }

        /// <summary>
        /// 自動設定快門。
        /// </summary>
        public bool AutoShutter
        {
            get => GetAutoFeatures(_camera.AutoFeatures.Software.Shutter.GetEnable);
            set => _camera.AutoFeatures.Software.Shutter.SetEnable(value);
        }

        /// <summary>
        /// 自動設定白平衡。
        /// </summary>
        public bool AutoWhiteBalance
        {
            get => GetAutoFeatures(_camera.AutoFeatures.Software.WhiteBalance.GetEnable);
            set => _camera.AutoFeatures.Software.WhiteBalance.SetEnable(value);
        }

        private bool GetAutoFeatures(GetFunc func)
        {
            func(out bool enable);
            return enable;
        }

        #endregion - Auto Features -

        #region - Form -

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

        /// <summary>
        /// 顯示相機參數設定視窗。
        /// </summary>
        public void ShowSettingForm()
        {
            if (_camera != null)
            {
                var settingForm = new SettingsForm(_camera);
                settingForm.SizeControl.AOIChanged += OnDisplayChanged;
                settingForm.FormatControl.DisplayChanged += OnDisplayChanged;
                settingForm.ShowDialog();
            }
            else
            {
                _message.Show("Camera never initialization.", LoggingLevel.Warn);
            }
        }

        #endregion - Form -

        #region - .NET Version -

        private bool CheckRuntimeVersion()
        {
            var versionMin = new Version(3, 5);
            var ok = false;

            foreach (Version version in InstalledDotNetVersions())
            {
                if (version >= versionMin)
                {
                    ok = true;
                    break;
                }
            }

            return ok;
        }

        private void GetDotNetVersion(RegistryKey parentKey,
                                      string subVersionName,
                                      Collection<Version> versions)
        {
            if (parentKey != null)
            {
                string installed = Convert.ToString(parentKey.GetValue("Install"));
                if (installed == "1")
                {
                    string version = Convert.ToString(parentKey.GetValue("Version"));
                    if (string.IsNullOrEmpty(version))
                    {
                        if (subVersionName.StartsWith("v"))
                            version = subVersionName.Substring(1);
                        else
                            version = subVersionName;
                    }

                    Version ver = new Version(version);

                    if (!versions.Contains(ver))
                        versions.Add(ver);
                }
            }
        }

        private Collection<Version> InstalledDotNetVersions()
        {
            var versions = new Collection<Version>();
            var NDPKey = Registry.LocalMachine.OpenSubKey(@"SOFTWARE\Microsoft\NET Framework Setup\NDP");
            if (NDPKey != null)
            {
                string[] subkeys = NDPKey.GetSubKeyNames();
                foreach (string subkey in subkeys)
                {
                    GetDotNetVersion(NDPKey.OpenSubKey(subkey), subkey, versions);
                    GetDotNetVersion(NDPKey.OpenSubKey(subkey).OpenSubKey("Client"), subkey, versions);
                    GetDotNetVersion(NDPKey.OpenSubKey(subkey).OpenSubKey("Full"), subkey, versions);
                }
            }
            return versions;
        }

        #endregion - .NET Version -

        #region - Events -

        private void FrameEvent(object sender, EventArgs e)
        {
            var camera = sender as Camera;
            if (camera.IsOpened)
            {
                DisplayMode mode;
                camera.Display.Mode.Get(out mode);

                // Only display in DiB mode.
                if (mode == DisplayMode.DiB)
                {
                    Int32 memId;
                    var status = camera.Memory.GetLast(out memId);
                    if ((status == Status.SUCCESS) && 0 < memId)
                    {
                        if (camera.Memory.Lock(memId) == Status.SUCCESS)
                        {
                            camera.Display.Render(memId, RenderMode);
                            camera.Memory.Unlock(memId);
                        }
                    }
                }
                ++FrameCount;
            }
        }

        private void OnDisplayChanged(object sender, EventArgs e)
        {
            DisplayMode displayMode;
            _camera.Display.Mode.Get(out displayMode);

            // set scaling options
            if (displayMode != DisplayMode.DiB)
            {
                if (RenderMode == DisplayRenderMode.DownScale_1_2)
                {
                    RenderMode = DisplayRenderMode.Normal;

                    _pictureBox.Anchor = AnchorStyles.Top | AnchorStyles.Left;

                    // get image size
                    Rectangle rect;
                    _camera.Size.AOI.Get(out rect);

                    _pictureBox.Width = rect.Width;
                    _pictureBox.Height = rect.Height;
                }
                else
                {
                    _camera.DirectRenderer.SetScaling(RenderMode == DisplayRenderMode.FitToWindow);
                }
            }
            else
            {
                if (RenderMode != DisplayRenderMode.FitToWindow)
                {
                    _pictureBox.Anchor = AnchorStyles.Top | AnchorStyles.Left;

                    // get image size
                    Rectangle rect;
                    _camera.Size.AOI.Get(out rect);

                    if (RenderMode != DisplayRenderMode.Normal)
                    {
                        _pictureBox.Width = rect.Width / 2;
                        _pictureBox.Height = rect.Height / 2;
                    }
                    else
                    {
                        _pictureBox.Width = rect.Width;
                        _pictureBox.Height = rect.Height;
                    }
                }
            }
        }

        private void UpdateControls(object sender, EventArgs e)
        {
            _camera.Timing.Framerate.GetCurrentFps(out var frameRate);
            Fps = frameRate;

            _camera.Information.GetCaptureStatus(out var captureStatus);
            if (null != captureStatus)
            {
                Failed = "" + captureStatus.Total;
            }
        }

        #endregion - Events -
    }
}