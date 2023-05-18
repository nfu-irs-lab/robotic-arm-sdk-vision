# Robotic Arm SDK Vision

[![GitHub release](https://img.shields.io/github/release/nfu-irs-lab/robotic-arm-sdk-vision.svg)](https://github.com/nfu-irs-lab/robotic-arm-sdk-vision/releases)
[![GitHub issues](https://img.shields.io/github/issues/nfu-irs-lab/robotic-arm-sdk-vision.svg)](https://github.com/nfu-irs-lab/robotic-arm-sdk-vision/issues)

機械手臂 SDK 的影像功能。目前主要支援：
- IDS 工業攝影機
- Zed2i 3D攝影機

支援的環境為 .NET Framework 4.7.2 (64-bit)

> Forked from [RASDK 0.2.0](https://github.com/nfu-irs-lab/robotic-arm-sdk/releases/tag/v0.2.0)

# 各 Project 功能

- [RASDK.Vision](/RASDK.Vision)：影像及視覺相關函式庫。
- [RASDK.Vision.TestForms](/RASDK.Vision.TestForms)：「RASDK.Vision」的測試用視窗程式。不會封裝進 NuGet。

# 未處理例外狀況
'Emgu.CV.Util.VectorOfDouble' 的類型初始設定式發生例外狀況。'
- 到實驗室NAS->public->PackageReferences_X64
- 找到「cvextern.dll」
- 將此檔案複製貼到執行目錄下的RASDK.Vision.TestForms->bin->X64->Debug內
- 使用Zed2i時需要下載官方Nuget，及該執行電腦需有官方SDK，兩者版本須相互匹配否則無法開啟相機。
