﻿using System.Windows.Forms;

namespace RASDK.Vision.IDS
{
    public class IControl : UserControl
    {
        protected uEye.Camera m_Camera;

        public IControl()
        { }

        public IControl(uEye.Camera camera)
        {
            m_Camera = camera;
        }

        public virtual void OnControlFocusActive()
        { }

        public virtual void OnControlFocusLost()
        { }

        public void SetCameraObject(uEye.Camera camera)
        {
            m_Camera = camera;
        }
    }
}