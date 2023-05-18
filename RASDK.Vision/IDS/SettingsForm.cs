﻿using System;
using System.Windows.Forms;

namespace RASDK.Vision.IDS
{
    public class ListViewControlItem : ListViewItem
    {
        public object Value { get; set; }
    }

    public partial class SettingsForm : Form
    {
        private IControl m_ActiveControl = null;
        private uEye.Camera m_Camera;
        private FormatControl m_FormatControl;
        private SizeControl m_SizeControl;

        public SettingsForm(uEye.Camera camera)
        {
            InitializeComponent();

            m_Camera = camera;

            m_FormatControl = new FormatControl(camera);
            m_SizeControl = new SizeControl(camera);
        }

        public FormatControl FormatControl
        {
            get { return m_FormatControl; }
        }

        public SizeControl SizeControl
        {
            get { return m_SizeControl; }
        }

        private void InitSettingsList()
        {
            ListViewControlItem item;

            item = new ListViewControlItem();
            item.Text = "Timing               ";
            item.Value = new CameraControl(m_Camera);
            item.ImageIndex = 0;

            listViewSettings.Items.Add(item);

            item = new ListViewControlItem();
            item.Text = "Picture                    ";
            item.Value = new PictureControl(m_Camera);
            item.ImageIndex = 1;
            listViewSettings.Items.Add(item);

            item = new ListViewControlItem();
            item.Text = "Size                     ";
            item.Value = m_SizeControl;
            item.ImageIndex = 2;
            listViewSettings.Items.Add(item);

            item = new ListViewControlItem();
            item.Text = "Format                         ";
            item.Value = m_FormatControl;
            item.ImageIndex = 3;
            listViewSettings.Items.Add(item);

            listViewSettings.Items[0].Selected = true;
            listViewSettings.FullRowSelect = true;
            listViewSettings.HideSelection = false;
        }

        private void listViewSettings_SelectedIndexChanged(object sender, EventArgs e)
        {
            if (m_ActiveControl != null)
            {
                m_ActiveControl.OnControlFocusLost();
                m_ActiveControl = null;
            }

            if (listViewSettings.SelectedItems.Count != 0)
            {
                m_ActiveControl = (IControl)(listViewSettings.SelectedItems[0] as ListViewControlItem).Value;

                if (m_ActiveControl != null)
                {
                    splitContainerMain.Panel2.Controls.Clear();
                    splitContainerMain.Panel2.Controls.Add(m_ActiveControl);
                    m_ActiveControl.Dock = DockStyle.Fill;

                    m_ActiveControl.OnControlFocusActive();
                }
            }
        }

        private void SettingsForm_Load(object sender, EventArgs e)
        {
            InitSettingsList();
        }
    }
}