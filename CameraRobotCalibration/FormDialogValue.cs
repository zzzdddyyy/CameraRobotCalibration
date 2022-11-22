using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace CameraRobotCalibration
{
    public partial class FormDialogValue : Form
    {
        private double orignalValue;
        public FormDialogValue()
        {
            InitializeComponent();
        }
        public FormDialogValue( double value)
        {
            InitializeComponent();
            orignalValue = value;
            txtValue.Text = value.ToString();
        }

        public double ReturnValue;
        private void button1_Click(object sender, EventArgs e)
        {
            try
            {
                ReturnValue = double.Parse(txtValue.Text);
                label1.ForeColor = Color.Green;
                label1.Text = "修改完成，关闭生效！";
            }
            catch (Exception)
            {

            }
           
        }

        private void button2_Click(object sender, EventArgs e)
        {
            this.Close();
        }
    }
}
