using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Xml.Linq;

namespace TisCameraRobotCalibration
{
    public partial class SysParamsForm : Form
    {
        public delegate void SysParamsChanged(float[] inMat,float[] exMat,float[] distor,float[] trans,string[] robotPrams);//初始化委托信息
        public event SysParamsChanged sysParamChanged;
        public SysParamsForm()
        {
            InitializeComponent();
        }

        private void SysParamsForm_Load(object sender, EventArgs e)
        {
            groupBox1.Enabled = false;
            groupBox2.Enabled = false;
            groupBox3.Enabled = false;
            groupBox4.Enabled = false;
            groupBox5.Enabled = false;
            btnConfirm.Enabled = false;
            LoadXML();
        }

        private void btnModify_Click(object sender, EventArgs e)
        {
            groupBox1.Enabled = true;
            groupBox2.Enabled = true;
            groupBox3.Enabled = true;
            groupBox4.Enabled = true;
            groupBox5.Enabled = true;
            btnConfirm.Enabled = true;
        }

        private void btnConfirm_Click(object sender, EventArgs e)
        {
            
            try
            {
                //保存
                XDocument document = XDocument.Load(".\\Xml\\XMLFile.xml");
                XElement root = document.Root;
                XElement Matrix00 = root.Element("camera").Element("Matrix00");
                Matrix00.SetValue(float.Parse(txtFx.Text));
                XElement Matrix11 = root.Element("camera").Element("Matrix11");
                Matrix11.SetValue(float.Parse(txtFy.Text));
                XElement Matrix02 = root.Element("camera").Element("Matrix02");
                Matrix02.SetValue(float.Parse(txtV.Text));
                XElement Matrix12 = root.Element("camera").Element("Matrix12");
                Matrix12.SetValue(float.Parse(txtU.Text));

                XElement R11 = root.Element("camera").Element("exMatrix00");
                R11.SetValue(float.Parse(txtR11.Text));
                XElement R12 = root.Element("camera").Element("exMatrix01");
                R12.SetValue(float.Parse(txtR12.Text));
                XElement R13= root.Element("camera").Element("exMatrix02");
                R13.SetValue(float.Parse(txtR13.Text));

                XElement R21 = root.Element("camera").Element("exMatrix10");
                R21.SetValue(float.Parse(txtR21.Text));
                XElement R22 = root.Element("camera").Element("exMatrix11");
                R22.SetValue(float.Parse(txtR22.Text));
                XElement R23 = root.Element("camera").Element("exMatrix12");
                R23.SetValue(float.Parse(txtR23.Text));

                XElement R31 = root.Element("camera").Element("exMatrix20");
                R31.SetValue(float.Parse(txtR31.Text));
                XElement R32 = root.Element("camera").Element("exMatrix21");
                R32.SetValue(float.Parse(txtR32.Text));
                XElement R33 = root.Element("camera").Element("exMatrix22");
                R33.SetValue(float.Parse(txtR33.Text));

                XElement DistCoeffs00 = root.Element("camera").Element("DistCoeffs00");
                DistCoeffs00.SetValue(float.Parse(txtK1.Text));
                XElement DistCoeffs10 = root.Element("camera").Element("DistCoeffs10");
                DistCoeffs10.SetValue(float.Parse(txtK2.Text));

                XElement rapid = root.Element("ABB").Element("rapid");
                rapid.SetValue(txtModule.Text);
                XElement ip = root.Element("ABB").Element("ip");
                ip.SetValue(txtIP.Text);

                XElement kx = root.Element("TranslationParam").Element("kx");
                kx.SetValue(float.Parse(txtKx.Text));
                XElement bx = root.Element("TranslationParam").Element("bx");
                bx.SetValue(float.Parse(txtBx.Text));
                XElement ky = root.Element("TranslationParam").Element("ky");
                ky.SetValue(float.Parse(txtKy.Text));
                XElement by = root.Element("TranslationParam").Element("by");
                by.SetValue(float.Parse(txtBy.Text));
                XElement kz = root.Element("TranslationParam").Element("kz");
                kz.SetValue(float.Parse(txtKz.Text));
                XElement bz = root.Element("TranslationParam").Element("bz");
                bz.SetValue(float.Parse(txtBz.Text));

                document.Save(".\\Xml\\XMLFile.xml");
                //更新委托
                float[] inmat = new float[4] { float.Parse(txtFx.Text), float.Parse(txtFy.Text), float.Parse(txtV.Text), float.Parse(txtU.Text) };
                float[] exmat = new float[9] {  float.Parse(txtR11.Text),float.Parse(txtR12.Text),float.Parse(txtR13.Text),
                                                float.Parse(txtR21.Text),float.Parse(txtR22.Text),float.Parse(txtR23.Text),
                                                 float.Parse(txtR31.Text),float.Parse(txtR32.Text),float.Parse(txtR33.Text)};
                float[] distor = new float[2] { float.Parse(txtK1.Text), float.Parse(txtK2.Text) };
                float[] t = new float[6] { float.Parse(txtKx.Text), float.Parse(txtBx.Text), float.Parse(txtKy.Text), float.Parse(txtBy.Text), float.Parse(txtKz.Text), float.Parse(txtBz.Text) };
                string[] robot = new string[2] { txtModule.Text, txtIP.Text };

                sysParamChanged?.Invoke(inmat, exmat, distor, t, robot);
                groupBox1.Enabled = false;
                groupBox2.Enabled = false;
                groupBox3.Enabled = false;
                groupBox4.Enabled = false;
                groupBox5.Enabled = false;
                btnConfirm.Enabled = false;
            }
            catch (Exception)
            {

                throw;
            }
        }

        /// <summary>
        /// 加载XML
        /// </summary>
        private void LoadXML()
        {
            try
            {
                XDocument document = XDocument.Load(".\\Xml\\XMLFile.xml");
                XElement root = document.Root;
                txtFx.Text = root.Element("camera").Element("Matrix00").Value;
                txtV.Text = root.Element("camera").Element("Matrix02").Value;

                txtFy.Text = root.Element("camera").Element("Matrix11").Value;
                txtU.Text = root.Element("camera").Element("Matrix12").Value;

                txtR11.Text = root.Element("camera").Element("exMatrix00").Value;
                txtR12.Text = root.Element("camera").Element("exMatrix01").Value;
                txtR13.Text = root.Element("camera").Element("exMatrix02").Value;
                      
                txtR21.Text = root.Element("camera").Element("exMatrix10").Value;
                txtR22.Text = root.Element("camera").Element("exMatrix11").Value;
                txtR23.Text = root.Element("camera").Element("exMatrix12").Value;
                      
                txtR31.Text = root.Element("camera").Element("exMatrix20").Value;
                txtR32.Text = root.Element("camera").Element("exMatrix21").Value;
                txtR33.Text = root.Element("camera").Element("exMatrix22").Value;

                txtK1.Text = root.Element("camera").Element("DistCoeffs00").Value;
                txtK2.Text = root.Element("camera").Element("DistCoeffs10").Value;
                
                txtModule.Text = root.Element("ABB").Element("rapid").Value;
                txtIP.Text = root.Element("ABB").Element("ip").Value;

                txtKx.Text = root.Element("TranslationParam").Element("kx").Value;
                txtBx.Text = root.Element("TranslationParam").Element("bx").Value;
                txtKy.Text = root.Element("TranslationParam").Element("ky").Value;
                txtBy.Text = root.Element("TranslationParam").Element("by").Value;
                txtKz.Text = root.Element("TranslationParam").Element("kz").Value;
                txtBz.Text = root.Element("TranslationParam").Element("bz").Value;
            }
            catch (Exception ex)
            {
                MessageBox.Show("加载配置参数出错，原因：\r\n" + ex.Message);
                return;
            }
        }
    }
}
