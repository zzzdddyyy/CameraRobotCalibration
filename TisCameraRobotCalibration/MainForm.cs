using ABB.Robotics.Controllers;
using ABB.Robotics.Controllers.Discovery;
using ABB.Robotics.Controllers.MotionDomain;
using ABB.Robotics.Controllers.RapidDomain;
using CalculateExMatrix;
using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Features2D;
using Emgu.CV.Structure;
using Emgu.CV.Util;
using MetroFramework.Forms;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Xml.Linq;
using TIS.Imaging;

namespace TisCameraRobotCalibration
{
    public partial class MainForm : MetroForm
    {
        ICImagingControl IcImageCtrl = new ICImagingControl();
        private Matrix<double> intrinsicMatrix;//相机内部参数
        private Matrix<double> extrinsicMatrix;//相机wai部参数
        private Matrix<double> distCoeffs;//畸变参数
        private float[] transParams;//平移向量与机器人Z坐标的关系
        //private Matrix<double> homogeneousMat;//正定齐次矩阵
        Matrix<double> pInvMat = new Matrix<double>(3, 3);
        private MCvTermCriteria criteria;//求角点迭代的终止条件（精度
        private Matrix<float> MAPX; //x坐标对应的映射矩阵
        private Matrix<float> MAPY;
        private Size IMAGESIZE;//图像的大小
        private Size patternSize; //corner pattern
        private int square = 50;//棋盘格方块边长
        private float threshold = 0.1f;//平行度阈值

        private Bitmap myBmp = null;

        Image<Bgr, byte> myImg = null;

        private NetworkScanner ns_NetworkScanner = null;
        private ControllerInfoCollection cic_ControllerInfoCollection = null;
        private Controller myController = null;
        private static readonly object lockerMastership = new object();
        string robotIp;//机器人IP
        string moduleName;//任务块名称
        System.Timers.Timer robotPosTimer;
        MotionSystem motionSystem;
        RobTarget robTarget;
        CoordinateSystemType coordinateSystemType = CoordinateSystemType.Base;

        PointF[] nineRobotPoints = new PointF[9];//标定的9点机器人坐标
        PointF[] nineCameraPoints = new PointF[9];//标定的9点相机坐标
        Matrix<double> rigidTransform = new Matrix<double>(2, 3);//刚性变换矩阵
        private RapidData rd_pCalibrate, rd_caliPositon;

        private bool robMoveDone = false;
        private bool SYSTEMSATART = false;
        private Thread calibThread;
        Matrix<double> eyeToHandMat = new Matrix<double>(3, 4);//最终手眼变换矩阵

        public MainForm()
        {
            InitializeComponent();
        }

        private void MainForm_Load(object sender, EventArgs e)
        {
            cmbCoordinateSystemType.DataSource = System.Enum.GetNames(typeof(CoordinateSystemType));
            cmbCoordinateSystemType.SelectedIndex = 4;
            intrinsicMatrix = new Matrix<double>(3, 3);//相机内部参数
            extrinsicMatrix = new Matrix<double>(3, 4);//相机wai部参数
            distCoeffs = new Matrix<double>(5, 1);//畸变参数
            transParams = new float[6];
            criteria = new MCvTermCriteria(100, 1e-5);//求角点迭代的终止条件（精度
            LoadXML();
            MAPX = new Matrix<float>(resolutionH, resolutionW); //x坐标对应的映射矩阵
            MAPY = new Matrix<float>(resolutionH, resolutionW);
            IMAGESIZE = new Size(resolutionW, resolutionH);//图像的大小
            patternSize = new Size(chessXcount, chessYcount);

            robotPosTimer = new System.Timers.Timer();
            robotPosTimer.Enabled = false;
            robotPosTimer.Interval = 500;
            robotPosTimer.Elapsed += new System.Timers.ElapsedEventHandler(SubscirbRobotPos);

            LoadMatrix();
            txtMsg.Text += DateTime.Now.ToString("HH:mm:ss") + "：将机器人切换到自动模式\r\n";
            txtMsg.Text += DateTime.Now.ToString("HH:mm:ss") + "：请先【连接机器人】和【连接相机】后启动程序\r\n";
        }

        private void MainForm_FormClosing(object sender, FormClosingEventArgs e)
        {
            robotPosTimer.Elapsed -= new System.Timers.ElapsedEventHandler(SubscirbRobotPos);
            robotPosTimer.Enabled = false;
        }

        #region xml
        private double im00;
        private double im01;
        private double im02;

        private double im11;
        private double im12;

        private double em00;
        private double em01;
        private double em02;
        private double em10;
        private double em11;
        private double em12;
        private double em20;
        private double em21;
        private double em22;
        private double k1;
        private double k2;
        private int chessXcount;
        private int chessYcount;
        private int chessSquarel;

        private int resolutionW;
        private int resolutionH;

        private float diskDepth;
        #endregion

        #region 1、调平助手
        private void btnSet_Click(object sender, EventArgs e)
        {
            SysParamsForm sysParamsForm = new SysParamsForm();
            sysParamsForm.FormClosed += SysParamsForm_FormClosed;
            sysParamsForm.ShowDialog();
        }

        private void SysParamsForm_FormClosed(object sender, FormClosedEventArgs e)
        {
            
        }

        private void btnRealTime_Click(object sender, EventArgs e)
        {

        }

        private void button1_Click(object sender, EventArgs e)
        {
            System.Threading.Tasks.Task eulerTask = System.Threading.Tasks.Task.Run(async () => {
                myBmp = null;
                //触发一帧
                SoftTrigger(IcImageCtrl);
                await System.Threading.Tasks.Task.Delay(500);
                if (myBmp == null)
                {
                    throw new Exception($"执行标定时，未获取到图像");
                }
                //图像处理
                Image<Gray, byte> imageInput = BitmapExtension.ToImage<Gray, byte>(myBmp);
                Image<Gray, byte> remapImg = imageInput.CopyBlank();
                CvInvoke.InitUndistortRectifyMap(intrinsicMatrix, distCoeffs, null, intrinsicMatrix, IMAGESIZE, DepthType.Cv32F, MAPX, MAPY);
                CvInvoke.Remap(imageInput, remapImg, MAPX, MAPY, Inter.Linear, BorderType.Constant, new MCvScalar(0));
                Image<Bgr, byte> bgrInput = BitmapExtension.ToImage<Bgr, byte>(remapImg.AsBitmap());
                Rectangle roiRect = new Rectangle(5472 / 2 - 1000, 3648 / 2 - 800, 2000, 1600);
                remapImg = GetROI(remapImg, roiRect);
                //CvInvoke.Threshold(remapImg, remapImg, 0, 255, ThresholdType.Otsu);//
                                                                                   //remapImg.Bitmap.Save("ThresholdImg.bmp");
                VectorOfPointF corners;
                Mat revc = new Mat();
                Mat tevc = new Mat();
                Mat inliners = new Mat();
                VectorOfPoint3D32F corners3D = new VectorOfPoint3D32F();
                objectCorners3D(patternSize, square, out corners3D);
                try
                {
                    FindCorners(ref remapImg, out corners);

                    CvInvoke.SolvePnPRansac(corners3D, corners, intrinsicMatrix, distCoeffs, revc, tevc, true, 10000, 1, 0.99, inliners, SolvePnpMethod.Iterative);
                    //获得的旋转矩阵是向量，是3×1的矩阵，想要还原回3×3的矩阵，需要罗德里格斯变换Rodrigues
                }
                catch (Exception ex)
                {
                    CvInvoke.Rectangle(bgrInput, roiRect, new MCvScalar(0, 255, 0), 15);
                    pictureBox2.Image = remapImg.AsBitmap();
                    MessageBox.Show("输入图像不合格，确认标定板在绿色矩形框内\r\n请重新取像计算或重新设置ROI大小！\r\n" + ex.Message, "计算失败");
                    return;
                }
                Mat dst = new Mat();
                CvInvoke.Rodrigues(revc, dst);
                double x, y, z;
                CalEulerAngel(dst, out x, out y, out z);
                if (Math.Abs(x) < threshold && Math.Abs(y) < threshold)
                {
                    lblRet.Invoke(new Action(() => {
                        lblRet.BackColor = Color.Green;
                        lblRet.Text = "OK";
                    }));
                }
                else
                {
                    lblRet.Invoke(new Action(() => {
                        lblRet.BackColor = Color.Red;
                        lblRet.Text = "NG";
                    }));
                }
                PointF p0 = corners[0];
                PointF p7 = corners[10];
                PointF p77 = corners[77];
                PointF p87 = corners[87];
                CvInvoke.Circle(bgrInput, new Point((int)p0.X, (int)p0.Y), 10, new MCvScalar(5, 255, 5), 10);
                CvInvoke.ArrowedLine(bgrInput, new Point((int)p0.X, (int)p0.Y), new Point((int)p7.X, (int)p7.Y), new MCvScalar(0, 0, 255), 10);
                CvInvoke.ArrowedLine(bgrInput, new Point((int)p0.X, (int)p0.Y), new Point((int)p77.X, (int)p77.Y), new MCvScalar(255, 0, 0), 10);
                CvInvoke.PutText(bgrInput, "X", new Point((int)p7.X - 85, (int)p7.Y + 30), FontFace.HersheyComplex, 3, new MCvScalar(0, 0, 255), 10);
                CvInvoke.PutText(bgrInput, "Y", new Point((int)p77.X + 10, (int)p77.Y - 50), FontFace.HersheyComplex, 3, new MCvScalar(255, 0, 0), 10);
                CvInvoke.Rectangle(bgrInput, roiRect, new MCvScalar(0, 255, 0), 15);
                pictureBox2.Image = bgrInput.AsBitmap();
                //ShowGrayImg(remapImg);
                txtMsg.Invoke(new Action(() => {
                    txtMsg.Text = "";
                    //txtMsg.BackColor = Color.Green;
                    txtMsg.Text += $"EX = {x.ToString("f8")}f\r\n";
                    txtMsg.Text += $"EY = {y.ToString("f8")}\r\n";
                    txtMsg.Text += $"EZ = {z.ToString("f8")}\r\n";
                }));

            });                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
        }

        private void btnImport_Click(object sender, EventArgs e)
        {
            OpenFileDialog openFileDialog = new OpenFileDialog();
            if (openFileDialog.ShowDialog() == DialogResult.OK)
            {
                myImg = new Image<Bgr, byte>(openFileDialog.FileName);
                //myImgCopy = myImg.Copy();
            }
            else return;
            if (myImg == null)
            {
                return;
            }
            else
            {
                //System.Threading.Tasks.Task t = System.Threading.Tasks.Task.Run(() => {

                    Image<Gray, byte> grayImg = myImg.Convert<Gray, byte>();
                    //畸变校正      
                    Image<Gray, byte> remapImg = grayImg.CopyBlank();
                    try
                    {
                        CvInvoke.InitUndistortRectifyMap(intrinsicMatrix, distCoeffs, null, intrinsicMatrix, IMAGESIZE, DepthType.Cv32F, MAPX, MAPY);
                        CvInvoke.Remap(grayImg, remapImg, MAPX, MAPY, Inter.Linear, BorderType.Constant, new MCvScalar(0));
                    }
                    catch (Exception ex)
                    {
                        MessageBox.Show("畸变校正失败" + ex.Message);
                        throw (ex);
                    }
                    Rectangle roiRect = new Rectangle(1500, 700, 2700, 1800);
                    remapImg = GetROI(remapImg, roiRect);
                    //CvInvoke.Threshold(remapImg, remapImg, 0, 255, ThresholdType.Otsu);//
                    remapImg.Save("ThresholdImg.bmp");
                    VectorOfPointF corners;
                    Mat revc = new Mat();
                    Mat tevc = new Mat();
                    Mat inliners = new Mat();
                    VectorOfPoint3D32F corners3D = new VectorOfPoint3D32F();
                    objectCorners3D(patternSize, square, out corners3D);
                    try
                    {
                        FindCorners(ref remapImg, out corners);

                        CvInvoke.SolvePnPRansac(corners3D, corners, intrinsicMatrix, distCoeffs, revc, tevc, true, 10000, 1, 0.99, inliners, SolvePnpMethod.Iterative);
                        //获得的旋转矩阵是向量，是3×1的矩阵，想要还原回3×3的矩阵，需要罗德里格斯变换Rodrigues
                    }
                    catch (Exception ex)
                    {
                        CvInvoke.Rectangle(myImg, roiRect, new MCvScalar(0, 255, 0), 15);
                        pictureBox2.Image = myImg.AsBitmap();
                        MessageBox.Show("输入图像不合格，确认标定板在绿色矩形框内\r\n请重新取像计算或重新设置ROI大小！\r\n" + ex.Message, "计算失败");
                        return;
                    }

                    Mat dst = new Mat();
                    CvInvoke.Rodrigues(revc, dst);
                    double x, y, z;
                    CalEulerAngel(dst, out x, out y, out z);
                    if (Math.Abs(x) < threshold && Math.Abs(y) < threshold)
                    {
                        lblRet.Invoke(new Action(() => {
                            lblRet.BackColor = Color.Green;
                            lblRet.Text = "OK";
                        }));
                        //txtMsg.Invoke(new Action(() => {
                        //    //txtMsg.BackColor = Color.Red;
                        //    txtMsg.Text += $"EX = {x}\r\n";
                        //    txtMsg.Text += $"EY = {y}\r\n";
                        //    txtMsg.Text += $"EZ = {z}\r\n";
                        //}));
                    }
                    else
                    {
                        lblRet.Invoke(new Action(() => {
                            lblRet.BackColor = Color.Red;
                            lblRet.Text = "NG";
                        }));
                    }
                    PointF p0 = corners[0];
                    PointF p7 = corners[10];
                    PointF p77 = corners[77];
                    PointF p87 = corners[87];
                    Console.WriteLine(111111);
                    CvInvoke.Circle(myImg, new Point((int)p0.X, (int)p0.Y), 10, new MCvScalar(5, 255, 5), 10);
                    CvInvoke.ArrowedLine(myImg, new Point((int)p0.X, (int)p0.Y), new Point((int)p7.X, (int)p7.Y), new MCvScalar(0, 255, 255), 10);
                    CvInvoke.ArrowedLine(myImg, new Point((int)p0.X, (int)p0.Y), new Point((int)p77.X, (int)p77.Y), new MCvScalar(255, 0, 0), 10);
                    CvInvoke.PutText(myImg, "X", new Point((int)p7.X - 85, (int)p7.Y + 30), FontFace.HersheyComplex, 3, new MCvScalar(0, 0, 255), 10);
                    CvInvoke.PutText(myImg, "Y", new Point((int)p77.X + 10, (int)p77.Y - 50), FontFace.HersheyComplex, 3, new MCvScalar(255, 0, 0), 10);

                    Console.WriteLine($"EX = {x}");
                    Console.WriteLine($"EY = {y}");
                    Console.WriteLine($"EZ = {z}");
                    CvInvoke.Rectangle(myImg, roiRect, new MCvScalar(0, 255, 0), 15);
                    txtMsg.Invoke(new Action(() => {
                        txtMsg.Text = "";
                        //txtMsg.BackColor = Color.Green;
                        txtMsg.Text += $"EX = {x.ToString("f8")}f\r\n";
                        txtMsg.Text += $"EY = {y.ToString("f8")}\r\n";
                        txtMsg.Text += $"EZ = {z.ToString("f8")}\r\n";
                        pictureBox2.Image = myImg.AsBitmap();
                    }));


                    //pictureBox2.Image = grayImg.Bitmap;
                //});
            }
        }
        #endregion

        #region 2、机器人监视
        /// <summary>
        /// 实时输出机器人位置
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void SubscirbRobotPos(object sender, System.Timers.ElapsedEventArgs e)
        {
            try
            {
                robTarget = motionSystem.ActiveMechanicalUnit.GetPosition(coordinateSystemType);
                this.Invoke(new Action(() => {
                    lblRx.Text = $"X : {robTarget.Trans.X}";
                    lblRy.Text = $"Y : {robTarget.Trans.Y}";
                    lblRz.Text = $"Z : {robTarget.Trans.Z}";
                }));
            }
            catch (Exception)
            {
            }

        }
        #endregion

        #region 3、Euler Calibration
        private void btnTrigger_Click(object sender, EventArgs e)
        {
            System.Threading.Tasks.Task eulerTask = System.Threading.Tasks.Task.Run(async () => {
                try
                {
                    myBmp = null;
                    //触发一帧
                    SoftTrigger(IcImageCtrl);
                    await System.Threading.Tasks.Task.Delay(500);
                    if (myBmp == null)
                    {
                        throw new Exception($"执行标定时，未获取到图像");
                    }
                    //图像处理
                    Image<Gray, byte> imageInput = BitmapExtension.ToImage<Gray, byte>(myBmp);
                    Image<Gray, byte> remapImg = imageInput.CopyBlank();
                    CvInvoke.InitUndistortRectifyMap(intrinsicMatrix, distCoeffs, null, intrinsicMatrix, IMAGESIZE, DepthType.Cv32F, MAPX, MAPY);
                    CvInvoke.Remap(imageInput, remapImg, MAPX, MAPY, Inter.Linear, BorderType.Constant, new MCvScalar(0));
                    //Rectangle roiRect = new Rectangle(IMAGESIZE.Width / 2 - 1300, IMAGESIZE.Height / 2 - 1300, 2600, 2600);
                    Rectangle roiRect = new Rectangle(2000, 1000, 2200, 1500);
                    remapImg = GetROI(remapImg, roiRect);
                    //CvInvoke.Threshold(remapImg, remapImg, 0, 255, ThresholdType.Otsu);//
                    Image<Bgr, byte> bgrInput = remapImg.Convert<Bgr,byte>();
                    bgrInput.Save("eulerBgr.bmp");
                    VectorOfPointF corners;
                    FindCorners(ref remapImg, out corners);
                    VectorOfPointF keyCorner = new VectorOfPointF();
                    keyCorner.Push(new PointF[4] { corners[0], corners[10], corners[77], corners[87] });
                    CvInvoke.Circle(bgrInput, Point.Round(corners[0]), 13, new MCvScalar(0, 0, 255), 11, LineType.AntiAlias);
                    CvInvoke.Circle(bgrInput, Point.Round(corners[10]), 13, new MCvScalar(0, 255, 255), 11, LineType.AntiAlias);
                    CvInvoke.Circle(bgrInput, Point.Round(corners[77]), 13, new MCvScalar(255, 255, 0), 11, LineType.AntiAlias);
                    CvInvoke.Circle(bgrInput, Point.Round(corners[87]), 13, new MCvScalar(0, 255, 0), 11, LineType.AntiAlias);
                    CvInvoke.Line(bgrInput, new Point((int)corners[0].X, (int)corners[0].Y), new Point((int)corners[10].X, (int)corners[10].Y), new MCvScalar(0, 0, 255), 3);
                    CvInvoke.Line(bgrInput, new Point((int)corners[10].X, (int)corners[10].Y), new Point((int)corners[77].X, (int)corners[77].Y), new MCvScalar(0, 255, 255), 3);
                    CvInvoke.Line(bgrInput, new Point((int)corners[77].X, (int)corners[77].Y), new Point((int)corners[87].X, (int)corners[87].Y), new MCvScalar(255, 0, 55), 3);
                    CvInvoke.Line(bgrInput, new Point((int)corners[0].X, (int)corners[0].Y), new Point((int)corners[87].X, (int)corners[87].Y), new MCvScalar(0, 255, 25), 3);
                    RotatedRect rotatedRect = CvInvoke.MinAreaRect(corners);
                    RotatedRect rotatedPoints = CvInvoke.MinAreaRect(keyCorner);
                    double k = (corners[87].Y - corners[10].Y) / (corners[87].X - corners[10].X);
                    double eulerAllCorner;
                    double eulerKeyCorner;

                    if (Math.Abs(rotatedRect.Angle) > 45)
                    {
                        eulerAllCorner = rotatedRect.Angle + 90;
                    }
                    else
                    {
                        eulerAllCorner = rotatedRect.Angle;
                    }
                    if (Math.Abs(rotatedPoints.Angle) > 45)
                    {
                        eulerKeyCorner = rotatedRect.Angle + 90;
                    }
                    else
                    {
                        eulerKeyCorner = rotatedRect.Angle;
                    }
                    txtEulerResult.Invoke(new Action(() => {
                        txtEulerResult.Text += "K = " + k.ToString() + "\r\n";
                        txtEulerResult.Text += "AllEuler = " + eulerAllCorner.ToString() + "\r\n";
                        txtEulerResult.Text += "KeyEuler = " + eulerKeyCorner.ToString() + "\r\n";
                    }));
                    CvInvoke.Rectangle(bgrInput, roiRect, new MCvScalar(0, 255, 0), 15);
                    //保存结果
                    File.AppendAllText("Euler.txt", DateTime.Now.ToString() + "\r\n" + $"Camera Name : {IcImageCtrl.Name}\r\n" +
                        $"+$Robot Name :{myController.SystemName}\r\n" + $"AllEuler : {eulerAllCorner}\r\n" + $"KeyEuler : {eulerKeyCorner}\r\n");
                    myBmp = null;
                    ptbEuler.Invoke(new Action(() => {
                        ptbEuler.Image = bgrInput.AsBitmap();
                        ptbEuler.Refresh();
                    }));
                }
                catch (Exception ex)
                {
                    MessageBox.Show("棋盘格识别错误，原因：\r\n" + ex.Message + "\r\n请更改相机曝光后再次尝试", "棋盘格识提示", MessageBoxButtons.OK, MessageBoxIcon.Error);
                }

            });
        }
        #endregion

        #region 4、手眼标定
        List<PointF> nineWP = new List<PointF>();
        List<PointF> nineRP = new List<PointF>();
        private void btnOffsetTranslate_Click(object sender, EventArgs e)
        {
            try
            {
                if (nineWP.Count >= 9)
                {
                    Mat warpMat = CvInvoke.EstimateAffine2D(nineWP.ToArray(), nineRP.ToArray());
                    Image<Gray, float> img = warpMat.ToImage<Gray, float>();
                    rigidTransform[0, 0] = img.Data[0, 0, 0];
                    rigidTransform[0, 1] = img.Data[0, 1, 0];
                    rigidTransform[0, 2] = img.Data[0, 2, 0];
                    rigidTransform[1, 0] = img.Data[1, 0, 0];
                    rigidTransform[1, 1] = img.Data[1, 1, 0];
                    rigidTransform[1, 2] = img.Data[1, 2, 0];
                    File.AppendAllText(@".\Results\EyeToHandMat.txt", $"{DateTime.Now.ToString()}\r\n{eyeToHandMat[0, 0]},{eyeToHandMat[0, 1]},{eyeToHandMat[0, 2]},{eyeToHandMat[0, 3]}\r\n" +
                        $"{eyeToHandMat[1, 0]},{eyeToHandMat[1, 1]},{eyeToHandMat[1, 2]},{eyeToHandMat[1, 3]}\r\n" +
                        $"{eyeToHandMat[2, 0]},{eyeToHandMat[2, 1]},{eyeToHandMat[2, 2]},{eyeToHandMat[2, 3]}\r\n\r\n");
                    //File.AppendAllText(@".\Results\Bot4_EyeToHandMat.txt", $"{DateTime.Now.ToString()} + {cmbCoordinateSystemType.SelectedItem.ToString()}\r\n{img.Data[0, 0, 0].ToString()},{img.Data[0, 1, 0].ToString()},{img.Data[0, 2, 0].ToString()}\r\n" +
                    //   $"{img.Data[1, 0, 0].ToString()},{img.Data[1, 1, 0].ToString()},{img.Data[1, 2, 0].ToString()}\r\n\r\n");
                }
                else
                {
                    MessageBox.Show($"点数不达标 ，目前点数：{nineWP.Count }");
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
            
        }
        /// <summary>
        /// 触发计算一个点数据
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void btnOffsetTrigger_Click(object sender, EventArgs e)
        {
            System.Threading.Tasks.Task eulerTask = System.Threading.Tasks.Task.Run(async () => {
                try
                {
                    if (int.Parse(txtoffx.Text) == 0 & int.Parse(txtoffy.Text) == 0)
                    {
                        throw new Exception($"标定圆盘相对于机器人法兰中心的偏移值不能全为0，请输入正确的偏移值");
                    }
                    myBmp = null;
                    //触发一帧
                    SoftTrigger(IcImageCtrl);
                    await System.Threading.Tasks.Task.Delay(500);
                    if (myBmp == null)
                    {
                        throw new Exception($"执行标定时，未获取到图像");
                    }
                    //图像处理
                    Image<Gray, byte> imageInput = BitmapExtension.ToImage<Gray, byte>(myBmp);
                    Image<Gray, byte> remapImg = imageInput.CopyBlank();
                    Image<Gray, byte> roiImg = imageInput.CopyBlank();
                    CvInvoke.InitUndistortRectifyMap(intrinsicMatrix, distCoeffs, null, intrinsicMatrix, IMAGESIZE, DepthType.Cv32F, MAPX, MAPY);
                    CvInvoke.Remap(imageInput, remapImg, MAPX, MAPY, Inter.Linear, BorderType.Constant, new MCvScalar(0));
                    Rectangle roiRect = new Rectangle(IMAGESIZE.Width / 2 - 2600, IMAGESIZE.Height / 2 - 1600, 5200, 3200);
                    roiImg = GetROI(remapImg, roiRect);
                    Image<Bgr, byte> bgrInput = roiImg.Convert<Bgr, byte>();

                    SimpleBlobDetectorParams blobparams = new SimpleBlobDetectorParams();
                    blobparams.FilterByArea = true;
                    blobparams.MinArea = 20000;
                    blobparams.MaxArea = 50000000;
                    //blobparams.MinThreshold = (float)minValue + 1;
                    //blobparams.MaxThreshold = (float)maxValue;
                    blobparams.FilterByCircularity = true;  //斑点圆度
                    blobparams.MinCircularity = (float)0.8;
                    blobparams.MaxCircularity = 1;
                    blobparams.FilterByConvexity = false;    //斑点凸度
                    blobparams.MinConvexity = (float)0.2;
                    blobparams.MaxConvexity = 1;
                    blobparams.FilterByInertia = false;  //斑点惯性率
                    blobparams.MinInertiaRatio = (float)0.9;
                    blobparams.MaxInertiaRatio = 1;
                    blobparams.FilterByColor = false;
                    blobparams.ThresholdStep = 2;
                    blobparams.MinRepeatability = new IntPtr(2);
                    SimpleBlobDetector detector = new SimpleBlobDetector(blobparams);

                    MKeyPoint[] keypoints = detector.Detect(roiImg);
                    //目标矩形
                    Rectangle targRect = new Rectangle();
                    if (keypoints.Length < 1)
                    {
                        throw new Exception("未找到合适的标定物！\r\n请检查标定物是否在ROI内。");
                    }
                    foreach (MKeyPoint keypoint in keypoints)
                    {
                        targRect = new Rectangle((int)(keypoint.Point.X - keypoint.Size / 2), (int)(keypoint.Point.Y - keypoint.Size / 2), (int)keypoint.Size, (int)keypoint.Size);
                        CvInvoke.Rectangle(bgrInput, targRect, new MCvScalar(255, 0, 0), 2);
                        CvInvoke.Circle(bgrInput, new Point((int)(keypoint.Point.X), (int)(keypoint.Point.Y)), (int)keypoint.Size / 2, new MCvScalar(0, 0, 255), 2);
                        CvInvoke.Circle(bgrInput, new Point((int)(keypoint.Point.X), (int)(keypoint.Point.Y)), 4, new MCvScalar(0, 0, 255), 2);
                        Console.WriteLine($"BlobCenter X = {keypoint.Point.X}");
                        Console.WriteLine($"BlobCenter Y = {keypoint.Point.Y}");
                    }
                    bgrInput.Save("circles.bmp");
                    if (keypoints.Length == 1)
                    {
                        //CvInvoke.Circle(bgrInput, System.Drawing.Point.Round(keypoints[0].Point), (int)keypoints[0].Size/2, new Bgr(Color.Red).MCvScalar, 2);
                        this.Invoke(new Action(() => {
                            txtpx.Text = keypoints[0].Point.X.ToString();
                            txtpy.Text = keypoints[0].Point.Y.ToString();
                        }));

                        //计算wx
                        double wx;
                        double wy;
                        //计算广义逆矩阵
                        Matrix<double> worldToImageMat = new Matrix<double>(3, 4);
                        //extrinsicMatrix[2, 3] = tevc[2,0];
                        double RZ = robTarget.Trans.Z + 12 + 6.6;//当前平台的RZ +标定厚度，14.7 + robTarget.Trans.Z
                        extrinsicMatrix[2, 3] = transParams[4] * RZ + transParams[5];
                        extrinsicMatrix[1, 3] = transParams[2] * RZ + transParams[3];
                        extrinsicMatrix[0, 3] = transParams[0] * RZ + transParams[1];
                        worldToImageMat = MatMultiply(intrinsicMatrix, extrinsicMatrix);
                        Matrix<double> homographyMatrix = new Matrix<double>(3, 3);
                        homographyMatrix[0, 0] = worldToImageMat[0, 0];
                        homographyMatrix[0, 1] = worldToImageMat[0, 1];
                        homographyMatrix[0, 2] = worldToImageMat[0, 3];
                        homographyMatrix[1, 0] = worldToImageMat[1, 0];
                        homographyMatrix[1, 1] = worldToImageMat[1, 1];
                        homographyMatrix[1, 2] = worldToImageMat[1, 3];
                        homographyMatrix[2, 0] = worldToImageMat[2, 0];
                        homographyMatrix[2, 1] = worldToImageMat[2, 1];
                        homographyMatrix[2, 2] = worldToImageMat[2, 3];
                        CvInvoke.Invert(homographyMatrix, pInvMat, DecompMethod.LU);
                        PointsToWorld(pInvMat, keypoints[0].Point.X, keypoints[0].Point.Y, out wx, out wy);
                        this.Invoke(new Action(() => {
                            txtwx.Text = wx.ToString();
                            txtwy.Text = wy.ToString();
                        }));
                        //计算矩形长度
                        double recX1, recY1, recX2, recY2, recX3, recY3;
                        PointsToWorld(pInvMat, targRect.X, targRect.Y, out recX1, out recY1);
                        PointsToWorld(pInvMat, (targRect.X + targRect.Width), targRect.Y, out recX2, out recY2);
                        PointsToWorld(pInvMat, (targRect.X), (targRect.Y + targRect.Height), out recX3, out recY3);
                        double recH = Math.Sqrt(Math.Pow(recY2 - recY1, 2) + Math.Pow(recX2 - recX1, 2));
                        double recV = Math.Sqrt(Math.Pow(recY3 - recY1, 2) + Math.Pow(recX3 - recX1, 2));
                        CvInvoke.PutText(bgrInput, $"Radius-H = {recH} | 300 mm", new Point(targRect.Location.X, targRect.Location.Y - 150), FontFace.HersheyComplex, 2, new MCvScalar(0, 255, 0), 3);
                        CvInvoke.PutText(bgrInput, $"Radius-V = {recV} | 300 mm", new Point(targRect.Location.X, targRect.Location.Y - 75), FontFace.HersheyComplex, 2, new MCvScalar(0, 255, 0), 3);
                        CvInvoke.Rectangle(bgrInput, roiRect, new MCvScalar(0, 255, 0), 2);
                        //转换
                        //nineWP.Add (new PointF((float)wx, (float)(wy)));
                        nineWP.Add(new PointF((float)wx, (float)(wy)));
                        //nineRP.Add(new PointF(robTarget.Trans.X, (robTarget.Trans.Y+530)));//在工件坐标系中Y增加530
                        nineRP.Add(new PointF(robTarget.Trans.X + int.Parse(txtoffx.Text), (robTarget.Trans.Y + int.Parse(txtoffy.Text))));//在工件坐标系中X增加530
                        if (nineRP.Count == 9)
                        {
                            for (int i = 0; i < 9; i++)
                            {
                                File.AppendAllText(@".\Results\FabricTop_NinePointsRecord.txt",
                                    $"\r\n{nineWP[i].X},{nineWP[i].Y},{nineRP[i].X},{nineRP[i].Y}\r\n\r\n");
                            }
                        }
                        myBmp = null;
                        this.Invoke(new Action(() => {
                            this.Text = $"No.{nineWP.Count} 个";
                            Console.WriteLine($"收集No.{nineWP.Count} 个点对");
                            ptbOffset.Image = bgrInput.AsBitmap();
                            ptbOffset.Refresh();
                        }));
                        GC.Collect();
                    }
                }
                catch (Exception ex)
                {
                    MessageBox.Show("偏移值计算错误，原因：\r\n" + ex.Message, "偏移值标定提示", MessageBoxButtons.OK, MessageBoxIcon.Error);
                }
            });
        }

        private void btnCheck_Click(object sender, EventArgs e)
        {
            if (rigidTransform[0, 2] == 0)
            {
                MessageBox.Show("请导入变换矩阵");
                return;
            }
            System.Threading.Tasks.Task eulerTask = System.Threading.Tasks.Task.Run(async () => {
                myBmp = null;


                //触发一帧
                SoftTrigger(IcImageCtrl);
                await System.Threading.Tasks.Task.Delay(500);
                if (myBmp == null)
                {
                    throw new Exception($"执行标定时，未获取到图像");
                }
                //图像处理
                Image<Gray, byte> imageInput = BitmapExtension.ToImage<Gray,byte>(myBmp);
                Image<Gray, byte> remapImg = imageInput.CopyBlank();
                Image<Gray, byte> roiImg = imageInput.CopyBlank();
                CvInvoke.InitUndistortRectifyMap(intrinsicMatrix, distCoeffs, null, intrinsicMatrix, IMAGESIZE, DepthType.Cv32F, MAPX, MAPY);
                CvInvoke.Remap(imageInput, remapImg, MAPX, MAPY, Inter.Linear, BorderType.Constant, new MCvScalar(0));
                Rectangle roiRect = new Rectangle(IMAGESIZE.Width / 2 - 2500, IMAGESIZE.Height / 2 - 1600, 5000, 3200);
                roiImg = GetROI(remapImg, roiRect);
                Image<Bgr, byte> bgrInput = roiImg.Convert<Bgr,byte>();

                SimpleBlobDetectorParams blobparams = new SimpleBlobDetectorParams();
                blobparams.FilterByArea = true;
                blobparams.MinArea = 5000;
                blobparams.MaxArea = 50000000;
                //blobparams.MinThreshold = (float)minValue + 1;
                //blobparams.MaxThreshold = (float)maxValue;
                blobparams.FilterByCircularity = true;  //斑点圆度
                blobparams.MinCircularity = (float)0.8;
                blobparams.MaxCircularity = 1;
                blobparams.FilterByConvexity = false;    //斑点凸度
                blobparams.MinConvexity = (float)0.2;
                blobparams.MaxConvexity = 1;
                blobparams.FilterByInertia = false;  //斑点惯性率
                blobparams.MinInertiaRatio = (float)0.9;
                blobparams.MaxInertiaRatio = 1;
                blobparams.FilterByColor = false;
                blobparams.ThresholdStep = 2;
                blobparams.MinRepeatability = new IntPtr(2);
                SimpleBlobDetector detector = new SimpleBlobDetector(blobparams);

                MKeyPoint[] keypoints = detector.Detect(roiImg);
                //目标矩形
                Rectangle targRect = new Rectangle();
                if (keypoints.Length < 1)
                {
                    throw new Exception("未找到合适的标定物！\r\n请检查标定物是否在ROI内。");
                }
                foreach (MKeyPoint keypoint in keypoints)
                {
                    targRect = new Rectangle((int)(keypoint.Point.X - keypoint.Size / 2), (int)(keypoint.Point.Y - keypoint.Size / 2), (int)keypoint.Size, (int)keypoint.Size);
                    CvInvoke.Rectangle(bgrInput, targRect, new MCvScalar(255, 0, 0), 2);
                    CvInvoke.Circle(bgrInput, new Point((int)(keypoint.Point.X), (int)(keypoint.Point.Y)), (int)keypoint.Size / 2, new MCvScalar(0, 0, 255), 2);
                    CvInvoke.Circle(bgrInput, new Point((int)(keypoint.Point.X), (int)(keypoint.Point.Y)), 4, new MCvScalar(0, 0, 255), 2);
                    Console.WriteLine($"BlobCenter X = {keypoint.Point.X}");
                    Console.WriteLine($"BlobCenter Y = {keypoint.Point.Y}");
                }
                //bgrInput.Save("check.bmp");
                if (keypoints.Length == 1)
                {
                    //CvInvoke.Circle(bgrInput, System.Drawing.Point.Round(keypoints[0].Point), (int)keypoints[0].Size/2, new Bgr(Color.Red).MCvScalar, 2);
                    this.Invoke(new Action(() => {
                        txtpx.Text = keypoints[0].Point.X.ToString();
                        txtpy.Text = keypoints[0].Point.Y.ToString();
                    }));

                    //计算wx
                    double wx;
                    double wy;
                    //计算广义逆矩阵
                    Matrix<double> worldToImageMat = new Matrix<double>(3, 4);
                    //extrinsicMatrix[2, 3] = tevc[2,0];
                    double RZ = robTarget.Trans.Z + 12 + 6.6;
                    //double RZ = -238.27 + 12;//当前平台的RZ +标定板厚度，一般是10mm
                    //double RZ = -242.23-14.7 + 12;//当前平台的RZ +标定板厚度，一般是10mm
                    extrinsicMatrix[2, 3] = transParams[4] * RZ + transParams[5];
                    extrinsicMatrix[1, 3] = transParams[2] * RZ + transParams[3];
                    extrinsicMatrix[0, 3] = transParams[0] * RZ + transParams[1];
                    worldToImageMat = MatMultiply(intrinsicMatrix, extrinsicMatrix);
                    Matrix<double> homographyMatrix = new Matrix<double>(3, 3);
                    homographyMatrix[0, 0] = worldToImageMat[0, 0];
                    homographyMatrix[0, 1] = worldToImageMat[0, 1];
                    homographyMatrix[0, 2] = worldToImageMat[0, 3];
                    homographyMatrix[1, 0] = worldToImageMat[1, 0];
                    homographyMatrix[1, 1] = worldToImageMat[1, 1];
                    homographyMatrix[1, 2] = worldToImageMat[1, 3];
                    homographyMatrix[2, 0] = worldToImageMat[2, 0];
                    homographyMatrix[2, 1] = worldToImageMat[2, 1];
                    homographyMatrix[2, 2] = worldToImageMat[2, 3];
                    CvInvoke.Invert(homographyMatrix, pInvMat, DecompMethod.LU);
                    PointsToWorld(pInvMat, keypoints[0].Point.X, keypoints[0].Point.Y, out wx, out wy);
                    this.Invoke(new Action(() => {
                        txtwx.Text = wx.ToString();
                        txtwy.Text = wy.ToString();
                    }));
                    //计算矩形长度
                    double recX1, recY1, recX2, recY2, recX3, recY3;
                    PointsToWorld(pInvMat, targRect.X, targRect.Y, out recX1, out recY1);
                    PointsToWorld(pInvMat, (targRect.X + targRect.Width), targRect.Y, out recX2, out recY2);
                    PointsToWorld(pInvMat, (targRect.X), (targRect.Y + targRect.Height), out recX3, out recY3);
                    double recH = Math.Sqrt(Math.Pow(recY2 - recY1, 2) + Math.Pow(recX2 - recX1, 2));
                    double recV = Math.Sqrt(Math.Pow(recY3 - recY1, 2) + Math.Pow(recX3 - recX1, 2));
                    CvInvoke.PutText(bgrInput, $"Radius-H = {recH} | 300 mm", new Point(targRect.Location.X, targRect.Location.Y - 150), FontFace.HersheyComplex, 2, new MCvScalar(0, 255, 0), 3);
                    CvInvoke.PutText(bgrInput, $"Radius-V = {recV} | 300 mm", new Point(targRect.Location.X, targRect.Location.Y - 75), FontFace.HersheyComplex, 2, new MCvScalar(0, 255, 0), 3);
                    CvInvoke.Rectangle(bgrInput, roiRect, new MCvScalar(0, 255, 0), 2);
                    //转换
                    //nineWP.Add(new PointF((float)wx, (float)(wy)));
                    //nineRP.Add(new PointF(robTarget.Trans.X, (robTarget.Trans.Y - 530)));
                    //校验
                    //计算机器人理论坐标
                    double Rx = rigidTransform[0, 0] * wx + rigidTransform[0, 1] * wy + rigidTransform[0, 2] - 530 + 0.0099 * keypoints[0].Point.Y - 18;
                    //+0.0053* keypoints[0].Point.Y* keypoints[0].Point.Y-20.279* keypoints[0].Point.Y+19360;
                    double Ry = rigidTransform[1, 0] * wx + rigidTransform[1, 1] * wy + rigidTransform[1, 2] + 0.0124 * keypoints[0].Point.X - 36.679;
                    //MessageBox.Show($"计算值 & 实际值 & 差值：\r\n{Rx} & {robTarget.Trans.X} & {(robTarget.Trans.X - (Rx))}\r\n\r\n" +
                    //    $"{Ry} & {robTarget.Trans.Y} & {(robTarget.Trans.Y - (Ry-530))}\r\n\r\n", "计算结果");
                    myBmp = null;
                    //    string sql = "INSERT INTO "+stableName+" (logTime,rh,rx,ry,px,py,wx,wy,crx,cry) " +
                    //"VALUES('" + DateTime.Now + "','" + RZ + "','" + robTarget.Trans.X + "','" + robTarget.Trans.Y + "','" + keypoints[0].Point.X + "','" + keypoints[0].Point.Y
                    //+ "','" + wx + "','" + wy + "','" + Rx + "','" + Ry + "')";
                    //    MySqlHelper.ExecuteNonQuery(MySqlConn, sql);
                    this.Invoke(new Action(() => {
                        txtcrx.Text = Rx.ToString();
                        txtcry.Text = Ry.ToString();
                        //txtoffx.Text = $"No.{nineWP.Count} 个";
                        ptbOffset.Image = bgrInput.AsBitmap();
                        ptbOffset.Refresh();
                    }));
                    GC.Collect();
                }
            });
        }

        private void btnRetry_Click(object sender, EventArgs e)
        {
            nineWP.Clear();
            nineRP.Clear();
            this.Text = $"No.{nineWP.Count} 个";
        }
        #endregion、

        #region 5、参数转换
        private void btnImportTxt_Click(object sender, EventArgs e)
        {
            try
            {
                OpenFileDialog openFileDialog = new OpenFileDialog();
                openFileDialog.Filter = "文本文件(*.txt)|*.txt";
                if (openFileDialog.ShowDialog() == DialogResult.OK)
                {
                    List<double> rd = new List<double>();
                    StreamReader sr = new StreamReader(openFileDialog.FileName);
                    string[] splitPath = openFileDialog.FileName.Split('\\');
                    while (!sr.EndOfStream)
                    {
                        string str = sr.ReadLine();
                        rd.Add(double.Parse(str));
                    }

                    sr.Close();
                    //为文件打开一个二进制写入器 
                    FileStream fs;
                    fs = new FileStream($".\\NewBat\\{splitPath[splitPath.Length - 1].Substring(0, splitPath[splitPath.Length - 1].Length - 4)}.bat", FileMode.OpenOrCreate, FileAccess.ReadWrite);
                    BinaryWriter bw = new BinaryWriter(fs);
                    foreach (double item in rd)
                    {
                        bw.Write(item);
                    }
                    //int length = Convert.ToInt32(bw.BaseStream.Length);
                    //Console.WriteLine($"length = {length / 8}");
                    fs.Close();
                    bw.Close();
                    byte[] y = File.ReadAllBytes($".\\NewBat\\{splitPath[splitPath.Length - 1].Substring(0, splitPath[splitPath.Length - 1].Length - 4)}.bat");
                    if (y.Length % 8 != 0)
                    {
                        Console.WriteLine("文件长度不对");
                        return;
                    }
                    double[] re = new double[y.Length / sizeof(double)];
                    unsafe
                    {
                        fixed (double* px = &re[0])
                        {
                            Marshal.Copy(y, 0, new IntPtr(px), y.Length);
                        }
                    }
                    txtResult.Text = "";
                    txtResult.Text = "请核对Bat文件是否正确：\r\n";

                    foreach (double item in re)
                    {
                        txtResult.Text += item.ToString() + "\r\n";
                    }
                    MessageBox.Show($"生成的文件字段数量：{y.Length / sizeof(double)}", "bat文件生成完成");
                }
                else return;
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message, "bat文件生成错误");
            }
        }
        #endregion


        #region ABB
        private void 连接机器人ToolStripMenuItem_Click(object sender, EventArgs e)
        {
            try
            {
                RobotConnect(1, robotIp);
                robotPosTimer.Enabled = true;
                txtMsg.Text += DateTime.Now.ToString("HH:mm:ss") + "：机器人连接成功\r\n";
            }
            catch (Exception ex)
            {
                MessageBox.Show("机器人连接失败\r\n" + ex.Message);
            }
        }
        /// <summary>
        /// 上电
        /// </summary>
        /// <param name="c"></param>
        /// <returns></returns>
        private bool RobotMotorOn(Controller c)
        {
            try
            {
                if (c != null && c.Connected == true)
                {
                    c.State = ControllerState.MotorsOn;
                    // controller.Logoff();
                    return true;
                }
                else
                {
                    throw new Exception($"当前控制器{c.SystemName} 不可用");
                }
            }
            catch (Exception ex)
            {
                string str = ex.StackTrace;
                // Program.MlilyFourthLog.Error("SetMotorsOn : " + str.Substring(str.LastIndexOf("\\") + 1, str.Length - str.LastIndexOf("\\") - 1) + "--------" + ex.Message);
                return false;
            }
        }
        private bool RobotMotorOff(Controller c)
        {
            try
            {
                if (c != null && c.Connected == true)
                {
                    c.State = ControllerState.MotorsOff;
                    // controller.Logoff();
                    return true;
                }
                else
                {
                    throw new Exception($"当前控制器{c.SystemName} 不可用");
                }
            }
            catch (Exception ex)
            {
                string str = ex.StackTrace;
                //Program.MlilyFourthLog.Error("SetMotorsOn : " + str.Substring(str.LastIndexOf("\\") + 1, str.Length - str.LastIndexOf("\\") - 1) + "--------" + ex.Message);
                return false;
            }
        }
        /// <summary>
        /// 连接机器人
        /// </summary>
        /// <param name="deciceCounts">当前局域网内控制器的数量</param>
        /// <returns></returns>
        private void RobotConnect(int deciceCounts, string ip)
        {
            try
            {
                string str_Error = "";
                if (ns_NetworkScanner == null)
                    ns_NetworkScanner = new NetworkScanner();

                for (int i = 0; i < 5; i++)
                {
                    ns_NetworkScanner.Scan();
                    cic_ControllerInfoCollection = ns_NetworkScanner.Controllers;
                    if (cic_ControllerInfoCollection.Count >= deciceCounts)
                    {
                        break;
                    }
                }

                if (cic_ControllerInfoCollection.Count < deciceCounts)
                {
                    str_Error = "网络上找到的机器人数量<" + cic_ControllerInfoCollection.Count + "，请检查！ from RobotConnect";
                    //MessageBox.Show(str_Error);
                    throw new Exception(str_Error);
                }

                foreach (ControllerInfo ci_ControllerInfo in cic_ControllerInfoCollection)
                {
                    if (ci_ControllerInfo.IPAddress.ToString() == ip)
                    {
                        if (ci_ControllerInfo.Availability != Availability.Available)
                        {
                            str_Error = "机器人控制器不可用，请检查！ from ConnectRobot";
                            MessageBox.Show(str_Error);
                            throw new Exception(str_Error);
                        }
                        else
                        {
                            if (myController != null)
                            {
                                myController.Logoff();
                                myController.Dispose();
                                myController = null;
                            }

                            myController = ControllerFactory.CreateFrom(ci_ControllerInfo);
                            myController.Logon(UserInfo.DefaultUser);

                            if (myController != null && myController.Connected == true)
                            {
                                AddRobotHandle();
                                return;
                            }
                        }
                    }

                }
                str_Error = "机器人IP地址不匹配，请检查！ from ConnectRobot";
                MessageBox.Show(str_Error);
                throw new Exception(str_Error);
            }
            catch (Exception)
            {
                //MessageBox.Show(exc_Exception.ToString() + " from ConnectRobot");
                throw;
            }
        }
        private void AddRobotHandle()
        {
            //rd_pCalibrate = myController.Rapid.GetRapidData("T_ROB1", "DataModule", "pCalibrate");
            //rd_caliPositon = myController.Rapid.GetRapidData("T_ROB1", "DataModule", "caliPositon");
            //lock (lockerMastership)
            //{
            //    using (Mastership ms = Mastership.Request(myController.Rapid))
            //    {
            //        Num loc_calibPosition = new Num() { Value = 0 };
            //        rd_caliPositon.Value = loc_calibPosition;
            //    }
            //}
            //rd_caliPositon.ValueChanged += Rd_caliPositon_ValueChanged;
            //myController.MotionSystem.
            motionSystem = myController.MotionSystem;
        }

       
        private int StartABBRoutine(string routineName, string moduleName = "MianModule")
        {
            try
            {
                ABB.Robotics.Controllers.RapidDomain.Task[] tasks = myController.Rapid.GetTasks();
                if (ControllerState.MotorsOff == myController.State)
                {
                    RobotMotorOn(myController);
                }
                if (myController.CurrentUser != UserInfo.DefaultUser)
                {
                    myController.Logon(ABB.Robotics.Controllers.UserInfo.DefaultUser);
                }
                if (myController.OperatingMode == ControllerOperatingMode.Auto)
                {
                    if (!myController.AuthenticationSystem.CheckDemandGrant(Grant.ExecuteRapid))//可执行
                        myController.AuthenticationSystem.DemandGrant(Grant.ExecuteRapid);
                    //检查任务执行状态，若任务使用中报错：  error "SYS_E_EXEC_LEVEL: Operation is illegal at current execution level" 
                    if (tasks[0].ExecutionStatus == TaskExecutionStatus.Ready || tasks[0].ExecutionStatus == TaskExecutionStatus.Stopped)
                    {
                        lock (lockerMastership)
                        {
                            using (Mastership master = Mastership.Request(myController.Rapid))
                            {
                                myController.Rapid.Stop(StopMode.Immediate);//立即停止
                                tasks[0].SetProgramPointer(moduleName, routineName);//设置程序指针
                                Thread.Sleep(200);
                                if (tasks[0].ProgramPointer.Routine == routineName)
                                {
                                    Thread.Sleep(200);
                                    myController.Rapid.Start(true);//开始程序
                                    return 0;
                                }
                                else
                                {
                                    return -4;
                                }
                            }

                        }
                    }
                    else
                    {
                        return -1;
                    }
                }
                else
                {
                    return -2;
                }

            }
            catch (System.Exception ex)
            {
                string str = ex.StackTrace;
                return -3;
            }
        }
        private void WaitForRoutineFinish()
        {
            try
            { //读取状态
                while (myController.Rapid.GetTask("T_ROB1").ExecutionStatus == TaskExecutionStatus.Running
                    || myController.Rapid.GetTask("T_ROB1").ExecutionStatus == TaskExecutionStatus.UnInitiated
                    || myController.Rapid.GetTask("T_ROB1").ExecutionStatus == TaskExecutionStatus.Unknown)
                {
                    Thread.Sleep(100);
                }
            }
            catch (Exception)
            {
            }
        }
        #endregion

        #region Camera

        private void 连接相机ToolStripMenuItem_Click(object sender, EventArgs e)
        {
            try
            {
                LoadLastUsedDevice();
                IcImageCtrl.LiveDisplayDefault = false;
                IcImageCtrl.LiveDisplaySize = IcImageCtrl.Size;
                FrameNotificationSink bSink = new FrameNotificationSink(FramProcessing, MediaSubtypes.Y800);
                IcImageCtrl.Sink = bSink;
                IcImageCtrl.LiveStart();
                txtMsg.Text += DateTime.Now.ToString("HH:mm:ss") + "：相机连接成功\r\n";
            }
            catch (Exception ex)
            {
                MessageBox.Show("相机连接失败\r\n" + ex.Message);
            }

        }

        private void 相机设置ToolStripMenuItem_Click(object sender, EventArgs e)
        {
            try
            {
                ShowProperties(IcImageCtrl);

                if (File.Exists("lastSelectedDeviceState.xml"))
                {
                    IcImageCtrl.SaveDeviceStateToFile("lastSelectedDeviceState.xml");
                }
                else
                {
                    File.Create("lastSelectedDeviceState.xml");
                    IcImageCtrl.SaveDeviceStateToFile("lastSelectedDeviceState.xml");
                }

                MessageBox.Show("The camera parameters are set successfully.", "Camera Setting");
            }
            catch (Exception ex)
            {
                MessageBox.Show("Description Failed to set camera parameters.\r\nReason：\r\n" + ex.Message, "Camera Setting");
            }
        }

        private void 软触发ToolStripMenuItem_Click(object sender, EventArgs e)
        {
            SoftTrigger(IcImageCtrl);
        }

        private void 保存当前帧ToolStripMenuItem_Click(object sender, EventArgs e)
        {
            if (myBmp != null)
            {
                myBmp.Save("Bg.bmp");
                txtMsg.Text += DateTime.Now.ToString("HH:mm:ss") + "：背景图像已保存\r\n";
            }
        }
        private void LoadLastUsedDevice()
        {
            if (File.Exists("lastSelectedDeviceState.xml"))
            {
                IcImageCtrl.LoadDeviceStateFromFile("lastSelectedDeviceState.xml", true);
            }
            else if (!IcImageCtrl.LoadShowSaveDeviceState("lastSelectedDeviceState.xml"))
            {
                throw new Exception("No device was selected.");
                //MessageBox.Show("No device was selected.", "Grabbing an Image", MessageBoxButtons.OK, MessageBoxIcon.Information);
                //this.Close();
            }
        }


        private void FramProcessing(IFrame frame)
        {
            myBmp = frame.CreateBitmapWrap();
            txtMsg.Invoke(new Action(() => {
                txtMsg.Text += DateTime.Now.ToString("HH:mm:ss") + $"图像获取成功\r\n";
            }));

        }
        /// <summary>
        /// 设置相机参数
        /// </summary>
        /// <param name="ICC"></param>
        private void ShowProperties(TIS.Imaging.ICImagingControl ICC)
        {
            if (ICC.DeviceValid)
            {
                ICC.ShowPropertyDialog();
            }
            else
            {
                MessageBox.Show("ICC camera found");
            }
        }
        /// <summary>
        /// 触发
        /// </summary>
        /// <param name="ICC"></param>
        private void SoftTrigger(TIS.Imaging.ICImagingControl ICC)
        {
            try
            {
                TIS.Imaging.VCDPropertyItem trigger = ICC.VCDPropertyItems.FindItem(TIS.Imaging.VCDGUIDs.VCDID_TriggerMode);//控制触发
                if (trigger != null)
                {
                    TIS.Imaging.VCDButtonProperty Softtrigger = trigger.Find<TIS.Imaging.VCDButtonProperty>(TIS.Imaging.VCDGUIDs.VCDElement_SoftwareTrigger);
                    if (Softtrigger == null)
                    {
                        throw new Exception("Soft Trigger is not supported by the current device!");
                    }
                    else
                    {
                        //Console.WriteLine("Soft trigger once");
                        Softtrigger.Push();
                    }
                }
                else
                {
                    throw new Exception("Soft Trigger is not supported by the current device!");
                }
            }
            catch (Exception ex)
            {
                throw ex;
            }
        }
        #endregion

        #region User Define

        /// <summary>
        /// corners detection//角点检测
        /// </summary>
        /// <param name="chessboardImage">chessboard image</param>
        /// <param name="cornersDetected">corners value in image coordinate</param>
        /// <returns>
        /// return true if success
        /// </returns>
        private bool FindCorners(ref Image<Gray, byte> chessboardImage, out VectorOfPointF cornersDetected)
        {
            //cornersDetected = new PointF[nPoints];
            cornersDetected = new VectorOfPointF(88);
            CvInvoke.FindChessboardCorners(chessboardImage, patternSize, cornersDetected,
                            CalibCbType.AdaptiveThresh | CalibCbType.NormalizeImage);

            if (null == cornersDetected)
            {
                return false;
            }
            else
            {
                //函数 cvFindCornerSubPix 通过迭代来发现具有子象素精度的角点位置：
                CvInvoke.CornerSubPix(chessboardImage, cornersDetected,
                    new Size(5, 5), new Size(-1, -1), criteria);

                return true;
            }
        }

        /// <summary>
        /// points in world coordinate保存标定板上角点的三维坐标
        /// </summary>
        /// <param name="corners3D">coordinate value</param>
        /// <param name="chessBoardSize">size of chessboard</param>
        /// <param name="nImages">number of images</param>
        /// <param name="squareSize">actual size of square棋盘格实际尺寸</param>
        private void objectCorners3D(Size chessBoardSize, float squareSize, out VectorOfPoint3D32F corners3D)
        {
            MCvPoint3D32f[] _3D = new MCvPoint3D32f[chessBoardSize.Width * chessBoardSize.Height];
            int currentRow, currentCol;
            for (currentRow = 0; currentRow < chessBoardSize.Height; currentRow++)
            {
                for (currentCol = 0; currentCol < chessBoardSize.Width; currentCol++)
                {
                    int nPoint = currentRow * chessBoardSize.Width + currentCol;
                    _3D[nPoint].X = (float)currentCol * squareSize;
                    _3D[nPoint].Y = (float)currentRow * squareSize;
                    _3D[nPoint].Z = (float)0.0f;
                }
            }
            corners3D = new VectorOfPoint3D32F(_3D);
        }
        /// <summary>
        /// 获取ROI
        /// </summary>
        /// <param name="image">需裁剪的原图</param>
        /// <param name="rect">裁剪留下的ROI大小</param>
        /// <returns>ROI</returns>
        private Image<Gray, byte> GetROI(Image<Gray, byte> image, Rectangle rect)
        {
            //程序中image是原始图像，类型Image<Gray, byte>，rectangle是矩形，CropImage是截得的图像。
            Image<Gray, byte> resImag = image.CopyBlank();
            using (var mask = new Image<Gray, byte>(image.Size))
            {
                mask.SetZero();//设置所有值为0
                mask.ROI = rect;
                mask.SetValue(255);//设置ROI的值为255
                mask.ROI = Rectangle.Empty;//去掉ROI
                                           //res(I)=img1(I)+img2(I) if mask(I)!=0
                CvInvoke.BitwiseAnd(image, mask, resImag);
            }
            return resImag;
        }
        private Matrix<double> MatMultiply(Matrix<double> A, Matrix<double> B)
        {
            Matrix<double> resM = new Matrix<double>(A.Rows, B.Cols);
            if (A.Cols != B.Rows)
            {
                resM = null;
                throw new Exception("A矩阵的列数不等于B矩阵的行数，无法计算");
            }
            else
            {
                for (int i = 0; i < A.Rows; i++)
                {
                    for (int j = 0; j < B.Cols; j++)
                    {
                        for (int k = 0; k < A.Cols; k++)
                        {
                            resM[i, j] += A[i, k] * B[k, j];
                        }
                    }
                }
            }
            return resM;
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
                im00 = double.Parse(root.Element("camera").Element("Matrix00").Value);
                im01 = double.Parse(root.Element("camera").Element("Matrix01").Value);
                im02 = double.Parse(root.Element("camera").Element("Matrix02").Value);

                im11 = double.Parse(root.Element("camera").Element("Matrix11").Value);
                im12 = double.Parse(root.Element("camera").Element("Matrix12").Value);

                em00 = double.Parse(root.Element("camera").Element("exMatrix00").Value);
                em01 = double.Parse(root.Element("camera").Element("exMatrix01").Value);
                em02 = double.Parse(root.Element("camera").Element("exMatrix02").Value);

                em10 = double.Parse(root.Element("camera").Element("exMatrix10").Value);
                em11 = double.Parse(root.Element("camera").Element("exMatrix11").Value);
                em12 = double.Parse(root.Element("camera").Element("exMatrix12").Value);

                em20 = double.Parse(root.Element("camera").Element("exMatrix20").Value);
                em21 = double.Parse(root.Element("camera").Element("exMatrix21").Value);
                em22 = double.Parse(root.Element("camera").Element("exMatrix22").Value);

                k1 = double.Parse(root.Element("camera").Element("DistCoeffs00").Value);
                k2 = double.Parse(root.Element("camera").Element("DistCoeffs10").Value);

                chessXcount = int.Parse(root.Element("Chess").Element("ChessCornerL").Value);
                chessYcount = int.Parse(root.Element("Chess").Element("ChessCornerS").Value);
                chessSquarel = int.Parse(root.Element("Chess").Element("Squarelength").Value);

                resolutionW = int.Parse(root.Element("ResolvingPower").Element("WIDTH").Value);
                resolutionH = int.Parse(root.Element("ResolvingPower").Element("HEIGHT").Value);

                moduleName = root.Element("ABB").Element("rapid").Value;
                robotIp = root.Element("ABB").Element("ip").Value;

                transParams[0] = float.Parse(root.Element("TranslationParam").Element("kx").Value);
                transParams[1] = float.Parse(root.Element("TranslationParam").Element("bx").Value);
                transParams[2] = float.Parse(root.Element("TranslationParam").Element("ky").Value);
                transParams[3] = float.Parse(root.Element("TranslationParam").Element("by").Value);
                transParams[4] = float.Parse(root.Element("TranslationParam").Element("kz").Value);
                transParams[5] = float.Parse(root.Element("TranslationParam").Element("bz").Value);

                diskDepth = float.Parse(root.Element("DiskDepth").Value);
            }
            catch (Exception ex)
            {
                MessageBox.Show("加载配置参数出错，原因：\r\n" + ex.Message);
                return;
            }
        }

        private void PointsToWorld(Matrix<double> InvMat, float imageX, float imageY, out double worldX, out double worldY)
        {
            Matrix<double> imgP = new Matrix<double>(3, 1);
            imgP[0, 0] = imageX;
            imgP[1, 0] = imageY;
            imgP[2, 0] = 1;
            Matrix<double> worP = new Matrix<double>(3, 1);

            worP = MatMultiply(InvMat, imgP);
            worldX = worP[0, 0] / worP[2, 0];
            worldY = worP[1, 0] / worP[2, 0];
        }

        private void 加载标定结果ToolStripMenuItem_Click(object sender, EventArgs e)
        {
            try
            {
                // 创建一个 StreamReader 的实例来读取文件 
                // using 语句也能关闭 StreamReader
                using (StreamReader sr = new StreamReader(".\\Results\\EyeToHandMat.txt"))
                {
                    string line;
                    int lineNum = 0;
                    // 从文件读取并显示行，直到文件的末尾 
                    while ((line = sr.ReadLine()) != null)
                    {
                        Console.WriteLine(line);
                        if (lineNum == 1)
                        {
                            rigidTransform[0, 0] = double.Parse(line.Split(',')[0]);
                            rigidTransform[0, 1] = double.Parse(line.Split(',')[1]);
                            rigidTransform[0, 2] = double.Parse(line.Split(',')[2]);
                        }
                        else if (lineNum == 2)
                        {
                            rigidTransform[1, 0] = double.Parse(line.Split(',')[0]);
                            rigidTransform[1, 1] = double.Parse(line.Split(',')[1]);
                            rigidTransform[1, 2] = double.Parse(line.Split(',')[2]);
                        }

                        lineNum++;
                    }
                }
                txtMsg.Text += DateTime.Now.ToString("HH:mm:ss") + "：标定参数加载完成\r\n";
            }
            catch (Exception ex)
            {
                // 向用户显示出错消息
                Console.WriteLine("The file could not be read:");
                Console.WriteLine(ex.Message);
            }
        }

        private void LoadMatrix()
        {
            //01 - Camera
            intrinsicMatrix[0, 0] = im00;
            intrinsicMatrix[0, 1] = im01;
            intrinsicMatrix[0, 2] = im02;
            intrinsicMatrix[1, 0] = 0;
            intrinsicMatrix[1, 1] = im11;
            intrinsicMatrix[1, 2] = im12;
            intrinsicMatrix[2, 0] = 0;
            intrinsicMatrix[2, 1] = 0;
            intrinsicMatrix[2, 2] = 1;
            distCoeffs[0, 0] = k1;
            distCoeffs[1, 0] = k2;
            distCoeffs[2, 0] = 0;
            distCoeffs[3, 0] = 0;
            distCoeffs[4, 0] = 0;
            //整理外参矩阵
            extrinsicMatrix[0, 0] = em00;
            extrinsicMatrix[0, 1] = em01;
            extrinsicMatrix[0, 2] = em02;
            extrinsicMatrix[0, 3] = 0;
            extrinsicMatrix[1, 0] = em10;
            extrinsicMatrix[1, 1] = em11;
            extrinsicMatrix[1, 2] = em12;
            extrinsicMatrix[1, 3] = 0;
            extrinsicMatrix[2, 0] = em20;
            extrinsicMatrix[2, 1] = em21;
            extrinsicMatrix[2, 2] = em22;
            //生产机器人的9点

            //Console.WriteLine("nineRobotPoints Count :"+ nineRobotPoints.Count<MCvPoint3D32f>());
        }



        /// <summary>
        /// 计算欧拉角
        /// </summary>
        /// <param name="mat"></param>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <param name="z"></param>
        private void CalEulerAngel(Mat mat, out double x, out double y, out double z)
        {
            x=y=z =0;
            /*  sy = math.sqrt(m[0,0]*m[0,0]+m[1,0]*m[1,0])
                if(sy < 1e-6):
                    x = math.atan2(-m[1,2],m[1,1])
                    y = math.atan2(-m[2,0],sy)
                    z = 0
                else:
                    x = math.atan2(m[2,1],m[2,2])
                    y = math.atan2(-m[2,0],sy)
                    z = math.atan2(m[1,0],m[0,0])
                x = x * 180.0 / math.pi
                y = y * 180.0 / math.pi
                z = z * 180.0 / math.pi
                print('Ex = ',x)
                print('Ey = ',y)
                print('Ez = ',z)
           */
            //double[] _00 = MatExtension.GetValue1(mat, 0, 0);
            //Console.WriteLine($"_00 = {_00[0]}");
            //double[] _01 = MatExtension.GetValue1(mat, 0, 1);
            //Console.WriteLine($"_01 = {_01[0]}");
            //double[] _02 = MatExtension.GetValue1(mat, 0, 2);
            //Console.WriteLine($"_02 = {_02[0]}");
            //double[] _10 = MatExtension.GetValue1(mat, 1, 0);
            //Console.WriteLine($"_10 = {_10[0]}");
            //double[] _11 = MatExtension.GetValue1(mat, 1, 1);
            //Console.WriteLine($"_11 = {_11[0]}");
            //double[] _12 = MatExtension.GetValue1(mat, 1, 2);
            //Console.WriteLine($"_12 = {_12[0]}");
            //double[] _20 = MatExtension.GetValue1(mat, 2, 0);
            //Console.WriteLine($"_20 = {_20[0]}");
            //double[] _21 = MatExtension.GetValue1(mat, 2, 1);
            //Console.WriteLine($"_21 = {_21[0]}");
            //double[] _22 = MatExtension.GetValue1(mat, 2, 2);
            //Console.WriteLine($"_22 = {_22[0]}");

            //double sy = Math.Sqrt(MatExtension.GetValue1(mat, 0, 0)[0] * MatExtension.GetValue1(mat, 0, 0)[0] + MatExtension.GetValue1(mat, 1, 0)[0] * MatExtension.GetValue1(mat, 1, 0)[0]);
            //if (sy < 1E-6)
            //{
            //    x = Math.Atan2(-MatExtension.GetValue1(mat, 1, 2)[0], MatExtension.GetValue1(mat, 1, 1)[0]);
            //    y = Math.Atan2(-MatExtension.GetValue1(mat, 2, 0)[0], sy);
            //    z = 0;
            //}
            //else
            //{
            //    x = Math.Atan2(MatExtension.GetValue1(mat, 2, 1)[0], MatExtension.GetValue1(mat, 2, 2)[0]);
            //    y = Math.Atan2(-MatExtension.GetValue1(mat, 2, 0)[0], sy);
            //    z = Math.Atan2(MatExtension.GetValue1(mat, 1, 0)[0], MatExtension.GetValue1(mat, 0, 0)[0]);
            //}

            Array matDatas = mat.GetData();
            double n_00 = double.Parse(matDatas.GetValue(0, 0).ToString());
            double n_01 = double.Parse(matDatas.GetValue(0, 1).ToString());
            double n_02 = double.Parse(matDatas.GetValue(0, 2).ToString());
            double n_10 = double.Parse(matDatas.GetValue(1, 0).ToString());
            double n_11 = double.Parse(matDatas.GetValue(1, 1).ToString());
            double n_12 = double.Parse(matDatas.GetValue(1, 2).ToString());
            double n_20 = double.Parse(matDatas.GetValue(2, 0).ToString());
            double n_21 = double.Parse(matDatas.GetValue(2, 1).ToString());
            double n_22 = double.Parse(matDatas.GetValue(2, 2).ToString());
            double n_sy = Math.Sqrt(n_00 * n_00 + n_10 * n_10);
            if (n_sy < 1E-6)
            {
                x = Math.Atan2(-n_12, n_11);
                y = Math.Atan2(-n_20, n_sy);
                z = 0;
            }
            else
            {
                x = Math.Atan2(n_21, n_22);
                y = Math.Atan2(-n_20, n_sy);
                z = Math.Atan2(n_10, n_00);
            }
            x = x * 180.0 / Math.PI;
            y = y * 180.0 / Math.PI;
            z = z * 180.0 / Math.PI;
             
        }
        #endregion

    }
}
