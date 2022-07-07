using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Features2D;
using Emgu.CV.Structure;
using Emgu.CV.Util;
using MvCamCtrl.NET;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Drawing.Imaging;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Xml.Linq;

namespace CameraRobotCalibration
{
    public partial class MainForm : Form
    {
        private int funTag = 0;

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

        string robotIp;//机器人IP
        string moduleName;//任务块名称
        #endregion
        #region HK Camera Variable
        MyCamera.MV_CC_DEVICE_INFO_LIST m_stDeviceList = new MyCamera.MV_CC_DEVICE_INFO_LIST();//设备列表
        MyCamera WmHKC = new MyCamera();
        bool hk_bGrabbing = false;

        bool hk_getpicture = false;
        bool outOfSafeZone = false;//底棉位置超出安全放置范围
        bool timesExhaustion = false;//3次机会耗尽

        Thread m_hReceiveThread = null;
        MyCamera.MV_FRAME_OUT_INFO_EX stFrameInfo = new MyCamera.MV_FRAME_OUT_INFO_EX();
        MyCamera.MV_FRAME_OUT_INFO_EX m_stFrameInfo = new MyCamera.MV_FRAME_OUT_INFO_EX();//输出帧信息
        Bitmap bmp = null;
        // ch:用于从驱动获取图像的缓存 | en:Buffer for getting image from driver
        uint m_nBufSizeForDriver = 0;
        private static Object BufForDriverLock = new Object();//标准锁对象
        IntPtr m_BufForDriver;

        private string cameraIP;
        private string pcIP;

        System.Timers.Timer testHKFrameTimer;
        #endregion

        public MainForm()
        {
            InitializeComponent();
        }

        private void MainForm_Load(object sender, EventArgs e)
        {
            try
            {

                //初始化相机
                //homogeneousMat = new Matrix<double>(3, 4);
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


                LoadMatrix();
                txtMsg.Text += DateTime.Now.ToString("HH:mm:ss") + "：将机器人切换到自动模式\r\n";
                txtMsg.Text += DateTime.Now.ToString("HH:mm:ss") + "：请先【连接机器人】和【连接相机】后启动程序\r\n";
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
            }
        }
        private void btnSet_Click(object sender, EventArgs e)
        {
            SysParamsForm sysParamsForm = new SysParamsForm();
            sysParamsForm.ShowDialog();
        }

        #region Camera Methods
        /// <summary>
        /// 打开相机
        /// </summary>
        private void OpenDevice(string CameraIP, string PcIP)
        {
            MyCamera.MV_CC_DEVICE_INFO stDevInfo = new MyCamera.MV_CC_DEVICE_INFO();
            MyCamera.MV_GIGE_DEVICE_INFO stGigEDev = new MyCamera.MV_GIGE_DEVICE_INFO();
            stDevInfo.nTLayerType = MyCamera.MV_GIGE_DEVICE;
            var parts = CameraIP.Split('.');
            try
            {
                int nIp1 = Convert.ToInt32(parts[0]);
                int nIp2 = Convert.ToInt32(parts[1]);
                int nIp3 = Convert.ToInt32(parts[2]);
                int nIp4 = Convert.ToInt32(parts[3]);
                stGigEDev.nCurrentIp = (uint)((nIp1 << 24) | (nIp2 << 16) | (nIp3 << 8) | nIp4);

                parts = PcIP.Split('.');
                nIp1 = Convert.ToInt32(parts[0]);
                nIp2 = Convert.ToInt32(parts[1]);
                nIp3 = Convert.ToInt32(parts[2]);
                nIp4 = Convert.ToInt32(parts[3]);
                stGigEDev.nNetExport = (uint)((nIp1 << 24) | (nIp2 << 16) | (nIp3 << 8) | nIp4);
            }
            catch
            {
                throw new Exception("输入IP无效");
            }
            // stGigEDev结构体转为stDevInfo.SpecialInfo.stGigEInfo(Byte[])
            IntPtr stGigeInfoPtr = Marshal.AllocHGlobal(Marshal.SizeOf(stGigEDev));
            Marshal.StructureToPtr(stGigEDev, stGigeInfoPtr, false);
            stDevInfo.SpecialInfo.stGigEInfo = new System.Byte[Marshal.SizeOf(stDevInfo.SpecialInfo)];
            Marshal.Copy(stGigeInfoPtr, stDevInfo.SpecialInfo.stGigEInfo, 0, Marshal.SizeOf(stDevInfo.SpecialInfo));
            Marshal.Release(stGigeInfoPtr);

            int nRet = WmHKC.MV_CC_CreateDevice_NET(ref stDevInfo);
            if (MyCamera.MV_OK != nRet)
            {
                return;
            }

            nRet = WmHKC.MV_CC_OpenDevice_NET();
            if (MyCamera.MV_OK != nRet)
            {
                WmHKC.MV_CC_DestroyDevice_NET();
                WmHKC.MV_CC_CloseDevice_NET();
                return;
            }

            // ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal package size(It only works for the GigE camera)
            if (stDevInfo.nTLayerType == MyCamera.MV_GIGE_DEVICE)
            {
                int nPacketSize = WmHKC.MV_CC_GetOptimalPacketSize_NET();
                if (nPacketSize > 0)
                {
                    nRet = WmHKC.MV_CC_SetIntValue_NET("GevSCPSPacketSize", (uint)nPacketSize);
                    if (nRet != MyCamera.MV_OK)
                    {
                        ShowErrorMsg("Set Packet Size failed!", nRet);
                    }
                }
                else
                {
                    ShowErrorMsg("Get Packet Size failed!", nPacketSize);
                }
            }
            WmHKC.MV_CC_SetEnumValue_NET("ExposureAuto", 0);
            //WmHKC.MV_CC_SetFloatValue_NET("ExposureTime", expTime);
            WmHKC.MV_CC_SetFloatValue_NET("ExposureTime", 800000);
            WmHKC.MV_CC_SetEnumValue_NET("GainAuto", 0);
            //nRet = WmHKC.MV_CC_SetFloatValue_NET("Gain", gainValue);
            nRet = WmHKC.MV_CC_SetFloatValue_NET("Gain", 0);
            if (nRet != MyCamera.MV_OK)
            {
                ShowErrorMsg("Set Gain Fail!", nRet);
            }
           
            // ch:设置采集连续模式 | en:Set Continues Aquisition Mode
            //ThirdHKC.MV_CC_SetEnumValue_NET("AcquisitionMode", (uint)MyCamera.MV_CAM_ACQUISITION_MODE.MV_ACQ_MODE_CONTINUOUS);
            //ThirdHKC.MV_CC_SetEnumValue_NET("ExposureAuto", (uint)MyCamera.MV_CAM_EXPOSURE_AUTO_MODE.MV_EXPOSURE_AUTO_MODE_CONTINUOUS);
            //SetParam();// ch:获取参数 | en:Get parameters
            WmHKC.MV_CC_SetEnumValue_NET("TriggerMode", (uint)MyCamera.MV_CAM_TRIGGER_MODE.MV_TRIGGER_MODE_ON);
            //ThirdHKC.MV_CC_SetEnumValue_NET("TriggerMode", (uint)MyCamera.MV_CAM_TRIGGER_MODE.MV_TRIGGER_MODE_OFF);
            //设置为软触发
            WmHKC.MV_CC_SetEnumValue_NET("TriggerSource", (uint)MyCamera.MV_CAM_TRIGGER_SOURCE.MV_TRIGGER_SOURCE_SOFTWARE);//软
            WmHKC.MV_CC_SetIntValue_NET("AcquisitionBurstFrameCount", 1);
            //设置IO
            nRet = WmHKC.MV_CC_SetEnumValue_NET("LineSelector", 1);
            nRet = WmHKC.MV_CC_SetIntValue_NET("StrobeEnable", 1);
            
            //if (MyCamera.MV_OK != nRet)
            //{
            //    ShowErrorMsg("Set Fail!", nRet);
            //    return;
            //}
            //m_MyCamera.MV_CC_SetEnumValue_NET("TriggerSource", (uint)MyCamera.MV_CAM_TRIGGER_SOURCE.MV_TRIGGER_SOURCE_LINE0);//硬
            //摄像头在使用中，如果长时间没有输出信号到网络，会产生假死机现象，
            //海康摄像头的心跳功能即不间断的与外围设备相互连接，这种连接不会产生很多数据，但可以不断的让摄像头保持“清醒”。
            WmHKC.MV_CC_SetIntValueEx_NET("GevHeartbeatTimeout", 2000);//设置心跳时间，不然程序异常退出，相机还在占用资源
        }
        /// <summary>
        /// 开始采集
        /// </summary>
        private void StartGrab()
        {
            hk_bGrabbing = true;

            m_hReceiveThread = new Thread(ReceiveThreadProcess);
            m_hReceiveThread.IsBackground = true;//设置为后台线程
            m_hReceiveThread.Start();

            m_stFrameInfo.nFrameLen = 0;//取流之前先清除帧长度
            m_stFrameInfo.enPixelType = MyCamera.MvGvspPixelType.PixelType_Gvsp_Undefined;
            // ch:开始采集 | en:Start Grabbing
            int nRet = WmHKC.MV_CC_StartGrabbing_NET();
            if (MyCamera.MV_OK != nRet)
            {
                hk_bGrabbing = false;
                m_hReceiveThread.Join();
                ShowErrorMsg("Start Grabbing Fail!", nRet);
                return;
            }
        }
        private void ReceiveThreadProcess()
        {
            MyCamera.MVCC_INTVALUE stParam = new MyCamera.MVCC_INTVALUE();
            //获取包大小
            int nRet = WmHKC.MV_CC_GetIntValue_NET("PayloadSize", ref stParam);
            if (MyCamera.MV_OK != nRet)
            {
                ShowErrorMsg("Get PayloadSize failed", nRet);
                return;
            }

            UInt32 nPayloadSize = stParam.nCurValue;
            if (nPayloadSize > m_nBufSizeForDriver)
            {
                if (m_BufForDriver != IntPtr.Zero)
                {
                    Marshal.Release(m_BufForDriver);
                }
                m_nBufSizeForDriver = nPayloadSize;
                //开辟内存空间，与包大小相同，从底层驱动接收图像数据
                m_BufForDriver = Marshal.AllocHGlobal((Int32)m_nBufSizeForDriver);
            }

            if (m_BufForDriver == IntPtr.Zero)
            {
                return;
            }
            //MyCamera.MV_DISPLAY_FRAME_INFO stDisplayInfo = new MyCamera.MV_DISPLAY_FRAME_INFO();
            int triggerTimes = 0;
            while (hk_bGrabbing)
            {
                lock (BufForDriverLock)
                {
                    try
                    {
                        //获取一阵图像，参数：用于保存图像的缓存地址：：缓存大小：：获取到的帧信息：：设置超时时间
                        //该接口为主动式获取帧数据，上层应用程序需要根据帧率，控制好调用该接口的频率。
                        //该接口支持设置超时时间，SDK内部等待直到有数据时返回，可以增加取流平稳性，适合用于对平稳性要求较高的场合。
                        nRet = WmHKC.MV_CC_GetOneFrameTimeout_NET(m_BufForDriver, nPayloadSize, ref stFrameInfo, 1000);
                        //nRet = SecondHKC.MV_CC_GetImageBuffer_NET(ref stFrameInfo, 1000);
                        if (nRet == MyCamera.MV_OK)
                        {
                            Console.WriteLine("Get One Frame OK");
                            m_stFrameInfo = stFrameInfo;
                            //************************Mono8 转 Bitmap*******************************
                            bmp = new Bitmap(stFrameInfo.nWidth, stFrameInfo.nHeight, stFrameInfo.nWidth * 1, PixelFormat.Format8bppIndexed, m_BufForDriver);

                            ColorPalette cp = bmp.Palette;
                            // init palette
                            for (int i = 0; i < 256; i++)
                            {
                                cp.Entries[i] = Color.FromArgb(i, i, i);
                            }
                            // set palette back
                            bmp.Palette = cp;
                            //释放内存
                            //SecondHKC.MV_CC_FreeImageBuffer_NET(ref stFrameInfo);
                            //TODO：图像处理--->结果：传递数据
                            Image<Gray, byte> grayImg = new Image<Gray, byte>(bmp);

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
                            Image<Rgb, byte> rgbImg = new Image<Rgb, byte>(remapImg.Bitmap);
                            switch (funTag)
                            {
                                case 0:
                                    #region 调平
                                    Rectangle roiRect = new Rectangle(800, 370, 3700, 2880);
                                    remapImg = GetROI(remapImg, roiRect);
                                    //CvInvoke.Threshold(remapImg, remapImg, 0, 255, ThresholdType.Otsu);//
                                    remapImg.Bitmap.Save("ThresholdImg.bmp");

                                    VectorOfPointF corners;
                                    Mat revc = new Mat();
                                    Mat tevc = new Mat();
                                    Mat inliners = new Mat();
                                    VectorOfPoint3D32F corners3D = new VectorOfPoint3D32F();
                                    objectCorners3D(patternSize, square, out corners3D);
                                    FindCorners(ref remapImg, out corners);

                                    CvInvoke.SolvePnPRansac(corners3D, corners, intrinsicMatrix, distCoeffs, revc, tevc, true, 10000, 1, 0.99, inliners, SolvePnpMethod.Iterative);
                                    //获得的旋转矩阵是向量，是3×1的矩阵，想要还原回3×3的矩阵，需要罗德里格斯变换Rodrigues
                                    //try
                                    //{

                                    //}
                                    //catch (Exception ex)
                                    //{
                                    //    CvInvoke.Rectangle(rgbImg, roiRect, new MCvScalar(0, 255, 0), 15);
                                    //    pictureBox2.Image = rgbImg.Bitmap;
                                    //    MessageBox.Show("输入图像不合格，确认标定板在绿色矩形框内\r\n请重新取像计算或重新设置ROI大小！\r\n" + ex.Message, "计算失败");
                                    //    return;
                                    //}

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
                                    CvInvoke.Circle(rgbImg, new Point((int)p0.X, (int)p0.Y), 10, new MCvScalar(5, 255, 5), 10);
                                    CvInvoke.ArrowedLine(rgbImg, new Point((int)p0.X, (int)p0.Y), new Point((int)p7.X, (int)p7.Y), new MCvScalar(0, 255, 255), 10);
                                    CvInvoke.ArrowedLine(rgbImg, new Point((int)p0.X, (int)p0.Y), new Point((int)p77.X, (int)p77.Y), new MCvScalar(255, 0, 0), 10);
                                    CvInvoke.PutText(rgbImg, "X", new Point((int)p7.X - 85, (int)p7.Y + 30), FontFace.HersheyComplex, 3, new MCvScalar(0, 0, 255), 10);
                                    CvInvoke.PutText(rgbImg, "Y", new Point((int)p77.X + 10, (int)p77.Y - 50), FontFace.HersheyComplex, 3, new MCvScalar(255, 0, 0), 10);

                                    Console.WriteLine($"EX = {x}");
                                    Console.WriteLine($"EY = {y}");
                                    Console.WriteLine($"EZ = {z}");
                                    CvInvoke.Rectangle(rgbImg, roiRect, new MCvScalar(0, 255, 0), 15);
                                    txtMsg.Invoke(new Action(() => {
                                        txtMsg.Text = "";
                                        //txtMsg.BackColor = Color.Green;
                                        txtMsg.Text += $"EX = {x.ToString("f8")}f\r\n";
                                        txtMsg.Text += $"EY = {y.ToString("f8")}\r\n";
                                        txtMsg.Text += $"EZ = {z.ToString("f8")}\r\n";
                                        pictureBox2.Image = rgbImg.Bitmap;
                                        //ShowGrayImg(remapImg);
                                    }));
                                    #endregion
                                    break;
                                case 1:
                                    #region 标志物圆盘定位
                                    SimpleBlobDetectorParams blobparams = new SimpleBlobDetectorParams();
                                    blobparams.FilterByArea = true;
                                    blobparams.MinArea = 20000;
                                    blobparams.MaxArea = 500000000;
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
                                    Rectangle targetRect = new Rectangle(500, 100, 4900, 3500);
                                    //remapImg = GetROI(remapImg, targetRect);
                                    //CvInvoke.Threshold(remapImg, remapImg, 0, 255, ThresholdType.Otsu);//
                                    Emgu.CV.UI.ImageViewer.Show(remapImg);
                                    MKeyPoint[] keypoints = detector.Detect(remapImg);
                                    //目标矩形
                                    Rectangle targRect = new Rectangle();
                                    
                                    foreach (MKeyPoint keypoint in keypoints)
                                    {
                                        targRect = new Rectangle((int)(keypoint.Point.X - keypoint.Size / 2), (int)(keypoint.Point.Y - keypoint.Size / 2), (int)keypoint.Size, (int)keypoint.Size);
                                        CvInvoke.Rectangle(rgbImg, targRect, new MCvScalar(255, 0, 0), 2);
                                        CvInvoke.Circle(rgbImg, new Point((int)(keypoint.Point.X), (int)(keypoint.Point.Y)), (int)keypoint.Size / 2, new MCvScalar(0, 0, 255), 5);
                                        CvInvoke.Circle(rgbImg, new Point((int)(keypoint.Point.X), (int)(keypoint.Point.Y)), 5, new MCvScalar(0, 0, 255), 5);
                                        //Console.WriteLine($"BlobCenter X = {keypoint.Point.X}");
                                        //Console.WriteLine($"BlobCenter Y = {keypoint.Point.Y}");
                                    }

                                    //CircleF[] circles = CvInvoke.HoughCircles(roiImg, HoughType.Gradient, 1.01, 80.0, 100, 70, 80, 300);
                                    //foreach (var circle in circles)
                                    //{
                                    //    CvInvoke.Circle(bgrInput, System.Drawing.Point.Round(circle.Center), (int)circle.Radius, new Bgr(Color.Red).MCvScalar, 2);
                                    //}
                                    if (keypoints.Length < 1)
                                    {
                                        throw new Exception("未找到合适的标定物！\r\n请检查标定物是否在ROI内。");
                                    }
                                    else
                                    {
                                        //rgbImg.Save("circles.bmp");
                                        txtMsg.Invoke(new Action(() => {
                                            txtMsg.Text = "";
                                            txtMsg.Text += DateTime.Now.ToString()+"\r\n";
                                            //txtMsg.BackColor = Color.Green;
                                            txtMsg.Text += $"CX = {keypoints[0].Point.X.ToString("f2")}\r\n";
                                            txtMsg.Text += $"CY = {keypoints[0].Point.Y.ToString("f2")}\r\n";
                                            
                                            pictureBox1.Image = rgbImg.Bitmap;
                                            //ShowGrayImg(remapImg);
                                        }));
                                    }
                                   
                                    #endregion
                                    break;
                                default:
                                    break;
                            }
                            

                        }
                    }
                    catch (Exception ex)
                    {
                        txtMsg.Invoke(new Action(() => {
                            txtMsg.Text = "";
                            txtMsg.Text += DateTime.Now.ToString() + "\r\n";
                            txtMsg.Text += ex.Message;
                        }));
                    }
                    
                }
            }
        }
        private void CloseHKCamera()
        {
            if (hk_bGrabbing)
            {
                hk_bGrabbing = false;
                //while (m_hReceiveThread.ThreadState != ThreadState.Stopped)//必须等线程完全停止了，否则会出现冲突。 
                // {
                //Thread.Sleep(100);
                m_hReceiveThread.Join(1000);//会阻塞到farmFrame线程结束
                //}
            }
            //释放采集图像所占内存
            if (m_BufForDriver != IntPtr.Zero)
            {
                Marshal.Release(m_BufForDriver);
            }
            //关闭设备
            WmHKC.MV_CC_CloseDevice_NET();
            //销毁设备
            WmHKC.MV_CC_DestroyDevice_NET();
        }
        /// <summary>
        /// 显示相机异常信息
        /// </summary>
        /// <param name="csMessage"></param>
        /// <param name="nErrorNum"></param>
        private void ShowErrorMsg(string csMessage, int nErrorNum)
        {
            string errorMsg;
            if (nErrorNum == 0)
            {
                errorMsg = csMessage;
            }
            else
            {
                errorMsg = csMessage + ": Error =" + System.String.Format("{0:X}", nErrorNum);
            }

            switch (nErrorNum)
            {
                case MyCamera.MV_E_HANDLE: errorMsg += " Error or invalid handle "; break;
                case MyCamera.MV_E_SUPPORT: errorMsg += " Not supported function "; break;
                case MyCamera.MV_E_BUFOVER: errorMsg += " Cache is full "; break;
                case MyCamera.MV_E_CALLORDER: errorMsg += " Function calling order error "; break;
                case MyCamera.MV_E_PARAMETER: errorMsg += " Incorrect parameter "; break;
                case MyCamera.MV_E_RESOURCE: errorMsg += " Applying resource failed "; break;
                case MyCamera.MV_E_NODATA: errorMsg += " No data "; break;
                case MyCamera.MV_E_PRECONDITION: errorMsg += " Precondition error, or running environment changed "; break;
                case MyCamera.MV_E_VERSION: errorMsg += " Version mismatches "; break;
                case MyCamera.MV_E_NOENOUGH_BUF: errorMsg += " Insufficient memory "; break;
                case MyCamera.MV_E_UNKNOW: errorMsg += " Unknown error "; break;
                case MyCamera.MV_E_GC_GENERIC: errorMsg += " General error "; break;
                case MyCamera.MV_E_GC_ACCESS: errorMsg += " Node accessing condition error "; break;
                case MyCamera.MV_E_ACCESS_DENIED: errorMsg += " No permission "; break;
                case MyCamera.MV_E_BUSY: errorMsg += " Device is busy, or network disconnected "; break;
                case MyCamera.MV_E_NETER: errorMsg += " Network error "; break;
            }

            MessageBox.Show(errorMsg, "相机PROMPT", MessageBoxButtons.OK, MessageBoxIcon.Error, MessageBoxDefaultButton.Button1, MessageBoxOptions.ServiceNotification);
        }
        #endregion

        private void btnRealTime_Click(object sender, EventArgs e)
        {
            WmHKC.MV_CC_SetEnumValue_NET("StrobeEnable", 0);
            WmHKC.MV_CC_SetEnumValue_NET("TriggerMode", (uint)MyCamera.MV_CAM_TRIGGER_MODE.MV_TRIGGER_MODE_OFF);
        }

        private void 打开海康相机ToolStripMenuItem_Click(object sender, EventArgs e)
        {
            OpenDevice("192.168.50.30", "192.168.50.99");
            StartGrab();
        }

        private void txtWriteParam_Click(object sender, EventArgs e)
        {
            try
            {
                WmHKC.MV_CC_SetEnumValue_NET("ExposureAuto", 0);
                //WmHKC.MV_CC_SetFloatValue_NET("ExposureTime", expTime);
                WmHKC.MV_CC_SetFloatValue_NET("ExposureTime", int.Parse(txtExp.Text));
                WmHKC.MV_CC_SetEnumValue_NET("GainAuto", 0);
                //nRet = WmHKC.MV_CC_SetFloatValue_NET("Gain", gainValue);
                WmHKC.MV_CC_SetFloatValue_NET("Gain", int.Parse(txtGain.Text));
            }
            catch (Exception)
            {

            }
            
        }

        private void 关闭海康相机ToolStripMenuItem_Click(object sender, EventArgs e)
        {
            CloseHKCamera();
        }
        /// <summary>
        /// 触发调平
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button1_Click(object sender, EventArgs e)
        {
            try
            {
                // ch:触发命令 | en:Trigger command
                int nRet = WmHKC.MV_CC_SetCommandValue_NET("TriggerSoftware");
                if (MyCamera.MV_OK != nRet)
                {
                    ShowErrorMsg("Trigger Software Fail!", nRet);
                    //txtMsg.Text += DateTime.Now.ToString("HH:mm:ss") + "：手动触发底棉相机拍照\r\n";
                }
            }
            catch (Exception)
            {

                throw;
            }
        }

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
            
            double[] _00 = MatExtension.GetValue(mat, 0, 0);
            Console.WriteLine($"_00 = {_00[0]}");
            double[] _01 = MatExtension.GetValue(mat, 0, 1);
            Console.WriteLine($"_01 = {_01[0]}");
            double[] _02 = MatExtension.GetValue(mat, 0, 2);
            Console.WriteLine($"_02 = {_02[0]}");
            double[] _10 = MatExtension.GetValue(mat, 1, 0);
            Console.WriteLine($"_10 = {_10[0]}");
            double[] _11 = MatExtension.GetValue(mat, 1, 1);
            Console.WriteLine($"_11 = {_11[0]}");
            double[] _12 = MatExtension.GetValue(mat, 1, 2);
            Console.WriteLine($"_12 = {_12[0]}");
            double[] _20 = MatExtension.GetValue(mat, 2, 0);
            Console.WriteLine($"_20 = {_20[0]}");
            double[] _21 = MatExtension.GetValue(mat, 2, 1);
            Console.WriteLine($"_21 = {_21[0]}");
            double[] _22 = MatExtension.GetValue(mat, 2, 2);
            Console.WriteLine($"_22 = {_22[0]}");
            */
            double sy = Math.Sqrt(MatExtension.GetValue1(mat, 0, 0)[0] * MatExtension.GetValue1(mat, 0, 0)[0] + MatExtension.GetValue1(mat, 1, 0)[0] * MatExtension.GetValue1(mat, 1, 0)[0]);
            if (sy < 1E-6)
            {
                x = Math.Atan2(-MatExtension.GetValue1(mat, 1, 2)[0], MatExtension.GetValue1(mat, 1, 1)[0]);
                y = Math.Atan2(-MatExtension.GetValue1(mat, 2, 0)[0], sy);
                z = 0;
            }
            else
            {
                x = Math.Atan2(MatExtension.GetValue1(mat, 2, 1)[0], MatExtension.GetValue1(mat, 2, 2)[0]);
                y = Math.Atan2(-MatExtension.GetValue1(mat, 2, 0)[0], sy);
                z = Math.Atan2(MatExtension.GetValue1(mat, 1, 0)[0], MatExtension.GetValue1(mat, 0, 0)[0]);
            }
            x = x * 180.0 / Math.PI;
            y = y * 180.0 / Math.PI;
            z = z * 180.0 / Math.PI;
        }
        #endregion


        private void tabControl1_SelectedIndexChanged(object sender, EventArgs e)
        {
            switch (this.tabControl1.SelectedIndex)
            {
                case 0:
                    funTag = 0;
                    break;
                case 1:
                    funTag = 1;
                    break;
                default:
                    break;
            }
        }
        /// <summary>
        /// 触发找圆盘
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button4_Click(object sender, EventArgs e)
        {
            try
            {
                // ch:触发命令 | en:Trigger command
                int nRet = WmHKC.MV_CC_SetCommandValue_NET("TriggerSoftware");
                if (MyCamera.MV_OK != nRet)
                {
                    ShowErrorMsg("Trigger Software Fail!", nRet);
                    //txtMsg.Text += DateTime.Now.ToString("HH:mm:ss") + "：手动触发底棉相机拍照\r\n";
                }
            }
            catch (Exception)
            {

                throw;
            }
        }

        private void btnImport_Click(object sender, EventArgs e)
        {

        }
    }
}
