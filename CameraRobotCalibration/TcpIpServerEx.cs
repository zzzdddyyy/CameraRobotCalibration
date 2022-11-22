using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CameraRobotCalibration
{
    using System;
    using System.Collections.Generic;
    using System.Linq;
    using System.Text;
    using System.Threading;
    using System.Net.Sockets;
    using System.Net;

    public delegate void ReciveMsg(EndPoint ep,string msg);//初始化委托信息
    public class TcpIpServerEx
    {
        //当前客户端的网络结点
        public EndPoint RemoteEndPoint { get; private set; }

        //调试信息
        public Action<string> Debug_Output_string;

        //接受信息
        //public Action<string> Rev_Msg;
        public event ReciveMsg Rev_Msg;//实例化委托
        //是否接受
        private bool isRev = true;

        //负责监听客户端的线程
        Thread threadWatch = null;

        //负责监听客户端的套接字
        Socket socket = null;

        // Dictionary<ip和端口, Socket> 定义一个集合，存储客户端信息
        public Dictionary<EndPoint, Socket> clientList = new Dictionary<EndPoint, Socket> { };

        List<Thread> threadList = new List<Thread>();

        private StringBuilder msg = new StringBuilder();
        public string Msg
        {
            get { return msg.ToString(); }
            private set
            {
                msg.AppendLine(value);
                //   Console.WriteLine(value + "\r\n");
                if (Debug_Output_string != null)
                    Debug_Output_string("\r\n" + value + "\r\n");
            }
        }
        /// <summary>
        /// 关闭服务器
        /// 当客户端没有关闭socket时 ,调用Close 就会导致错误
        /// </summary>
        public void Close()
        {
            try
            {
                //先关闭soket 再关闭线程
                isRev = false;
                Thread.Sleep(1000);
                var dict = clientList;
                var keys = dict.Keys.ToList();
                for (int i = dict.Count - 1; i >= 0; i--)
                {
                    var Value = dict[keys[i]];
                    Value?.Close();
                }
                socket?.Close();

                for (int i = 0; i < threadList.Count; i++)
                {
                    threadList[i]?.Abort();
                }
                threadWatch?.Abort();
                dict.Clear();
                threadList.Clear();
            }
            catch (Exception)
            {

            }

        }
        public void CloseServer()
        {
            socket.Close();
        }

        private TcpIpServerEx() { }
        public TcpIpServerEx(int port)
        {
            //定义一个套接字用于监听客户端发来的消息，包含三个参数（IP4寻址协议，流式连接，Tcp协议）
            socket = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);

            //将IP地址和端口号绑定到网络节点point上
            IPEndPoint point = new IPEndPoint(IPAddress.Any, port); //设置服务器端口，IP是本程序所在PC的内网IP

            //监听绑定的网络节点
            socket.Bind(point);

            //将套接字的监听队列长度限制为20
            socket.Listen(20);

            //创建一个监听线程
            threadWatch = new Thread(ListenConnecting);

            //将窗体线程设置为与后台同步，随着主线程结束而结束
            threadWatch.IsBackground = true;

            //启动线程   
            threadWatch.Start();

            //启动线程后显示相应提示
            Msg = ("开始监听客户端传来的信息!" + "\r\n");
        }

        public TcpIpServerEx(IPEndPoint point)
        {
            //定义一个套接字用于监听客户端发来的消息，包含三个参数（IP4寻址协议，流式连接，Tcp协议）
            socket = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);

            //监听绑定的网络节点
            socket.Bind(point);

            //将套接字的监听队列长度限制为20
            socket.Listen(20);

            //创建一个监听线程
            threadWatch = new Thread(ListenConnecting);

            //将窗体线程设置为与后台同步，随着主线程结束而结束
            threadWatch.IsBackground = true;

            //启动线程   
            threadWatch.Start();

            //启动线程后显示相应提示
            Msg = ("开始监听客户端传来的信息!" + "\r\n");
        }

        public TcpIpServerEx(IPEndPoint point, Action<string> debugOutputString)
        {
            Debug_Output_string = debugOutputString;
            //定义一个套接字用于监听客户端发来的消息，包含三个参数（IP4寻址协议，流式连接，Tcp协议）
            socket = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);

            //监听绑定的网络节点
            socket.Bind(point);

            //将套接字的监听队列长度限制为20
            socket.Listen(20);

            //创建一个监听线程
            threadWatch = new Thread(ListenConnecting);

            //将窗体线程设置为与后台同步，随着主线程结束而结束
            threadWatch.IsBackground = true;

            //启动线程   
            threadWatch.Start();

            //启动线程后显示相应提示
            Msg = ("开始监听客户端传来的信息!" + "\r\n");

        }

        //监听客户端发来的请求
        private void ListenConnecting()
        {
            Socket connection = null;
            while (isRev)  //持续不断监听客户端发来的请求   
            {
                try
                {
                    if (socket == null)//即服务器停止了（停止监听）
                    {
                        return;
                    }
                    connection = socket.Accept();
                    //让客户显示"连接成功的"的信息
                    //string sendmsg = "连接服务端成功！你的IP是" + connection.RemoteEndPoint;
                    //byte[] arrSendMsg = Encoding.UTF8.GetBytes(sendmsg);
                    //connection.Send(arrSendMsg);

                    RemoteEndPoint = connection.RemoteEndPoint; //客户端网络结点号
                    Msg = ("成功与" + RemoteEndPoint + "客户端建立连接！\t\n");     //显示与客户端连接情况
                    clientList.Add(RemoteEndPoint, connection);    //添加客户端信息

                    //创建一个通信线程    
                    var pts = new ParameterizedThreadStart(ReciveMsg);
                    var thread = new Thread(pts);
                    threadList.Add(thread);
                    //   thread.IsBackground = true;//设置为后台线程，随着主线程退出而退出           
                    thread.Start(connection);//启动线程  
                }
                catch (Exception ex)
                {
                    Msg = (ex.Message); //提示套接字监听异常   
                    break;
                }


            }
        }

        /// <summary>
        /// 接收客户端发来的信息 
        /// </summary>
        /// <param name="socketclientpara"></param>
        private void ReciveMsg(object socketclientpara)
        {

            Socket socketServer = socketclientpara as Socket;
            while (isRev)
            {
                //创建一个内存缓冲区 其大小为1024*1024字节  即1M   
                byte[] arrServerRecMsg = new byte[1024 * 1024];
                //将接收到的信息存入到内存缓冲区,并返回其字节数组的长度  
                try
                {
                    int length = socketServer.Receive(arrServerRecMsg);

                    if (length != 0)
                    {
                        //将机器接受到的字节数组转换为人可以读懂的字符串   
                        string strSRecMsg = Encoding.UTF8.GetString(arrServerRecMsg, 0, length);
                        if (Rev_Msg != null) Rev_Msg(socketServer.RemoteEndPoint,strSRecMsg);

                        //将发送的字符串信息附加到文本框txtMsg上   
                        //Msg = ("收到来自客户端:" + socketServer.RemoteEndPoint + "的信息:" + strSRecMsg + "\r\n");
                    }
                }
                catch (Exception ex)
                {
                    try
                    {
                        Msg = ("客户端:" + socketServer.RemoteEndPoint + "已经中断连接," + ex.Message + "\r\n"); //提示套接字监听异常 
                        clientList.Remove(socketServer.RemoteEndPoint);
                        socketServer.Close();//关闭之前accept出来的和客户端进行通信的套接字
                        break;
                    }
                    catch (Exception)
                    {

                    }
                }
            }


        }


        //获取当前系统时间
        private string GetCurrentTime()
        {
            string timeStr = System.DateTime.Now.ToString("yyyy年MM月dd日hh时mm分ss秒fff毫秒。");
            return timeStr;
        }

        /// <summary>
        /// 发送信息到客户端
        /// </summary>
        /// <param name="smallname"></param>
        /// <param name="sendMsg">要发送的信息</param>
        public void SentMsg(EndPoint endPoint, string sendMsg)
        {
            byte[] bytes = System.Text.Encoding.UTF8.GetBytes(sendMsg);   //将要发送的信息转化为字节数组，因为Socket发送数据时是以字节的形式发送的
            clientList[endPoint].Send(bytes);   //发送数据
            //Msg = ("向："+endPoint + "发送消息:" + sendMsg + "\r\n");
        }
        /// <summary>
        /// 发送信息到客户端
        /// </summary>
        /// <param name="smallname"></param>
        /// <param name="sendMsg">要发送的信息</param>
        public void SentMsg(string ip, string sendMsg)
        {
            byte[] bytes = System.Text.Encoding.UTF8.GetBytes(sendMsg);   //将要发送的信息转化为字节数组，因为Socket发送数据时是以字节的形式发送的
            foreach (KeyValuePair<EndPoint, Socket> item in clientList)
            {
                if (ip == ((IPEndPoint)item.Key).Address.ToString())
                {
                    item.Value.Send(bytes);   //发送数据
                    //Msg = (GetCurrentTime() + ip.ToString() + "的消息:" + sendMsg + "\r\n");
                }
            }


        }

        public void CloseEndPoint(EndPoint iPoint)
        {
            if (clientList.ContainsKey(iPoint))
            {
                clientList[iPoint].Close();
                clientList.Remove(iPoint);
            }
        }
    }
}
