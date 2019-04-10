using System;
using System.IO;
using System.Collections;
using Tanis.Collections;

namespace Huangbo.AStarPetri.Test
{
    class MainClass
    {

        #region Public Methods

        [STAThread]

        static void Main(string[] args)//主程序
        {
            Huangbo.AStarPetri.AStar astar;
            Huangbo.AStarPetri.AStarNode GoalNode;
            Huangbo.AStarPetri.AStarNode StartNode;

            string filename = "xiong981111";
            common.L =1;//含时间库所托肯最大值
            string[] initfile = new string[] { "./" + filename + "_init.txt" };
            string[] matrixfile = new string[] { "./" + filename + "_matrix.txt" };
        
            int[] hmethods = new int[] { 1 };//所用启发函数h
            //h1=max{ei(m)};
            //h2=0;
            //h4=-dep(m);
            int[] opensizes = new int[1] { 0 };//0表示open无限制;findpath函数添加opensize参数，opensize:0:表示open可为无穷大；大于0：表示进行BF locally,BT globally
            int hFmethod = 2;	//所用第二个启发函数
            //1=h;2=-(FMarkingDepth+1);10=h;
            //从小到大排序
             double ep =-1;	//ep<0时表示没有ep的情况
            //ep=0时，选OPEN中具有相同最小f值marking中有最小hFCost的(0比-1好)
            //ep>0时选择范围更大,选OPEN中具有不大于最小f值1+ep倍的marking中有最小hFCost的

            bool printScreen = true;//是否向屏幕打印每个扩展节点的信息

            foreach (int hmethod in hmethods)
            {
                foreach (int opensize in opensizes)
                {
                    astar = new Huangbo.AStarPetri.AStar(initfile[0], matrixfile[0]);
                    GoalNode = new Huangbo.AStarPetri.AStarNode(null, null, 0, 0, 0, AStar.GoalM, AStar.GoalMr, 0, 0, 0 , 0);
                    StartNode = new Huangbo.AStarPetri.AStarNode(null, GoalNode, 0, 0, 0, AStar.StartM, AStar.StartMr, 0, 0, 0, 0);

                    Console.WriteLine();
                    Console.WriteLine("hmethod={0},filename={1},filename={2},running...", hmethod, initfile[0], matrixfile[0]);

                    DateTime startTime = DateTime.Now;//开始时间
                    astar.FindPath(StartNode, GoalNode, ep, hmethod, hFmethod, opensize, printScreen);
                    DateTime endTime = DateTime.Now;//结束时间
                    TimeSpan elapsedTime = new TimeSpan(endTime.Ticks - startTime.Ticks);//运行时间

                    astar.PrintSolution();  //向屏幕和文件输出		
                    astar.SimplePrintSolution(); //向屏幕输出
                    Console.WriteLine("运行时间：" + elapsedTime);
                }
            }

            Console.WriteLine("Algorithm finished!");
            Console.ReadLine();
        }

        #endregion
    }
}