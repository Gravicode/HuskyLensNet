using HuskyLensNet;
using System.Diagnostics;

namespace TestDevice
{
    internal class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine("Test HuskyLens!");
            var IDLearn = 0;
            var huskylens = new HuskyLensModule();
            huskylens.initMode(protocolAlgorithm.ALGORITHM_OBJECT_RECOGNITION);
            while (true)
            {
                huskylens.request();
                if (huskylens.isAppear_s(HUSKYLENSResultType_t.HUSKYLENSResultBlock))
                {
                    if (huskylens.isLearned(IDLearn))
                    {
                        Debug.WriteLine($"x {huskylens.readeBox(IDLearn, Content1.xCenter)}");
                        Debug.WriteLine($"y {huskylens.readeBox(IDLearn, Content1.yCenter)}");
                        Debug.WriteLine($"k {huskylens.readeBox(IDLearn, Content1.width)}");
                        Debug.WriteLine($"h {huskylens.readeBox(IDLearn, Content1.height)}");
                    }
                    else
                    {
                        Debug.WriteLine("-1");
                    }
                }
                else
                {
                    Debug.WriteLine("-1");
                }
                Thread.Sleep(100);
            }
           
        }
    }
}