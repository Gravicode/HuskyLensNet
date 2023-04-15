
using GHIElectronics.TinyCLR.Devices.I2c;
using GHIElectronics.TinyCLR.Pins;
using System;
using System.Diagnostics;
using System.Threading;

namespace HuskyLensTiny
{
    public enum Content1
    {
        //% block="X center"
        xCenter = 1,
        //% block="Y center"
        yCenter = 2,
        //% block="width"
        width = 3,
        //% block="height"
        height = 4
    }

    public enum Content2
    {
        //% block="X beginning"
        xOrigin = 1,
        //% block="Y beginning"
        yOrigin = 2,
        //% block="X endpoint"
        xTarget = 3,
        //% block="Y endpoint"
        yTarget = 4
    }

    public enum Content3
    {
        //% block="ID"
        ID = 5,
        //% block="X center"
        xCenter = 1,
        //% block="Y center"
        yCenter = 2,
        //% block="width"
        width = 3,
        //% block="height"
        height = 4
    }

    public enum Content4
    {
        //% block="ID"
        ID = 5,
        //% block="X beginning"
        xOrigin = 1,
        //% block="Y beginning"
        yOrigin = 2,
        //% block="X endpoint"
        xTarget = 3,
        //% block="Y endpoint"
        yTarget = 4

    }

    public enum HUSKYLENSResultType_t
    {
        //%block="frame"
        HUSKYLENSResultBlock = 1,
        //%block="arrow"
        HUSKYLENSResultArrow = 2,
    }
    /*
    public FIRST = new {
        first = -1,
        xCenter = -1,
        xOrigin = -1,
        protocolSize = -1,
        algorithmType = -1,
        requestID = -1,
    };*/

    public enum HUSKYLENSMode
    {
        //%block="save"
        SAVE,
        //%block="load"
        LOAD,
    }
    public enum HUSKYLENSphoto
    {
        //%block="photo"
        PHOTO,
        //%block="screenshot"
        SCREENSHOT
    }
    public enum protocolCommand
    {
        COMMAND_REQUEST = 0x20,
        COMMAND_REQUEST_BLOCKS = 0x21,
        COMMAND_REQUEST_ARROWS = 0x22,
        COMMAND_REQUEST_LEARNED = 0x23,
        COMMAND_REQUEST_BLOCKS_LEARNED = 0x24,
        COMMAND_REQUEST_ARROWS_LEARNED = 0x25,
        COMMAND_REQUEST_BY_ID = 0x26,
        COMMAND_REQUEST_BLOCKS_BY_ID = 0x27,
        COMMAND_REQUEST_ARROWS_BY_ID = 0x28,
        COMMAND_RETURN_INFO = 0x29,
        COMMAND_RETURN_BLOCK = 0x2A,
        COMMAND_RETURN_ARROW = 0x2B,
        COMMAND_REQUEST_KNOCK = 0x2C,
        COMMAND_REQUEST_ALGORITHM = 0x2D,
        COMMAND_RETURN_OK = 0x2E,
        COMMAND_REQUEST_LEARN = 0x2F,
        COMMAND_REQUEST_FORGET = 0x30,
        COMMAND_REQUEST_SENSOR = 0x31,

    }

    public enum protocolAlgorithm
    {
        //%block="Face Recognition"
        ALGORITHM_FACE_RECOGNITION = 0,
        //%block="Object Tracking"
        ALGORITHM_OBJECT_TRACKING = 1,
        //%block="Object Recognition"
        ALGORITHM_OBJECT_RECOGNITION = 2,
        //%block="Line Tracking"
        ALGORITHM_LINE_TRACKING = 3,
        //%block="Color Recognition"
        ALGORITHM_COLOR_RECOGNITION = 4,
        //%block="Tag Recognition"
        ALGORITHM_TAG_RECOGNITION = 5,
        //%block="Object Classification"
        OBJECTCLASSIFICATION,
        //%block="QR Recogmition (EDU only)"
        QRRECOGMITION,
        //%block="Barcode Recognition (EDU only)"
        BARCODERECOGNITION,

    }

    //% weight=100  color=#e7660b icon="\uf083"  block="HuskyLens"
    public class HuskyLensModule
    {
        static int m_i = 16;
        int[] receive_buffer = new int[FRAME_BUFFER_SIZE];
        //static int receive_index = 0;
        //static int content_end = 0;
        byte[] send_buffer = new byte[FRAME_BUFFER_SIZE];
        //static int send_index = 0;
        //static bool send_fail = false;
        //static int[] Protocol_t = new int[6];
        //static int HEADER_0_INDEX = 0;
        //static int HEADER_1_INDEX = 1;
        //static int ADDRESS_INDEX = 2;
        //static int CONTENT_SIZE_INDEX = 3;
        //static int CONTENT_INDEX = 4;
        //static int PROTOCOL_SIZE = 5;
        //static int FRAME_BUFFER_SIZE = 32;
        //static int timeOutTimer = 0;
        //static int timeOutDuration = 1000;


        int[][] protocolPtr = new int[][] { new int[] { 0, 0, 0, 0, 0, 0 }, new int[] { 0, 0, 0, 0, 0, 0 }, new int[] { 0, 0, 0, 0, 0, 0 },
            new int[] { 0, 0, 0, 0, 0, 0 }, new int[] { 0, 0, 0, 0, 0, 0 }, new int[] { 0, 0, 0, 0, 0, 0 }, new int[] { 0, 0, 0, 0, 0, 0 },
            new int[] { 0, 0, 0, 0, 0, 0 }, new int[] { 0, 0, 0, 0, 0, 0 }, new int[] { 0, 0, 0, 0, 0, 0 } };
        int[] Protocol_t = new int[] { 0, 0, 0, 0, 0, 0 };
        //int i = 1;
        const int FRAME_BUFFER_SIZE = 128;
        const int HEADER_0_INDEX = 0;
        const int HEADER_1_INDEX = 1;
        const int ADDRESS_INDEX = 2;
        const int CONTENT_SIZE_INDEX = 3;
        int COMMAND_INDEX = 4;
        int CONTENT_INDEX = 5;
        int PROTOCOL_SIZE = 6;
        int send_index = 0;
        int receive_index = 0;

        //int COMMAND_REQUEST = 0x20;

        //int[] receive_buffer = new int[] { };
        //byte[] send_buffer = new byte[] { };
        //int[] buffer = new int[] { };

        bool send_fail = false;
        bool receive_fail = false;
        int content_current = 0;
        int content_end = 0;
        bool content_read_end = false;

        //int command;
        //int content;

        private int timeOutDuration = 100;
        private DateTime timeOutTimer;
        private I2cDevice i2cDevice;
        const int I2CADDRESS = 0x32;
        string I2CBus;
        //% advanced=true shim=i2c::init
        void init()
        {
            var settings = new I2cConnectionSettings(I2CADDRESS, 100_000); //The slave's address and the bus speed.
            var controller = I2cController.FromName(I2CBus);
            i2cDevice = controller.GetDevice(settings);


        }
        public HuskyLensModule(string I2CBus= SC13048.I2cBus.I2c1)
        {
            this.I2CBus = I2CBus;
            initI2c();
        }
        /**
         * HuskyLens init I2C until success
         */
        //%block="HuskyLens initialize I2C until success"
        //% weight=90
        public void initI2c()
        {
            init();
            while (!readKnock()) ;

            yes();
        }
        /**
         * HuskyLens change mode algorithm until success.
         */
        //%block="HuskyLens switch algorithm to %mode"
        //% weight=85
        public void initMode(protocolAlgorithm mode)
        {
            writeAlgorithm((int)mode, (int)protocolCommand.COMMAND_REQUEST_ALGORITHM);
            while (!wait((byte)protocolCommand.COMMAND_RETURN_OK)) ;
            yes();
        }
        /**
         * HuskyLens requests data and stores it in the result.
         */

        //% block="HuskyLens request data once and save into the result"
        //% weight=80
        public void request()
        {
            protocolWriteCommand((int)protocolCommand.COMMAND_REQUEST);
            processReturn();
        }
        /**
         * HuskyLens get the number of the learned ID from result.
         */
        //%block="HuskyLens get a total number of learned IDs from the result"
        //% weight=79
        public int getIds()
        {
            return Protocol_t[2];
        }
        /**
         * The box or arrow HuskyLens got from result appears in screen?
         */
        //%block="HuskyLens check if %Ht is on screen from the result"
        //% weight=78
        public bool isAppear_s(HUSKYLENSResultType_t Ht)
        {
            switch (Ht)
            {
                case HUSKYLENSResultType_t.HUSKYLENSResultBlock:
                    return countBlocks_s() != 0 ? true : false;
                case HUSKYLENSResultType_t.HUSKYLENSResultArrow:
                    return countArrows_s() != 0 ? true : false;
                default:
                    return false;
            }
        }
        /**
         * HuskyLens get the parameter of box near the screen center from result.
         */
        //% block="HuskyLens get %data of frame closest to the center of screen from the result"
        //% weight=77
        public int readBox_s(Content3 data)
        {
            int hk_x;
            int hk_y = readBlockCenterParameterDirect();
            if (hk_y != -1)
            {
                switch (data)
                {
                    case Content3.ID:
                        hk_x = protocolPtr[hk_y][1]; break;
                    case Content3.xCenter:
                        hk_x = protocolPtr[hk_y][2]; break;
                    case Content3.yCenter:
                        hk_x = protocolPtr[hk_y][3]; break;
                    case Content3.width:
                        hk_x = protocolPtr[hk_y][4]; break;
                    default:
                        hk_x = protocolPtr[hk_y][5];
                        break;
                }
            }
            else hk_x = -1;
            return hk_x;
        }
        /**
         * HuskyLens get the parameter of arrow near the screen center from result.
         */
        //% block="HuskyLens get %data of arrow closest to the center of screen from the result"
        //% weight=77
        public int readArrow_s(Content4 data)
        {
            int hk_x;
            int hk_y = readArrowCenterParameterDirect();
            if (hk_y != -1)
            {
                switch (data)
                {
                    case Content4.ID:
                        hk_x = protocolPtr[hk_y][1]; break;
                    case Content4.xOrigin:
                        hk_x = protocolPtr[hk_y][2]; break;
                    case Content4.yOrigin:
                        hk_x = protocolPtr[hk_y][3]; break;
                    case Content4.xTarget:
                        hk_x = protocolPtr[hk_y][4]; break;
                    default:
                        hk_x = protocolPtr[hk_y][5];
                        break;
                }
            }
            else hk_x = -1;
            return hk_x;
        }
        /**
    * The ID Huskylens got from result has been learned before?
    * @param id to id ,eg: 1
    */
        //% block="HuskyLens check if ID %id is learned from the result"
        //% weight=76
        public bool isLearned(int id)
        {
            int hk_x = countLearnedIDs();
            if (id <= hk_x) return true;
            return false;
        }
        /**
         * The box or arrow corresponding to ID obtained by HuskyLens from result appears in screen？
         * @param id to id ,eg: 1
         */
        //% block="HuskyLens check if ID %id %Ht is on screen from the result"
        //% weight=75
        public bool isAppear(int id, HUSKYLENSResultType_t Ht)
        {
            switch (Ht)
            {
                case HUSKYLENSResultType_t.HUSKYLENSResultBlock:
                    return countBlocks(id) != 0 ? true : false;
                case HUSKYLENSResultType_t.HUSKYLENSResultArrow:
                    return countArrows(id) != 0 ? true : false;
                default:
                    return false;
            }
        }
        /**
    * HuskyLens get the parameter of the box corresponding to ID from result.
    * @param id to id ,eg: 1
    */
        //%block="HuskyLens get  $number1 of ID $id frame from the result"
        //% weight=65
        public int readeBox(int id, Content1 number1)
        {
            int hk_y = cycle_block(id, 1);
            int hk_x = 0;
            if (countBlocks(id) != 0)
            {
                if (hk_y != -999)
                {
                    switch (number1)
                    {
                        case Content1.xCenter:
                            hk_x = (int)protocolPtr[hk_y][1]; break;
                        case Content1.yCenter:
                            hk_x = (int)protocolPtr[hk_y][2]; break;
                        case Content1.width:
                            hk_x = (int)protocolPtr[hk_y][3]; break;
                        case Content1.height:
                            hk_x = (int)protocolPtr[hk_y][4]; break;
                    }
                }
                else hk_x = -1;
            }
            else
                hk_x = -1;
            return hk_x;
        }
        /**
        * HuskyLens get the parameter of the arrow corresponding to ID from result.
        * @param id to id ,eg: 1
        */

        //%block="HuskyLens get $number1 of ID $id arrow from the result"
        //% weight=60
        public int readeArrow(int id, Content2 number1)
        {
            int hk_y = cycle_arrow(id, 1);
            int hk_x;
            if (countArrows(id) != 0)
            {
                if (hk_y != -999)
                {

                    switch (number1)
                    {
                        case Content2.xOrigin:
                            hk_x = protocolPtr[hk_y][1]; break;
                        case Content2.yOrigin:
                            hk_x = protocolPtr[hk_y][2]; break;
                        case Content2.xTarget:
                            hk_x = protocolPtr[hk_y][3]; break;
                        case Content2.yTarget:
                            hk_x = protocolPtr[hk_y][4]; break;
                        default:
                            hk_x = -1;
                            break;
                    }
                }
                else hk_x = -1;
            }
            else hk_x = -1;
            return hk_x;
        }
        /**
         * HuskyLens get the box or arrow total number from result.
         * 
         */
        //%block="HuskyLens get a total number of %Httotal from the result"
        //% weight=90
        //% advanced=true
        public int getBox(HUSKYLENSResultType_t Ht)
        {
            switch (Ht)
            {
                case HUSKYLENSResultType_t.HUSKYLENSResultBlock:
                    return countBlocks_s();
                case HUSKYLENSResultType_t.HUSKYLENSResultArrow:
                    return countArrows_s();
                default:
                    return 0;
            }
        }
        /**
         * HuskyLens get the parameter of Nth box from result.
         * @param index to index ,eg: 1
         */
        //% block="HuskyLens get $data of the No. $index frame from the result"
        //% weight=60
        //% advanced=true
        public int readBox_ss(int index, Content3 data)
        {
            int hk_x = -1;
            int hk_i = index - 1;
            if (protocolPtr[hk_i][0] == (int)protocolCommand.COMMAND_RETURN_BLOCK)
            {
                switch (data)
                {
                    case Content3.ID:
                        hk_x = protocolPtr[hk_i][1]; break;
                    case Content3.xCenter:
                        hk_x = protocolPtr[hk_i][2]; break;
                    case Content3.yCenter:
                        hk_x = protocolPtr[hk_i][3]; break;
                    case Content3.width:
                        hk_x = protocolPtr[hk_i][4]; break;
                    default:
                        hk_x = protocolPtr[hk_i][5];
                        break;
                }
            }
            else hk_x = -1;
            return hk_x;

        }
        /**
         * HuskyLens get the parameter of the Nth arrow from result.
         * @param index to index ,eg: 1
        */
        //% block="HuskyLens get $data of the No. $index arrow from the result"
        //% weight=60
        //% advanced=true
        public int readArrow_ss(int index, Content4 data)
        {
            int hk_x;
            int hk_i = index - 1;
            if (protocolPtr[hk_i][0] == (int)protocolCommand.COMMAND_RETURN_ARROW)
            {
                switch (data)
                {
                    case Content4.ID:
                        hk_x = protocolPtr[hk_i][1]; break;
                    case Content4.xOrigin:
                        hk_x = protocolPtr[hk_i][2]; break;
                    case Content4.yOrigin:
                        hk_x = protocolPtr[hk_i][3]; break;
                    case Content4.xTarget:
                        hk_x = protocolPtr[hk_i][4]; break;
                    default:
                        hk_x = protocolPtr[hk_i][5];
                        break;
                }
            }
            else hk_x = -1;
            //protocolPtr[hk_i][0] = 0;
            return hk_x;
        }

        /**
    * HuskyLens get the total number of box or arrow from result.
    * @param id to id ,eg: 1
    */
        //%block="HuskyLens get a total number of ID %id %Httotal from the result"
        //% weight=55
        //% advanced=true
        public int getBox_S(int id, HUSKYLENSResultType_t Ht)
        {
            switch (Ht)
            {
                case HUSKYLENSResultType_t.HUSKYLENSResultBlock:
                    return countBlocks(id);
                case HUSKYLENSResultType_t.HUSKYLENSResultArrow:
                    return countArrows(id);
                default:
                    return 0;
            }
        }
        /**
         * HuskyLens get the parameter of the Nth box corresponding to ID from result.
         * @param id to id ,eg: 1
         * @param index to index ,eg: 1
         */
        //%block="HuskyLens get $number1 of the ID $id  No. $index frame from the result"
        //% weight=45
        //% advanced=true
        public int readeBox_index(int id, int index, Content1 number1)
        {
            int hk_y = cycle_block(id, index);
            int hk_x;
            if (countBlocks(id) != 0)
            {
                if (hk_y != -999)
                {
                    switch (number1)
                    {
                        case Content1.xCenter:
                            hk_x = protocolPtr[hk_y][1]; break;
                        case Content1.yCenter:
                            hk_x = protocolPtr[hk_y][2]; break;
                        case Content1.width:
                            hk_x = protocolPtr[hk_y][3]; break;
                        case Content1.height:
                            hk_x = protocolPtr[hk_y][4]; break;
                        default:
                            hk_x = -1;
                            break;
                    }
                }
                else hk_x = -1;
            }
            else hk_x = -1;
            return hk_x;
        }
        /**
         * HuskyLens get the parameter of the Nth arrow corresponding to ID from result.
         * @param id to id ,eg: 1
         * @param index to index ,eg: 1
         */
        //%block="HuskyLens get $number1 of the ID $id No. $index arrow from the result"
        //% weight=35
        //% advanced=true
        public int readeArrow_index(int index, int id, Content2 number1)
        {
            int hk_y = cycle_arrow(id, index);
            int hk_x;
            if (countArrows(id) != 0)
            {
                if (hk_y != -999)
                {
                    switch (number1)
                    {
                        case Content2.xOrigin:
                            hk_x = protocolPtr[hk_y][1]; break;
                        case Content2.yOrigin:
                            hk_x = protocolPtr[hk_y][2]; break;
                        case Content2.xTarget:
                            hk_x = protocolPtr[hk_y][3]; break;
                        case Content2.yTarget:
                            hk_x = protocolPtr[hk_y][4]; break;
                        default:
                            hk_x = -1;
                            break;
                    }
                }
                else hk_x = -1;
            }
            else hk_x = -1;
            return hk_x;
        }
        /**
         * Huskylens automatic learning ID
         * @param id to id ,eg: 1
         */
        //%block="HuskyLens learn ID %id once automatically"
        //% weight=30
        //% advanced=true
        public void writeLearn1(int id)
        {
            writeAlgorithm(id, 0X36);
            //while(!wait((byte)protocolCommand.COMMAND_RETURN_OK));
        }
        /**
         * Huskylens forget all learning data of the current algorithm
         */
        //%block="HuskyLens forget all learning data of the current algorithm"
        //% weight=29
        //% advanced=true
        public void forgetLearn()
        {
            writeAlgorithm(0x47, 0X37);
            //while(!wait((byte)protocolCommand.COMMAND_RETURN_OK));
        }
        /**
         * Set ID name
         * @param id to id ,eg: 1
         * @param name to name ,eg: "DFRobot"
         */
        //%block="HuskyLens name ID %id of the current algorithm as %name"
        //% weight=28
        //% advanced=true
        public void writeName(int id, string name)
        {
            //do{
            string newname = name;
            byte[] buffer = husky_lens_protocol_write_begin(0x2f);
            send_buffer[send_index] = (byte)id;
            send_buffer[send_index + 1] = (byte)((newname.Length + 1) * 2);
            send_index += 2;
            for (int i = 0; i < newname.Length; i++)
            {
                send_buffer[send_index] = (byte)newname[i];
                //serial.writeNumber(newname.charCodeAt(i))
                send_index++;
            }
            send_buffer[send_index] = 0;
            send_index += 1;
            int length = husky_lens_protocol_write_end();
            //byte[] Buffer = pins.createBufferFromArray(buffer);
            protocolWrite(buffer);
            //}while(!wait((byte)protocolCommand.COMMAND_RETURN_OK));
        }
        /**
         * Display characters on the screen
         * @param name to name ,eg: "DFRobot"
         * @param x to x ,eg: 150
         * @param y to y ,eg: 30
         */
        //%block="HuskyLens show custom texts %name at position x %x y %y on screen"
        //% weight=27
        //% advanced=true
        //% x.min=0 x.max=319
        //% y.min=0 y.max=210
        public void writeOSD(string name, int x, int y)
        {
            //do{
            byte[] buffer = husky_lens_protocol_write_begin(0x34);
            send_buffer[send_index] = (byte)name.Length;
            if (x > 255)
            {
                send_buffer[send_index + 2] = (byte)(x % 255);
                send_buffer[send_index + 1] = 0xff;
            }
            else
            {
                send_buffer[send_index + 1] = 0;
                send_buffer[send_index + 2] = (byte)x;
            }
            send_buffer[send_index + 3] = (byte)y;
            send_index += 4;
            for (int i = 0; i < name.Length; i++)
            {
                send_buffer[send_index] = (byte)name[i];
                //serial.writeNumber(name.charCodeAt(i));
                send_index++;
            }
            int length = husky_lens_protocol_write_end();
            //serial.writeNumber(length)
            //byte[] Buffer = pins.createBufferFromArray(buffer);
            protocolWrite(buffer);
            //}while(!wait((byte)protocolCommand.COMMAND_RETURN_OK));
        }

        /**
     * HuskyLens clear characters in the screen
     */
        //%block="HuskyLens clear all custom texts on screen"
        //% weight=26
        //% advanced=true
        public void clearOSD()
        {
            writeAlgorithm(0x45, 0X35);
            //while(!wait((byte)protocolCommand.COMMAND_RETURN_OK));
        }
        /**
         * Photos and screenshots
         */
        //%block="HuskyLens take %request and save to SD card"
        //% weight=25
        //% advanced=true
        public void takePhotoToSDCard(HUSKYLENSphoto request)
        {
            switch (request)
            {
                case HUSKYLENSphoto.PHOTO:
                    writeAlgorithm(0x40, 0X30);
                    //while(!wait((byte)protocolCommand.COMMAND_RETURN_OK))
                    break;
                case HUSKYLENSphoto.SCREENSHOT:
                    writeAlgorithm(0x49, 0X39);
                    //while(!wait((byte)protocolCommand.COMMAND_RETURN_OK));
                    break;
                default:
                    writeAlgorithm(0x40, 0X30);
                    //while(!wait((byte)protocolCommand.COMMAND_RETURN_OK));
                    break;
            }
            Thread.Sleep(500);
        }
        /**
         * Save data model
         */
        //%block="HuskyLens %command current algorithm data as No. %data model of SD card"
        //% weight=24
        //% advanced=true
        //% data.min=0 data.max=5
        public void saveModelToTFCard(HUSKYLENSMode command, int data)
        {
            switch (command)
            {
                case HUSKYLENSMode.SAVE:
                    writeAlgorithm(data, 0x32);
                    //while(!wait((byte)protocolCommand.COMMAND_RETURN_OK));
                    break;
                case HUSKYLENSMode.LOAD:
                    writeAlgorithm(data, 0x33);
                    //while(!wait((byte)protocolCommand.COMMAND_RETURN_OK));
                    break;
                default:
                    writeAlgorithm(data, 0x32);
                    //while(!wait((byte)protocolCommand.COMMAND_RETURN_OK));
                    break;
            }
            Thread.Sleep(500);
        }

        private bool validateCheckSum()
        {

            int stackSumIndex = receive_buffer[3] + CONTENT_INDEX;
            int hk_sum = 0;
            for (int i = 0; i < stackSumIndex; i++)
            {
                hk_sum += receive_buffer[i];
            }
            hk_sum = hk_sum & 0xff;

            return (hk_sum == receive_buffer[stackSumIndex]);
        }

        private int husky_lens_protocol_write_end()
        {
            if (send_fail) { return 0; }
            if (send_index + 1 >= FRAME_BUFFER_SIZE) { return 0; }
            send_buffer[CONTENT_SIZE_INDEX] = (byte)(send_index - CONTENT_INDEX);
            //serial.writeValue("618", send_buffer[CONTENT_SIZE_INDEX])
            int hk_sum = 0;
            for (int i = 0; i < send_index; i++)
            {
                hk_sum += send_buffer[i];
            }

            hk_sum = hk_sum & 0xff;
            send_buffer[send_index] = (byte)hk_sum;
            send_index++;
            return send_index;
        }

        private byte[] husky_lens_protocol_write_begin(byte command = 0)
        {
            send_fail = false;
            send_buffer[HEADER_0_INDEX] = 0x55;
            send_buffer[HEADER_1_INDEX] = 0xAA;
            send_buffer[ADDRESS_INDEX] = 0x11;
            //send_buffer[CONTENT_SIZE_INDEX] = datalen;
            send_buffer[COMMAND_INDEX] = command;
            send_index = CONTENT_INDEX;
            return send_buffer;
        }

        private void protocolWrite(byte[] buffer)
        {
            i2cDevice.Write(buffer);
            //pins.i2cWriteBuffer(0x32, buffer, false);
            Thread.Sleep(50);
        }

        private bool processReturn()
        {
            if (!wait((byte)protocolCommand.COMMAND_RETURN_INFO)) return false;
            protocolReadFiveInt16((int)protocolCommand.COMMAND_RETURN_INFO);
            for (int i = 0; i < Protocol_t[1]; i++)
            {

                if (!wait()) return false;
                if (protocolReadFiveInt161(i, (int)protocolCommand.COMMAND_RETURN_BLOCK)) continue;
                else if (protocolReadFiveInt161(i, (int)protocolCommand.COMMAND_RETURN_ARROW)) continue;
                else return false;
            }
            return true;
        }

        private bool wait(byte command = 0)
        {
            timerBegin();
            while (!timerAvailable())
            {
                if (protocolAvailable())
                {
                    if (command != 0)
                    {
                        if (husky_lens_protocol_read_begin((byte)command))
                        {
                            //serial.writeNumber(0);
                            return true;
                        }
                    }
                    else
                    {
                        return true;
                    }
                }
                else
                {
                    return false;
                }
            }
            return false;
        }

        private bool husky_lens_protocol_read_begin(byte command = 0)
        {
            if (command == receive_buffer[COMMAND_INDEX])
            {
                content_current = CONTENT_INDEX;
                content_read_end = false;
                receive_fail = false;
                return true;
            }
            return false;
        }


        private void timerBegin()
        {
            timeOutTimer = DateTime.Now;//input.runningTime();
        }




        public bool timerAvailable()
        {
            return ((DateTime.Now - timeOutTimer).TotalMilliseconds > timeOutDuration);
        }

        public bool protocolAvailable()
        {
            byte[] buf = new byte[16];
            if (m_i == 16)
            {
                i2cDevice.Read(buf); //buf = pins.i2cReadBuffer(0x32, 16, false);
                m_i = 0;
            }
            for (int i = m_i; i < 16; i++)
            {
                if (husky_lens_protocol_receive(buf[i]))
                {
                    m_i++;
                    return true;
                }
                m_i++;
            }
            return false;
        }

        public bool husky_lens_protocol_receive(int data)
        {
            switch (receive_index)
            {
                case HEADER_0_INDEX:
                    if (data != 0x55) { receive_index = 0; return false; }
                    receive_buffer[HEADER_0_INDEX] = 0x55;
                    break;
                case HEADER_1_INDEX:
                    if (data != 0xAA) { receive_index = 0; return false; }
                    receive_buffer[HEADER_1_INDEX] = 0xAA;
                    break;
                case ADDRESS_INDEX:
                    receive_buffer[ADDRESS_INDEX] = data;
                    break;
                case CONTENT_SIZE_INDEX:
                    if (data >= FRAME_BUFFER_SIZE - PROTOCOL_SIZE) { receive_index = 0; return false; }
                    receive_buffer[CONTENT_SIZE_INDEX] = data;
                    break;
                default:
                    receive_buffer[receive_index] = data;

                    if (receive_index == receive_buffer[CONTENT_SIZE_INDEX] + CONTENT_INDEX)
                    {
                        content_end = receive_index;
                        receive_index = 0;
                        return validateCheckSum();

                    }
                    break;
            }
            receive_index++;
            return false;
        }

        public void husky_lens_protocol_write_int16(int content = 0)
        {

            int x = ((content.ToString()).Length);
            if (send_index + x >= FRAME_BUFFER_SIZE) { send_fail = true; return; }
            send_buffer[send_index] = (byte)(content & 0xff);
            send_buffer[send_index + 1] = (byte)((content >> 8) & 0xff);
            send_index += 2;
        }

        public bool protocolReadFiveInt16(int command = 0)
        {
            if (husky_lens_protocol_read_begin((byte)command))
            {
                Protocol_t[0] = command;
                Protocol_t[1] = husky_lens_protocol_read_int16();
                Protocol_t[2] = husky_lens_protocol_read_int16();
                Protocol_t[3] = husky_lens_protocol_read_int16();
                Protocol_t[4] = husky_lens_protocol_read_int16();
                Protocol_t[5] = husky_lens_protocol_read_int16();
                husky_lens_protocol_read_end();
                return true;
            }
            else
            {
                return false;
            }
        }
        bool protocolReadFiveInt161(int i, int command = 0)
        {
            if (husky_lens_protocol_read_begin((byte)command))
            {
                protocolPtr[i][0] = command;
                protocolPtr[i][1] = husky_lens_protocol_read_int16();
                protocolPtr[i][2] = husky_lens_protocol_read_int16();
                protocolPtr[i][3] = husky_lens_protocol_read_int16();
                protocolPtr[i][4] = husky_lens_protocol_read_int16();
                protocolPtr[i][5] = husky_lens_protocol_read_int16();
                husky_lens_protocol_read_end();
                return true;
            }
            else
            {
                return false;
            }
        }

        int husky_lens_protocol_read_int16()
        {
            if (content_current >= content_end || content_read_end) { receive_fail = true; return 0; }
            int result = receive_buffer[content_current + 1] << 8 | receive_buffer[content_current];
            content_current += 2;
            return result;
        }

        bool husky_lens_protocol_read_end()
        {
            if (receive_fail)
            {
                receive_fail = false;
                return false;
            }
            return content_current == content_end;
        }

        int countLearnedIDs()
        {
            return Protocol_t[2];
        }

        int countBlocks(int ID)
        {
            int counter = 0;
            for (int i = 0; i < Protocol_t[1]; i++)
            {
                if (protocolPtr[i][0] == (int)protocolCommand.COMMAND_RETURN_BLOCK && protocolPtr[i][5] == ID) counter++;
            }
            return counter;
        }

        int countBlocks_s()
        {
            int counter = 0;
            for (int i = 0; i < Protocol_t[1]; i++)
            {
                if (protocolPtr[i][0] == (int)protocolCommand.COMMAND_RETURN_BLOCK) counter++;
            }
            //serial.writeNumber(counter)
            return counter;
        }

        int countArrows(int ID)
        {
            int counter = 0;
            for (int i = 0; i < Protocol_t[1]; i++)
            {
                if (protocolPtr[i][0] == (int)protocolCommand.COMMAND_RETURN_ARROW && protocolPtr[i][5] == ID) counter++;
            }
            return counter;
        }

        int countArrows_s()
        {
            int counter = 0;
            for (int i = 0; i < Protocol_t[1]; i++)
            {
                if (protocolPtr[i][0] == (int)protocolCommand.COMMAND_RETURN_ARROW) counter++;
            }
            return counter;
        }

        bool readKnock()
        {
            for (int i = 0; i < 5; i++)
            {
                protocolWriteCommand((int)protocolCommand.COMMAND_REQUEST_KNOCK);//I2C
                if (wait((byte)protocolCommand.COMMAND_RETURN_OK))
                {
                    return true;
                }
            }
            return false;
        }

        bool writeForget()
        {
            for (int i = 0; i < 5; i++)
            {
                protocolWriteCommand((int)protocolCommand.COMMAND_REQUEST_FORGET);
                if (wait((byte)protocolCommand.COMMAND_RETURN_OK))
                {
                    return true;
                }
            }
            return false;
        }

        void protocolWriteCommand(int command = 0)
        {
            Protocol_t[0] = command;
            var buffer = husky_lens_protocol_write_begin((byte)Protocol_t[0]);
            int length = husky_lens_protocol_write_end();
            //byte[] Buffer = pins.createBufferFromArray(buffer);
            protocolWrite(buffer);
        }

        bool protocolReadCommand(int command = 0)
        {
            if (husky_lens_protocol_read_begin((byte)command))
            {
                Protocol_t[0] = command;
                husky_lens_protocol_read_end();
                return true;
            }
            else
            {
                return false;
            }
        }

        void writeAlgorithm(int algorithmType, int comemand = 0)
        {
            protocolWriteOneInt16(algorithmType, comemand);
            //return true//wait((byte)protocolCommand.COMMAND_RETURN_OK);
            //while(!wait((byte)protocolCommand.COMMAND_RETURN_OK));
            //return true
        }

        bool writeLearn(int algorithmType)
        {
            protocolWriteOneInt16(algorithmType, (int)protocolCommand.COMMAND_REQUEST_LEARN);
            return wait((byte)protocolCommand.COMMAND_RETURN_OK);
        }

        void protocolWriteOneInt16(int algorithmType, int command = 0)
        {
            var buffer = husky_lens_protocol_write_begin((byte)command);
            husky_lens_protocol_write_int16(algorithmType);
            int length = husky_lens_protocol_write_end();
            //byte[] Buffer = pins.createBufferFromArray(buffer);
            protocolWrite(buffer);
        }

        int cycle_block(int ID, int index = 1)
        {
            int counter = 0;
            for (int i = 0; i < Protocol_t[1]; i++)
            {
                if (protocolPtr[i][0] == (int)protocolCommand.COMMAND_RETURN_BLOCK && protocolPtr[i][5] == ID)
                {
                    counter++;
                    if (index == counter) return i;

                }
            }
            return default;
        }

        int cycle_arrow(int ID, int index = 1)
        {
            int counter = 0;
            for (int i = 0; i < Protocol_t[1]; i++)
            {
                if (protocolPtr[i][0] == (int)protocolCommand.COMMAND_RETURN_ARROW && protocolPtr[i][5] == ID)
                {
                    counter++;
                    if (index == counter) return i;

                }
            }
            return -999;
        }

        int readBlockCenterParameterDirect()
        {
            int distanceMinIndex = -1;
            int distanceMin = 65535;
            for (int i = 0; i < Protocol_t[1]; i++)
            {
                if (protocolPtr[i][0] == (int)protocolCommand.COMMAND_RETURN_BLOCK)
                {
                    var distance = (Math.Round(Math.Sqrt(Math.Abs(protocolPtr[i][1] - 320 / 2))) + Math.Round(Math.Sqrt(Math.Abs(protocolPtr[i][2] - 240 / 2))));
                    if (distance < distanceMin)
                    {
                        distanceMin = (int)distance;
                        distanceMinIndex = i;
                    }
                }
            }
            return distanceMinIndex;
        }

        int readArrowCenterParameterDirect()
        {
            int distanceMinIndex = -1;
            int distanceMin = 65535;
            for (int i = 0; i < Protocol_t[1]; i++)
            {
                if (protocolPtr[i][0] == (int)protocolCommand.COMMAND_RETURN_ARROW)
                {
                    var a1 = Math.Round(Math.Sqrt(Math.Abs(protocolPtr[i][1] - 320 / 2)));
                    var a2 = Math.Round(Math.Sqrt(Math.Abs(protocolPtr[i][2] - 240 / 2)));
                    int distance = (int)(a1 + a2);
                    if (distance < distanceMin)
                    {
                        distanceMin = distance;
                        distanceMinIndex = i;
                    }
                }
            }
            return distanceMinIndex;
        }

        void no()
        {
            //basic.showIcon(IconNames.No);
            //Thread.Sleep(100);
            //basic.clearScreen();
            //Thread.Sleep(100);
        }
        void yes()
        {
            //basic.showIcon(IconNames.Yes);
            //Thread.Sleep(100);
            //basic.clearScreen();
        }
    }

    public class Testing
    {
        void DoTest()
        {
            var huskylens = new HuskyLensModule();
            huskylens.initMode(protocolAlgorithm.ALGORITHM_OBJECT_TRACKING);
            while (true)
            {
                huskylens.request();
                if (huskylens.isAppear_s(HUSKYLENSResultType_t.HUSKYLENSResultBlock))
                {
                    if (huskylens.isLearned(1))
                    {
                        Debug.WriteLine($"x {huskylens.readeBox(1, Content1.xCenter)}" );
                        Debug.WriteLine($"y {huskylens.readeBox(1, Content1.yCenter)}" );
                        Debug.WriteLine($"k {huskylens.readeBox(1, Content1.width)}" );
                        Debug.WriteLine($"h {huskylens.readeBox(1, Content1.height)}" );
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
            }
        }
    }
}