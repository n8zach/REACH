using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace ArmController
{
    public class Controller : IObservable<string>, IObservable<List<ServoData>>
    {

        // Control table address universal
        private const int P_CW_ANGLE_LIMIT_L = 6;
        private const int P_CW_ANGLE_LIMIT_H = 7;
        private const int P_CCW_ANGLE_LIMIT_L = 8;
        private const int P_CCW_ANGLE_LIMIT_H = 9;
        private const int P_STATUS_RETURN_LEVEL = 16;
        private const int P_TORQUE_ENABLE = 24;
        private const int P_GOAL_POSITION_L = 30;
        private const int P_GOAL_POSITION_H = 31;
        private const int P_GOAL_SPEED_L = 32;
        private const int P_GOAL_SPEED_H = 33;
        private const int P_TORQUE_LIMIT = 34;
        private const int P_PRESENT_POSITION_L = 36;
        private const int P_PRESENT_POSITION_H = 37;
        private const int P_PRESENT_LOAD = 40;
        private const int P_TEMP = 43;
        private const int P_MOVING = 46;

        public const double MX_PER_ROTATION = 4096;
        public const double RADIANS2MX = MX_PER_ROTATION / (2 * Math.PI);
        public const double MX2RADIANS = MX_PER_ROTATION / (2 * Math.PI);

        public const double AX_PER_ROTATION = 1024;
        public const double RADIANS2AX = AX_PER_ROTATION / (2 * Math.PI);
        public const double AX2RADIANS = AX_PER_ROTATION / (2 * Math.PI);

        // Control table address MX
        public const int P_GOAL_ACCEL = 73;


        // Defulat setting
        private const int DEFAULT_STATUS_RETURN_LEVEL = 2;
        private const int DEFAULT_PORTNUM = 4;
        private const int DEFAULT_BAUDNUM = 34;
        private const int DEFAULT_SPEED = 65;//45;
        private const int DEFAULT_MX106_SPEED = (int)(DEFAULT_SPEED * 2);
        private const int DEFAULT_MX64_SPEED = (int)(DEFAULT_SPEED * 1.2);
        private const int DEFAULT_MX28_SPEED = (int)(DEFAULT_SPEED * 1.3);
        private const int DEFAULT_AX_SPEED = (int)(DEFAULT_SPEED * 4.5);
        private const int DEFAULT_GRIPPER_SPEED = (int)(DEFAULT_SPEED * 2);
        private const int DEFAULT_MX28_ACCEL = 10;
        private const int DEFAULT_MX64_ACCEL = 2;
        private const int DEFAULT_MX106_ACCEL = 2;
        private const int DEFAULT_CCW_LIMIT = 3532;
        private const int DEFAULT_CW_LIMIT = 575;

        private readonly int[] DefaultSpeeds = { DEFAULT_MX28_SPEED, DEFAULT_MX106_SPEED, DEFAULT_MX64_SPEED, DEFAULT_MX64_SPEED, DEFAULT_MX28_SPEED, DEFAULT_AX_SPEED, DEFAULT_AX_SPEED };

        private readonly List<IObserver<string>> messageObservers;
        private readonly List<IObserver<List<ServoData>>> armObservers;

        private readonly object lockObj;
        Task monitor;

        //joint current positions;
        private int[] joints;

        public Controller()
        {
            messageObservers = new List<IObserver<string>>();
            armObservers = new List<IObserver<List<ServoData>>>();
            joints = new int[7];
            lockObj = new object();
            MonitorEnabled = false;
        }

        public bool Moving { get; private set; }
        public bool MonitorEnabled { get; set; }

        public IDisposable Subscribe(IObserver<string> observer)
        {
            lock (lockObj)
            {
                messageObservers.Add(observer);
            }
            return new MessageUnsubscriber(messageObservers, observer);
        }

        public IDisposable Subscribe(IObserver<List<ServoData>> observer)
        {
            lock (lockObj)
            {
                armObservers.Add(observer);
            }
            return new ServoDataUnsubscriber(armObservers, observer);
        }

        private class MessageUnsubscriber : IDisposable
        {
            private List<IObserver<string>> _observers;
            private IObserver<string> _observer;

            public MessageUnsubscriber(List<IObserver<string>> observers, IObserver<string> observer)
            {
                this._observers = observers;
                this._observer = observer;
            }

            public void Dispose()
            {
                if (_observer != null && _observers.Contains(_observer))
                    _observers.Remove(_observer);
            }
        }

        private class ServoDataUnsubscriber : IDisposable
        {
            private List<IObserver<List<ServoData>>> observers;
            private IObserver<List<ServoData>> observer;

            public ServoDataUnsubscriber(List<IObserver<List<ServoData>>> observers, IObserver<List<ServoData>> observer)
            {
                this.observers = observers;
                this.observer = observer;
            }

            public void Dispose()
            {
                if (observer != null && observers.Contains(observer))
                    observers.Remove(observer);
            }
        }

        public bool Initialize(bool stiff)
        {
            // Open device
            if (Dynamixel.dxl_initialize(DEFAULT_PORTNUM, DEFAULT_BAUDNUM) == 0)
            {
                PublishMessage("Failed to open USB2Dynamixel!");
                return false;
            }

            PublishMessage("Succeeded opening USB2Dynamixel!");

            // universal settings
            PublishMessage(string.Format("Setting Response Type on all: {0}", DEFAULT_STATUS_RETURN_LEVEL));
            Dynamixel.dxl_write_byte(Dynamixel.BROADCAST_ID, P_STATUS_RETURN_LEVEL, DEFAULT_STATUS_RETURN_LEVEL);
            CheckCommStatus();

            // MX Settings
            for (int i = 0; i < 5; i++)
            {
                PublishMessage(string.Format("Setting CCW Limit Servo {0}: {1}", i, DEFAULT_CCW_LIMIT));
                Dynamixel.dxl_write_word(i, P_CCW_ANGLE_LIMIT_L, DEFAULT_CCW_LIMIT);
                CheckCommStatus();
                PublishMessage(string.Format("Setting CW limit {0}: {1}", i, DEFAULT_CW_LIMIT));
                Dynamixel.dxl_write_word(i, P_CW_ANGLE_LIMIT_L, DEFAULT_CW_LIMIT);
                CheckCommStatus();
            }

            // ID 0 = MX28
            PublishMessage(string.Format("Setting Goal speed Servo 0: {0}", DEFAULT_MX28_SPEED));
            Dynamixel.dxl_write_word(0, P_GOAL_SPEED_L, DEFAULT_MX28_SPEED);
            CheckCommStatus();
            PublishMessage(string.Format("Setting Acceleration Servo 0: {0}", DEFAULT_MX28_ACCEL));
            Dynamixel.dxl_write_byte(0, P_GOAL_ACCEL, DEFAULT_MX28_ACCEL);
            CheckCommStatus();

            // ID 1  = MX 106 
            PublishMessage(string.Format("Setting Goal speed Servo 1: {0}", DEFAULT_MX106_SPEED));
            Dynamixel.dxl_write_word(1, P_GOAL_SPEED_L, DEFAULT_MX106_SPEED);
            CheckCommStatus();
            PublishMessage(string.Format("Setting Acceleration Servo 1: {0}", DEFAULT_MX106_ACCEL));
            Dynamixel.dxl_write_byte(1, P_GOAL_ACCEL, DEFAULT_MX106_ACCEL);
            CheckCommStatus();

            // ID 2 = MX_64
            PublishMessage(string.Format("Setting Goal speed Servo 2: {0}", DEFAULT_MX64_SPEED));
            Dynamixel.dxl_write_word(2, P_GOAL_SPEED_L, DEFAULT_MX64_SPEED);
            CheckCommStatus();
            PublishMessage(string.Format("Setting Acceleration Servo 2: {0}", DEFAULT_MX64_ACCEL));
            Dynamixel.dxl_write_byte(2, P_GOAL_ACCEL, DEFAULT_MX64_ACCEL);
            CheckCommStatus();

            // ID 3 = MX64
            PublishMessage(string.Format("Setting Goal speed Servo 3: {0}", DEFAULT_MX64_SPEED));
            Dynamixel.dxl_write_word(3, P_GOAL_SPEED_L, DEFAULT_MX64_SPEED);
            CheckCommStatus();
            PublishMessage(string.Format("Setting Acceleration Servo 3: {0}", DEFAULT_MX64_SPEED));
            Dynamixel.dxl_write_byte(3, P_GOAL_ACCEL, DEFAULT_MX64_SPEED);
            CheckCommStatus();

            // ID 4 = MX28
            PublishMessage(string.Format("Setting Goal speed Servo 4: {0}", DEFAULT_MX28_SPEED));
            Dynamixel.dxl_write_word(4, P_GOAL_SPEED_L, DEFAULT_MX28_SPEED);
            CheckCommStatus();
            PublishMessage(string.Format("Setting Acceleration Servo 4: {0}", DEFAULT_MX28_ACCEL));
            Dynamixel.dxl_write_byte(4, P_GOAL_ACCEL, DEFAULT_MX28_ACCEL);
            CheckCommStatus();

            // ID 5 = AX wrist setting
            PublishMessage(string.Format("Setting Goal speed Servo 5: {0}", DEFAULT_AX_SPEED));
            Dynamixel.dxl_write_word(5, P_GOAL_SPEED_L, DEFAULT_AX_SPEED);
            CheckCommStatus();

            // ID 6 = AX gripper setting
            PublishMessage(string.Format("Setting Goal speed Servo 6: {0}", DEFAULT_GRIPPER_SPEED));
            Dynamixel.dxl_write_word(6, P_GOAL_SPEED_L, DEFAULT_GRIPPER_SPEED);
            CheckCommStatus();

            //start the monitor
            ActivateMoitoring();

            SetTorqueEnable(stiff);

            return true;
        }

        public void SetTorqueEnable(bool enabled)
        {
            lock (lockObj)
            {
                this.PublishMessage("broadcast torque enable " + enabled);
                Dynamixel.dxl_write_byte(Dynamixel.BROADCAST_ID, P_TORQUE_ENABLE, enabled ? 1 : 0);
            }
        }

        private void PublishMessage(string message)
        {
            foreach (var observer in messageObservers)
            {
                observer.OnNext(message);
            }
        }

        private void PublishArmData(List<ServoData> data)
        {
            foreach (var observer in armObservers)
            {
                observer.OnNext(data);
            }
        }

        // Check communication result
        private bool CheckCommStatus()
        {
            int comStatus = Dynamixel.dxl_get_result();

            if (comStatus == Dynamixel.COMM_RXSUCCESS)
            {
                //PublishMessage("Success!");
                //Check packet for servo errors
                PublishErrorCode();
                return true;
            }
            else
            {
                switch (comStatus)
                {
                    case Dynamixel.COMM_TXFAIL:
                        PublishMessage("COMM_TXFAIL: Failed transmit instruction packet!");
                        break;

                    case Dynamixel.COMM_TXERROR:
                        PublishMessage("COMM_TXERROR: Incorrect instruction packet!");
                        break;

                    case Dynamixel.COMM_RXFAIL:
                        PublishMessage("COMM_RXFAIL: Failed get status packet from device!");
                        break;

                    case Dynamixel.COMM_RXWAITING:
                        PublishMessage("COMM_RXWAITING: Now recieving status packet!");
                        break;

                    case Dynamixel.COMM_RXTIMEOUT:
                        PublishMessage("COMM_RXTIMEOUT: There is no status packet!");
                        break;

                    case Dynamixel.COMM_RXCORRUPT:
                        PublishMessage("COMM_RXCORRUPT: Incorrect status packet!");
                        break;

                    default:
                        PublishMessage("This is unknown error code!");
                        break;
                }
            }

            return false;
        }

        // Print error bit of status packet
        private void PublishErrorCode()
        {
            bool error = false;
            if (Dynamixel.dxl_get_rxpacket_error(Dynamixel.ERRBIT_VOLTAGE) == 1)
            {
                error = true;
                PublishMessage("Input voltage error!");
            }
            if (Dynamixel.dxl_get_rxpacket_error(Dynamixel.ERRBIT_ANGLE) == 1)
            {
                error = true;
                PublishMessage("Angle limit error!");
            }
            if (Dynamixel.dxl_get_rxpacket_error(Dynamixel.ERRBIT_OVERHEAT) == 1)
            {
                error = true;
                PublishMessage("Overheat error!");
            }
            if (Dynamixel.dxl_get_rxpacket_error(Dynamixel.ERRBIT_RANGE) == 1)
            {
                error = true;
                PublishMessage("Out of range error!");
            }
            if (Dynamixel.dxl_get_rxpacket_error(Dynamixel.ERRBIT_CHECKSUM) == 1)
            {
                error = true;
                PublishMessage("Checksum error!");
            }
            if (Dynamixel.dxl_get_rxpacket_error(Dynamixel.ERRBIT_OVERLOAD) == 1)
            {
                error = true;
                PublishMessage("Overload error!");
            }
            if (Dynamixel.dxl_get_rxpacket_error(Dynamixel.ERRBIT_INSTRUCTION) == 1)
            {
                error = true;
                PublishMessage("Instruction code error!");
            }
            if (error)
            {
                Console.Beep();
            }
        }

        public void SetGoalPosition(int id, int pos)
        {
            SetGoalPositionAndSpeed(id, pos, DefaultSpeeds[id]);
        }

        /// <summary>
        /// Speed here is a decimal fractional speed
        /// </summary>
        public void SetGoalPositionAndSpeed(int id, int pos, int speed, bool sync = true)
        {
            this.Moving = true;
            if (sync)
            {
                lock (this.lockObj)
                {
                    this.PublishMessage("seting goal speed " + id + " " + speed);
                    Dynamixel.dxl_write_word(id, P_GOAL_SPEED_L, speed);
                    //PublishCommStatus(Dynamixel.dxl_get_result());

                    this.PublishMessage("seting goal position " + id + " " + pos);
                    Dynamixel.dxl_write_word(id, P_GOAL_POSITION_L, pos);
                    //PublishCommStatus(Dynamixel.dxl_get_result());
                }
            }
            else
            {
                Dynamixel.dxl_write_word(id, P_GOAL_SPEED_L, speed);
                //PublishCommStatus(Dynamixel.dxl_get_result());

                Dynamixel.dxl_write_word(id, P_GOAL_POSITION_L, pos);
                //PublishCommStatus(Dynamixel.dxl_get_result());
            }
        }

        public void SetMultiGoalPositions(List<Command> jointCommands, bool sync = true)
        {
            //this.Moving = true;
            //Dynamixel.dxl_set_txpacket_id(Dynamixel.BROADCAST_ID);
            //Dynamixel.dxl_set_txpacket_instruction(Dynamixel.INST_SYNC_WRITE);
            //Dynamixel.dxl_set_txpacket_parameter(0, P_GOAL_POSITION_L);
            //Dynamixel.dxl_set_txpacket_parameter(1, 2);
            //int i = 0;
            //foreach (Tuple<int, int> jointPos in jointPositions)
            //{
            //    Dynamixel.dxl_set_txpacket_parameter(2 + 3 * i, jointPos.Item1);
            //    Dynamixel.dxl_set_txpacket_parameter(2 + 3 * i + 1, Dynamixel.dxl_get_lowbyte(jointPos.Item2));
            //    Dynamixel.dxl_set_txpacket_parameter(2 + 3 * i + 2, Dynamixel.dxl_get_highbyte(jointPos.Item2));
            //    i++;
            //}
            //Dynamixel.dxl_set_txpacket_length(4 + (jointPositions.Count * 3));
            //lock (sync)
            //{
            //    Dynamixel.dxl_txrx_packet();
            //    PublishCommStatus(Dynamixel.dxl_get_result());

            //}
            this.joints[6] = jointCommands[6].GoalPosition;
            double maxAngle = jointCommands.Take(6).Max(
                c =>
                {
                    int delta = Math.Abs(c.GoalPosition - this.joints[c.Id]);
                    return c.Id == 5 ? delta * 4 : delta;
                });

            foreach (Command c in jointCommands)
            {
                double speed;

                if (c.Id == 6)
                {
                    speed = .5;
                }
                else
                {
                    speed = Math.Abs((double)(c.GoalPosition - this.joints[c.Id]) / maxAngle);
                    if (c.Id == 5)
                    {
                        //speed = speed / 4;
                    }
                }
                if (speed > 0)
                {
                    speed = Math.Max(.2, speed);
                    speed = (speed * (double)this.DefaultSpeeds[c.Id]);
                    //PublishMessage(string.Format("J{0} : {1},{2}", c.Id, speed.ToString(), c.GoalPosition));
                    SetGoalPositionAndSpeed(c.Id, c.GoalPosition, (int)speed, sync);
                }
            }
            //PublishMessage("-------------------");
        }

        public void ActivateMoitoring()
        {
            monitor = new TaskFactory<bool>().StartNew(Monitor);
        }

        public void DeactivateMonitoring()
        {
            monitor.Dispose();
        }

        private bool Monitor()
        {
            int moving;
            int position;
            int load;
            int temp;
            bool anyMoving = false;

            while (true)
            {
                if (MonitorEnabled)
                {
                    List<ServoData> armData = new List<ServoData>();
                    lock (lockObj)
                    {
                        anyMoving = false;
                        for (int i = 0; i < 7; i++)
                        {
                            //this.PublishMessage("read position " + i);
                            position = Dynamixel.dxl_read_word(i, P_PRESENT_POSITION_L);
                            CheckCommStatus();
                            this.joints[i] = position;

                            //this.PublishMessage("read moving " + i);
                            moving = Dynamixel.dxl_read_byte(i, P_MOVING);
                            CheckCommStatus();

                            //this.PublishMessage("read load " + i);
                            load = Dynamixel.dxl_read_word(i, P_PRESENT_LOAD);
                            CheckCommStatus();
                            if (load > 1023)
                            {
                                load = load - 1023;
                            }

                            //this.PublishMessage("read temp " + i);
                            temp = Dynamixel.dxl_read_byte(i, P_TEMP);
                            CheckCommStatus();

                            armData.Add(new ServoData(i, moving == 1, position, load, temp));

                            if (i == 0 || i == 2 || i == 4)
                            {
                                AlertOverAngle(position);
                            }

                            PublishArmData(armData);
                        }

                        anyMoving = armData.Any(d => d.Moving == true);
                        this.Moving = anyMoving;
                    }
                }

                Thread.Sleep(50);
            }
        }

        public void Reset()
        {
            lock (lockObj)
            {
                PublishMessage("Resetting all");
                Dynamixel.dxl_write_word(Dynamixel.BROADCAST_ID, P_TORQUE_LIMIT, 1023);
                CheckCommStatus();
            }
        }

        private void AlertOverAngle(int pos)
        {
            if (pos > DEFAULT_CCW_LIMIT || pos < DEFAULT_CW_LIMIT)
            {
                Console.Beep();
            }
        }
    }
}
