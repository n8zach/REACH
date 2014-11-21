using System;
using System.ComponentModel;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Xml;
using Microsoft.Win32;
using ArmController;
using KUKA.Tutorials.Transformation;

namespace ArmUI
{
    class ArmViewModel : INotifyPropertyChanged, IObserver<string>, IObserver<List<ServoData>>
    {
        private readonly List<int> activeList = new List<int>();

        
        public int J1 { get { return j1.Position; } set { j1.Position = value; OnJointChanged(); } }
        public int J2 { get { return j2.Position; } set { j2.Position = value; OnJointChanged(); } }
        public int J3 { get { return j3.Position; } set { j3.Position = value; OnJointChanged(); } }
        public int J4 { get { return j4.Position; } set { j4.Position = value; OnJointChanged(); } }
        public int J5 { get { return j5.Position; } set { j5.Position = value; OnJointChanged(); } }
        public int J6 { get { return j6.Position; } set { j6.Position = value; OnJointChanged(); } }
        public int J7 { get { return j7.Position; } set { j7.Position = value; OnJointChanged(); } }

        public int Torque1 { get { return (int)(100 * (double)(j1.Torque) / 1024); } }
        public int Torque2 { get { return (int)(100 * (double)(j2.Torque) / 1024); } }
        public int Torque3 { get { return (int)(100 * (double)(j3.Torque) / 1024); } }
        public int Torque4 { get { return (int)(100 * (double)(j4.Torque) / 1024); } }
        public int Torque5 { get { return (int)(100 * (double)(j5.Torque) / 1024); } }
        public int Torque6 { get { return (int)(100 * (double)(j6.Torque) / 1024); } }
        public int Torque7 { get { return (int)(100 * (double)(j7.Torque) / 1024); } }

        public int Temp1 { get { return j1.Temp; } }
        public int Temp2 { get { return j2.Temp; } }
        public int Temp3 { get { return j3.Temp; } }
        public int Temp4 { get { return j4.Temp; } }
        public int Temp5 { get { return j5.Temp; } }
        public int Temp6 { get { return j6.Temp; } }
        public int Temp7 { get { return j7.Temp; } }

        private int x;
        public int X { get { return x; } set { x = value; OnCartesianChanged(); } }

        private int y;
        public int Y { get { return y; } set { y = value; OnCartesianChanged(); } }

        private int z;
        public int Z { get { return z; } set { z = value; OnCartesianChanged(); } }

        private int roll;
        public int Roll { get { return roll; } set { roll = value; OnCartesianChanged(); } }

        private int pitch;
        public int Pitch { get { return pitch; } set { pitch = value; OnCartesianChanged(); } }

        private int yaw;
        public int Yaw { get { return yaw; } set { yaw = value; OnCartesianChanged(); } }

        public bool CanConnect
        {
            get
            {
                return this.canConnect;
            }
        }

        public bool Connected
        {
            get
            {
                return this.connected;
            }
        }

        public string Output
        {
            get
            {
                return this.output;
            }
        }

        public string ScriptAsText
        {
            get
            {
                StringBuilder text = new StringBuilder();
                if (!this.recording)
                {
                    foreach (List<Command> command in this.script.Commands)
                    {
                        foreach (Command jointPos in command)
                        {
                            text.AppendFormat("{0} ", jointPos.GoalPosition);
                        }
                        text.AppendLine();
                    }
                }
                return text.ToString();
            }
        }

        public event PropertyChangedEventHandler PropertyChanged;

        private IDisposable messageUnsubscriber;
        private IDisposable servoDataUnsubscriber;
        private Controller armController;

        private bool connected = false;
        private bool canConnect = true;

        private ServoData j1;
        private ServoData j2;
        private ServoData j3;
        private ServoData j4;
        private ServoData j5;
        private ServoData j6;
        private ServoData j7;

        private string output;

        private Script script;
        private bool recording;

        InvKinematik invKin;

        private int xHome;
        private int yHome;
        private int zHome;
        private DateTime lastTime;

        public ArmViewModel()
        {
            this.armController = new Controller();
            this.messageUnsubscriber = armController.Subscribe((IObserver<string>)this);
            this.servoDataUnsubscriber = armController.Subscribe((IObserver<List<ServoData>>)this);
            script = new Script();
            x = 360;
            y = 300;
            z = 0;
            roll = 90;
            pitch = 0;
            yaw = 10;
        }

        public void Initialize(bool stiff)
        {
            this.canConnect = false;
            this.NotifyPropertyChanged("CanConnect");
            this.AddToOutput("Connecting...");

            this.connected = armController.Initialize(stiff);
            this.NotifyPropertyChanged("Connected");
            this.canConnect = !this.connected;

            this.lastTime = DateTime.Now;
        }

        private void OnJointChanged([CallerMemberName] String propertyName = "")
        {
            if (this.connected)
            {
                switch (propertyName)
                {
                    case "J1":
                        armController.SetGoalPosition(0, J1);
                        break;
                    case "J2":
                        armController.SetGoalPosition(1, J2);
                        break;
                    case "J3":
                        armController.SetGoalPosition(2, J3);
                        break;
                    case "J4":
                        armController.SetGoalPosition(3, J4);
                        break;
                    case "J5":
                        armController.SetGoalPosition(4, J5);
                        break;
                    case "J6":
                        armController.SetGoalPosition(5, J6);
                        break;
                    case "J7":
                        armController.SetGoalPosition(6, J7);
                        break;
                    default:
                        this.AddToOutput("Unknown Joint!");
                        break;
                }
            }
            this.NotifyPropertyChanged(propertyName);
        }

        private void AddToOutput(string message)
        {
            this.output = message + System.Environment.NewLine + this.output;
            this.NotifyPropertyChanged("Output");
        }

        public virtual void OnCompleted()
        {
            this.AddToOutput("The Arm Controller has completed transmitting data");
            this.messageUnsubscriber.Dispose();
            this.servoDataUnsubscriber.Dispose();
        }

        public virtual void OnError(Exception e)
        {
            this.AddToOutput(string.Format("The Arm Controller encountered an error: {0}", e));
        }

        public virtual void OnNext(string message)
        {
            this.AddToOutput(message);
        }

        private void NotifyPropertyChanged(string property)
        {
            if (this.PropertyChanged != null)
            {
                this.PropertyChanged(this, new PropertyChangedEventArgs(property));
            }
        }

        public virtual void OnNext(List<ServoData> armData)
        {
            if (this.recording && this.Moved(armData))
            {
                AddCommand();
            }

            foreach (ServoData sd in armData)
            {
                switch (sd.Id)
                {
                    case 0:
                        j1 = sd;
                        break;
                    case 1:
                        j2 = sd;
                        break;
                    case 2:
                        j3 = sd;
                        break;
                    case 3:
                        j4 = sd;
                        break;
                    case 4:
                        j5 = sd;
                        break;
                    case 5:
                        j6 = sd;
                        break;
                    case 6:
                        j7 = sd;
                        break;
                }
                this.NotifyPropertyChanged(string.Format("J{0}", sd.Id + 1));
                this.NotifyPropertyChanged(string.Format("Torque{0}", sd.Id + 1));
                this.NotifyPropertyChanged(string.Format("Temp{0}", sd.Id + 1));
            }
        }

        private bool Moved(List<ServoData> armData)
        {
            bool moved = false;
            int minChange = 1;
            foreach (ServoData sd in armData)
            {
                switch (sd.Id)
                {
                    case 0:
                        moved = moved || (Math.Abs(j1.Position - sd.Position) > minChange);
                        break;
                    case 1:
                        moved = moved || (Math.Abs(j2.Position - sd.Position) > minChange);
                        break;
                    case 2:
                        moved = moved || (Math.Abs(j3.Position - sd.Position) > minChange);
                        break;
                    case 3:
                        moved = moved || (Math.Abs(j4.Position - sd.Position) > minChange);
                        break;
                    case 4:
                        moved = moved || (Math.Abs(j5.Position - sd.Position) > minChange);
                        break;
                    case 5:
                        moved = moved || (Math.Abs(j6.Position - sd.Position) > minChange);
                        break;
                    case 6:
                        moved = moved || (Math.Abs(j7.Position - sd.Position) > minChange);
                        break;
                }
                if (moved)
                {
                    break;
                }
            }
            return moved;
        }

        internal void SetTorqueEnable(bool enabled)
        {
            armController.SetTorqueEnable(enabled);
        }

        internal void AddCommand()
        {
            List<Command> jointPositions = new List<Command>();

            jointPositions.Add(new Command(0, J1));
            jointPositions.Add(new Command(1, J2));
            jointPositions.Add(new Command(2, J3));
            jointPositions.Add(new Command(3, J4));
            jointPositions.Add(new Command(4, J5));
            jointPositions.Add(new Command(5, J6));
            jointPositions.Add(new Command(6, J7));
            this.script.AddCommand(jointPositions);
            this.NotifyPropertyChanged("ScriptAsText");
        }

        internal void AddCommandHome()
        {
            List<Command> jointPositions = new List<Command>();

            jointPositions.Add(new Command(0, 2048)); //2048));
            jointPositions.Add(new Command(1, 2016));
            jointPositions.Add(new Command(2, 2048));
            jointPositions.Add(new Command(3, 2048));
            jointPositions.Add(new Command(4, 2048));
            jointPositions.Add(new Command(5, 512));
            jointPositions.Add(new Command(6, 512));
            this.script.AddCommand(jointPositions);
            this.NotifyPropertyChanged("ScriptAsText");
        }

        internal void LoadAndPlayScriptDuringKinect(string script)
        {
            armController.MonitorEnabled = true;
            Console.Beep();
            WaitForMotionToComplete();
            Console.Beep();
            if (!this.script.LoadScript(script))
            {
                AddToOutput(string.Format("Script {0} could not be loaded", script));
            }
            else
            {
                PlayScript(true);
            }
        }

        internal void PlayScript(bool reenableMonitor = false)
        {
            SetTorqueEnable(true);
            armController.MonitorEnabled = true;
            Task play = new TaskFactory<bool>().StartNew(() =>
            {

                foreach (List<Command> command in this.script.Commands)
                {
                    armController.SetMultiGoalPositions(command);
                    this.WaitForMotionToComplete();
                }
                if (reenableMonitor)
                {
                    armController.MonitorEnabled = false;
                }
                return true;
            });
        }

        private void WaitForMotionToComplete()
        {
            while (armController.Moving == true)
            {
                Thread.Sleep(100);
            }
        }

        public void ClearScript()
        {
            this.script.Clear();
            this.NotifyPropertyChanged("ScriptAsText");
        }

        public void SaveScript()
        {
            SaveFileDialog dlg = new SaveFileDialog();
            dlg.DefaultExt = ".arm";
            dlg.Filter = "Arm scripts (.arm)|*.arm";
            if (dlg.ShowDialog() == true)
            {
                this.script.Save(dlg.FileName);
            }
        }

        internal void LoadScript()
        {
            OpenFileDialog dlg = new OpenFileDialog();
            dlg.Filter = "Arm scripts (*.arm)|*.arm";
            dlg.DefaultExt = "arm";
            if (dlg.ShowDialog() == true)
            {
                if (!this.script.LoadScript(dlg.FileName))
                {
                    this.AddToOutput("The file could not be loaded");
                }
                else
                {
                    NotifyPropertyChanged("ScriptAsText");
                }
            }
        }

        public void Reset()
        {
            armController.Reset();
        }

        internal void ClearOutput()
        {
            this.output = string.Empty;
            NotifyPropertyChanged("Output");
        }

        internal void StartRecording()
        {
            this.recording = true;
        }

        internal void StopRecording()
        {
            this.recording = false;
            NotifyPropertyChanged("ScriptAsText");
        }

        public void InitializeKinematics()
        {
            float[] angles = new float[7] {
                    (float)Controller.MX2RADIANS * (2048 - (float)Controller.MX_PER_ROTATION / 2),
                    (float)Controller.MX2RADIANS * (2048 - (float)Controller.MX_PER_ROTATION / 2),
                    (float)Controller.MX2RADIANS * (2048 - (float)Controller.MX_PER_ROTATION / 2),
                    (float)Controller.MX2RADIANS * (2048 - (float)Controller.MX_PER_ROTATION / 2),
                    (float)Controller.MX2RADIANS * (2048 - (float)Controller.MX_PER_ROTATION / 2),
                    (float)Controller.AX2RADIANS * (512 - (float)Controller.AX_PER_ROTATION / 2),
                    (float)Controller.AX2RADIANS * (512 - (float)Controller.AX_PER_ROTATION / 2)};
      
            invKin = new InvKinematik(angles);


            
            //DirKinematik dirKin = new DirKinematik();
            //dirKin.Calculate(new TransformationState(), angles);

            //x = (int)(dirKin.X * 1000);
            //y = (int)(dirKin.Y * 1000);
            //z = (int)(dirKin.Z * 1000);
            //roll = (int)(dirKin.Roll * 1000);
            //pitch = (int)(dirKin.Pitch * 1000);
            //yaw = (int)(dirKin.Yaw * 1000);
            
            xHome = 360;
            yHome = 300;
            zHome = 0;

            x = xHome;
            y = yHome;
            z = zHome;

            roll = 90;
            pitch = 0;
            yaw = 10;
            
            NotifyPropertyChanged("X");
            NotifyPropertyChanged("Y");
            NotifyPropertyChanged("Z");
            NotifyPropertyChanged("Roll");
            NotifyPropertyChanged("Pitch");
            NotifyPropertyChanged("Yaw");

            armController.MonitorEnabled = false;
        }

        private void OnCartesianChanged([CallerMemberName] String propertyName = "")
        {
            if (invKin == null)
            {
                AddToOutput("Need to inialize positions before moving.");
                return;
            }

            try
            {
                invKin.Calculate(
                    (float)x / 1000,
                    (float)y / 1000,
                    (float)z / 1000,
                    (float)Degrees2Radians((float)roll),
                    (float)Degrees2Radians((float)pitch),
                    (float)Degrees2Radians((float)yaw),
                    new TransformationState());

                //AddToOutput(string.Format("{0:F2} {1:F2} {2:F2} {3:F2} {4:F2} {5:F2}",
                //    ((float)(Math.Round(invKin.BestSolution[0] / Math.PI * 180f, 2))).ToString(),
                //    ((float)(Math.Round(invKin.BestSolution[1] / Math.PI * 180f, 2))).ToString(),
                //    ((float)(Math.Round(invKin.BestSolution[2] / Math.PI * 180f, 2))).ToString(),
                //    ((float)(Math.Round(invKin.BestSolution[3] / Math.PI * 180f, 2))).ToString(),
                //    ((float)(Math.Round(invKin.BestSolution[4] / Math.PI * 180f, 2))).ToString(),
                //    ((float)(Math.Round(invKin.BestSolution[5] / Math.PI * 180f, 2))).ToString()));

                List<Command> jointCommands = new List<Command>();

                jointCommands.Add(new Command(0, (int)(Controller.MX_PER_ROTATION / 2 + Controller.RADIANS2MX * invKin.BestSolution[0])));
                jointCommands.Add(new Command(1, (int)(Controller.MX_PER_ROTATION / 2 + Controller.RADIANS2MX * invKin.BestSolution[1])));
                jointCommands.Add(new Command(2, (int)(Controller.MX_PER_ROTATION / 2 + Controller.RADIANS2MX * invKin.BestSolution[2])));
                jointCommands.Add(new Command(3, (int)(Controller.MX_PER_ROTATION / 2 - Controller.RADIANS2MX * invKin.BestSolution[3])));
                jointCommands.Add(new Command(4, (int)(Controller.MX_PER_ROTATION / 2 + Controller.RADIANS2MX * invKin.BestSolution[4])));
                jointCommands.Add(new Command(5, (int)(Controller.AX_PER_ROTATION / 2 + Controller.RADIANS2AX * invKin.BestSolution[5])));
                jointCommands.Add(new Command(6, j7.Position));//(int)(Controller.AX_PER_ROTATION / 2 + Controller.RADIANS2AX * invKin.Angles[6])));

                //AddToOutput(string.Format("{0:F2} {1:F2} {2:F2} {3:F2} {4:F2} {5:F2}",
                //    jointCommands[0].GoalPosition,
                //    jointCommands[1].GoalPosition,
                //    jointCommands[2].GoalPosition,
                //    jointCommands[3].GoalPosition,
                //    jointCommands[4].GoalPosition,
                //    jointCommands[5].GoalPosition));

                armController.SetMultiGoalPositions(jointCommands, false);
                
                this.NotifyPropertyChanged(propertyName);

            }
            catch (Exception e)
            {
                AddToOutput(e.Message);
                Console.Beep();
            }

        }
        private double Radians2Degrees(double angle)
        {
            return (angle * (180.0 / Math.PI));
        }

        private double Degrees2Radians(double angle)
        {
            return (Math.PI * angle / 180.0);
        }


        private void OpenGripper()
        {
            j7.Position = 800;
            NotifyPropertyChanged("J7");
        }

        private void CloseGripper()
        {
            j7.Position = 512;
            NotifyPropertyChanged("J7");
        }

        internal void ToggleMonitor()
        {
            this.armController.MonitorEnabled = !armController.MonitorEnabled;
        }
    }
}
