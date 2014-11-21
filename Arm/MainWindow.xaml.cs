using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;

namespace ArmUI
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        private readonly ArmViewModel viewModel;
        private bool recording;

        public MainWindow()
        {
            this.viewModel = new ArmViewModel() { J1 = 2048, J2 = 2048, J3 = 2048, J4 = 2048, J5 = 2048, J6 = 512, J7 = 512 };

            this.DataContext = viewModel;

            InitializeComponent();
        }

        private void WindowLoaded(object sender, EventArgs e)
        {
        }

        private void WindowClosed(object sender, EventArgs e)
        {
        }

        private void Button_Click_ConectStiff(object sender, RoutedEventArgs e)
        {
            viewModel.Initialize(stiff: true);
        }

        private void Button_Click_ConectRelaxed(object sender, RoutedEventArgs e)
        {
            viewModel.Initialize(stiff: false);
        }

        private void Button_Click_Add(object sender, RoutedEventArgs e)
        {
            viewModel.AddCommand();
        }

        private void Button_Click_Play(object sender, RoutedEventArgs e)
        {
            viewModel.PlayScript();
        }

        private void Button_Click_Clear(object sender, RoutedEventArgs e)
        {
            viewModel.ClearScript();
        }

        private void Button_Click_Relax(object sender, RoutedEventArgs e)
        {
            viewModel.SetTorqueEnable(false);
        }

        private void Button_Click_Stiffen(object sender, RoutedEventArgs e)
        {
            viewModel.SetTorqueEnable(true);
        }

        private void Button_Click_AddHome(object sender, RoutedEventArgs e)
        {
            viewModel.AddCommandHome();
        }

        private void Button_Click_Save(object sender, RoutedEventArgs e)
        {
            viewModel.SaveScript();
        }

        private void Button_Click_Load(object sender, RoutedEventArgs e)
        {
            viewModel.LoadScript();
        }

        private void Button_Click_Reset(object sender, RoutedEventArgs e)
        {
            viewModel.Reset();
        }

        private void Button_Click_ClearOutput(object sender, RoutedEventArgs e)
        {
            viewModel.ClearOutput();

        }

        private void Button_Click_Record(object sender, RoutedEventArgs e)
        {
            if (!this.recording)
            {
                this.recording = true;
                viewModel.StartRecording();
            }
            else
            {
                this.recording = false;
                viewModel.StopRecording();
            }
        }

        private void Button_Click_InitKin(object sender, RoutedEventArgs e)
        {
            viewModel.InitializeKinematics();
        }

        private void Button_Click_ToggleMonitor(object sender, RoutedEventArgs e)
        {
            viewModel.ToggleMonitor();
        }

        private void Button_Click_Resting(object sender, RoutedEventArgs e)
        {
            viewModel.LoadAndPlayScriptDuringKinect(@"\\imself-cluster\mydocs\gershonp\My Documents\arm scripts\resting.arm");
        }

        private void Button_Click_KinStart(object sender, RoutedEventArgs e)
        {
            viewModel.LoadAndPlayScriptDuringKinect(@"\\imself-cluster\mydocs\gershonp\My Documents\arm scripts\KinStart.arm");
        }
    }
}
